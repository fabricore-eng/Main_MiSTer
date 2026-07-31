// -----------------------------------------------------------------------------
// s573mp3.cpp - Konami System 573 Digital I/O MP3 service, HPS side.
//
// The 573's DIO board (GX894) stores its music SCRAMBLED in a 32 MiB DRAM window
// that lives, in this core, in DDR3 where we can mmap it. Decision B / option (c)
// (System573_MiSTer docs/2026-07-29-p4b-decision-b-spike-result.md): we read that
// window directly, descramble it here, decode it with minimp3, and push PCM into
// a ring the fabric drains at 44100 Hz. The fabric keeps a position tracker paced
// by the consumption credit we report, because the game reads a "still streaming"
// bit derived from it.
//
// Shape and lifecycle are cloned from support/megadrive/mdplus.cpp: a ~5 ms
// throttled poll off user_io_poll(), no thread, no teardown. Core unload is a
// full Main process teardown (fpga_io.cpp app_restart), so the service dies with
// the core and cannot leak an mmap into the next one.
//
// The protocol is specified in System573_MiSTer/rtl/s573_hps_ext.v's header. That
// file is normative; this is the client. Two rules worth repeating because
// getting either wrong is silent rather than loud:
//
//   * The PTRS pointer words are IGNORED until we ack the current rst_epoch, and
//     the core-load reset itself bumps it -- so the first poll always acks and
//     achieves nothing else. That is by design: one code path for every reset.
//   * hps_cons_bytes is CUMULATIVE. Sending a per-poll delta would credit the
//     fabric again every poll and run its position (and the game-visible 0xae
//     bit 12) far ahead of the audio, while every honesty counter still read
//     GREEN. s573mp3_core.c owns that counter; do not compute it here.
//
// Released under the GNU GPL v2.
// -----------------------------------------------------------------------------

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>   // test-tone sine (S573MP3_TONE)

#include "../../user_io.h"
#include "../../spi.h"
#include "../../fpga_io.h"
#include "../../shmem.h"
#include "s573mp3.h"

extern "C" {
#include "s573mp3_core.h"
}

#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_STDIO
#define MINIMP3_IMPLEMENTATION
#include "../../lib/minimp3/minimp3.h"

// One MPEG-1 Layer III frame is 1152 samples; stereo int16 => 4608 bytes.
// The fabric ring counts 8-byte beats (2 stereo frames each), so 576 per frame.
#define BEATS_PER_FRAME      (1152u * 2u * 2u / 8u)

static struct
{
	int              active;
	s573_core_t      core;
	mp3dec_t         dec;
	volatile uint8_t *dio;     // scrambled source window (read-only for us)
	volatile uint8_t *pcm;     // PCM ring (write-only for us)
	int              adopted_baselines;
	int              warned_mono;
	int              warned_rate;
	uint32_t         last_poll;
	int              tone_en;      // S573MP3_TONE=1: substitute a generated sine
	int              tone_ph;      // sine phase, samples mod 44100
	int              hb_en;        // S573MP3_HB=1: periodic state line
	float            gain_l;       // MAS3507D output gain, applied to decoded PCM
	float            gain_r;       // (1.0 until the game sets one -- never boot muted)
	uint32_t         last_hb;
} s573;

static uint32_t now_ms()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

// ---- EXT_BUS exchanges (same-word response semantics; see the RTL header) ----

struct ptrs_reply { uint16_t fab_pcm_rd, fab_pos_lo, cfg_epoch, rst_epoch; };

static void ext_ptrs(uint16_t hps_pcm_wr, uint16_t cons, uint16_t rst_ack,
                     struct ptrs_reply *r)
{
	r->fab_pcm_rd = spi_uio_cmd_cont(CMD_573_PTRS);
	r->fab_pos_lo = spi_w(hps_pcm_wr);
	r->cfg_epoch  = spi_w(cons);
	r->rst_epoch  = spi_w(rst_ack);
	DisableIO();
}

static void ext_ctrl(uint16_t events, uint16_t flags, uint16_t *baselines)
{
	uint16_t b = spi_uio_cmd_cont(CMD_573_CTRL);
	spi_w(events);
	spi_w(flags);
	DisableIO();
	if (baselines) *baselines = b;
}

// Reads the whole descramble tuple. All eight words come from ONE fabric-side
// snapshot taken at the command strobe, so this cannot tear even if the game
// rewrites its setup registers underneath us.
static void ext_read_cfg(s573_cfg_t *c)
{
	c->epoch    = spi_uio_cmd_cont(CMD_573_MP3CFG);
	c->start_lo = spi_w(0);
	c->start_hi = spi_w(0);
	c->end_lo   = spi_w(0);
	c->end_hi   = spi_w(0);
	c->key1     = spi_w(0);
	c->key2     = spi_w(0);
	c->key3     = spi_w(0);
	c->flags    = spi_w(0);
	uint16_t g_ll_lo = spi_w(0);
	uint16_t g_rr_lo = spi_w(0);
	uint16_t g_hi    = spi_w(0);   /* {7'd0, gain_seen, rr[19:16], ll[19:16]} */
	c->gain_ll   = ((uint32_t)(g_hi & 0x000f) << 16) | g_ll_lo;
	c->gain_rr   = ((uint32_t)((g_hi >> 4) & 0x000f) << 16) | g_rr_lo;
	c->gain_seen = (g_hi >> 8) & 1;
	DisableIO();
}


// ---- MAS3507D output gain -----------------------------------------------------
// The game sets its own output level in the decoder's gain matrix and MUTES by
// writing zeros. The fabric decodes those writes (rtl/mas3507d_i2c.v) and hands
// them over on CMD_573_MP3CFG; we apply them here because our decode is host-side
// and there is no MAS3507D chip to apply them for us.
//
// Curve is MAME's mas3507d verbatim:
//     gain_to_db(v)  = round(20 * log10((0x100000 - v) / 0x80000))
//     percentage(v)  = v == 0 ? 0 : 10 ^ ((db + 6) / 20)
//
// NOTE, MEASURED 2026-07-31 -- this is NOT the clipping fix, and expecting it to
// be was wrong. ddrsbm asks for 0xAF3CD, which is -4 dB on that curve and so a
// multiplier of 1.2589 -- a BOOST. Unity sits near 0xBFBDE. Applying the game's
// gain faithfully therefore makes us ~2 dB LOUDER, not quieter, so it can only
// worsen the peak-pinned-at-0-dBFS measured on silicon. The clipping is a
// headroom problem in our SPU+MP3 mix budget and needs its own fix.
//
// What this DOES buy: correct level relative to the game's intent, and the mute
// (v == 0), which is real -- the game issues it at song end and on a failed stage.
// gain_seen == 0 means the game has never set a level, so apply unity rather than
// booting muted.

// Scale in place with saturation. Saturating (not wrapping) matters: a wrap turns
// a loud passage into buzzing garbage, which is far worse than a clipped peak.
static void s573_apply_gain(int16_t *pcm, int samples_stereo, float gl, float gr)
{
	if (gl == 1.0f && gr == 1.0f) return;
	for (int i = 0; i < samples_stereo; i++)
	{
		float l = (float)pcm[2*i]     * gl;
		float r = (float)pcm[2*i + 1] * gr;
		if (l >  32767.0f) l =  32767.0f; else if (l < -32768.0f) l = -32768.0f;
		if (r >  32767.0f) r =  32767.0f; else if (r < -32768.0f) r = -32768.0f;
		pcm[2*i]     = (int16_t)l;
		pcm[2*i + 1] = (int16_t)r;
	}
}

// ---- PCM ring ----------------------------------------------------------------
// Writes `frames` stereo int16 pairs at the current beat cursor, wrapping. The
// mapping is uncached (/dev/mem O_SYNC), so this is a straight store loop; a
// memcpy into it is fine but must not be re-ordered past the SPI exchange that
// advertises the new write pointer -- hence the barrier in the caller.
static void pcm_write(const int16_t *pcm, uint32_t bytes)
{
	uint32_t byte_off = (uint32_t)s573.core.pcm_wr * 8u;
	const uint8_t *src = (const uint8_t *)pcm;

	while (bytes)
	{
		uint32_t run = S573_PCM_BYTES - byte_off;
		if (run > bytes) run = bytes;
		memcpy((void *)(s573.pcm + byte_off), src, run);
		src      += run;
		bytes    -= run;
		byte_off  = (byte_off + run) & (S573_PCM_BYTES - 1);
	}
}

// ---- lifecycle ---------------------------------------------------------------

void s573mp3_reset()
{
	memset(&s573, 0, sizeof(s573));
}

static int s573mp3_open()
{
	s573.dio = (volatile uint8_t *)shmem_map(S573_DIO_PHYS, S573_DIO_SIZE);
	if (!s573.dio) { printf("s573mp3: cannot map the DIO window\n"); return 0; }

	s573.pcm = (volatile uint8_t *)shmem_map(S573_PCM_PHYS, S573_PCM_BYTES);
	if (!s573.pcm) { printf("s573mp3: cannot map the PCM ring\n"); return 0; }
	memset((void *)s573.pcm, 0, S573_PCM_BYTES);

	s573_core_init(&s573.core);
	mp3dec_init(&s573.dec);
	s573.active = 1;
	// Diagnostics, both opt-in via env so a normal boot is byte-identical in behaviour.
	{
		const char *t = getenv("S573MP3_TONE");
		const char *h = getenv("S573MP3_HB");
		s573.tone_en = (t && *t && *t != '0');
		s573.hb_en   = (h && *h && *h != '0');
		if (s573.tone_en) printf("s573mp3: TEST TONE enabled -- 440 Hz sine replaces decoded MP3\n");
		if (s573.hb_en)   printf("s573mp3: heartbeat enabled\n");
	}
	// Loud on purpose: design risk #6 is a stale forked binary silently
	// disabling MP3. If this line is missing from the log, the service is not
	// running, whatever the core is doing.
	printf("s573mp3: service up (DIO %08x, PCM ring %08x, %u beats)\n",
	       S573_DIO_PHYS, S573_PCM_PHYS, S573_PCM_BEATS);
	return 1;
}

// Periodic state line (S573MP3_HB=1). Every field here answers a specific "which
// half is broken" question: epochs prove the SPI mailbox round-trips at all;
// in_len/cons proves we are fetching the game's scrambled window; frames proves
// minimp3 is decoding; wr/rd proves the fabric is DRAINING what we wrote (rd
// advancing is the only evidence the audio side is alive).
static void s573mp3_heartbeat(const s573_core_t *c, const struct ptrs_reply *r)
{
	if (!s573.hb_en) return;
	uint32_t now = now_ms();
	if (now - s573.last_hb < 2000) return;
	s573.last_hb = now;
	/* desc.cur is the ONLY way to see the enable-toggle rewind bug from outside:
	 * on a pause/resume with no setup rewrite it must stay put, not snap back to
	 * mp3_start. Printed relative to the window so a rewind is obvious by eye. */
	printf("s573mp3: rst=%u/%u ack=%d cfg=%u/%u have=%d | cur=+%u/%u | in_len=%u pos=%u cons=%u "
	       "| frames=%u sync=%u idle=%u | wr=%u rd=%u free=%u | ctrl=%04x echo_bad=%u\n",
	       c->rst_epoch, r->rst_epoch, c->rst_acked, c->cfg_epoch, r->cfg_epoch, c->have_cfg,
	       (unsigned)(c->desc.cur - c->desc.mp3_start),
	       (unsigned)(c->desc.mp3_end - c->desc.mp3_start),
	       c->in_len, c->in_pos, c->cons_bytes,
	       c->frames, c->sync_cnt, c->idle_cnt,
	       c->pcm_wr, c->pcm_rd, s573_core_pcm_free(c),
	       c->ctrl_flags, c->ddrsbm_echo_bad);
}

// ---- the poll ----------------------------------------------------------------

void s573mp3_poll()
{
	if (!is_573()) return;

	uint32_t now = now_ms();
	if (now - s573.last_poll < 5) return;
	s573.last_poll = now;

	if (!s573.active && !s573mp3_open()) { s573.active = -1; return; }
	if (s573.active < 0) return;

	s573_core_t *c = &s573.core;

	// 1. pointer / epoch exchange. Anything we send is ignored by the fabric
	//    until the reset epoch is acked, which is exactly what we want.
	struct ptrs_reply r;
	__sync_synchronize();          // our PCM stores land before we advertise them
	// rst_ack always carries the epoch we last saw: if it matches the fabric's,
	// the pointer words apply; if it does not, the fabric ignores them and we
	// re-ack below. Same word either way -- no branch needed.
	ext_ptrs(c->pcm_wr, s573_core_credit_word(c), c->rst_epoch, &r);
	c->pcm_rd = r.fab_pcm_rd;

	// 2. reset handling. The core-load reset bumps rst_epoch too, so this fires
	//    on the very first poll and gives every reset one code path.
	if (r.rst_epoch != c->rst_epoch || !c->rst_acked)
	{
		if (r.rst_epoch != c->rst_epoch)
		{
			s573_core_on_reset(c, r.rst_epoch);
			mp3dec_init(&s573.dec);
			s573.adopted_baselines = 0;
			memset((void *)s573.pcm, 0, S573_PCM_BYTES);
		}
		// ack it; pointers thaw from the NEXT poll
		ext_ptrs(0, 0, c->rst_epoch, &r);
		c->rst_acked = 1;
		return;
	}

	// 3. adopt the fabric's cumulative event baselines once per reset, or our
	//    zero-based counters would diff as garbage deltas on the first CTRL.
	if (!s573.adopted_baselines)
	{
		uint16_t base = 0;
		// Nothing to push here any more: the descramble scheme is the fabric's
		// (OSD bit O[101]) and the PCM drain follows the game's own play/stop,
		// both arriving in the MP3CFG word. This exchange exists only to ADOPT
		// the fabric's cumulative event baselines.
		ext_ctrl(0, c->ctrl_flags, &base);
		c->sync_cnt = (uint8_t)(base >> 8);
		c->idle_cnt = (uint8_t)(base & 0xFF);
		s573.adopted_baselines = 1;
		return;
	}

	// 4. re-read the descramble config when the fabric says it moved
	if (r.cfg_epoch != c->cfg_epoch || !c->have_cfg)
	{
		s573_cfg_t cfg;
		ext_read_cfg(&cfg);
		int drain_before = (c->ctrl_flags & S573_CTRL_DRAIN_EN) ? 1 : 0;
		s573_core_apply_cfg(c, &cfg);
		/* WHY THIS LINE: two completely different things can silence the music --
		 * the GAME asking us to stop (enables clear, this path) and the song data
		 * simply running out (the window-exhausted backstop in 5b, which prints its
		 * own line). In the log they were indistinguishable, so "the music stopped"
		 * could not be attributed to either. Reported 2026-07-31 that songs keep
		 * playing past a failed stage and across menus, with the hypothesis that the
		 * enable path never fires at all and only exhaustion ever stops anything.
		 * This makes that directly countable. */
		{
			int drain_after = (c->ctrl_flags & S573_CTRL_DRAIN_EN) ? 1 : 0;
			if (s573.hb_en && drain_after != drain_before)
				printf("s573mp3: DRAIN %s via ENABLES (flags=%04x mp3_en=%d stream_en=%d)\n",
				       drain_after ? "ON" : "OFF", cfg.flags,
				       (int)!!(cfg.flags & S573_CFG_MP3_ENABLE),
				       (int)!!(cfg.flags & S573_CFG_STREAM_ENABLE));
		}
		// gain_seen == 0: the game has not set a level yet -> unity, NOT mute.
		s573.gain_l = cfg.gain_seen ? s573_core_gain_mult(cfg.gain_ll) : 1.0f;
		s573.gain_r = cfg.gain_seen ? s573_core_gain_mult(cfg.gain_rr) : 1.0f;
		// The drain is derived from flags bits MP3_ENABLE/STREAM_ENABLE (fabric
		// fpga_ctrl[14:13]). If those never clear, the drain never clears and the
		// music plays on past the game's stop -- so log the RAW flags on every
		// config adoption. One line per epoch move, not per poll.
		if (s573.hb_en)
			printf("s573mp3: CFG epoch=%u flags=%04x (mp3_en=%d stream_en=%d ddrsbm=%d) start=%08x end=%08x\n",
			       cfg.epoch, cfg.flags,
			       (int)!!(cfg.flags & S573_CFG_MP3_ENABLE),
			       (int)!!(cfg.flags & S573_CFG_STREAM_ENABLE),
			       (int)!!(cfg.flags & S573_CTRL_DDRSBM),
			       ((uint32_t)cfg.start_hi << 16) | cfg.start_lo,
			       ((uint32_t)cfg.end_hi   << 16) | cfg.end_lo);
	}

	// 5. decode while there is ring space. STRICTLY space-gated -- never a
	//    byte/time budget. See s573mp3_core.h.
	int produced = 0;

	// TEST TONE (S573MP3_TONE=1): substitute a generated 440 Hz sine for the decoded
	// MP3 and write it through the SAME ring path. This bisects the transport: hearing
	// the tone proves ring writes -> pointer accounting -> the fabric's 44100 Hz drain
	// -> s573_audio_mix -> HDMI all work, so any silence is upstream (fetching or
	// descrambling or decoding the game's data). Hearing nothing proves the delivery
	// half is dead and the decode half is irrelevant for now. Diagnostic only -- it is
	// off unless the env var is set, so a normal boot is unaffected.
	if (s573.tone_en)
	{
		while (s573_core_should_decode(c, BEATS_PER_FRAME))
		{
			static short tone[1152 * 2];
			for (int i = 0; i < 1152; i++)
			{
				// 440 Hz at 44100 Hz, quarter scale so it is unmistakable but not harsh
				double th = 2.0 * 3.14159265358979 * 440.0 * (double)s573.tone_ph / 44100.0;
				short v = (short)(8000.0 * sin(th));
				tone[2*i] = v; tone[2*i+1] = v;
				s573.tone_ph = (s573.tone_ph + 1) % 44100;
			}
			pcm_write(tone, sizeof(tone));
			s573_core_wrote_pcm(c, (uint16_t)(sizeof(tone) / 8u));
			produced = 1;
		}
		if (!produced) s573_core_note_idle(c);
		ext_ctrl(s573_core_ctrl_events(c), c->ctrl_flags, NULL);
		s573mp3_heartbeat(c, &r);
		return;
	}

	while (s573_core_should_decode(c, BEATS_PER_FRAME))
	{
		mp3dec_frame_info_t info;
		short pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
		int samples;

		s573_core_fill(c, (const uint8_t *)s573.dio);

		uint32_t avail = c->in_len - c->in_pos;
		if (!avail) break;                       // window exhausted

		samples = mp3dec_decode_frame(&s573.dec, c->in + c->in_pos, (int)avail,
		                              pcm, &info);
		if (info.frame_bytes <= 0) break;        // need more input
		s573_core_consume(c, (uint32_t)info.frame_bytes);

		if (samples > 0)
		{
			// minimp3 returns samples PER CHANNEL and fills samples*channels
			// values. The fabric ring is interleaved stereo, so a MONO frame
			// must be expanded -- writing it raw would put half the data in and
			// desync the ring against the 44100 Hz drain, which is silent
			// corruption, not an error. 573 audio should always be stereo; if
			// it is not, say so once and still play it correctly.
			uint32_t bytes;
			if (info.channels == 2)
			{
				bytes = (uint32_t)samples * 4u;
				s573_apply_gain(pcm, samples, s573.gain_l, s573.gain_r);
				pcm_write(pcm, bytes);
			}
			else if (info.channels == 1)
			{
				static short st[MINIMP3_MAX_SAMPLES_PER_FRAME];
				for (int i = 0; i < samples; i++) { st[2*i] = pcm[i]; st[2*i+1] = pcm[i]; }
				bytes = (uint32_t)samples * 4u;
				s573_apply_gain(st, samples, s573.gain_l, s573.gain_r);
				pcm_write(st, bytes);
				if (!s573.warned_mono) { printf("s573mp3: MONO frame (%d ch) -- expanding to stereo\n", info.channels); s573.warned_mono = 1; }
			}
			else
			{
				if (!s573.warned_mono) { printf("s573mp3: unexpected channel count %d -- dropping frame\n", info.channels); s573.warned_mono = 1; }
				continue;
			}
			// The fabric drains at a hard 44100 Hz. A different sample rate is
			// not something we can resample here -- it would simply play at the
			// wrong speed, and the chart would drift against the music.
			if (info.hz != 44100 && !s573.warned_rate)
			{
				printf("s573mp3: WARNING sample rate %d Hz, fabric drains at 44100 -- audio will play at the wrong speed\n", info.hz);
				s573.warned_rate = 1;
			}
			s573_core_wrote_pcm(c, (uint16_t)(bytes / 8u));
			produced = 1;
		}
	}
	if (!produced) s573_core_note_idle(c);

	// 5b. SONG END -- a SECOND end condition, not the only one.
	//
	//     CORRECTION 2026-07-31 (later the same day): the claim that used to sit
	//     here -- "ddrsbm asserts both enables once and never clears them again,
	//     every config adoption reads flags=000f" -- is FALSE. It came from a
	//     16-sample window that happened to fall entirely inside playback. A full
	//     13-hour board log says the opposite:
	//         flags=000f (stream_en=1)  1,828,242
	//         flags=000b (stream_en=0)    846,906
	//     and the drain does follow it -- 2,170 heartbeats show ctrl=0000 with
	//     frames>0, i.e. drain OFF after decoding had started. MAME agrees: every
	//     playback stop is fpga_ctrl bit14 going low (bit13 MP3_ENABLE, bit14
	//     STREAMING_ENABLE, per k573fpga.h and rtl/k573_mp3stream.v:140).
	//
	//     So apply_cfg's enable-driven stop in s573mp3_core.c:58 IS the primary
	//     path and it works. This block stays as a backstop for the case where the
	//     window runs out without the game clearing the enables: the descrambled
	//     window is exhausted (cur >= mp3_end -- the same bound the fabric's own
	//     stream_en uses) AND everything we decoded has been drained. Waiting for
	//     the ring to empty lets the tail play out instead of truncating it.
	//
	//     STILL OPEN: the reported "music keeps playing when a stage is FAILED".
	//     Neither path above has been shown to miss that case; the remaining
	//     suspect is the MAS3507D output gain matrix (I2C bank1 0x7f8..0x7fb,
	//     zero = mute), which rtl/mas3507d_i2c.v ACKs and DROPS -- so if the game
	//     silences a failed stage by muting the decoder rather than by clearing
	//     the enables, we would never see it. Unconfirmed: reach a real failed
	//     stage in the MAME oracle and check which of the two the game uses.
	if ((c->ctrl_flags & S573_CTRL_DRAIN_EN)
	    && c->desc.cur >= c->desc.mp3_end
	    && c->pcm_wr == c->pcm_rd)
	{
		c->ctrl_flags &= (uint16_t)~S573_CTRL_DRAIN_EN;
		mp3dec_init(&s573.dec);
		if (s573.hb_en)
			printf("s573mp3: DRAIN OFF via EXHAUSTION -- window done (cur=%08x end=%08x) and ring drained, drain OFF\n",
			       c->desc.cur, c->desc.mp3_end);
	}

	// 6. hand the fabric this poll's cumulative event counts + control flags
	ext_ctrl(s573_core_ctrl_events(c), c->ctrl_flags, NULL);

	s573mp3_heartbeat(c, &r);
}
