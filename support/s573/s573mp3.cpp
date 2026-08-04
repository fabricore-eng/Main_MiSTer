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
	// Consecutive polls that decoded nothing. See the WRITER RACE note at the
	// decode loop: a stall means we have caught up with the game's uploader, not
	// that the data is bad, so we wait rather than skipping forward.
	uint32_t         stall_polls;
	int              warned_stall;
	uint32_t         last_underrun;  // previous heartbeat's underrun_cnt, for the delta
	// Underrun count latched when the CURRENT song's config was adopted. The
	// per-song delta is the whole measurement: every 44100 Hz sample-clock that
	// found the buffer empty is one sample of audio that never played while the
	// game's position register advanced anyway, so
	//     desync_seconds = (underrun - song_underrun_base) / 44100
	// EXACTLY. Zero across a song is not "probably fine", it is proof the chart
	// and the audio advanced together sample for sample.
	uint32_t         song_underrun_base;
	/* One PLAYBACK EPISODE = a contiguous stretch with the drain on. Tracked so the
	 * log says WHICH BYTES were played, not just whether they arrived on time. The
	 * remaining attract artifact produces no underrun at all -- correctly paced
	 * audio that is the wrong audio -- so timing instrumentation cannot see it, and
	 * an episode ledger can: a window played twice shows two episodes over the same
	 * range, a stale window shows an episode on the OLD window after a new song
	 * started, and a restart-from-the-top shows a second episode beginning at
	 * mp3_start. */
	int              ep_open;
	uint32_t         ep_t0, ep_cur0, ep_frames0, ep_start, ep_end;
} s573;

// How many consecutive no-progress polls to wait before stepping past the byte
// we are stuck on. Polls are ~5 ms, so this is ~1 s of patience -- far longer
// than the uploader's ~85 ms gap between 4 KB chunks, and short enough that a
// genuinely corrupt byte degrades to a brief glitch instead of a hung song.
#define S573_STALL_LIMIT 200u
/* Largest MPEG-1 Layer III frame the 573 produces (128 kbps / 44.1 kHz, padded
 * = 418 bytes) with room to spare. Below this we cannot have a whole frame, so
 * a short buffer at the writer's edge means "wait", never "bad data". */
#define S573_MIN_FRAME_BYTES 1024u

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

// CMD_573_STATUS: fabric observability. Implemented in the fabric since the
// transport was built and, until 2026-08-03, never once read -- so starvation was
// unobservable from the HPS side even though the fabric was counting it.
//
// underrun_cnt is SATURATED to 16 bits on the way up (emu.sv:2737-2738), so
// 0xFFFF means "instrument exhausted", not "65535". And it is CUMULATIVE with a
// structural component: drain_en leads the write pointer by one poll at every
// song start, so a couple of hundred at the top of a song is normal. Read it as a
// DELTA across a mid-song window, never as an absolute.
struct status_reply { uint16_t flags, buf_level; uint32_t underrun, ram_wr; };

static void ext_status(struct status_reply *st)
{
	uint16_t lo, hi;
	st->flags     = spi_uio_cmd_cont(CMD_573_STATUS);
	lo            = spi_w(0);
	st->buf_level = spi_w(0);
	hi            = spi_w(0);   // word3: high half of the same snapshot
	st->underrun  = ((uint32_t)hi << 16) | lo;
	lo            = spi_w(0);   // word4/5: the game's DIO-RAM write frontier
	hi            = spi_w(0);
	st->ram_wr    = ((uint32_t)hi << 16) | lo;
	DisableIO();
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
// Collapse the PCM ring to AT MOST ONE undrained beat. Returns what was pending.
//
// WHY rd+1 AND NOT rd. The ring reader can be parked mid-beat in S_PUSH0/S_PUSH1
// (rtl/s573_pcm_ring.v:125-129) with a beat already fetched from DDR3 and
// fab_rd_ptr NOT yet incremented -- exactly the state wr_full creates at a stop.
// Targeting rd would let that pending increment land on re-arm and put fab_rd_ptr
// one beat AHEAD of us; have_data is a bare inequality (s573_pcm_ring.v:83), not
// an ordering compare, so the reader would then chase us the long way round the
// 16-bit pointer space -- 65535 beats, ~2.97 s of stale ring, twice the bug we
// are fixing. rd+1 is a fixed point in BOTH parked states: parked mid-beat the
// retiring push lands exactly on us; parked idle it costs one stale beat
// (2 stereo samples, 45 us). For the same reason we never rest at pend == 0.
// How far AHEAD of the last-seen read pointer a live collapse must land.
//
// Collapsing to rd+1 is only safe when the reader has STOPPED. While it is
// draining, c->pcm_rd is a snapshot that is already stale by up to one poll, and
// have_data is a bare inequality (rtl/s573_pcm_ring.v:83) -- so landing even one
// beat BEHIND the true read pointer sends the reader 65535 beats the long way
// round, which is far worse than the stale audio we came to drop.
//
// The reader drains 44100 samples/s = 22050 beats/s. A poll is ~5 ms, so it can
// advance ~110 beats between our snapshot and the write. 256 is >2x that, and
// costs only 256*2/44100 = 11.6 ms of the old song left in the ring -- against
// the ~1.47 s it removes.
#define PCM_COLLAPSE_MARGIN 256u

// Drop undrained PCM. `margin` is where the write pointer lands relative to the
// last-seen read pointer: 1 when the reader is stopped, PCM_COLLAPSE_MARGIN when
// it is still running. Returns the beats dropped.
static uint16_t pcm_ring_collapse_m(s573_core_t *c, uint16_t margin)
{
	uint16_t pend = (uint16_t)(c->pcm_wr - c->pcm_rd);
	// Nothing to do if the ring already holds less than the margin -- moving the
	// write pointer FORWARD there would fabricate beats of garbage audio.
	if (pend > margin) c->pcm_wr = (uint16_t)(c->pcm_rd + margin);
	return pend;
}

static uint16_t pcm_ring_collapse(s573_core_t *c)
{
	return pcm_ring_collapse_m(c, 1u);
}

static void pcm_write(const int16_t *pcm, uint32_t bytes)
{
	/* pcm_wr is a 16-bit LAPPING pointer (beat index + wrap MSB), so bit15 MUST
	 * be masked off before it becomes a byte offset. Unmasked, any pcm_wr above
	 * 0x8000 makes `S573_PCM_BYTES - byte_off` underflow below, `run > bytes`
	 * then clamps it, and the memcpy walks off the end of the 256 KiB mapping
	 * established at s573mp3_open(). The mask belongs HERE, at the consumer that
	 * needs a byte offset -- not in s573_core_wrote_pcm, whose value is what the
	 * fabric's full-width emptiness compare consumes. */
	uint32_t byte_off = ((uint32_t)s573.core.pcm_wr * 8u) & (S573_PCM_BYTES - 1);
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
	s573.stall_polls = 0; s573.warned_stall = 0;  // new window: fresh patience budget
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
	/* Starvation is the reason the game-visible position register and real time
	 * can drift apart, so it belongs on the same line as the pointers -- printed
	 * as a DELTA since the previous heartbeat, which is the only reading that
	 * means anything (see ext_status). */
	struct status_reply st;
	ext_status(&st);
	uint32_t ur_delta = st.underrun - s573.last_underrun;
	s573.last_underrun = st.underrun;
	/* song=N is the verdict field: underruns accrued since THIS song's config was
	 * adopted, i.e. exactly how many 44100 Hz sample-clocks played silence while
	 * the position register advanced regardless. song=0 for a whole song means the
	 * chart and the audio cannot have drifted; anything else is a desync of
	 * song/44100 seconds, which is the measurement rather than an impression. */
	uint32_t ur_song = st.underrun - s573.song_underrun_base;

	printf("s573mp3: rst=%u/%u ack=%d cfg=%u/%u have=%d | cur=+%u/%u | in_len=%u pos=%u cons=%u "
	       "| frames=%u sync=%u idle=%u | wr=%u rd=%u free=%u | ctrl=%04x echo_bad=%u "
	       "| underrun=%u(+%u) song=%u(%.3f s) buf=%u sflags=%04x | wr_ptr=%08x lead=%+d\n",
	       c->rst_epoch, r->rst_epoch, c->rst_acked, c->cfg_epoch, r->cfg_epoch, c->have_cfg,
	       (unsigned)(c->desc.cur - c->desc.mp3_start),
	       (unsigned)(c->desc.mp3_end - c->desc.mp3_start),
	       c->in_len, c->in_pos, c->cons_bytes,
	       c->frames, c->sync_cnt, c->idle_cnt,
	       c->pcm_wr, c->pcm_rd, s573_core_pcm_free(c),
	       c->ctrl_flags, c->ddrsbm_echo_bad,
	       st.underrun, ur_delta, ur_song, (double)ur_song / 44100.0,
	       st.buf_level, st.flags,
	       st.ram_wr, (int)((int64_t)st.ram_wr - (int64_t)c->desc.cur));
}

// Open/close a playback episode. Closing prints the ledger line.
static void ep_open_now(const s573_core_t *c)
{
	if (s573.ep_open) return;
	s573.ep_open   = 1;
	s573.ep_t0     = now_ms();
	s573.ep_cur0   = c->desc.cur;
	s573.ep_frames0 = c->frames;
	s573.ep_start  = c->desc.mp3_start;
	s573.ep_end    = c->desc.mp3_end;
}

static void ep_close_now(const s573_core_t *c, const char *why)
{
	if (!s573.ep_open) return;
	s573.ep_open = 0;
	if (!s573.hb_en) return;
	uint32_t bytes  = (c->desc.cur > s573.ep_cur0) ? (c->desc.cur - s573.ep_cur0) : 0;
	uint32_t frames = c->frames - s573.ep_frames0;
	/* frames*1152/44100 is the EXACT audio length decoded -- no bitrate guess. */
	double audio = (double)frames * 1152.0 / 44100.0;
	double wall  = (double)(now_ms() - s573.ep_t0) / 1000.0;
	printf("s573mp3: PLAYED -- window %08x..%08x bytes %08x..%08x (%u) "
	       "audio %.3f s wall %.3f s drift %+.3f s [%s]\n",
	       s573.ep_start, s573.ep_end, s573.ep_cur0, c->desc.cur, bytes,
	       audio, wall, wall - audio, why);
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

	// Advance the PLAYBACK credit to the read cursor we just learned. This is what
	// makes the game's chart follow the speaker instead of the decoder -- see the
	// credit-pacing note in s573mp3_core.h. It must run AFTER pcm_rd is refreshed
	// and BEFORE the next credit_word() is sent, which is exactly here.
	s573_core_credit_drained(c);

	// 1b. (was: DEFERRED PCM-RING FLUSH.)
	//
	//     REMOVED 2026-08-03. The flush is now unconditional at the re-arm site in
	//     step 4, with PCM_COLLAPSE_MARGIN making it safe against a moving reader.
	//
	//     The deferral was the right instinct about the wrong risk. It correctly
	//     identified that landing hps_wr_ptr even ONE beat BEHIND fab_rd_ptr is
	//     catastrophic -- have_data is `!=`, not an ordering compare
	//     (rtl/s573_pcm_ring.v:83), so the reader would walk 65535 beats before the
	//     two could meet again. But the chosen remedy was to WAIT for a poll with
	//     the drain off, and then to DROP the intent entirely if the drain came back
	//     first. On a title that re-arms while still draining, that "if" is the
	//     common case, so the flush usually never happened: three "PCM FLUSH
	//     deferred" in one measured session, each leaving ~1.47 s of the previous
	//     song in the ring, which desynchronises the chart by exactly that much
	//     (see the note at the re-arm site).
	//
	//     Landing AHEAD of the reader by a margin larger than it can travel in a
	//     poll gets the same safety without ever having to wait -- so there is no
	//     longer any deferred state to service here.

	// 2. reset handling. The core-load reset bumps rst_epoch too, so this fires
	//    on the very first poll and gives every reset one code path.
	if (r.rst_epoch != c->rst_epoch || !c->rst_acked)
	{
		if (r.rst_epoch != c->rst_epoch)
		{
			s573_core_on_reset(c, r.rst_epoch);
			mp3dec_init(&s573.dec);
			s573.stall_polls = 0; s573.warned_stall = 0;  // new window: fresh patience budget
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
		/* Close the episode BEFORE apply_cfg if the WINDOW is about to change --
		 * afterwards desc.cur has been re-based and the ledger would report the new
		 * window's range against the old window's playback. A window swap with the
		 * drain left on is exactly the case that would otherwise be invisible. */
		{
			uint32_t nstart = ((uint32_t)cfg.start_hi << 16) | cfg.start_lo;
			uint32_t nend   = ((uint32_t)cfg.end_hi   << 16) | cfg.end_lo;
			if (s573.ep_open && (nstart != s573.ep_start || nend != s573.ep_end))
				ep_close_now(c, "window swap");
		}
		int rearmed      = s573_core_apply_cfg(c, &cfg);
		/* ...and reopen against the new window if the drain never dropped. */
		if (!s573.ep_open && (c->ctrl_flags & S573_CTRL_DRAIN_EN)) ep_open_now(c);

		/* A NEW WINDOW -- so the PCM still sitting in the ring belongs to a song we
		 * are never going to play again. Arm a flush.
		 *
		 * THE ORACLE. MAME's update_mp3_decode_state() runs on any write to
		 * start/end/key1-3 and does cur = start, re-seed keys, frame_counter = 0,
		 * reset_counter() AND mas3507d->reset_playback(), which throws away already
		 * DECODED audio as well as the input FIFO (573
		 * docs/2026-07-03-p4-mp3-pacing-model.md:87-89). We never have -- the known
		 * reset_playback gap (docs/2026-07-31-mp3-audio-WORKING.md:94-97). The
		 * difference is depth, not kind: MAME buffers about one 1152-sample frame,
		 * we buffer a 32768-beat ring that should_decode deliberately keeps within
		 * 576 beats of full, i.e. 1.46-1.49 s of the previous song.
		 *
		 * GATED ON `rearmed`, NOT ON THE ENABLE EDGE -- this is the load-bearing
		 * choice. cfg_epoch also moves on fpga_ctrl[14:13], so a bare play/stop
		 * toggle lands in apply_cfg too. On such a toggle the PCM at fab_rd_ptr is
		 * NOT stale: it is exactly the audio the resume is meant to continue with.
		 * Dropping it while desc.cur is preserved would fast-forward the song by up
		 * to 1.49 s -- the rewind's evil twin, and just as much a violation of "an
		 * enable-only toggle must not move the stream" as re-initialising the
		 * descrambler was. MAME draws the line in the same place: set_fpga_ctrl's
		 * reset_playback() never touches mp3_cur_addr or the key schedule, while
		 * update_mp3_decode_state() -- our `rearmed` -- does both. So: flush where
		 * MAME re-inits the window, and nowhere else. */
		if (rearmed)
		{
			mp3dec_init(&s573.dec);        // drop any partially decoded frame
			/* A new window starts its WRITER RACE patience budget fresh. Carrying
			 * the previous song's stall count over would spend this song's budget
			 * before it began -- and this is the site that matters, because a
			 * streamed song arms its window with almost nothing uploaded yet. */
			s573.stall_polls = 0; s573.warned_stall = 0;

			/* IMMEDIATE vs DEFERRED, decided by the drain state BEFORE this
			 * adoption -- not after it.
			 *
			 * ddrsbm delivers a new song as ONE adoption carrying the new window
			 * AND stream_en=1 together (observed: epoch N flags=000f with a fresh
			 * start/end, after a bare stop at N-1 flags=000b). So at a re-arm the
			 * drain is going ON, and a flush deferred to "once the drain is off"
			 * never fires at all -- measured on de10: zero flushes across a full
			 * attract cycle while music was demonstrably playing.
			 *
			 * When drain_before is 0 the deferral is also unnecessary: the fabric
			 * reader has been parked since the stop MANY polls ago (seconds, vs the
			 * ~240 us it needs to back up on wr_full), so fab_pcm_rd read at the top
			 * of this poll is stable. Flush right here, before step 5 writes any of
			 * the new song's PCM into the ring.
			 *
			 * When drain_before is 1 -- a new window while still draining -- the
			 * read pointer IS moving, so collapsing against it could land us behind
			 * it, and have_data is a bare inequality: the reader would chase us
			 * 65535 beats the long way round. Defer that case to 1b and say so
			 * rather than risk it. Not observed on ddrsbm. */
			// FLUSH NOW IN BOTH CASES, with the margin doing the safety work.
			//
			// Deferring the draining case (and dropping the intent when the drain
			// came back first) meant the ring kept ~1.47 s of the PREVIOUS song.
			// That misaligns the chart, because the fabric anchors its sample
			// counter at t=0 on the first DECODE of the new song and then advances
			// it on DRAINED samples (rtl/k573dio.v:405-410). With stale audio still
			// queued, the counter spends the old song's remaining 1.47 s before the
			// new song is audible at all -- so every arrow lands that far early.
			// Measured 2026-08-03: three "PCM FLUSH deferred" in one session, and
			// the two flushes that did fire dropped 1.470 s and 1.467 s.
			//
			// A live collapse is safe as long as we land AHEAD of where the reader
			// can have reached -- see PCM_COLLAPSE_MARGIN.
			uint16_t dropped = pcm_ring_collapse_m(
				c, drain_before ? (uint16_t)PCM_COLLAPSE_MARGIN : 1u);
			if (s573.hb_en && dropped > 1)
				printf("s573mp3: PCM FLUSH -- dropped %u undrained beats (%.3f s) from the previous song%s, wr=%u rd=%u\n",
				       dropped, (double)dropped * 2.0 / 44100.0,
				       drain_before ? " (live, margin 256)" : "",
				       c->pcm_wr, c->pcm_rd);
		}

		/* ADOPT THE EPOCH THIS TEST ACTUALLY COMPARES.
		 * The condition above tests r.cfg_epoch -- the FULL 16-bit counter, snapshotted
		 * into the hot PTRS poll (s573_hps_ext.v:324). But apply_cfg adopts cfg.epoch,
		 * and MP3CFG word0 carries only the LOW 8 BITS by design:
		 *   io_dout <= {8'd0, cfg_epoch[7:0]};      (s573_hps_ext.v:335, doc'd at :127)
		 * Below 256 the two agree by luck. The moment the fabric's counter passes 255
		 * they can NEVER be equal again, so this branch re-fires on EVERY poll: it
		 * re-applies a stale config -- re-arming DRAIN_EN from that config's stream_en
		 * -- and resets in_len/in_pos/cons_bytes, so decode cannot advance either. The
		 * game's stop is overwritten a few ms later, every time: THE MUSIC NEVER STOPS.
		 * Measured on de10 2026-08-02: healthy while the epoch was <=247, then 229/229
		 * heartbeats diverged with adopted == fabric % 256 exactly, and the drain
		 * oscillated ON-via-ENABLES/OFF-via-EXHAUSTION ~70k times.
		 * This also retires the old "846,906 stream_en=0 adoptions prove the firmware
		 * sees stops" reading -- that count was this runaway's signature, not evidence.
		 * The fabric is correct as designed; the 8-bit word is an identity echo. */
		if (s573.hb_en && (uint8_t)(r.cfg_epoch & 0xff) != (uint8_t)(cfg.epoch & 0xff))
			printf("s573mp3: WARNING torn cfg read -- PTRS epoch %u (low byte %u) != MP3CFG echo %u; re-reading next poll\n",
			       r.cfg_epoch, (unsigned)(r.cfg_epoch & 0xff), (unsigned)cfg.epoch);
		c->cfg_epoch = r.cfg_epoch;
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
			if (drain_after != drain_before)
			{
				if (s573.hb_en)
					printf("s573mp3: DRAIN %s via ENABLES (flags=%04x mp3_en=%d stream_en=%d)\n",
					       drain_after ? "ON" : "OFF", cfg.flags,
					       (int)!!(cfg.flags & S573_CFG_MP3_ENABLE),
					       (int)!!(cfg.flags & S573_CFG_STREAM_ENABLE));
				if (drain_after) ep_open_now(c); else ep_close_now(c, "enables");
			}
		}
		// gain_seen == 0: the game has not set a level yet -> unity, NOT mute.
		s573.gain_l = cfg.gain_seen ? s573_core_gain_mult(cfg.gain_ll) : 1.0f;
		s573.gain_r = cfg.gain_seen ? s573_core_gain_mult(cfg.gain_rr) : 1.0f;
		// The drain is derived from flags bits MP3_ENABLE/STREAM_ENABLE (fabric
		// fpga_ctrl[14:13]). If those never clear, the drain never clears and the
		// music plays on past the game's stop -- so log the RAW flags on every
		// config adoption. One line per epoch move, not per poll.
		/* Re-base the per-song underrun measurement HERE, at config adoption --
		 * the moment the game hands us a new song window. Basing it anywhere
		 * later (first decode, first drain) would fold the song's own start-up
		 * starvation into the previous song's total and read as a clean run. */
		{
			struct status_reply st0;
			ext_status(&st0);
			s573.song_underrun_base = st0.underrun;
			/* The heartbeat's first sample of `lead` arrives up to 2 s after a
			 * song starts, by which point a streaming writer has run well ahead
			 * and the reading looks safe. The audible wrong-song bit at the top
			 * of the attract video happens INSIDE that blind window, so measure
			 * the frontier at adoption too: negative here means we are about to
			 * decode bytes the game has not written for this window yet, which
			 * is stale content from whatever occupied that region before. */
			if (s573.hb_en)
			{
				uint32_t st = ((uint32_t)cfg.start_hi << 16) | cfg.start_lo;
				printf("s573mp3: SONG START t=%u -- window %08x..%08x, writer at %08x (lead %+d)\n",
				       now_ms(), st, ((uint32_t)cfg.end_hi << 16) | cfg.end_lo, st0.ram_wr,
				       (int)((int64_t)st0.ram_wr - (int64_t)st));
			}
		}
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

	/* How far may we legitimately read this poll?
	 *
	 * ram_adr is wherever the game last pointed its DRAM writer, NOT a per-window
	 * high-water mark, so it can only be trusted as a frontier when it actually
	 * lands INSIDE the window being played. Measured both cases on hardware:
	 * a streamed window has it inside and climbing (attract began at +12288 of a
	 * 1,375,511-byte window), while a preloaded window has it parked somewhere
	 * else entirely -- up to ~15 MB BELOW the window in one run, and ~3-13 MB
	 * above it in another. An unconditional clamp would deadlock every preloaded
	 * song on those readings, so the resident case reads freely, exactly as before.
	 */
	struct status_reply fr;
	ext_status(&fr);
	/* UNLIMITED for the resident case -- not mp3_end, which would clip the
	 * descrambler's held tail byte (T14). */
	uint32_t read_limit = 0xFFFFFFFFu;
	int streaming = (fr.ram_wr >= c->desc.mp3_start && fr.ram_wr <= c->desc.mp3_end);
	if (streaming) read_limit = fr.ram_wr;

	while (s573_core_should_decode(c, BEATS_PER_FRAME))
	{
		mp3dec_frame_info_t info;
		short pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
		int samples;

		s573_core_fill_upto(c, (const uint8_t *)s573.dio, read_limit);

		uint32_t avail = c->in_len - c->in_pos;
		if (!avail) break;                       // window exhausted, or waiting for the writer

		/* Caught up with a live writer: stop for this poll WITHOUT arming the
		 * stall guard. Waiting for bytes that do not exist yet is correct
		 * behaviour, not a fault, and counting it as a stall is what used to
		 * make us step past and read the previous song out of that region. */
		if (streaming && avail < S573_MIN_FRAME_BYTES && c->desc.cur >= read_limit)
		{
			s573.stall_polls = 0;
			break;
		}

		samples = mp3dec_decode_frame(&s573.dec, c->in + c->in_pos, (int)avail,
		                              pcm, &info);
		if (info.frame_bytes <= 0) break;        // need more input

		// ---- WRITER RACE: do NOT consume a frame that decoded to nothing ----
		//
		// DDR does not hand us a fully-populated window. For songs it streams
		// during play it arms the window and enables the stream FIRST, then
		// uploads the data behind us: the k573dio write pointer (ram_adr, 0xb0/
		// 0xb2) climbs from 0 to the window end over ~29 s while playback runs.
		// MAME oracle, ddrs2k, one song (tools/mame_dio_regs.lua):
		//     170.181  ram_adr = 0x00000000        upload begins
		//     170.282  start=0 end=0x0015bdc3, keys set, STREAM_EN=1
		//     199.032  ram_adr = 0x0015b000        1.42 MB, ~29 s later
		//
		// Reading past that pointer returns memory the game has not written.
		// minimp3 reports unwritten/garbage input as frame_bytes>0 with
		// samples==0 -- "no frame here, skip these bytes" -- which is the right
		// answer for a corrupt stream and exactly the wrong one for a stream
		// that has not arrived yet. Consuming on it let a SINGLE 5 ms poll burn
		// the whole 1.42 MB window: 40 real frames (1.045 s of audio) followed
		// by 1.4 MB of nothing, then DRAIN OFF via EXHAUSTION -- so the music cut
		// out after a second and the game, which paces a stage off the MP3 sample
		// counter, ended the stage on the spot.
		//
		// So a zero-sample decode does not advance: we re-read the same position
		// next poll and stay BEHIND the writer instead of racing it. The margin
		// is comfortable -- the game uploads at ~49 KB/s against ~16 KB/s of
		// playback, so it stays ahead once we stop outrunning it.
		//
		// MEASURED 2026-08-03, and it REFUTES the paragraph above as the whole
		// story. With the writer frontier now readable, every stall in a full
		// session had the data ALREADY WRITTEN at the point we choked on:
		//   attract window (start=0, genuinely streamed): stalled at +16228 with
		//     the writer 45,212 bytes AHEAD of us;
		//   stage windows (non-zero start, preloaded): stalled with ram_adr about
		//     15 MB away in another region entirely -- so those windows are fully
		//     resident and STILL hit this path, which this comment said could not
		//     happen.
		// So "the writer never arrived" is not what these stalls are. The offsets
		// also repeat exactly across runs (+265996 three times, +20408/10/12),
		// which is content-dependent, not a race. Suspicion has moved to the
		// descrambled bytes themselves -- hence the dump below.
		if (samples <= 0 && info.frame_bytes > 0)
		{
			if (s573.stall_polls < S573_STALL_LIMIT)
				break;                            // wait for the writer

			// STALL GUARD. Waiting forever would turn a single bad byte into a
			// hung song with nothing reporting an error, which is the failure
			// mode this project refuses to ship. Step past it, loudly.
			if (!s573.warned_stall)
			{
				/* Dump what we choked on. A valid MPEG-1 Layer III frame starts
				 * FF Fx; descrambled-but-wrong data is the hypothesis this
				 * separates, and 32 bytes is enough to see a sync word (or its
				 * absence) without flooding the log. */
				char hex[3 * 32 + 1];
				uint32_t n = c->in_len - c->in_pos;
				if (n > 32) n = 32;
				for (uint32_t k = 0; k < n; k++)
					sprintf(hex + 3 * k, "%02x ", c->in[c->in_pos + k]);
				hex[n ? 3 * n - 1 : 0] = 0;
				printf("s573mp3: STALL GUARD -- no decodable frame at +%u for %u polls; "
				       "stepping past %d bytes; bytes here: %s\n",
				       (unsigned)(c->desc.cur - c->desc.mp3_start),
				       (unsigned)s573.stall_polls, info.frame_bytes, hex);
				s573.warned_stall = 1;
			}
			s573.stall_polls = 0;
		}
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

	// Track how long we have been getting nothing out. This is the patience the
	// WRITER RACE break above spends: while the game's uploader is still catching
	// up we decode nothing and simply wait, and the moment a real frame lands the
	// count resets. Only a stall that outlives S573_STALL_LIMIT polls is treated
	// as bad data rather than as data-not-yet-written.
	if (produced) { s573.stall_polls = 0; s573.warned_stall = 0; }
	else if (s573.stall_polls < 0xffffffffu) s573.stall_polls++;

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
		s573.stall_polls = 0; s573.warned_stall = 0;  // new window: fresh patience budget
		if (s573.hb_en)
			printf("s573mp3: DRAIN OFF via EXHAUSTION -- window done (cur=%08x end=%08x) and ring drained, drain OFF\n",
			       c->desc.cur, c->desc.mp3_end);
		ep_close_now(c, "exhausted");
	}

	// 6. hand the fabric this poll's cumulative event counts + control flags
	ext_ctrl(s573_core_ctrl_events(c), c->ctrl_flags, NULL);

	s573mp3_heartbeat(c, &r);
}
