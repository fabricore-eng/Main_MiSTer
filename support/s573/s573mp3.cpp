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
	uint32_t         last_poll;
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
	DisableIO();
}

// ---- PCM ring ----------------------------------------------------------------
// Writes `frames` stereo int16 pairs at the current beat cursor, wrapping. The
// mapping is uncached (/dev/mem O_SYNC), so this is a straight store loop; a
// memcpy into it is fine but must not be re-ordered past the SPI exchange that
// advertises the new write pointer -- hence the barrier in the caller.
static void pcm_write(const int16_t *pcm, uint32_t frames)
{
	uint32_t byte_off = (uint32_t)s573.core.pcm_wr * 8u;
	uint32_t bytes    = frames * 4u;            // 2ch * int16
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
	// Loud on purpose: design risk #6 is a stale forked binary silently
	// disabling MP3. If this line is missing from the log, the service is not
	// running, whatever the core is doing.
	printf("s573mp3: service up (DIO %08x, PCM ring %08x, %u beats)\n",
	       S573_DIO_PHYS, S573_PCM_PHYS, S573_PCM_BEATS);
	return 1;
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
		// TODO(decision C): the scheme is OURS to choose and must be set BEFORE
		// the first config adoption, or we descramble ddrsbm with the default
		// schedule and produce noise. Nothing determines the mounted game yet,
		// so this is hardcoded off and ddrsbm WILL NOT PLAY CORRECTLY until a
		// per-game source exists. Loud comment rather than a silent default.
		s573_core_set_ctrl(c, S573_CTRL_DRAIN_EN /* | S573_CTRL_DDRSBM */);
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
		s573_core_apply_cfg(c, &cfg);
	}

	// 5. decode while there is ring space. STRICTLY space-gated -- never a
	//    byte/time budget. See s573mp3_core.h.
	int produced = 0;
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
			pcm_write(pcm, (uint32_t)samples);
			s573_core_wrote_pcm(c, (uint16_t)((uint32_t)samples * 4u / 8u));
			produced = 1;
		}
	}
	if (!produced) s573_core_note_idle(c);

	// 6. hand the fabric this poll's cumulative event counts + control flags
	ext_ctrl(s573_core_ctrl_events(c), c->ctrl_flags, NULL);
}
