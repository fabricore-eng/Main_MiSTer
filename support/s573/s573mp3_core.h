/* -----------------------------------------------------------------------------
 * s573mp3_core.h - System 573 Digital I/O MP3 service: the PURE logic half
 *
 * Everything the s573 MP3 service does that is NOT I/O: adopting the fabric's
 * descramble config, tracking the config and reset epochs, deciding how many
 * bytes to pull, accounting the PCM ring, and computing the cumulative
 * consumption credit reported back to the fabric.
 *
 * Deliberately free of MiSTer headers, minimp3 and libc I/O so it can be
 * host-compiled and unit-tested (support/s573/test/). s573mp3.cpp is the thin
 * glue that supplies real SPI words, a real mmap and a real decoder.
 *
 * THE PROTOCOL (fabric side: System573_MiSTer rtl/s573_hps_ext.v, whose header
 * is the normative spec -- read it before changing anything here):
 *
 *   CMD_573_PTRS   0x68  w0^ fab_pcm_rd  | w1v hps_pcm_wr     ^fab_pos_lo
 *                        w2v hps_cons_bytes ^cfg_epoch | w3v hps_rst_ack ^rst_epoch
 *   CMD_573_STATUS 0x69  w0^ status_flags | w1^ underrun | w2^ buf_level
 *   CMD_573_CTRL   0x6A  w0^ baselines | w1v {sync,idle} cumulative | w2v ctrl_flags
 *   CMD_573_MP3CFG 0x6B  w0^ cfg_epoch | w1..w8^ start/end/keys/{ctrl,ddrsbm}
 *
 * TWO RULES THAT ARE EASY TO GET WRONG, both load-bearing:
 *
 *  1. The pointer words of CMD_573_PTRS are IGNORED by the fabric until we have
 *     acked the current rst_epoch. A core reset bumps it, and the core-LOAD
 *     reset does too -- so our very first poll always sees a new epoch and must
 *     ack before anything else works. On any epoch change: drop everything,
 *     re-init the decoder, re-read the config, restart our counters.
 *
 *  2. hps_cons_bytes is CUMULATIVE, not a delta. Never send a per-poll count.
 *     The fabric diffs it against its own baseline, so a retried or duplicated
 *     exchange is harmless -- but sending a delta would make every poll credit
 *     the fabric again and run its `cur` (and the game-visible 0xae bit12) far
 *     past real playback.
 *
 * WHAT WE MUST NOT DO: synthesize the game-visible "streaming" bit. The fabric
 * owns it, derived from a position we pace. See s573_descramble.h.
 *
 * Released under the GNU GPL v2.
 * ---------------------------------------------------------------------------- */
#ifndef S573MP3_CORE_H
#define S573MP3_CORE_H

#include <stdint.h>
#include <stddef.h>
#include "s573_descramble.h"

#ifdef __cplusplus
extern "C" {
#endif

/* EXT_BUS command IDs -- must match rtl/s573_hps_ext.v */
#define CMD_573_PTRS    0x68
#define CMD_573_STATUS  0x69
#define CMD_573_CTRL    0x6A
#define CMD_573_MP3CFG  0x6B
/* The ATAPI CDB bytes on their OWN short exchange. They are ALSO in the STATUS
 * snapshot at words 12..23, but reading that far has never worked on hardware: a
 * 24-word STATUS transaction comes back with words 10..23 zeroed AND it
 * retroactively broke play_lba/play_endlba, which worked before the long read
 * existed. The fabric serves all 23 words correctly (tb_s573_hps_ext walks every
 * one); the framework drops io_enable part-way through a long transaction, so
 * byte_cnt resets and the rest reads out shifted or zero. This therefore reads
 * SHORTER rather than further -- 12 words at low indices, 13 exchanges against
 * the 12 that already work. */
#define CMD_573_CDB     0x6C

/* DDR3 geometry -- must match the fabric. The DIO sample-RAM aperture is where
 * the game stages the SCRAMBLED stream; we only ever READ it. */
#define S573_DIO_PHYS        0x32000000u
#define S573_DIO_SIZE        0x02000000u   /* 32 MiB */
/* PCM ring (HPS -> fabric), carved from the TOP of the DIO window. Must match
 * the fabric's RING_OFF_BEAT. NOTE transport-design must-fix #2: confirm this is
 * clear of ddrsbm's actual top-of-window use on HW before trusting it. */
#define S573_PCM_PHYS        0x33F10000u
#define S573_PCM_BEATS_LOG2  15            /* beats; 1 beat = 2 stereo frames */
#define S573_PCM_BEATS       (1u << S573_PCM_BEATS_LOG2)
#define S573_PCM_BYTES       (S573_PCM_BEATS * 8u)

/* Input window we keep descrambled but not yet decoded. Deliberately SMALL:
 * this is read-ahead into memory the game owns, and MD+'s 8192 would cost
 * ~1.3 ms in a single poll. ~1 KB is ~25-50 ms of audio. */
#define S573_IN_CHUNK        1024
#define S573_IN_WINDOW       4096

/* Credit queue depth: one entry per decoded frame still resident in the PCM
 * ring. The ring holds S573_PCM_BEATS/576 = ~57 MPEG-1 Layer III frames, so
 * 128 is comfortable headroom. On overflow we MERGE into the newest entry
 * rather than dropping -- losing an entry would silently under-credit the
 * position and desync the chart, which is the exact bug this queue exists to
 * fix. */
#define S573_CREDIT_Q        128

/* ctrl_flags bits (CMD_573_CTRL w2, fabric-bound) */
#define S573_CTRL_DDRSBM     0x0001   /* RESERVED: the fabric's OSD bit owns this */
#define S573_CTRL_DRAIN_EN   0x0002

/* MP3CFG word8 layout: {12'b0, fpga_ctrl[15:13], ddrsbm} */
#define S573_CFG_FPGA_EN_SHIFT 1
#define S573_CFG_MP3_ENABLE    (1u << (S573_CFG_FPGA_EN_SHIFT + 0))  /* fpga_ctrl[13] */
#define S573_CFG_STREAM_ENABLE (1u << (S573_CFG_FPGA_EN_SHIFT + 1))  /* fpga_ctrl[14] */

/* MAS3507D output-gain curve, MAME mas3507d verbatim:
 *   gain_to_db(v) = round(20*log10((0x100000 - v)/0x80000))
 *   mult(v)       = v == 0 ? 0 : 10^((db + 6)/20)
 * Lives here (not in s573mp3.cpp) so it is host-testable: the .cpp pulls in
 * MiSTer/Linux headers that will not compile on a dev Mac.
 * NOTE: ddrsbm asks for 0xAF3CD = -4 dB = x1.2589, a BOOST. Unity is near
 * 0xBFBDE. This is NOT a fix for the clipping -- see s573mp3.cpp. */
float s573_core_gain_mult(uint32_t v);

typedef struct {
    uint16_t start_lo, start_hi, end_lo, end_hi;
    uint16_t key1, key2, key3;
    uint16_t flags;            /* bit0 ddrsbm, bits3:1 fpga_ctrl[15:13] */
    uint16_t epoch;
    /* MAS3507D output gain matrix, decoded off the I2C bus by the fabric.
     * The GAME's own output level; 0 means mute. We used to drop these
     * entirely and play at unity, which is why silicon measured peak
     * 0.000265 dBFS -- pinned to digital full scale.
     * gain_seen is sticky: 0 = the game has never set a level, so apply
     * unity rather than booting muted. */
    uint32_t gain_ll;          /* 20-bit */
    uint32_t gain_rr;          /* 20-bit */
    int      gain_seen;
} s573_cfg_t;

typedef struct {
    /* --- epochs --- */
    uint16_t rst_epoch;        /* last value seen from the fabric */
    uint16_t cfg_epoch;        /* ditto */
    int      rst_acked;        /* have we acked rst_epoch yet? */
    int      have_cfg;         /* config adopted at least once this epoch */

    /* --- descramble --- */
    s573_desc_t desc;

    /* --- input staging (descrambled bytes awaiting decode) --- */
    uint8_t  in[S573_IN_WINDOW];
    uint32_t in_len;           /* valid bytes in `in` */
    uint32_t in_pos;           /* consumed prefix */

    /* --- PCM ring accounting, in BEATS (1 beat = 2 stereo frames = 8 bytes) --- */
    /* BOTH are FULL 16-BIT LAPPING cursors: a 15-bit beat index plus a wrap MSB.
     * The fabric's empty/full compare is a full-width equality on exactly these
     * bits (rtl/s573_pcm_ring.v:83), so the MSB must survive here -- mask to a
     * byte offset at the point of use. See s573_core_pcm_free. */
    uint16_t pcm_wr;           /* our write cursor, beats, 16-bit lapping */
    uint16_t pcm_rd;           /* the fabric's read cursor, last seen, 16-bit */

    /* --- credit pacing: PLAYBACK, not decode -------------------------------
     * cons_bytes is what the fabric turns into a stream position, and the GAME
     * reads that position to place every arrow in the chart. It must therefore
     * track what the player HEARS, not what we have decoded.
     *
     * It used to be bumped inside s573_core_consume(), i.e. the instant bytes
     * were handed to the decoder. We decode as fast as ring space allows, so
     * the credit ran a full ring ahead of the speaker. Measured on de10
     * 2026-08-03 across 344 draining polls: ring occupancy median 32,484 of
     * 32,768 beats = 1.473 s. The chart was paced ~1.5 s ahead of the music --
     * unplayable for a rhythm game, and invisible to every counter we had,
     * because nothing was wrong with the decode itself.
     *
     * So bytes now wait in a small queue and are credited only as the fabric
     * DRAINS the PCM they produced. Bytes that yielded no PCM (a stall-guard
     * skip) ride along with the next frame that did, so the running total stays
     * exact. Granularity is one frame -- credit lands when a frame's last beat
     * leaves the ring, ~26 ms -- which is three orders below the 1.473 s of
     * error it removes.
     *
     * This does NOT touch song-end detection: exhaustion is tested against
     * desc.cur (the DRAM pull position), never against cons_bytes. */
    struct { uint32_t bytes; uint16_t beats; } credit_q[S573_CREDIT_Q];
    uint8_t  cq_head, cq_tail;  /* ring indices; head == tail means empty */
    uint32_t cq_pending;        /* consumed, not yet attached to a decoded frame */
    uint32_t cq_drained;        /* beats drained, not yet matched to an entry */
    uint16_t cq_last_rd;        /* pcm_rd as of the last credit_drained() call */

    /* --- what we report back --- */
    uint32_t cons_bytes;       /* CUMULATIVE bytes PLAYED since the last reload */
    uint8_t  sync_cnt;         /* CUMULATIVE decoded-frame count (8-bit, wraps) */
    uint8_t  idle_cnt;         /* CUMULATIVE no-frame-decode count */
    uint16_t ctrl_flags;       /* what we PUSH down (CMD_573_CTRL w2) */
    uint8_t  ddrsbm_want;      /* OUR intent for the scheme -- see set_ctrl */
    uint8_t  ddrsbm_echo_bad;  /* the fabric disagreed with us (should be 0) */

    /* --- observability --- */
    uint32_t frames;
    uint32_t resets;
    uint32_t cfg_reloads;
} s573_core_t;

/* Zero the service. Does NOT touch hardware. */
void s573_core_init(s573_core_t *c);

/* A fabric reset epoch changed: drop all decode state and re-arm. Callers must
 * also re-init their MP3 decoder. */
void s573_core_on_reset(s573_core_t *c, uint16_t new_rst_epoch);

/* Set what we push down in CMD_573_CTRL w2.
 *
 * THE DESCRAMBLE SCHEME IS OURS TO CHOOSE, NOT THE FABRIC'S TO TELL US (open
 * decision C: the HPS knows which game is mounted). emu.sv feeds our ctrl_flags
 * bit0 into k573dio's cfg_ddrsbm, and CMD_573_MP3CFG word8 reports that same bit
 * back up -- so the config word is an ECHO of our own setting, not a source. A
 * caller that took the scheme from the echo would read 0 before we had set
 * anything, silently descramble ddrsbm with the DEFAULT scheme, and never
 * re-adopt. So: set it here, and changing it forces the config to be re-read. */
void s573_core_set_ctrl(s573_core_t *c, uint16_t flags);

/* Adopt a freshly-read CMD_573_MP3CFG tuple. Re-arms the descrambler and zeroes
 * the consumption counter, because the fabric zeroes its own baseline on the
 * same re-arm. Uses OUR ddrsbm intent for the scheme and only CHECKS the echo --
 * a mismatch sets ddrsbm_echo_bad, which means the fabric is descrambling
 * differently than we are and the audio will be noise.
 *
 * RETURNS 1 if the descrambler was re-armed to a NEW WINDOW (start/end/keys or
 * the scheme changed) -- i.e. MAME's update_mp3_decode_state() event -- and 0
 * for a bare enable toggle, which only GATES. Callers use this to decide whether
 * PCM already in the ring belongs to a song that will never be played again. */
int s573_core_apply_cfg(s573_core_t *c, const s573_cfg_t *cfg);

/* Free space in the PCM ring, in beats, leaving one beat so full != empty. */
uint16_t s573_core_pcm_free(const s573_core_t *c);

/* Should we pull+decode more right now? Gated STRICTLY on ring space -- never on
 * elapsed time. A timer here is the "invent a clock" bug the pacing model
 * forbids; it would run our consumption (and so the fabric's cur) ahead of real
 * playback. `need_beats` is the worst-case beats one decoded frame produces. */
int s573_core_should_decode(const s573_core_t *c, uint16_t need_beats);

/* Top up `in` from the DRAM window. Returns bytes added. */
uint32_t s573_core_fill(s573_core_t *c, const uint8_t *dram);
/* Fill, but never read past `limit` (absolute window address). Used to keep the
 * decoder behind a writer that is still streaming into the current window; see
 * the note on the definition. */
uint32_t s573_core_fill_upto(s573_core_t *c, const uint8_t *dram, uint32_t limit);

/* Mark `n` staged bytes as consumed by the decoder and add them to the
 * cumulative credit. Compacts the staging buffer. */
void s573_core_consume(s573_core_t *c, uint32_t n);

/* Advance the PCM write cursor by `beats` and count a decoded frame. */
void s573_core_wrote_pcm(s573_core_t *c, uint16_t beats);

/* Count a poll that produced no frame (MAME mpeg_frame_sync(0)); the fabric
 * derives mpeg IDLE from the cadence of these, so it is RECURRING, not
 * one-shot. */
void s573_core_note_idle(s573_core_t *c);

/* The CMD_573_CTRL w1 payload: {sync_cnt, idle_cnt}, cumulative 8-bit each. */
uint16_t s573_core_ctrl_events(const s573_core_t *c);

/* The CMD_573_PTRS w2 payload: cumulative consumed bytes, truncated to 16 bits.
 * Truncation is CORRECT -- the fabric diffs mod 2^16 and 65536 bytes is 1.6 s at
 * 320 kbps against a ~5 ms poll. */
/* Advance the PLAYBACK credit to match the fabric's read cursor. Call once per
 * poll AFTER c->pcm_rd has been refreshed from CMD_573_PTRS. Pops every queue
 * entry whose PCM has fully left the ring and adds its bytes to cons_bytes. */
void s573_core_credit_drained(s573_core_t *c);

uint16_t s573_core_credit_word(const s573_core_t *c);


/* ---- TRUE PLAY-AUDIO range, recovered from the CDB bytes -------------------
 *
 * The fabric's play_lba/play_endlba are the LOW 16 BITS of a 32-bit LBA and
 * nothing more (s573_hps_ext.v:109 says so in words; atapi.v:712 does
 * `play_lba <= cdda_start[15:0]`). Printed raw they are actively misleading: a
 * play straddling a multiple of 65536 comes out with its END BELOW ITS START,
 * which on 2026-08-08 was read as "this play crosses a 64K boundary" and became
 * a standing lead for a bug. It is not an event, it is a wrap -- the observed
 * rate (1 of 12 plays) is exactly the rate chance predicts (0.79 of 12) for the
 * play lengths involved. Worse, a play at a true 294557..297483 prints as a
 * placid-looking 32413..35339, so a wrapped field cannot be spotted by eye.
 *
 * PLAY AUDIO (10) carries the full start LBA in CDB bytes 2..5 and the length
 * in bytes 7..8, and the CDB witness already ships those bytes -- so the true
 * range was recoverable all along.
 *
 * The latch is caller-owned (no hidden statics: the point is that it is
 * testable) and its result is VALIDATED rather than trusted. CDBs really are
 * missed between polls -- the logs carry "seq GAP" lines -- and a stale latch
 * printed against a newer play would be a new way to lie. The fabric's own
 * truncated pair is the check: if the latch disagrees with it in the low 16
 * bits, we report "not known" instead of guessing. */
typedef struct { uint32_t lba, end; int have; } s573_play_latch_t;

/* Update `l` from any PLAY AUDIO in `cdb` (slot 0 is the newer), then check it
 * against the fabric's sticky low-16 pair. Returns 1 and fills out_lba/out_end
 * with the TRUE 32-bit range when it can be proven, 0 when it cannot. */
int s573_play_range(s573_play_latch_t *l, const uint8_t cdb[2][12],
                    int play_seen, uint16_t lo_lba, uint16_t lo_end,
                    uint32_t *out_lba, uint32_t *out_end);


/* ---- bounded sink for the per-CDB sequence log ----------------------------
 *
 * WHY: S573_CDB_SEQ=1 emits one line per PACKET command, and DrumMania polls
 * READ SUBCHANNEL continuously in attract, so the rate does NOT fall off when
 * nothing is happening -- measured at ~17.4 MB/h on 2026-08-08. The board's
 * /tmp is a 247 MB tmpfs on a machine with 492 MB of RAM, so an unattended run
 * fills it in roughly half a day and the instrument deployed to catch the fault
 * becomes the fault. That matters here more than usual: the state being hunted
 * only shows up during long armed runs, so "just do not leave it on" is not an
 * option.
 *
 * Two-file rotation rather than a cap-and-stop: the fault appears LATE, so the
 * tail is the part worth keeping and truncating from the front is the whole
 * point. Total on disk stays under 2*cap.
 *
 * State is caller-owned so the rotation can be driven by the host tests. With
 * path == NULL the sink writes to stdout and never rotates.
 *
 * WHICH MODE AN ARMED RUN GETS is decided by s573_seq_sink_default_path() below,
 * NOT by this call. Until 2026-08-09 an unset S573_CDB_SEQ_FILE meant stdout,
 * so the cap only protected the board when someone remembered to name a file --
 * and on 2026-08-09 a session armed the instrument with S573_CDB_SEQ=1 alone and
 * redirected stdout by hand, which is precisely the unbounded case this exists to
 * prevent. A safety default that has to be opted into is not a safety default. */
typedef struct {
    void     *f;            /* FILE* once open; NULL means "write to stdout" */
    uint64_t  written;      /* bytes in the CURRENT file */
    uint64_t  cap;          /* rotate once a line would push written past this */
    unsigned  rotations;    /* diagnostic: how many times we have wrapped */
    char      path[256];
} s573_seq_sink_t;

/* path == NULL (or "") -> stdout, unbounded, no rotation. cap_bytes == 0 picks
 * the 48 MB default. Returns 0 on success, -1 if the file cannot be opened (in
 * which case the sink falls back to stdout rather than losing the log). */
int  s573_seq_sink_open(s573_seq_sink_t *s, const char *path, uint64_t cap_bytes);

/* Resolve S573_CDB_SEQ_FILE (pass getenv()'s result, NULL included) to the path
 * s573_seq_sink_open() should be given:
 *
 *   NULL or ""  -> S573_SEQ_PATH_DEFAULT, the CAPPED file. Arming the instrument
 *                  is now bounded by default; nothing has to be remembered.
 *   "-"         -> NULL, i.e. unbounded stdout. Still available for a short
 *                  attended run, but it has to be asked for by name.
 *   anything    -> itself, unchanged.
 *
 * Returns a pointer the caller does not own (a literal or the argument itself).
 * Split out of the SPI poll so the policy is testable without a board. */
const char *s573_seq_sink_default_path(const char *env_file);
#define S573_SEQ_PATH_DEFAULT "/tmp/s573_seq.log"
void s573_seq_sink_printf(s573_seq_sink_t *s, const char *fmt, ...);
void s573_seq_sink_close(s573_seq_sink_t *s);

#ifdef __cplusplus
}
#endif
#endif /* S573MP3_CORE_H */
