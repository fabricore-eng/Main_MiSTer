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
    uint16_t pcm_wr;           /* our write cursor, beats, mod S573_PCM_BEATS */
    uint16_t pcm_rd;           /* the fabric's read cursor, last seen */

    /* --- what we report back --- */
    uint32_t cons_bytes;       /* CUMULATIVE bytes consumed since the last reload */
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
 * differently than we are and the audio will be noise. */
void s573_core_apply_cfg(s573_core_t *c, const s573_cfg_t *cfg);

/* Free space in the PCM ring, in beats, leaving one beat so full != empty. */
uint16_t s573_core_pcm_free(const s573_core_t *c);

/* Should we pull+decode more right now? Gated STRICTLY on ring space -- never on
 * elapsed time. A timer here is the "invent a clock" bug the pacing model
 * forbids; it would run our consumption (and so the fabric's cur) ahead of real
 * playback. `need_beats` is the worst-case beats one decoded frame produces. */
int s573_core_should_decode(const s573_core_t *c, uint16_t need_beats);

/* Top up `in` from the DRAM window. Returns bytes added. */
uint32_t s573_core_fill(s573_core_t *c, const uint8_t *dram);

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
uint16_t s573_core_credit_word(const s573_core_t *c);

#ifdef __cplusplus
}
#endif
#endif /* S573MP3_CORE_H */
