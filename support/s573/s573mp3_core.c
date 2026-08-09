/* -----------------------------------------------------------------------------
 * s573mp3_core.c - System 573 MP3 service, pure logic half. See the header.
 * Released under the GNU GPL v2.
 * ---------------------------------------------------------------------------- */
#include <math.h>
#include "s573mp3_core.h"

static void zero(void *p, size_t n)
{
    uint8_t *b = (uint8_t *)p;
    while (n--) *b++ = 0;
}

void s573_core_init(s573_core_t *c)
{
    zero(c, sizeof *c);
    /* rst_acked stays 0: the core-LOAD reset itself bumps rst_epoch, so our very
     * first poll is guaranteed to see a change and go through the ack path. That
     * costs one poll (~5 ms) at startup and gives every reset ONE code path. */
}

void s573_core_on_reset(s573_core_t *c, uint16_t new_rst_epoch)
{
    uint32_t resets = c->resets;
    zero(c, sizeof *c);
    c->rst_epoch = new_rst_epoch;
    c->resets    = resets + 1;
    /* rst_acked / have_cfg deliberately left 0: we ack THIS poll, and the config
     * must be re-read before we may descramble a single byte. Reading the window
     * with a stale key schedule is silent noise, not an error. */
}

void s573_core_set_ctrl(s573_core_t *c, uint16_t flags)
{
    /* bit0 is RESERVED: the descramble scheme is owned by the fabric's OSD bit
     * (emu.sv O[101]) and arrives in the MP3CFG word, not from us. */
    c->ctrl_flags = (uint16_t)(flags & (uint16_t)~S573_CTRL_DDRSBM);
}

int s573_core_apply_cfg(s573_core_t *c, const s573_cfg_t *cfg)
{
    int setup_changed;
    uint32_t start = ((uint32_t)cfg->start_hi << 16) | cfg->start_lo;
    uint32_t end   = ((uint32_t)cfg->end_hi   << 16) | cfg->end_lo;
    /* The scheme is the FABRIC's to state, not ours to guess: it comes from the
     * OSD bit O[101], which k573dio also folds into cfg_epoch -- so a mid-game
     * toggle re-triggers this adoption instead of silently leaving us on the old
     * key schedule. (Earlier this read back a bit WE had set, which made the
     * config word an echo of ourselves and defaulted ddrsbm titles to the wrong
     * scheme; the OSD bit removed that circularity.) */
    c->ddrsbm_want     = (cfg->flags & S573_CTRL_DDRSBM) ? 1 : 0;
    c->ddrsbm_echo_bad = 0;

    /* The PCM drain follows the GAME's intent, not ours: fabric stream_en is
     * fpga_ctrl[13] & [14], and those bits are the game pressing play or stop.
     * k573dio ticks cfg_epoch on them precisely so we see this. Running the drain
     * when the game is stopped would tick the sample counter off an empty ring --
     * or rather, underrun it -- and leaving it off when the game is playing is
     * silence with nothing reporting an error. */
    if ((cfg->flags & S573_CFG_MP3_ENABLE) && (cfg->flags & S573_CFG_STREAM_ENABLE))
        c->ctrl_flags |= S573_CTRL_DRAIN_EN;
    else
        c->ctrl_flags &= (uint16_t)~S573_CTRL_DRAIN_EN;

    /* ENABLE-ONLY CHANGES MUST NOT REWIND THE STREAM.
     * cfg_epoch moves on fpga_ctrl[14:13] as well as on a setup write, so a bare
     * play/stop toggle lands here too. MAME is explicit that the enable bits only
     * GATE: set_fpga_ctrl calls reset_playback() on the decoder FIFO and never
     * touches mp3_cur_addr or the key schedule, so a pause RESUMES in place rather
     * than restarting the window (573 docs/2026-07-03-p4-mp3-pacing-model.md).
     *
     * Re-initialising unconditionally rewound `cur` to mp3_start on every toggle.
     * The oracle caught the case that exposes it: a stop at t=439.378 and a restart
     * at t=439.748 with NO start/end rewrite in between -- a mid-song pause/resume,
     * which we would have restarted from the top of the window.
     *
     * So only re-init when the SETUP actually changed. */
    setup_changed = (start != c->desc.mp3_start)
                 || (end   != c->desc.mp3_end)
                 || (cfg->key1 != c->desc.key1_seed)
                 || (cfg->key2 != c->desc.key2_seed)
                 || (cfg->key3 != c->desc.key3_seed)
                 || (c->ddrsbm_want != c->desc.ddrsbm)
                 || !c->have_cfg;
    if (setup_changed)
        s573_desc_init(&c->desc, start, end,
                       cfg->key1, cfg->key2, cfg->key3,
                       c->ddrsbm_want);

    c->in_len   = 0;
    c->in_pos   = 0;
    /* The fabric re-armed on the same pulse that moved cfg_epoch, and its credit
     * baseline went with it -- so ours must too, or our first post-song report
     * would look like a huge delta and dump credit into the new window. */
    c->cons_bytes  = 0;
    /* The credit queue describes the PREVIOUS song's PCM. Carrying it over would
     * pay this window's position with the last one's bytes. cq_last_rd is re-based
     * on the LIVE pcm_rd rather than zeroed: the fabric's read cursor does not
     * restart at 0, so a zero here would make the next credit_drained() see an
     * enormous phantom delta and jump the position straight to the song's end. */
    c->cq_head    = 0;
    c->cq_tail    = 0;
    c->cq_pending = 0;
    c->cq_drained = 0;
    c->cq_last_rd = c->pcm_rd;
    c->cfg_epoch   = cfg->epoch;
    c->have_cfg    = 1;
    c->cfg_reloads++;
    /* Tell the caller whether this was a NEW WINDOW (MAME's
     * update_mp3_decode_state event) or a bare enable toggle. Only the former
     * makes the PCM still in the ring stale -- see the flush in s573mp3.cpp. */
    return setup_changed;
}

uint16_t s573_core_pcm_free(const s573_core_t *c)
{
    /* BOTH cursors are FULL 16-BIT LAPPING pointers: a 15-bit beat index plus a
     * wrap MSB. The MSB is not decoration. The fabric's emptiness test is a
     * full-width equality -- `have_data = (hps_wr_ptr != fab_rd_ptr)`
     * (rtl/s573_pcm_ring.v:83, both ports [BEATS_LOG2:0] at :54-55) -- and only
     * the wrap bit lets it tell "empty" from "one whole lap behind".
     *
     * We used to mask our side to 15 bits here and in s573_core_wrote_pcm, so
     * hps_wr_ptr[15] was ALWAYS 0 while fab_rd_ptr[15] toggles every 32768
     * beats. For half of every lap the two could therefore never compare equal:
     * the reader saw data in a physically empty ring, lapped it, and replayed up
     * to 1.486 s of already-drained PCM -- audible as the previous song bleeding
     * into the next one, with underrun_cnt reading a clean 0 throughout. It also
     * made the exhaustion backstop's `pcm_wr == pcm_rd` test (s573mp3.cpp)
     * unsatisfiable half the time.
     *
     * The fabric has always specified 16 bits (rtl/s573_hps_ext.v:46-49 and
     * rtl/s573_pcm_ring.v:43-47, which even guards elaboration against silently
     * truncating the wrap bit) and the ring TB drives it that way
     * (sim/tb_s573_pcm_ring.v:33,:103). We were the only side masking. */
    uint16_t used = (uint16_t)(c->pcm_wr - c->pcm_rd);
    /* used >= BEATS means the fabric read PAST us -- impossible unless a pointer
     * moved backwards. Report no room rather than overwrite undrained PCM. */
    if (used >= S573_PCM_BEATS) return 0;
    /* one beat held back so full and empty are distinguishable, matching the
     * fabric ring's own convention */
    return (uint16_t)(S573_PCM_BEATS - 1 - used);
}

int s573_core_should_decode(const s573_core_t *c, uint16_t need_beats)
{
    if (!c->rst_acked || !c->have_cfg) return 0;
    if (!(c->ctrl_flags & S573_CTRL_DRAIN_EN)) return 0;
    return s573_core_pcm_free(c) >= need_beats;
}

uint32_t s573_core_fill(s573_core_t *c, const uint8_t *dram)
{
    /* No frontier known -> UNLIMITED, and deliberately not mp3_end: the
     * descrambler holds a tail byte that it releases after cur has reached the
     * window end, so clamping at mp3_end silently drops the window's last byte.
     * Caught by T14; the window's own bound lives in s573_desc_pull. */
    return s573_core_fill_upto(c, dram, 0xFFFFFFFFu);
}

/* Same, but never reads past `limit` (an ABSOLUTE window address).
 *
 * WHY THIS EXISTS -- measured on hardware 2026-08-03. We do not decode at real
 * time: the loop runs while the PCM ring has space, and the ring is 32768 beats
 * = 1.486 s, which at 128 kbps is ~23,776 bytes of MP3 demanded AT ONCE when a
 * song starts. A song the game has preloaded does not care. A song the game is
 * still streaming does: ddrs2k's attract track began with the writer only 12,288
 * bytes into a 1,375,511-byte window, so the burst fill outran it inside the
 * first ~20 KB, and we read the PREVIOUS song still resident in that region.
 * Those stale bytes are valid MPEG (the dump showed ff fb 92 0c), so they decode
 * and PLAY -- the audible fragment of the wrong song -- until the stale data
 * stops lining up, at which point minimp3 refuses, the stall guard waits ~0.9 s
 * and steps past, and the song resumes permanently offset.
 *
 * Clamping the READ is the whole fix: bytes the writer has not produced are not
 * ours to interpret. */
uint32_t s573_core_fill_upto(s573_core_t *c, const uint8_t *dram, uint32_t limit)
{
    uint32_t room, got, ahead;

    /* compact first so a partial frame never pins the buffer */
    if (c->in_pos) {
        uint32_t left = c->in_len - c->in_pos;
        uint32_t i;
        for (i = 0; i < left; i++) c->in[i] = c->in[c->in_pos + i];
        c->in_len = left;
        c->in_pos = 0;
    }

    room = S573_IN_WINDOW - c->in_len;
    if (room > S573_IN_CHUNK) room = S573_IN_CHUNK;

    /* clamp to the frontier. cur is where the next pull starts, so a cur at or
     * past the limit means there is nothing legitimate to read yet -- return 0
     * and let the caller wait, which is NOT a stall. */
    ahead = (limit > c->desc.cur) ? (limit - c->desc.cur) : 0;
    if (room > ahead) room = ahead;
    if (!room) return 0;

    got = (uint32_t)s573_desc_pull(&c->desc, dram, c->in + c->in_len, room);
    c->in_len += got;
    return got;
}

void s573_core_consume(s573_core_t *c, uint32_t n)
{
    uint32_t avail = c->in_len - c->in_pos;
    if (n > avail) n = avail;
    c->in_pos    += n;
    /* NOT cons_bytes. These bytes are not "played" until the PCM they decode to
     * has actually left the ring -- see the credit-pacing note in the header.
     * They wait here and are attached to the next frame that produces PCM, so
     * bytes that yielded nothing (a stall-guard skip) still get credited. */
    c->cq_pending += n;
}

void s573_core_wrote_pcm(s573_core_t *c, uint16_t beats)
{
    /* plain 16-bit wrap -- NOT & (S573_PCM_BEATS - 1). The wrap MSB is part of
     * the pointer the fabric compares; see s573_core_pcm_free. Every consumer
     * that turns this into a byte offset must mask it there instead. */
    c->pcm_wr = (uint16_t)(c->pcm_wr + beats);

    /* Attach everything consumed since the last frame to THIS frame's beats, so
     * the credit for those bytes is paid when this audio is heard. Overflow
     * MERGES into the newest entry instead of dropping one: an entry lost here
     * would permanently under-credit the position. */
    {
        uint8_t next = (uint8_t)((c->cq_head + 1u) % S573_CREDIT_Q);
        if (next == c->cq_tail) {
            uint8_t last = (uint8_t)((c->cq_head + S573_CREDIT_Q - 1u) % S573_CREDIT_Q);
            c->credit_q[last].bytes += c->cq_pending;
            c->credit_q[last].beats  = (uint16_t)(c->credit_q[last].beats + beats);
        } else {
            c->credit_q[c->cq_head].bytes = c->cq_pending;
            c->credit_q[c->cq_head].beats = beats;
            c->cq_head = next;
        }
        c->cq_pending = 0;
    }

    c->sync_cnt++;           /* cumulative 8-bit; the fabric diffs mod 256 */
    c->frames++;
}

void s573_core_note_idle(s573_core_t *c)
{
    c->idle_cnt++;
}

uint16_t s573_core_ctrl_events(const s573_core_t *c)
{
    return (uint16_t)(((uint16_t)c->sync_cnt << 8) | c->idle_cnt);
}

void s573_core_credit_drained(s573_core_t *c)
{
    /* 16-bit LAPPING subtraction, same as pcm_free: pcm_rd carries a wrap MSB
     * and a plain compare would go backwards once per lap. */
    uint16_t delta = (uint16_t)(c->pcm_rd - c->cq_last_rd);
    c->cq_last_rd = c->pcm_rd;
    c->cq_drained += delta;

    /* Pay out every frame whose PCM has fully left the ring. Partial frames wait:
     * one frame of granularity is ~26 ms against the 1.473 s of decode-ahead this
     * removes, and holding to whole frames keeps cons_bytes exactly equal to the
     * bytes those frames consumed. */
    while (c->cq_head != c->cq_tail && c->cq_drained >= c->credit_q[c->cq_tail].beats)
    {
        c->cq_drained  -= c->credit_q[c->cq_tail].beats;
        c->cons_bytes  += c->credit_q[c->cq_tail].bytes;
        c->cq_tail      = (uint8_t)((c->cq_tail + 1u) % S573_CREDIT_Q);
    }

    /* Queue empty means the ring has been fully consumed, so nothing is left to
     * wait for -- release any trailing bytes that never produced PCM (a window
     * that ended mid-skip). Without this the last few bytes of a song would
     * never be credited and the fabric would sit just short of the end. */
    if (c->cq_head == c->cq_tail && c->cq_pending) {
        c->cons_bytes += c->cq_pending;
        c->cq_pending  = 0;
        c->cq_drained  = 0;
    }
}

uint16_t s573_core_credit_word(const s573_core_t *c)
{
    return (uint16_t)(c->cons_bytes & 0xFFFFu);
}

float s573_core_gain_mult(uint32_t v)
{
    double db;
    if (v == 0) return 0.0f;                 /* MAME's explicit mute case */
    db = round(20.0 * log10(((double)0x100000 - (double)v) / (double)0x80000));
    return (float)pow(10.0, (db + 6.0) / 20.0);
}

int s573_play_range(s573_play_latch_t *l, const uint8_t cdb[2][12],
                    int play_seen, uint16_t lo_lba, uint16_t lo_end,
                    uint32_t *out_lba, uint32_t *out_end)
{
    int i;
    for (i = 0; i < 2; i++) {                 /* slot 0 is the newer CDB */
        uint32_t lba, len;
        if (cdb[i][0] != 0x45) continue;      /* PLAY AUDIO (10) only */
        lba = ((uint32_t)cdb[i][2] << 24) | ((uint32_t)cdb[i][3] << 16) |
              ((uint32_t)cdb[i][4] <<  8) |  (uint32_t)cdb[i][5];
        len = ((uint32_t)cdb[i][7] <<  8) |  (uint32_t)cdb[i][8];
        if (!len) continue;                   /* a zero-length play tells us nothing */
        l->lba = lba; l->end = lba + len - 1u; l->have = 1;
        break;
    }
    *out_lba = l->lba; *out_end = l->end;
    /* Agree with the fabric's low-16 view, or admit we do not know. */
    return l->have && play_seen &&
           (uint16_t)l->lba == lo_lba && (uint16_t)l->end == lo_end;
}

/* ---- bounded sequence-log sink (see header for why) ---------------------- */
#include <stdarg.h>
#include <stdio.h>

#define S573_SEQ_CAP_DEFAULT (48ull * 1024ull * 1024ull)

const char *s573_seq_sink_default_path(const char *env_file)
{
    if (!env_file || !*env_file) return S573_SEQ_PATH_DEFAULT;
    /* "-" is the conventional spelling for "the standard stream", and unlike ""
     * or "stdout" it cannot be mistaken for a path someone meant to write to. */
    if (env_file[0] == '-' && env_file[1] == 0) return NULL;
    return env_file;
}

int s573_seq_sink_open(s573_seq_sink_t *s, const char *path, uint64_t cap_bytes)
{
    zero(s, sizeof *s);
    s->cap = cap_bytes ? cap_bytes : S573_SEQ_CAP_DEFAULT;
    if (!path || !*path) return 0;                 /* stdout mode */
    snprintf(s->path, sizeof s->path, "%s", path);
    s->f = (void *)fopen(s->path, "w");
    if (!s->f) { s->path[0] = 0; return -1; }      /* fall back to stdout */
    setvbuf((FILE *)s->f, NULL, _IOLBF, 0);
    return 0;
}

void s573_seq_sink_printf(s573_seq_sink_t *s, const char *fmt, ...)
{
    va_list ap;
    char line[512];
    int n;

    va_start(ap, fmt);
    n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if ((size_t)n >= sizeof line) n = (int)sizeof line - 1;

    if (!s->f) { fwrite(line, 1, (size_t)n, stdout); return; }

    /* Rotate BEFORE the write that would breach the cap, so the live file never
     * exceeds it -- checking afterwards would let one line straddle the limit. */
    if (s->written + (uint64_t)n > s->cap) {
        char older[300];
        fclose((FILE *)s->f);
        snprintf(older, sizeof older, "%s.1", s->path);
        remove(older);
        rename(s->path, older);
        s->f = (void *)fopen(s->path, "w");
        if (!s->f) { s->path[0] = 0; fwrite(line, 1, (size_t)n, stdout); return; }
        setvbuf((FILE *)s->f, NULL, _IOLBF, 0);
        s->written = 0;
        s->rotations++;
    }
    fwrite(line, 1, (size_t)n, (FILE *)s->f);
    s->written += (uint64_t)n;
}

void s573_seq_sink_close(s573_seq_sink_t *s)
{
    if (s->f) fclose((FILE *)s->f);
    s->f = NULL;
}
