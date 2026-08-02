/* -----------------------------------------------------------------------------
 * test_core.c - host unit tests for the s573 MP3 service logic.
 *
 *   cc -O2 -Wall -Wextra -std=c99 -I.. -o test_core test_core.c \
 *      ../s573mp3_core.c ../s573_descramble.c && ./test_core
 *
 * Covers the things that are cheap to get wrong and expensive to debug on
 * hardware: cumulative-vs-delta credit accounting, ring free-space wrap, the
 * reset/config gate, and the "never decode on a timer" pacing rule. The
 * descramble itself is already proven byte-exact against the RTL (see
 * System573_MiSTer/sim/spike_b_descramble); here we only check that the service
 * hands the same bytes through without losing or duplicating any.
 * ---------------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include "s573mp3_core.h"

static int fails = 0;
#define CHK(cond, ...) do { if (!(cond)) { \
    printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* a deterministic stand-in for the DIO window */
static uint8_t dram[0x4000];
static void seed_dram(void)
{
    uint32_t s = 0x12345678u, i;
    for (i = 0; i < sizeof dram; i++) {
        s = s * 1103515245u + 12345u;
        dram[i] = (uint8_t)(s >> 16);
    }
}

static s573_cfg_t mkcfg(uint32_t start, uint32_t end, uint16_t epoch, int sbm)
{
    s573_cfg_t c;
    memset(&c, 0, sizeof c);
    c.start_lo = start & 0xFFFF; c.start_hi = start >> 16;
    c.end_lo   = end   & 0xFFFF; c.end_hi   = end   >> 16;
    c.key1 = 0x1357; c.key2 = 0x2468; c.key3 = 0x9BDF;
    c.flags = (uint16_t)((sbm ? S573_CTRL_DDRSBM : 0)
                         | S573_CFG_MP3_ENABLE | S573_CFG_STREAM_ENABLE);
    c.epoch = epoch;
    return c;
}

int main(void)
{
    s573_core_t c;
    s573_cfg_t  cfg;
    uint32_t    n, total;
    uint16_t    f0;

    seed_dram();

    /* ---- T1: nothing happens before the reset is acked and config adopted ---- */
    s573_core_init(&c);
    CHK(!s573_core_should_decode(&c, 1), "T1: decoding before rst ack");
    c.rst_acked = 1;
    CHK(!s573_core_should_decode(&c, 1), "T1: decoding before config");
    /* game NOT playing: config adopted, but the drain must stay off */
    cfg = mkcfg(0x100, 0x900, 1, 0);
    cfg.flags &= (uint16_t)~(S573_CFG_MP3_ENABLE | S573_CFG_STREAM_ENABLE);
    s573_core_apply_cfg(&c, &cfg);
    CHK(c.have_cfg, "T1: config should still be adopted while stopped");
    CHK(!s573_core_should_decode(&c, 1), "T1: decoding while the game is stopped");
    /* game presses play */
    cfg = mkcfg(0x100, 0x900, 2, 0);
    s573_core_apply_cfg(&c, &cfg);
    CHK(s573_core_should_decode(&c, 1), "T1: still refusing with everything armed");

    /* ---- T2: pacing is ring-space only, never a timer ---- */
    c.pcm_wr = 0; c.pcm_rd = 0;
    CHK(s573_core_pcm_free(&c) == S573_PCM_BEATS - 1, "T2: free space when empty");
    c.pcm_wr = (uint16_t)(S573_PCM_BEATS - 1);
    CHK(s573_core_pcm_free(&c) == 0, "T2: free space when full");
    CHK(!s573_core_should_decode(&c, 1), "T2: decoded with a full ring");
    /* wrapped: wr behind rd numerically but the ring is nearly empty */
    c.pcm_wr = 4; c.pcm_rd = 2;
    CHK(s573_core_pcm_free(&c) == S573_PCM_BEATS - 3, "T2: free space, no wrap");
    /* The SAME state, now expressed WITH the wrap bit: one beat past a lap
     * boundary is BEATS+1, not 1. Under the old 15-bit mask those two were
     * indistinguishable -- which is precisely what let the fabric's full-width
     * compare (rtl/s573_pcm_ring.v:83) mistake an empty ring for a whole lap of
     * pending data and replay it. Expected free count is unchanged. */
    c.pcm_wr = (uint16_t)(S573_PCM_BEATS + 1); c.pcm_rd = (uint16_t)(S573_PCM_BEATS - 2);
    CHK(s573_core_pcm_free(&c) == S573_PCM_BEATS - 4, "T2: free space across the wrap");
    /* EXACTLY ONE LAP AHEAD -- the ring is completely FULL. This is the case the
     * old 15-bit mask got catastrophically wrong: (wr - rd) = 0x8000 masked to 15
     * bits reads as 0, i.e. "empty", so we would have overwritten 32767 undrained
     * beats. The wrap bit is the only thing that distinguishes this from empty. */
    c.pcm_wr = (uint16_t)S573_PCM_BEATS; c.pcm_rd = 0;
    CHK(s573_core_pcm_free(&c) == 0, "T2: a full lap ahead reads FULL, not empty");
    CHK(!s573_core_should_decode(&c, 1), "T2: decoded into a fully-lapped ring");
    /* fabric read PAST us -- report no room rather than overwrite */
    c.pcm_wr = 0; c.pcm_rd = 1;
    CHK(s573_core_pcm_free(&c) == 0, "T2: rd ahead of wr reports no room");
    c.pcm_wr = 0; c.pcm_rd = 0;

    /* ---- T3: the stream comes through intact, in arbitrary chunks ---- */
    {
        static uint8_t ref[0x2000], via[0x2000];
        s573_desc_t d;
        size_t rn;
        uint32_t vn = 0;

        s573_desc_init(&d, 0x100, 0x900, 0x1357, 0x2468, 0x9BDF, 0);
        rn = s573_desc_pull(&d, dram, ref, sizeof ref);

        s573_core_init(&c);
        c.rst_acked = 1; s573_core_set_ctrl(&c, S573_CTRL_DRAIN_EN);
        cfg = mkcfg(0x100, 0x900, 1, 0);
        s573_core_apply_cfg(&c, &cfg);

        /* drain it through the staging buffer in odd little bites */
        for (;;) {
            uint32_t take, avail;
            s573_core_fill(&c, dram);
            avail = c.in_len - c.in_pos;
            if (!avail) break;
            take = (vn % 7) + 1;
            if (take > avail) take = avail;
            memcpy(via + vn, c.in + c.in_pos, take);
            vn += take;
            s573_core_consume(&c, take);
        }
        CHK(vn == rn, "T3: %u bytes through the service vs %u direct", vn, (unsigned)rn);
        CHK(memcmp(ref, via, vn) == 0, "T3: byte stream differs from the direct pull");
        CHK(c.cons_bytes == vn, "T3: cons_bytes %u != bytes consumed %u", c.cons_bytes, vn);
    }

    /* ---- T4: credit is CUMULATIVE and monotonic; the word truncates, the
     *          counter does not ---- */
    s573_core_init(&c);
    c.rst_acked = 1; s573_core_set_ctrl(&c, S573_CTRL_DRAIN_EN);
    cfg = mkcfg(0x000, 0x3000, 5, 0);
    s573_core_apply_cfg(&c, &cfg);
    total = 0;
    for (n = 0; n < 200; n++) {
        uint32_t avail;
        s573_core_fill(&c, dram);
        avail = c.in_len - c.in_pos;
        if (avail > 100) avail = 100;
        if (!avail) break;
        s573_core_consume(&c, avail);
        total += avail;
        CHK(c.cons_bytes == total, "T4: credit drifted at step %u", n);
    }
    CHK(total > 4000, "T4: expected a long run, got %u bytes", total);
    c.cons_bytes = 0x1FFFF;
    CHK(s573_core_credit_word(&c) == 0xFFFF,
        "T4: credit word should truncate to 0xFFFF, got %04x", s573_core_credit_word(&c));

    /* ---- T5: a song change re-arms AND zeroes the credit baseline ---- */
    CHK(c.cons_bytes != 0, "T5 pre: expected outstanding credit");
    cfg = mkcfg(0x200, 0x400, 6, 1);
    s573_core_apply_cfg(&c, &cfg);
    CHK(c.cons_bytes == 0, "T5: credit not rebaselined on the new song");
    CHK(c.in_len == 0 && c.in_pos == 0, "T5: stale staged bytes survived the re-arm");
    /* the fabric states scheme 1 for this song, so we adopt it (see T9) */
    CHK(c.desc.ddrsbm == 1, "T5: scheme not adopted from the fabric config");
    CHK(c.cfg_epoch == 6, "T5: cfg epoch not adopted");

    /* ---- T6: a reset wipes everything and re-closes the gate ---- */
    c.pcm_wr = 1234; c.sync_cnt = 77; c.cons_bytes = 999;
    s573_core_on_reset(&c, 42);
    CHK(c.rst_epoch == 42, "T6: reset epoch not recorded");
    CHK(!c.rst_acked,  "T6: reset must re-close the ack gate");
    CHK(!c.have_cfg,   "T6: config must be re-read after a reset");
    CHK(c.pcm_wr == 0 && c.sync_cnt == 0 && c.cons_bytes == 0, "T6: state survived a reset");
    CHK(!s573_core_should_decode(&c, 1), "T6: decoding straight after a reset");
    CHK(c.resets == 1, "T6: reset not counted");

    /* ---- T7: event counters are cumulative 8-bit and wrap cleanly ---- */
    s573_core_init(&c);
    for (n = 0; n < 300; n++) s573_core_wrote_pcm(&c, 1);
    CHK(c.sync_cnt == (uint8_t)300, "T7: sync_cnt should wrap mod 256, got %u", c.sync_cnt);
    CHK(c.frames == 300, "T7: frame count should NOT wrap, got %u", c.frames);
    for (n = 0; n < 5; n++) s573_core_note_idle(&c);
    f0 = s573_core_ctrl_events(&c);
    CHK((f0 >> 8) == c.sync_cnt && (f0 & 0xFF) == c.idle_cnt, "T7: ctrl event packing");

    /* ---- T8: pcm_wr laps at 2^16, CARRYING the wrap bit the fabric compares ----
     * It must NOT fold at the ring size: hps_wr_ptr and fab_rd_ptr are both
     * [15:0] in the fabric and compared for full-width equality
     * (rtl/s573_pcm_ring.v:54-55,:83). Folding here is what made an empty ring
     * read as non-empty for half of every lap. */
    s573_core_init(&c);
    for (n = 0; n < S573_PCM_BEATS + 17; n++) s573_core_wrote_pcm(&c, 1);
    CHK(c.pcm_wr == (uint16_t)(S573_PCM_BEATS + 17),
        "T8: pcm_wr should keep the wrap bit and read %u, got %u",
        (unsigned)(uint16_t)(S573_PCM_BEATS + 17), c.pcm_wr);

    /* ---- T9: the descramble scheme comes FROM the fabric (OSD bit O[101]) ----
     * We must not set it ourselves and must not ignore it. k573dio folds the OSD
     * bit into cfg_epoch, so a mid-game toggle re-triggers adoption here. */
    s573_core_init(&c);
    c.rst_acked = 1;
    s573_core_set_ctrl(&c, S573_CTRL_DRAIN_EN | S573_CTRL_DDRSBM);
    CHK(!(c.ctrl_flags & S573_CTRL_DDRSBM),
        "T9: we must NOT push the scheme down -- bit0 is the fabric's");
    CHK(c.ctrl_flags & S573_CTRL_DRAIN_EN, "T9: other ctrl bits must survive");

    cfg = mkcfg(0x100, 0x400, 9, 1);          /* fabric says: variant scheme */
    s573_core_apply_cfg(&c, &cfg);
    CHK(c.desc.ddrsbm == 1, "T9: must adopt the scheme the fabric states");
    cfg = mkcfg(0x100, 0x400, 10, 0);         /* OSD toggled back mid-game */
    s573_core_apply_cfg(&c, &cfg);
    CHK(c.desc.ddrsbm == 0, "T9: must re-adopt when the fabric changes it");

    /* ---- T10: the drain follows the GAME's play/stop, not our own state ---- */
    s573_core_init(&c);
    c.rst_acked = 1;
    cfg = mkcfg(0x100, 0x400, 20, 0);            /* helper sets both enables */
    s573_core_apply_cfg(&c, &cfg);
    CHK(c.ctrl_flags & S573_CTRL_DRAIN_EN, "T10: drain must follow the game playing");
    CHK(s573_core_should_decode(&c, 1),     "T10: should decode while playing");

    cfg.flags &= (uint16_t)~S573_CFG_STREAM_ENABLE;   /* game pressed stop */
    cfg.epoch = 21;
    CHK(s573_core_apply_cfg(&c, &cfg) == 0,
        "T10: a bare STOP is not a re-arm -- must report 0");
    CHK(!(c.ctrl_flags & S573_CTRL_DRAIN_EN), "T10: drain must clear when the game stops");
    CHK(!s573_core_should_decode(&c, 1),      "T10: must not decode while stopped");

    cfg.flags |= S573_CFG_STREAM_ENABLE;              /* play again */
    cfg.epoch = 22;
    /* THE FLUSH GATE. The service drops the ring's undrained PCM only when this
     * returns 1. A bare resume must report 0: that PCM is exactly the audio the
     * resume continues with, and discarding it would fast-forward the song by up
     * to 1.49 s -- the mirror image of the rewind bug group 12 guards against. */
    CHK(s573_core_apply_cfg(&c, &cfg) == 0,
        "T10: a bare RESUME is not a re-arm -- must report 0");
    CHK(c.ctrl_flags & S573_CTRL_DRAIN_EN, "T10: drain must come back on replay");

    /* A genuinely NEW WINDOW must report 1, or the stale PCM is never dropped and
     * the previous song bleeds ~1.49 s into the next one. */
    cfg = mkcfg(0x8000, 0xC000, 23, 0);
    CHK(s573_core_apply_cfg(&c, &cfg) == 1,
        "T10: a new start/end window IS a re-arm -- must report 1");

    /* ---- group 12: an ENABLE-ONLY cfg change must not rewind the stream ----
     * cfg_epoch moves on fpga_ctrl[14:13] too, so a bare play/stop toggle reaches
     * apply_cfg. MAME's enable bits only GATE -- a pause resumes in place. The
     * oracle caught the exposing case: stop at t=439.378, restart at t=439.748 with
     * no start/end rewrite between. Re-initialising there restarts the song. */
    {
        s573_core_t c2;
        s573_cfg_t  cf;
        uint32_t    mid;
        s573_core_init(&c2);
        memset(&cf, 0, sizeof(cf));
        cf.start_lo = 0x1000; cf.start_hi = 0x0000;
        cf.end_lo   = 0x0000; cf.end_hi   = 0x0010;
        cf.key1 = 0x1111; cf.key2 = 0x2222; cf.key3 = 0x3333;
        cf.flags = 0x000f;                      /* both enables set */
        s573_core_apply_cfg(&c2, &cf);
        c2.have_cfg = 1;

        /* pretend we decoded a way into the song */
        c2.desc.cur = c2.desc.mp3_start + 0x4000;
        mid = c2.desc.cur;

        /* enable-only change: same setup, enables cleared then set again */
        cf.flags = 0x000b;                      /* STREAMING_ENABLE low = stop */
        s573_core_apply_cfg(&c2, &cf);
        cf.flags = 0x000f;                      /* and back = resume */
        s573_core_apply_cfg(&c2, &cf);
        if (c2.desc.cur != mid) {
            printf("FAIL: enable toggle rewound cur %08x -> %08x (should resume in place)\n",
                   mid, c2.desc.cur);
            fails++;
        }

        /* a REAL setup change must still re-init */
        cf.start_lo = 0x8000;
        s573_core_apply_cfg(&c2, &cf);
        if (c2.desc.cur != c2.desc.mp3_start) {
            printf("FAIL: a start/end rewrite did NOT re-init (cur=%08x start=%08x)\n",
                   c2.desc.cur, c2.desc.mp3_start);
            fails++;
        }
    }

    /* ---- group 11: MAS3507D output gain curve (MAME mas3507d verbatim) ----
     * The value that matters is the one ddrsbm actually sends, captured off the
     * MAME oracle: 0xAF3CD. It is a BOOST (x1.2589), which is why applying the
     * game's gain is NOT a fix for the measured clipping. Pin the curve so that
     * conclusion cannot silently rot. */
    {
        float m_mute  = s573_core_gain_mult(0);
        float m_game  = s573_core_gain_mult(0xAF3CDu);
        float m_unity = s573_core_gain_mult(0xBFBDEu);
        int ok = 1;
        if (m_mute != 0.0f) {
            printf("FAIL: gain(0) = %f, expected exact 0 (mute)\n", m_mute); ok = 0;
        }
        if (m_game < 1.25f || m_game > 1.27f) {
            printf("FAIL: gain(0xAF3CD) = %f, expected ~1.2589 (a BOOST)\n", m_game); ok = 0;
        }
        if (m_unity < 0.99f || m_unity > 1.01f) {
            printf("FAIL: gain(0xBFBDE) = %f, expected ~1.0 (unity point)\n", m_unity); ok = 0;
        }
        if (m_game <= 1.0f) {
            printf("FAIL: the game's gain must be > unity; if this ever flips, the "
                   "'gain is not the clipping fix' conclusion needs revisiting\n"); ok = 0;
        }
        if (!ok) fails++;
    }


    if (!fails) printf("RESULT: PASS (s573mp3_core, 12 groups)\n");
    else        printf("RESULT: FAIL (s573mp3_core, %d checks failed)\n", fails);
    return fails ? 1 : 0;
}
