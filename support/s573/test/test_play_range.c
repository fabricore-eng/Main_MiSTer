/* -----------------------------------------------------------------------------
 * test_play_range.c - the TRUE PLAY AUDIO range decode.
 *
 *   cc -O2 -Wall -Wextra -std=c99 -I.. -o test_play_range test_play_range.c \
 *      ../s573mp3_core.c -lm && ./test_play_range
 *
 * The fixtures are the five DISTINCT PLAY AUDIO CDBs captured off the live board
 * on 2026-08-08 (drmn attract, core 64a6ca35), together with the low-16 pair the
 * fabric published for each. They are real bytes, not invented ones, because the
 * bug being guarded against was a REASONING error about real data: `63615..3942`
 * was read as "this play crosses a 64K boundary" when it is simply 63615..69478
 * with the top bits sliced off.
 * ---------------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include "s573mp3_core.h"

static int fails = 0;
#define CHK(cond, ...) do { if (!(cond)) { \
    printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

/* build a PLAY AUDIO (10) CDB: opcode, 32-bit LBA at 2..5, 16-bit length at 7..8 */
static void mk45(uint8_t d[12], uint32_t lba, uint32_t len)
{
    memset(d, 0, 12);
    d[0] = 0x45;
    d[2] = (uint8_t)(lba >> 24); d[3] = (uint8_t)(lba >> 16);
    d[4] = (uint8_t)(lba >> 8);  d[5] = (uint8_t)lba;
    d[7] = (uint8_t)(len >> 8);  d[8] = (uint8_t)len;
}

struct fixture { uint32_t lba, len; const char *what; };

/* The five distinct plays actually seen on the board. */
static const struct fixture REAL[] = {
    { 23721, 5548, "in-window" },
    { 55205, 5981, "in-window" },
    { 63615, 5864, "STRADDLES 65536 -- prints as 63615..3942" },
    { 255990, 1314, "4 wraps up, prints innocuous" },
    { 294557, 2927, "4 wraps up, prints innocuous" },
};

int main(void)
{
    uint8_t cdb[2][12];
    uint32_t lo, hi;
    unsigned i;

    /* 1. Every real play decodes to its TRUE range. This is the green case, and
     *    it is also the whole point: row 3 must come back 63615..69478 and NOT
     *    63615..3942. */
    for (i = 0; i < sizeof REAL / sizeof REAL[0]; i++) {
        s573_play_latch_t l; memset(&l, 0, sizeof l);
        uint32_t want_end = REAL[i].lba + REAL[i].len - 1;
        memset(cdb, 0, sizeof cdb);
        mk45(cdb[0], REAL[i].lba, REAL[i].len);
        int ok = s573_play_range(&l, cdb, 1,
                                 (uint16_t)REAL[i].lba, (uint16_t)want_end, &lo, &hi);
        CHK(ok, "[%u] %s: not validated", i, REAL[i].what);
        CHK(lo == REAL[i].lba && hi == want_end,
            "[%u] %s: got %u..%u want %u..%u", i, REAL[i].what, lo, hi, REAL[i].lba, want_end);
        /* and the end must never come back BELOW the start -- the artifact itself */
        CHK(hi > lo, "[%u] %s: end %u below start %u -- truncation leaked through",
            i, REAL[i].what, hi, lo);
    }

    /* 2. RED-GREEN on the actual defect. The straddling play, read the OLD way
     *    (low 16 bits), yields an end below its start; read the new way it does
     *    not. If this ever stops holding, the artifact is back. */
    {
        uint32_t lba = 63615, end = 63615 + 5864 - 1;      /* = 69478 */
        CHK((uint16_t)end < (uint16_t)lba,
            "fixture no longer reproduces the artifact (%u vs %u)", (uint16_t)end, (uint16_t)lba);
        CHK((uint16_t)end == 3942, "the artifact value changed: got %u want 3942", (uint16_t)end);

        s573_play_latch_t l; memset(&l, 0, sizeof l);
        memset(cdb, 0, sizeof cdb);
        mk45(cdb[0], lba, 5864);
        CHK(s573_play_range(&l, cdb, 1, (uint16_t)lba, (uint16_t)end, &lo, &hi),
            "straddling play not validated");
        CHK(lo == 63615 && hi == 69478, "straddling play: got %u..%u want 63615..69478", lo, hi);
    }

    /* 3. A STALE latch must be refused, not printed. This is the guard that makes
     *    the value trustworthy: a play is missed (real "seq GAP" events exist),
     *    the fabric moves on, and the old latch no longer matches the low-16 pair. */
    {
        s573_play_latch_t l; memset(&l, 0, sizeof l);
        memset(cdb, 0, sizeof cdb);
        mk45(cdb[0], 23721, 5548);
        CHK(s573_play_range(&l, cdb, 1, 23721, 29268, &lo, &hi), "setup: first play not validated");

        /* now the fabric reports a DIFFERENT play and no 0x45 CDB arrives */
        memset(cdb, 0, sizeof cdb);
        cdb[0][0] = 0x42;                       /* READ SUBCHANNEL, as in the real logs */
        CHK(!s573_play_range(&l, cdb, 1, 55205, 61185, &lo, &hi),
            "stale latch was accepted -- it would print the wrong play's range");
    }

    /* 4. Nothing seen yet, or no play latched, must not validate. */
    {
        s573_play_latch_t l; memset(&l, 0, sizeof l);
        memset(cdb, 0, sizeof cdb);
        cdb[0][0] = 0x42;
        CHK(!s573_play_range(&l, cdb, 1, 0, 0, &lo, &hi), "empty latch validated");
        CHK(!s573_play_range(&l, cdb, 0, 0, 0, &lo, &hi), "validated with play_seen=0");
    }

    /* 5. A zero-length play is ignored rather than latched as a 0..-1 range. */
    {
        s573_play_latch_t l; memset(&l, 0, sizeof l);
        memset(cdb, 0, sizeof cdb);
        mk45(cdb[0], 1000, 0);
        CHK(!s573_play_range(&l, cdb, 1, 1000, 999, &lo, &hi), "zero-length play was latched");
    }

    /* 6. Slot 1 (the older CDB) is used when slot 0 is not a play -- the witness
     *    ships two and the newest is usually 0x42. */
    {
        s573_play_latch_t l; memset(&l, 0, sizeof l);
        memset(cdb, 0, sizeof cdb);
        cdb[0][0] = 0x42;
        mk45(cdb[1], 55205, 5981);
        CHK(s573_play_range(&l, cdb, 1, 55205, 61185, &lo, &hi), "slot-1 play not picked up");
        CHK(lo == 55205 && hi == 61185, "slot-1: got %u..%u want 55205..61185", lo, hi);
    }

    printf(fails ? "test_play_range: %d FAILURE(S)\n" : "test_play_range: all pass\n", fails);
    return fails ? 1 : 0;
}
