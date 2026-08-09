/* -----------------------------------------------------------------------------
 * test_seq_sink.c - the bounded per-CDB sequence log.
 *
 *   cc -O2 -Wall -Wextra -std=c99 -I.. -o test_seq_sink test_seq_sink.c \
 *      ../s573mp3_core.c ../s573_descramble.c -lm && ./test_seq_sink
 *
 * The property under test is the one the board cares about: however long the
 * instrument runs, it must not be able to fill /tmp. Measured on 2026-08-08 the
 * unbounded log grew at ~17.4 MB/h into a 247 MB tmpfs on a 492 MB board, i.e.
 * ~12.6 h to full -- and the fault being hunted only appears during long armed
 * runs, so the log has to be safe to leave on rather than merely short.
 * ---------------------------------------------------------------------------- */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "s573mp3_core.h"

static int fails = 0;
#define CHK(cond, ...) do { if (!(cond)) { \
    printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

static long fsize(const char *p)
{
    FILE *f = fopen(p, "rb");
    long n;
    if (!f) return -1;
    fseek(f, 0, SEEK_END); n = ftell(f); fclose(f);
    return n;
}

static int file_contains(const char *p, const char *needle)
{
    char buf[1 << 16]; size_t n; int hit = 0;
    FILE *f = fopen(p, "rb");
    if (!f) return 0;
    n = fread(buf, 1, sizeof buf - 1, f); buf[n] = 0;
    hit = strstr(buf, needle) != NULL;
    fclose(f);
    return hit;
}

int main(void)
{
    const char *path = "/tmp/s573_seq_test.log";
    char older[300];
    s573_seq_sink_t s;
    int i;
    const uint64_t CAP = 4096;

    snprintf(older, sizeof older, "%s.1", path);
    remove(path); remove(older);

    CHK(s573_seq_sink_open(&s, path, CAP) == 0, "open failed");

    /* ~1000 lines of ~45 bytes = ~45 KB against a 4 KB cap: many rotations. */
    for (i = 0; i < 1000; i++)
        s573_seq_sink_printf(&s, "s573: seq #%-5d 42 00 40 01 cdda=1\n", i);
    s573_seq_sink_close(&s);

    /* 1. The live file NEVER exceeds the cap. Rotation happens before the write
     *    that would breach it, so a line can never straddle the limit. */
    CHK(fsize(path) >= 0, "live file missing");
    CHK((uint64_t)fsize(path) <= CAP, "live file %ld bytes exceeds cap %llu",
        fsize(path), (unsigned long long)CAP);

    /* 2. Total on disk is bounded by 2*cap -- the actual safety property. */
    {
        long a = fsize(path), b = fsize(older);
        if (b < 0) b = 0;
        CHK((uint64_t)(a + b) <= 2 * CAP, "total %ld bytes exceeds 2*cap %llu",
            a + b, (unsigned long long)(2 * CAP));
    }

    /* 3. It rotated at all -- without this the two checks above pass trivially
     *    on a sink that silently dropped everything. */
    CHK(s.rotations > 0, "never rotated (wrote 45 KB through a 4 KB cap)");
    CHK(fsize(older) > 0, "no rotated-out file kept");

    /* 4. The NEWEST line survives. Keeping the tail is the whole reason this is
     *    rotation and not cap-and-stop: the fault shows up late. */
    CHK(file_contains(path, "seq #999  "), "newest line lost -- tail not preserved");

    /* 5. The OLDEST line is gone, i.e. it really is bounded and not appending. */
    CHK(!file_contains(path, "seq #0     ") || s.rotations == 0,
        "oldest line still in the live file after rotation");

    /* 6. stdout mode (path NULL) must not rotate or touch the filesystem --
     *    still reachable, but from 2026-08-09 only when the caller ASKS for it
     *    by name (see 8); it is no longer what an unset environment selects. */
    {
        s573_seq_sink_t d;
        CHK(s573_seq_sink_open(&d, NULL, 0) == 0, "stdout-mode open failed");
        CHK(d.f == NULL, "stdout mode opened a file");
        CHK(d.rotations == 0, "stdout mode rotated");
        s573_seq_sink_close(&d);
    }

    /* 7. An unopenable path falls back to stdout rather than losing the log. */
    {
        s573_seq_sink_t d;
        CHK(s573_seq_sink_open(&d, "/nonexistent-dir-xyz/log", CAP) == -1,
            "bad path did not report failure");
        CHK(d.f == NULL, "bad path left a file handle");
        s573_seq_sink_printf(&d, "");       /* must not crash */
        s573_seq_sink_close(&d);
    }

    /* 8. THE SAFETY DEFAULT. Everything above only bounds the log once a caller
     *    passes a path; with S573_CDB_SEQ_FILE unset the sink used to fall back
     *    to UNBOUNDED stdout, so "armed" still meant "can fill /tmp". Measured
     *    2026-08-09: a session armed the instrument with S573_CDB_SEQ=1 alone
     *    and redirected stdout by hand -- exactly the unbounded path. So the
     *    resolver, not the caller, decides: unset/empty picks the capped file,
     *    and unbounded stdout has to be spelled out as "-". */
    {
        const char *d_unset = s573_seq_sink_default_path(NULL);
        const char *d_empty = s573_seq_sink_default_path("");
        const char *d_dash  = s573_seq_sink_default_path("-");
        const char *d_named = s573_seq_sink_default_path("/tmp/mine.log");

        CHK(d_unset != NULL, "unset S573_CDB_SEQ_FILE still selects unbounded stdout");
        CHK(d_empty != NULL, "empty S573_CDB_SEQ_FILE still selects unbounded stdout");
        CHK(d_unset && d_empty && !strcmp(d_unset, d_empty),
            "unset and empty disagree on the default path");
        CHK(d_dash == NULL, "\"-\" did not select stdout");
        CHK(d_named && !strcmp(d_named, "/tmp/mine.log"),
            "an explicit path was not passed through");

        /* And the default must really be usable as a bounded sink, not just a
         * non-NULL string: open it and confirm a file handle plus a live cap. */
        if (d_unset) {
            s573_seq_sink_t d;
            char d_older[300];
            CHK(s573_seq_sink_open(&d, d_unset, 0) == 0,
                "default path %s could not be opened", d_unset);
            CHK(d.f != NULL, "default path did not open a file");
            CHK(d.cap > 0, "default sink has no cap");
            s573_seq_sink_printf(&d, "hello\n");
            s573_seq_sink_close(&d);
            snprintf(d_older, sizeof d_older, "%s.1", d_unset);
            remove(d_unset); remove(d_older);
        }
    }

    remove(path); remove(older);
    printf(fails ? "test_seq_sink: %d FAILURE(S)\n" : "test_seq_sink: all pass\n", fails);
    return fails ? 1 : 0;
}
