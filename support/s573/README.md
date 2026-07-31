# Konami System 573 — Digital I/O MP3 service (HPS side)

The HPS half of the System 573 core's MP3 audio path. Counterpart to
`fabricore-eng/System573_MiSTer` (branch `feat-digital-bringup`), whose
`rtl/s573_hps_ext.v` header is the **normative** protocol spec — read that before
changing anything here.

## Why this exists

The DIO board (GX894) keeps its music **scrambled** in a 32 MiB DRAM window. On real
hardware the board's Xilinx FPGA descrambles it and feeds a MAS3507D decoder chip. We
can't do the decode in fabric — the Cyclone V is at 97% logic / 100% multipliers and a
synthesizable MP3 decoder needs ~3–8k more logic units — so the decode runs here.

Decision B / **option (c)** (see the core repo's
`docs/2026-07-29-p4b-decision-b-spike-result.md`): we read the scrambled window
directly over `mmap`, descramble in C, decode with minimp3, and push PCM into a ring
the fabric drains at 44100 Hz. The fabric keeps a **position tracker** paced by the
consumption credit we report, because the game reads a "still streaming" bit derived
from it and its START/STOP routines are idempotence-guarded on that bit.

The descrambler here is not a fresh port: it is the exact C proven **byte-identical to
the RTL** over 45,377 bytes / 31 cases (`System573_MiSTer/sim/spike_b_descramble/`),
plus an exhaustive 2^32 cross-check of the core transform against MAME.

## Layout

| File | What it is | Testable on the host? |
|---|---|---|
| `s573_descramble.{h,c}` | the descrambler + stream semantics (2N−1 boundary, key schedule, byte order) | yes — proven against the RTL |
| `s573mp3_core.{h,c}` | pure service logic: epochs, config adoption, ring accounting, cumulative credit | yes — `test/test_core.c` |
| `s573mp3.{h,cpp}` | thin glue: SPI words, `shmem_map`, minimp3, the 5 ms poll | no (needs ARM + minimp3) |
| `test/test_core.c` | host unit tests | — |

The split exists so the parts that are easy to get *silently* wrong can be tested
without hardware. Run them:

```
cd support/s573/test && cc -O2 -Wall -Wextra -std=c99 -I.. -o test_core test_core.c ../s573mp3_core.c ../s573_descramble.c && ./test_core
```

Verified they can fail, not just pass: mutating cumulative→per-poll credit fails 123
checks; dropping the config rebaseline or the ring wrap each fail one.

## Two rules that are silent when broken

1. **`hps_cons_bytes` is CUMULATIVE.** A per-poll delta would credit the fabric again
   every poll and run its position — and the game-visible `0xae` bit 12 — far ahead of
   the audio, with every honesty counter still reading GREEN.
2. **The PTRS pointer words are ignored until the reset epoch is acked**, and the
   core-load reset bumps it too. So the first poll acks and does nothing else. One code
   path for every reset, at the cost of ~5 ms at startup.

And one prohibition: **do not synthesize the game-visible "streaming" bit here.** The
fabric owns it. `s573_desc_has_data()` is deliberately *not* that bit.

## Still to do

- [x] **Vendor minimp3** — `lib/minimp3/minimp3.h`, 76,831 bytes, sha256
      `57e437c5c1f0e8b243885d3929c8973b5e6c778451e0100ab4251d19915cb3ad`, from
      github.com/lieff/minimp3 (CC0 public domain, GPL-compatible).
- [x] **`s573mp3.cpp` compiles** — host-compiled clean under `-Wall -Wextra` against
      stubbed MiSTer headers (a mutation confirmed the check is real). It has NOT been
      compiled for ARM or linked into Main.
- [ ] **Build for ARM.** `arm-none-linux-gnueabihf` (per the top-level Makefile) is
      installed on neither the dev Mac nor `dell` — dell has only the Quartus image.
      Needs a toolchain decision: install on dell, use a cross-compile container, or
      build on the MiSTer itself.
- [x] **Deploy** — `tools/mister_load.sh --with-main` in the core repo. Opt-in, refuses a
      non-ARM ELF, warns if `s573mp3` did not link, keeps a `/media/fat/MiSTer.orig`
      backup and stages + atomically moves rather than truncating the live binary.
- [x] **`ctrl_flags` owners settled.** `ddrsbm` is the FABRIC's (OSD bit `O[101]`),
      arriving in the MP3CFG word — we must not push it. `DRAIN_EN` follows the GAME's
      `fpga_ctrl[13] & [14]` (play/stop), which k573dio now folds into `cfg_epoch` so we
      actually see it change.
- [ ] First hardware bring-up: `0xa8` advancing is the proof this service is live
      (design risk #6 — a stale binary silently disables MP3 with no error).

## Keeping the fork sane

`origin` is upstream `MiSTer-devel/Main_MiSTer` and is **never** pushed to;
`remote.pushDefault` is pinned to `fabricore`. This binary is **system-wide**, not
per-core, so it must stay rebased on upstream and must not regress other cores. Our
changes outside this directory are deliberately tiny: a core gate + poll call in
`user_io.cpp`, one declaration in `user_io.h`, one include in `support.h`, and one
`wildcard` line in the `Makefile`.
