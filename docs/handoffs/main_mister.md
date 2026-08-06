# Handoff — Main fork (573 work) — 2026-08-06 21:15
Branch: `claude/system573-framework-refresh-115qgm`   ·   Repo: `fabricore-eng/main_mister`

## TL;DR

This fork exists to serve the **573** core; it is not an independent lane and has no charter
of its own. **The entry point for this work is `docs/handoffs/573.md` in
`fabricore-eng/System573_MiSTer-dev` (branch `claude/system573-framework-refresh-115qgm`)** —
read that first. This file records only what is true on the Main side.

## Current branches (all three repos in play)

| Repo | Branch | Head |
|---|---|---|
| `fabricore-eng/main_mister` | `claude/system573-framework-refresh-115qgm` | `950d99d` |
| `fabricore-eng/System573_MiSTer-dev` | `claude/system573-framework-refresh-115qgm` | see that repo |
| `fabricore-eng/fabricore` | `claude/system573-framework-refresh-csj49s` | workspace root |

## State of play

- **Done, deployed and md5-verified on the board:** `19f99c5`, built to md5
  `4163616fd59adcca0d60359523d40c31`, matches `/media/fat/MiSTer`. It reads the core's ATAPI
  CDB trace out of `CMD_573_STATUS` and reports it. That instrument produced the session's
  only real reading (see the 573 doc).
- **Built but NOT deployed:** `950d99d`, md5 `e5feafb6e38fd2c7d9e543383629501a`, sitting on
  dell at `/home/human/Main_MiSTer/bin/MiSTer`. It reads STATUS words 12..23 (the two newest
  whole 12-byte CDBs) and filters torn reads. **Deploy it together with the core build
  `c1b9ac0`, not before** — a new Main against the old core clocks 12 words the fabric does
  not serve.
- **Earlier this session, already on the board and verified:** the `<disc>` → PSX CD mount
  route, `role=`-based multi-disc selection, `kMaxImages` 4→8, and flash-save seeding so a
  first install sticks.

## Key decisions (and why)

- **`ext_status()` reads a widening reply, and that is deliberately backward-compatible.**
  The fabric serves words 1..23 from the same snapshot bank; an older Main simply drops
  `io_enable` early and never clocks the trace words. Do not turn this into a second SPI
  exchange — one snapshot is what makes count, opcodes and CDBs a single consistent reading.
- **The torn-read guard is a FILTER, not a fix.** It drops a reply only when the trace tail
  is all-zero AND a non-zero trace has already been seen, so a genuine post-reset zero still
  reports, and it counts what it drops. If that count is large the fabric mailbox needs a
  look — see the 573 doc's *Open questions*.
- **Deploy the `.rbf` to BOTH board paths.** `.mgl` launchers load from `_Console/`, `.mra`
  ones from `_Arcade/cores/`. `/tmp/deploy_and_probe.sh` on dell does both.

## Refuted — do not re-derive

- **`get_rbf_name()` as the basis for `is_573()`** — it returns the `.mra` path on an arcade
  launch, so the test was false for the entire arcade route (and silently killed MP3 audio
  for 71 titles). Replaced with CONF_STR matching. Built, deployed, measured wrong.
- **Positional multi-disc selection** — wrong for the 3 sets that list the install disc
  second and meaningless for the 7 with no install disc. MAME's `region=` is authoritative.
- **Arming the empty flash slot at mount** — froze boot at `START UP...`, A/B verified.
  The correct fix was seeding the `.sav` from the `<rom index=2>` preload.

## Next steps

1. Wait for core build `c1b9ac0`, then deploy this fork's `950d99d` **and** that `.rbf`
   in one pass: `ssh dell 'bash /tmp/deploy_and_probe.sh'`.
   **Verify with:** `md5sum /media/fat/MiSTer` on the board = `e5feafb6e38fd2c7d9e543383629501a`,
   and the trace line printing `last=[…] prev=[…]` with twelve hex bytes each.
2. Everything after that is a 573-side question. Follow that lane's doc.

## Landmarks

- `support/s573/s573mp3.cpp:191` — `ext_status()`, the 23-word STATUS read.
- `support/s573/s573mp3.cpp:538` — the torn-read guard and its counter.
- `support/s573/s573mp3.cpp:586` — the trace report line.
- `support/arcade/mra_loader.cpp:363` — flash-save seeding from the factory preload.
- `support/arcade/mra_loader.cpp:1539` — the PSX CD dispatch for `<disc>`.
- `support/psx/psx.cpp:754` — `psx_mount_cd_media()`, the console-convention-free sibling
  of `psx_mount_cd()` (no BIOS, no game id, no region probe, no memory card).

## Open questions / risks

- The build toolchain here is `mister-arm:10.2` on dell via docker; `make` outputs
  `bin/MiSTer`, and `scp` onto a running `/media/fat/MiSTer` fails `ETXTBSY` — copy to a temp
  name and `mv`.
- The board has no `pkill`; kill MiSTer by PID from `ps w`.
