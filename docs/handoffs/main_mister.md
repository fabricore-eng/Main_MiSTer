# Handoff — main_mister — 2026-08-10 22:20 UTC
Branch: `claude/system573-cd-titles-test-je3lrq`   ·   Repo: `fabricore-eng/main_mister`

> This is the **fork**, not MiSTer-devel upstream. Everything here exists to serve the 573 lane.
> Read that lane's handoff first: `docs/handoffs/573.md` in `fabricore-eng/system573_mister-dev`.

## TL;DR
Tree is clean and everything is pushed; nothing was changed here this session. Two earlier
changes on this branch are load-bearing for 573 and both are already **live on the board**.

## State of play
- **Done — the pregap fix** (`f10ab64`, correcting `e32f702`). `support/psx/psx.cpp`
  `load_chd()` double-counted a pregap that a CHD does not store, putting every track from the
  third onward **+150 frames = 2.000 s** late. The first attempt used each track's *own*
  preceding gap, which would have broken DrumMania; modelling both discs caught it
  (ddrjb 26/28→0/28, drmn 0/69→**26+/69 regression**) and the corrected version spans the gap
  **AFTER** each track (drmn back to 0/69). Deployed; the operator confirmed the charts are in
  sync.
- **Done — the CDB sequence log is bounded by DEFAULT** (`680dfda`, then `3a59ef7`). Armed with
  `S573_CDB_SEQ=1` and no file set, it writes a self-rotating capped file instead of streaming
  to stdout forever. **Verified in production**, not just in code: on the board right now
  `/tmp/s573_seq.log` is 35 MB with a rotated `.1` at 48 MB — it hit the cap and rotated.
  Steady-state ceiling ≈96 MB = **39% of the 247 MB tmpfs**, under the 60% action line. The
  instrument is safe to leave armed.
- **In progress:** nothing.
- **Blocked:** nothing here. (`main` being behind is a 573-lane trunk-hygiene question.)

## Key decisions (and why)
- **Gap AFTER, not the track's own gap.** Both readings fit `ddrjb`; only one fits DrumMania.
  Whenever this is touched again, model at least those two discs before believing a fix.
- **The log cap is the default, not an opt-in.** An instrument that can fill a 247 MB tmpfs on a
  492 MB board turns itself into the fault. `S573_CDB_SEQ_FILE=-` still gives unbounded stdout
  for a short attended run.

## Refuted — do not re-derive
- **"`PreGap: 0` in the board log proves the pregap theory wrong."** It does not — that log line
  prints a local that `mister_chd.cpp:112` zeroes four lines before the print at `:157`. A bad
  measurement killed the correct hypothesis for a while; round two reversed it.

## Next steps
1. Nothing required in this repo. Any further work here will come from the 573 lane — most
   likely `A2` from the flash-window investigation: stop skipping the 573 flash storage slot in
   `support/arcade/mra_loader.cpp:1513` when the `.mra` declares no index-2 preload. That is
   belt-and-braces only; the generator-side fix (already committed in the 573 lane) should make
   it unnecessary.
   **Verify:** if attempted, rebuild and copy Main to the card — no Quartus build needed — and
   confirm a cold `ddra` still reaches the install prompt.

## Hardware state
- **MiSTer binary on the board:** md5 `4ac28e743a8774a130ccdca2a8b656ce`, 1,100,540 B,
  2026-08-10 06:58. Believed built from `f10ab64`; `3a59ef7` is an ancestor, and the observed
  log rotation confirms the cap is in it.
- **18 MiSTer binaries are kept on the board and every one is a distinct build — do not prune.**

## Landmarks
- `support/psx/psx.cpp` — `load_chd()`, `gap_after[]`
- `support/s573/s573mp3.cpp:654-690` — the bounded sequence log and its env gates
- `support/arcade/mra_loader.cpp:1513` — the skipped storage slot behind the 573 flash bug
- `support/arcade/mra_loader.cpp:363,396` — `s573_seed_flash_save`, which creates the `.sav`
  from an index-2 preload

## Open questions / risks
- The exact commit the deployed binary was built from is **inferred**, not recorded. If that
  ever matters, rebuild and redeploy rather than trusting the inference.
