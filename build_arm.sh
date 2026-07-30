#!/usr/bin/env bash
# Cross-build Main_MiSTer for the MiSTer's ARM (DE10-Nano, Cortex-A9) on `dell`.
#
# Uses a container rather than installing a toolchain on the shared box, and pins
# the EXACT compiler the upstream Makefile names ("using gcc version 10.2.1",
# BASE = arm-none-linux-gnueabihf) -- the ARM GNU Toolchain 10.2-2020.11. Not
# Debian's gcc-arm-linux-gnueabihf: MiSTer's rootfs is an older userland and a
# newer distro toolchain can emit binaries needing a glibc the board lacks.
#
# Build the image once:  docker build -f Dockerfile.mister -t mister-arm:10.2 .
# Then, from the repo root ON DELL:  ./build_arm.sh
#
# Output: bin/MiSTer (ARM 32-bit ELF). Verify our half actually linked with:
#   strings bin/MiSTer | grep s573mp3
set -euo pipefail
IMG=${IMG:-mister-arm:10.2}
docker run --rm -v "$PWD":/work -w /work "$IMG" make "$@"
echo
echo "== built =="
file bin/MiSTer
strings bin/MiSTer | grep -c s573mp3 | xargs -I{} echo "s573 service strings present: {}"
