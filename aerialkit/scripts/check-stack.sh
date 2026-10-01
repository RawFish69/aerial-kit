#!/usr/bin/env bash
#
# check-stack.sh - how much stack this firmware needs, from the compiler's graph.
#
#   scripts/check-stack.sh                       # the F405 image
#   scripts/check-stack.sh AERIALKIT_GHF435 at32f435 at32f435rg aerialkit-ghf435
#   scripts/check-stack.sh ... --limit 24576     # fail above the ceiling
#
# Why: the host tests run with eight megabytes of stack and the target has
# about forty-nine kilobytes free after its blackbox rings, so "does it fit" is
# a question only the compiler's own records can answer here. `-fstack-usage`
# gives each frame's size, `-fcallgraph-info=su` gives who calls whom, and
# tools/stack_report.py walks the second to add up the first. See that file for
# what the number is and is not.
#
# Both flags have to reach the *compiler*, and the Makefile sets `CC :=
# $(CROSS)gcc` with the flags after it, so the callgraph flag goes on `CC`. That
# also keeps it out of the way of a caller who passes `EXTRA_CFLAGS` for a board
# variant: a command-line `EXTRA_CFLAGS` overrides the environment, so a flag
# sent that way would displace the variant's defines rather than join them.
#
# Until 2026-09-21 this paragraph said something else, and it is false. It said
# `EXTRA_CFLAGS=` "is not a Makefile variable" and that "no target in this
# Makefile reads" it. `Makefile:157` is `CFLAGS += $(EXTRA_CFLAGS)`, added by
# `e454275` - the same commit that added this script - and that line is what
# builds the fitted and tell-tale images the profiles matrix describes. Whether
# it was true of the draft it describes cannot be re-derived from the tree
# today, and `docs/21-port-on-the-host.md` carried the same sentence; both now
# say what is true instead. The rule the paragraph reaches - put the callgraph
# flag on `CC` - survives the reason, and has a better one: a *command-line*
# `EXTRA_CFLAGS` overrides the environment, so sending the flag that way would
# displace a board variant's defines rather than join them. A false reason under
# a true conclusion is the kind of thing nobody re-reads, and what made this one
# get read was turning `make stack-check` into a CI stage, which meant reading
# this whole file to decide what the stage needs. Measured while doing it: the
# same profile with and without a define passed in `EXTRA_CFLAGS` gives two
# different ELFs (`aerialkit-f405-fitted`, 760,300 bytes, 8c8f1e74... against
# the bare 758,816 bytes, fd58e2c9...) and two different call graphs - 426
# functions against 423 - which is the flag demonstrably reaching the compiler.
#
# `-fstack-usage` needs no help: the image build already has it (it is what
# makes the per-frame numbers exist at all).
#
# The ARM toolchain has to be on PATH (the workspace harness has it under
# toolchain/); nothing else is needed and no board is involved.
#
set -euo pipefail

board="${1:-AERIALKIT_F405}"
arch="${2:-stm32f405}"
part="${3:-stm32f405rg}"
product="${4:-aerialkit-f405}"
cc="${CROSS:-arm-none-eabi-}gcc"

root="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
out="${STACK_OUT:-$root/build-stack/$product}"

cd "$root"

if ! command -v "$cc" >/dev/null; then
    echo "no $cc on PATH - the workspace toolchain is under toolchain/*/bin" >&2
    exit 1
fi

echo "== building $product with the call graph and the frame sizes"
make BOARD="$board" ARCH="$arch" PART="$part" PRODUCT="$product" \
     OUT="$out" HOST_OUT="$out/host" \
     CC="$cc -fcallgraph-info=su" all >/dev/null

python3 tools/stack_report.py "$out" "${@:5}"
