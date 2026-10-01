#!/usr/bin/env bash
#
# check-image.sh - structural checks on a built AerialKit image.
#
#   scripts/check-image.sh build/aerialkit-f405.bin scripts/image-facts/stm32f405rg.txt [elf] [--telltale] [--usb-trace]
#
# These are the things that can be checked without the board, and they are the
# ones that are expensive to get wrong: if the vector table is not the first
# thing in the image, or the reset vector does not point into flash, the board
# does nothing at all and the reason is invisible from the outside.
#
# **The numbers this script measures against are the part's, not the image's**,
# so they live in scripts/image-facts/<part>.txt rather than in constants here:
# the stack top, how many words the vector table has, and which handler each
# named entry must point at. Two parts with two different tables is exactly the
# case that made this a file - the F405's table runs to interrupt 39 and the
# AT32's to channel 2 of DMA1 at 57, and a shared constant would have been four
# words too short for one of them.
#
# A pass here proves the image is *shaped* correctly. It says nothing about
# whether the code runs, which needs the board.
#
# `--telltale` and `--usb-trace` are the checks about *which* image this is
# rather than about its shape, and they are one family: an image whose status
# pin is an instrument rather than a heartbeat.
#
#   --telltale   -DAK_BOOT_STAGE=1: every boot stage blinked as its own number.
#                An image named a tell-tale without the flag would blink
#                nothing, which reads on a board with no console exactly like a
#                boot that stopped at stage zero. See docs/05-bringup.md 6a.
#
#   --usb-trace  -DAK_USB_TRACE=1: the USB counters blinked out at ten
#                seconds, for a board whose console doors are both shut.
#
# Both of them read the pin through the same plan - `ak_boot_blink_plan()` and
# `ak_boot_run()` - which is the whole reason the reader of one can read the
# other. So the flight-image rule below ("no plan linked, small mark") is a
# rule about *neither* flag being set, and an image built with either is
# checked for carrying the instrument instead of for not carrying it.
#
set -euo pipefail

usage='usage: check-image.sh <image.bin> <facts-file> [elf] [--telltale] [--usb-trace]'

telltale=0
usb_trace=0
keep=()
for argument in "$@"; do
    case "$argument" in
    --telltale) telltale=1 ;;
    --usb-trace) usb_trace=1 ;;
    *) keep+=("$argument") ;;
    esac
done
set -- ${keep[0]+"${keep[@]}"}

bin="${1:?${usage}}"
facts="${2:?${usage}}"
elf="${3:-}"

[ -f "$facts" ] || { echo "check-image: no such facts file: $facts" >&2; exit 1; }

# The part's own numbers, and nothing about them is defaulted: a facts file
# that is missing one is a facts file that would otherwise silently check the
# image against somebody else's hardware.
stack_top=""
vector_words=""
symbol_rows=()

while read -r key a b c; do
    case "$key" in
    ""|\#*) continue ;;
    stack-top)    stack_top="$a" ;;
    vector-words) vector_words="$a" ;;
    # index, symbol, and what stops working if the entry is wrong. The `what`
    # keeps its spaces: it is a sentence in the output, not a field.
    vector)       symbol_rows+=("$a $b $c") ;;
    *) echo "check-image: $facts: unknown fact '$key'" >&2; exit 1 ;;
    esac
done < "$facts"

[ -n "$stack_top" ] || { echo "check-image: $facts has no stack-top" >&2; exit 1; }
[ -n "$vector_words" ] ||
    { echo "check-image: $facts has no vector-words" >&2; exit 1; }

# od hands back bare hex, and bash would read a leading zero as octal, so every
# value goes through here.
hex() { printf '%d' "$((16#${1#0x}))"; }

[ -f "$bin" ] || { echo "check-image: no such file: $bin" >&2; exit 1; }

flash_base=0x08000000
flash_end=$((flash_base + 0x100000))

size=$(stat -c %s "$bin")
[ "$size" -gt 64 ] || { echo "check-image: image is only $size bytes" >&2; exit 1; }
(( size % 4 == 0 )) || { echo "check-image: size $size is not a multiple of 4" >&2; exit 1; }

# The vector table is the first thing in the image. How long it is belongs to
# the part: the word count is the highest interrupt the table names, plus the
# sixteen core vectors. od wraps its output every 16 bytes, so split on
# whitespace and collect across lines.
mapfile -t v < <(od -An -v -tx4 -N$((vector_words * 4)) "$bin" | tr -s ' ' '\n' | grep -v '^$')
[ "${#v[@]}" -ge "$vector_words" ] || {
    echo "check-image: image is shorter than the $vector_words-word vector table" >&2
    exit 1
}

fail=0
ok()   { printf '  ok       %s\n' "$1"; }
bad()  { printf '  FAILED   %s\n' "$1" >&2; fail=1; }
# For the things that are neither: which of two honest images this is. A note
# cannot fail, which is why it is not a check - but it is printed, because an
# image whose reader is a wire instead of a lamp should say so.
note() { printf '  note     %s\n' "$1"; }

printf '%s (%s bytes, %s)\n' "$bin" "$size" "$facts"
printf '  vector table:\n'
printf '    [ 0] initial sp   0x%08x\n' "$(hex "${v[0]}")"
printf '    [ 1] reset        0x%08x\n' "$(hex "${v[1]}")"
printf '    [15] systick      0x%08x\n' "$(hex "${v[15]}")"

# 1. The hardware loads the stack pointer from word 0 before it runs anything.
if [ "$(hex "${v[0]}")" = "$(hex "$stack_top")" ]; then
    ok "initial stack pointer is $stack_top (top of SRAM)"
else
    bad "initial stack pointer is 0x${v[0]}, expected $(printf '0x%08x' "$((stack_top))")"
fi

# 2. Word 1 is the reset vector: into flash, and thumb (bit 0 set).
reset=$(hex "${v[1]}")
if (( reset >= flash_base && reset < flash_end )); then
    ok "reset vector 0x$(printf '%08x' "$reset") is inside flash"
else
    bad "reset vector 0x$(printf '%08x' "$reset") is outside flash"
fi
if (( reset & 1 )); then
    ok "reset vector has the thumb bit set"
else
    bad "reset vector has the thumb bit clear (not a thumb address)"
fi

# 3. SysTick is the one peripheral interrupt M0 uses. It has to be a real
#    handler, and not the same address as the fault handler.
systick=$(hex "${v[15]}")
if (( systick & 1 )) && (( systick >= flash_base && systick < flash_end )); then
    ok "systick vector 0x$(printf '%08x' "$systick") is a thumb address in flash"
else
    bad "systick vector 0x$(printf '%08x' "$systick") is not a usable handler"
fi
if [ "$systick" != "$(hex "${v[3]}")" ]; then
    ok "systick has its own handler, not the fault handler"
else
    bad "systick points at the fault handler"
fi

# 4. Reserved entries must stay zero: a stray address there is a copy-paste in
#    the table, and the entry number is how you find it.
for i in 7 8 9 10 13; do
    if [ "$(hex "${v[i]}")" != 0 ]; then
        bad "reserved vector [$i] is 0x${v[i]}, expected 0"
    fi
done
[ "$fail" = 0 ] && ok "reserved vector entries are zero"

# 5. With the ELF in hand, the vectors this firmware depends on have to point
#    at the functions they are supposed to. A handler that silently lands on
#    the default loop is a feature that does not exist and says nothing about
#    it: a hard fault loses its record, a SysTick stops the clock, a DMA
#    transfer-complete never marks the frame done, and a receiver stays empty.
#    The table grows as handlers are added, so this is a list, not a pair of
#    checks.
if [ -n "$elf" ]; then
    nm="${NM:-arm-none-eabi-nm}"
    if ! command -v "$nm" >/dev/null 2>&1; then
        echo "check-image: $nm not on PATH; skipping the symbol checks" >&2
    else
        # No early exit in the awk: with pipefail set, an early-closing reader
        # gives nm a SIGPIPE and the pipeline a 141, which set -e turns into a
        # build failure. It only started happening once the symbol table grew
        # past the pipe buffer, when nm had more to write than awk wanted to
        # read. The workspace's compare-firmware.sh documents the same trap.
        symbol() {
            "$nm" "$elf" 2>/dev/null |
                awk -v want="$1" '$3 == want { found = $1 } END { if (found != "") print found }'
        }

        # vector index, symbol, and what stops working if it is wrong - the
        # rows the facts file carries.
        for row in "${symbol_rows[@]}"; do
            read -r index want what <<< "$row"
            addr="$(symbol "$want")"
            if [ -n "$addr" ] && [ "$(hex "${v[index]}")" = "$(( $(hex "$addr") | 1 ))" ]; then
                ok "vector $index is $want ($what)"
            else
                bad "vector $index is not $want (0x$(printf '%08x' "$(hex "${v[index]}")")), so $what"
            fi
        done

        # 6. Which image this is. The tell-tale is the same sources compiled
        #    with `-DAK_BOOT_STAGE=1`, and the two builds differ in exactly one
        #    visible way: the mark is the board's real one (a plan built and
        #    run against the status pin) instead of the one-instruction stub
        #    the plain board file compiles. Two instructions and a `ret` are
        #    what "the flag did not reach the compiler" looks like in the
        #    image - and on a board whose console never came up, that image is
        #    indistinguishable from a boot that stopped before stage one.
        #
        #    The flight image is checked in the other direction for the same
        #    reason: the flag left on by accident costs flash, and a heartbeat
        #    boot that spends a minute and a half blinking stage numbers is not
        #    an aircraft anybody wants to arm.
        sized() {
            "$nm" --print-size "$elf" 2>/dev/null |
                awk -v want="$1" '$4 == want { size = $2 } END { if (size != "") print size }'
        }
        linked() {
            "$nm" "$elf" 2>/dev/null |
                awk -v want="$1" '$3 == want { found = 1 } END { if (found) print "yes" }'
        }

        mark="$(sized ak_board_boot_mark)"
        if [ -z "$mark" ]; then
            bad "there is no ak_board_boot_mark in the image at all"
        else
            mark_bytes=$((16#$mark))
            if [ "$telltale" = 1 ]; then
                # The mark is the tell-tale's own half: the *boot path* has to
                # be the one calling into the plans. A stub here is what "the
                # flag did not reach the compiler" looks like.
                if [ "$mark_bytes" -ge 64 ] &&
                   [ -n "$(linked ak_boot_blink_plan)" ] &&
                   [ -n "$(linked ak_boot_run)" ]; then
                    ok "this is the tell-tale: $mark_bytes bytes of mark, and the blink plans are linked"
                else
                    bad "this image is named a tell-tale but ak_board_boot_mark is $mark_bytes bytes with no blink plan behind it (was it built without -DAK_BOOT_STAGE=1?)"
                fi
            elif [ "$usb_trace" = 1 ]; then
                # There is no boot path to check here, because the report runs
                # from the main loop and the board's own mark is not involved.
                # So the two things to measure are the plan it blinks through
                # and the accessor that reads the counters into it: the plan
                # alone is also what the tell-tale links, so a build that
                # dropped `ak_usb_trace_get` would still show a plan and still
                # say nothing about USB. Both ends, or the pin is a heartbeat.
                if [ -n "$(linked ak_boot_blink_plan)" ] &&
                   [ -n "$(linked ak_boot_run)" ] &&
                   [ -n "$(linked ak_usb_trace_get)" ]; then
                    ok "this is the usb trace: the blink plans and the trace accessor are linked, so the pin is read from the main loop"
                else
                    bad "this image is named a usb trace but the blink plans or ak_usb_trace_get are missing, so its status pin says nothing (was it built without -DAK_USB_TRACE=1?)"
                fi

                # Two honest readings of one flag, and the reader should know
                # which one is in front of them: a trace that only blinks, or a
                # trace that also writes the numbers to a flash page so a host
                # can fetch them without counting. Neither is wrong, so neither
                # is a failure - but the second one needs BOOT0 and a reset to
                # get at, because the hand-over it was built for does not work
                # on this part (measured 2026-09-27, docs/05-bringup.md 6c).
                if [ -n "$(linked ak_board_trace_save)" ]; then
                    note "and it writes the numbers to a flash page, read back over DFU after a BOOT0 press, so a host can decode them without counting (docs/05-bringup.md 6c)"
                else
                    note "the numbers are on the status pin only; this image keeps no record in flash"
                fi
            else
                if [ "$mark_bytes" -le 32 ] &&
                   [ -z "$(linked ak_boot_blink_plan)" ]; then
                    ok "a flight image does not carry the tell-tale: a $mark_bytes-byte mark, no blink plan linked"
                else
                    bad "a flight image is carrying the tell-tale: ak_board_boot_mark is $mark_bytes bytes and the blink plan is linked"
                fi

                # 7. And the same question for the second bring-up flag, which
                #    is the one that is a hazard rather than a nuisance.
                #
                #    `-DAK_FAULT_REBOOT=1` makes the fault handler request a
                #    reset instead of stopping. On a bench that is the whole
                #    instrument: the record in .noinit is printed by the *next*
                #    boot, so something has to perform the restart, and with no
                #    watchdog nothing does. In the air it is a different thing
                #    entirely - a reset is a restart nobody asked for, on a
                #    machine that has just proved it cannot be trusted, and the
                #    watchdog is the only thing allowed to make that call.
                #
                #    This sits in the flight branch on purpose: the flag is
                #    legitimate in a bring-up image, and the tell-tale is one.
                #    They answer different questions - the LED for a board with
                #    no console, this for a board with one - and nothing says
                #    they cannot be combined, so a tell-tale is not checked
                #    here. The threshold has to allow for that anyway: the
                #    tell-tale build measures 14 bytes at this symbol, not 8,
                #    because its fault path also calls ak_boot_mark(). Both
                #    ends were measured - 8 stopping, 24 resetting, 14 for the
                #    tell-tale - and 16 admits the first two and excludes the
                #    third from being read as either.
                capture="$(sized ak_fault_capture)"
                if [ -z "$capture" ]; then
                    bad "there is no ak_fault_capture in a target image"
                else
                    capture_bytes=$((16#$capture))
                    if [ "$capture_bytes" -le 16 ]; then
                        ok "a flight image stops on a fault rather than resetting: ak_fault_capture is $capture_bytes bytes"
                    else
                        bad "ak_fault_capture is $capture_bytes bytes, too big for the stop-alone path (was this built with -DAK_FAULT_REBOOT=1? a flight image must not reset itself on a fault)"
                    fi
                fi
            fi
        fi
    fi
fi

if [ "$fail" = 0 ]; then
    echo 'result: shape ok (nothing here proves the code runs)'
else
    echo 'result: image is not usable as built' >&2
fi
exit "$fail"
