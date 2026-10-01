#!/usr/bin/env python3
#
# trace-page.py - decode the AK_USB_TRACE record ring a host read back with DFU.
#
#   dfu-util -d 0483:df11 -a 0 -s 0x0801F000:0x1000 -U trace-page.bin
#   scripts/trace-page.py trace-page.bin
#
# The firmware writes this record ten seconds into its main loop, and a host
# reads it out of the ROM's DFU. Neither end of that is this file: the writer is
# `ak_board_trace_save()` in src/boards/AERIALKIT_F405/board.c and the *reason*
# each field exists is the note above `ak_usb_trace_t` in src/core/ak_board.h.
# What lives here is the third thing a format needs and the two ends cannot
# supply - a decode that a person can run without reading the firmware, against
# a layout written down in docs/05-bringup.md section 6c.
#
# **The layout is duplicated on purpose and that is a hazard worth naming.** The
# writer is C on a target and the reader is Python on a host, so there is no
# header both can include; what keeps them honest is the FNV-1a sum in the last
# word, which this script recomputes and refuses a record whose sum disagrees.
# A record that stops matching after a field is added reads here as `torn`,
# which is the right answer - a reader that trusted the layout instead would
# print the wrong number with full confidence.
#
# It reads a *page*, not a record: slots fill from the bottom, so the newest
# record is the valid one in the highest slot and every valid slot is printed
# oldest first. More than one valid slot is not an error - it is a board that
# reached ten seconds more than once without being reflashed, which is how a
# board that reset itself is told from one that did not.

import struct
import sys

MAGIC = 0x52544B41  # "AKTR"
WORDS = 16
SLOT_BYTES = WORDS * 4


def fnv1a(words):
    """The same FNV-1a the firmware writes, over the fifteen words before it."""
    h = 0x811C9DC5
    for w in words[:WORDS - 1]:
        h = ((h ^ w) * 16777619) & 0xFFFFFFFF
    return h


def pllcfgr(v):
    return "PLLM=%d PLLN=%d PLLP=%d PLLSRC=%s PLLQ=%d" % (
        v & 0x3F, (v >> 6) & 0x1FF, ((v >> 16) & 0x3) * 2 + 2,
        "HSE" if (v >> 22) & 1 else "HSI", (v >> 24) & 0xF)


def dsts(v):
    speeds = {0: "HS", 1: "FS(30/60MHz)", 2: "LS", 3: "FS(48MHz)"}
    return "ENUMSPD=%s SUSSTS=%d EERR=%d FNSOF=%d" % (
        speeds.get((v >> 1) & 3, "?"), v & 1, (v >> 3) & 1, (v >> 8) & 0x3FFF)


def dctl(v):
    named = [(0, "RWUSIG"), (1, "SDIS"), (2, "GNPINNAK"), (3, "GONSTS"),
             (4, "TCTL"), (9, "POPRST")]
    on = [n for b, n in named if v & (1 << b)]
    return " ".join(on) if on else "all clear"


GINTSTS = [(0, "CMOD"), (1, "MMIS"), (2, "OTGINT"), (3, "SOF"), (4, "RXFLVL"),
           (5, "NPTXFE"), (6, "GINNONPMT"), (7, "GOUTNPMT"), (10, "ERLYSUSP"),
           (11, "USBSUSP"), (12, "USBRST"), (13, "ENUMDNE"), (14, "ISOOUTDROP"),
           (15, "EOPF"), (17, "IEPINT"), (18, "OEPINT"), (19, "IISOIXFR"),
           (20, "IPXFR"), (21, "IISOODR"), (22, "EONUMF"), (23, "INCOMPISOIN"),
           (24, "INCOMPISOOUT"), (26, "WkUpInt"), (28, "SRQINT"),
           (29, "DCONNINT"), (30, "SESSREQINT")]


def gintsts(v):
    return " ".join(n for b, n in GINTSTS if v & (1 << b)) or "none"


def main(argv):
    if len(argv) != 2:
        sys.stderr.write("usage: trace-page.py <page.bin>\n")
        return 2

    page = open(argv[1], "rb").read()
    nslots = len(page) // SLOT_BYTES
    valid = 0

    for slot in range(nslots):
        w = list(struct.unpack_from("<%dI" % WORDS, page, slot * SLOT_BYTES))
        if all(x == 0xFFFFFFFF for x in w):
            continue  # blank: a slot this board has not reached
        if w[0] != MAGIC:
            print("slot %d: not a record (magic %08x) - this page holds "
                  "something else" % (slot, w[0]))
            continue
        if w[WORDS - 1] != fnv1a(w):
            print("slot %d: torn - sum %08x, recomputed %08x. The layout read "
                  "here and the layout written disagree; do not trust any field "
                  "below the magic" % (slot, w[WORDS - 1], fnv1a(w)))
            continue

        valid += 1
        print("slot %d  sysclk %u Hz  resets %u  setups %u  sends %u" % (
            w[1], w[2], w[3], w[4], w[5]))
        print("        PLLCFGR %08x  %s" % (w[6], pllcfgr(w[6])))
        print("        DSTS    %08x  %s" % (w[7], dsts(w[7])))
        print("        DCTL    %08x  %s" % (w[8], dctl(w[8])))
        print("        GINTSTS|%08x  %s" % (w[9], gintsts(w[9])))
        print("        passes %u  rx %u = setup %u + data %u + other %u" % (
            w[10], w[11], w[12], w[13], w[14]))

    if valid == 0:
        print("no valid record in %d slots." % nslots)
        print("An all-blank page is a save that never happened, which means the "
              "loop did not reach ten seconds - not that the numbers were zero.")
        return 1
    if valid > 1:
        print("%d valid records: the last is the newest, and the ones before it "
              "are boots this board made without being reflashed." % valid)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
