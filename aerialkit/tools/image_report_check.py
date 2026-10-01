#!/usr/bin/env python3
"""tools/image_report.py, held to the linker's own arithmetic.

    make image-report-check

The tool answers one question - how much of the part does this image cost - and
it answers it by reading the ELF's program headers rather than by trusting
`size`, because `size`'s three numbers are the wrong shape for the question
(`data` is counted as flash and as RAM, and `bss` leaves out the padding the
linker reserved between sections). That is a derivation, and a derivation is
exactly the kind of thing that agrees with itself and disagrees with the
linker. So the fixtures here are real: `tests/fixtures/image-report/` holds the
Memory Configuration block and the program headers of a real F405 link, and the
numbers this file asserts - 104,500 of flash, 78,452 of RAM, nothing in CCM -
are the ones that link's own `--print-memory-usage` printed. Byte for byte, or
this check fails.

Four of the checks are about ways this file's subject has already been wrong:

* reading past the end of the Memory Configuration block and turning every
  section in the image into a "part of the microcontroller" - which it did
  first, and which ended in a division by the length of `.iplt`;
* adding the spans up instead of measuring to the furthest end, which loses the
  four bytes of alignment padding between `.data` and `.bss` and reports 78,448
  where the linker reserved 78,452;
* taking `size`'s `data` for a flash figure on its own, which double-counts it;
* reporting an image by its revision alone, which does not identify it. Two
  builds of `34bc690-dirty` and two builds of `34bc690-dirty` with every
  uncommitted byte changed print the same revision, so a report carrying only
  that field cannot be told from a stale one - which is not a hypothetical: a
  run-book sent an operator to a five-hour-old build whose revision field was
  byte-identical to the one the batch had just built (04-traps.md §156). Each
  row now carries the build stamp the image itself carries, and the check
  below is that the report of two such builds differs.

What this does not cover: whether `readelf` and `nm` are the right ones for the
profile being measured - `scripts/ci.sh` answers that by measuring five real
profiles and comparing each against the line the linker printed while building
it, which is the only place that can be answered.

The one program this needs that it cannot write for itself is `strings`, which
is host binutils and not a cross tool: the identity checks below are about what
the *image* says, so the strings have to be read out of a file rather than
handed to the reader. Everything else is a stub.
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import image_report                                             # noqa: E402

FIXTURES = os.path.join(HERE, os.pardir, "tests", "fixtures", "image-report")
# `stm32f405rg.map.txt`, not `.map`: `.gitignore` has `*.map` for the linker map
# of every build, so a fixture named `.map` is a fixture that never gets
# committed and a check that fails in a clean checkout and nowhere else. The
# extension is not worth that.
MAP = os.path.join(FIXTURES, "stm32f405rg.map.txt")
SEGMENTS = os.path.join(FIXTURES, "stm32f405rg.readelf.txt")

# What that link's own --print-memory-usage line said. Every number below is
# checked against these, not against the tool's previous answer.
LINKER_FLASH = 104500
LINKER_RAM = 78452
LINKER_CCM = 0

failures = 0


def expect(name, condition, detail=""):
    global failures
    print("  %s %s%s" % ("ok      " if condition else "FAILED  ", name, detail))
    if not condition:
        failures += 1


def regions_of(text, where="a test map"):
    return {r["name"]: r for r in image_report.parse_regions_text(text, where)}


def check_regions_from_the_fixture():
    regions = regions_of(open(MAP).read(), MAP)

    expect("the fixture's block reads as the three regions the script declares",
           list(regions) == ["FLASH", "RAM", "CCM"],
           " (%s)" % ", ".join(regions))
    expect("with the origins the linker script gives them",
           regions["FLASH"]["origin"] == 0x08000000 and
           regions["RAM"]["origin"] == 0x20000000 and
           regions["CCM"]["origin"] == 0x10000000)
    expect("and the lengths",
           regions["FLASH"]["length"] == 0x00100000 and
           regions["RAM"]["length"] == 0x00020000 and
           regions["CCM"]["length"] == 0x00010000)

    # The failure this file exists for: the block is followed by the memory map
    # proper, whose rows have the same four-column shape, and a parser that does
    # not stop at the end of the block turns `.iplt` into a region of length
    # zero.
    expect("and nothing from the memory map below it",
           ".iplt" not in regions and ".rel.iplt" not in regions and
           ".data" not in regions,
           " (%s)" % ", ".join(regions))


def check_the_block_ends_without_a_default_row():
    # `*default*` is this linker's last row and the loop breaks on it, which
    # would hide a parser that reads to the end of the file. A linker that does
    # not write that row must not get a different answer.
    text = ("Memory Configuration\n\n"
            "Name             Origin             Length             Attributes\n"
            "FLASH            0x08000000         0x00100000         xr\n"
            "RAM              0x20000000         0x00020000         xrw\n"
            "\n"
            "Linker script and memory map\n\n"
            ".iplt           0x08019810        0x0\n")
    regions = regions_of(text)
    expect("a block with no *default* row still ends at the block",
           list(regions) == ["FLASH", "RAM"], " (%s)" % ", ".join(regions))


def check_segments_from_the_fixture():
    segments = image_report.parse_segments_text(open(SEGMENTS).read(), SEGMENTS)
    expect("the fixture's four PT_LOAD segments are read",
           len(segments) == 4, " (%d)" % len(segments))
    expect("and the mapping table under them is not",
           all(s["filesz"] >= 0 for s in segments) and
           segments[0]["lma"] == 0x08000000 and
           segments[0]["filesz"] == 0x19810,
           " (%s)" % segments[0])
    # The second segment is .data: stored in flash at 0x08019810 and living in
    # RAM at 0x20000000. A reader that took one address for both would put
    # thirty-six bytes of RAM at the bottom of the flash.
    expect("and .data is read as a different address in each",
           segments[1]["lma"] == 0x08019810 and segments[1]["vma"] == 0x20000000,
           " (%s)" % segments[1])


def check_the_numbers_are_the_linkers():
    regions = image_report.parse_regions_text(open(MAP).read(), MAP)
    segments = image_report.parse_segments_text(open(SEGMENTS).read(), SEGMENTS)
    got = {}
    for region in regions:
        flash = image_report.span_in(segments, region["origin"],
                                     region["length"], "flash")
        ram = image_report.span_in(segments, region["origin"], region["length"],
                                   "ram")
        got[region["name"]] = max(x for x in (flash, ram, 0) if x is not None)

    expect("flash is the number the linker printed",
           got["FLASH"] == LINKER_FLASH, " (%s, want %s)" % (got["FLASH"], LINKER_FLASH))
    expect("RAM is the number the linker printed",
           got["RAM"] == LINKER_RAM, " (%s, want %s)" % (got["RAM"], LINKER_RAM))
    expect("and CCM is empty, which is what the linker printed",
           got["CCM"] == LINKER_CCM, " (%s, want %s)" % (got["CCM"], LINKER_CCM))

    # And the difference that shows why the spans are measured rather than
    # added: the sum of the memory sizes is four bytes short, because the linker
    # aligned `.bss` to four after a thirty-six byte `.data`. Those four bytes
    # are reserved and a report that leaves them out is four bytes optimistic.
    summed = sum(s["memsz"] for s in segments
                 if 0x20000000 <= s["vma"] < 0x20000000 + 0x00020000)
    expect("the padding is counted, not dropped",
           summed == LINKER_RAM - 4,
           " (sum is %s, the linker reserved %s)" % (summed, LINKER_RAM))


def check_a_profile_that_does_not_fit_fails():
    def profile(name, used, capacity):
        return {"name": name, "regions": [
            {"name": "FLASH", "used": used, "capacity": capacity,
             "percent": 100.0 * used / capacity}]}

    expect("an image inside its part is not a failure",
           image_report.over_capacity([profile("a", 900, 1000)]) == [])
    over = image_report.over_capacity([profile("a", 1001, 1000)])
    expect("an image over its part is", len(over) == 1 and over[0][0] == "a")
    # Exactly full is not over: the linker would have refused to link it.
    expect("and exactly full is not",
           image_report.over_capacity([profile("a", 1000, 1000)]) == [])


def check_the_identity_comes_out_of_the_image():
    """`embedded_identity`, on files this writes - no ELF, no toolchain.

    Four cases, and they are four different findings that the tool must not
    merge: both strings carried; one of the two; neither; and the one that
    looks like a mistake and is not - an image whose build really did say
    `unknown`, because that is what `src/core/ak_version.h` defines the macros
    to and `unknown` is therefore something an image can carry.
    """
    import shutil
    import tempfile

    work = tempfile.mkdtemp(prefix="image-report-identity.")
    try:
        def carried(name, text):
            path = os.path.join(work, name)
            with open(path, "w") as handle:
                handle.write(text)
            return image_report.embedded_identity(path)

        both = carried("both.elf",
                       "AK_REV 34bc690-dirty\n"
                       "AK_STAMP_VALUE Sep 20 2026 / 21:00:30\n")
        expect("the revision is read back out of the file",
               both[0] == "34bc690-dirty", " (%r)" % (both[0],))
        expect("and the stamp with it, spaces and slashes intact",
               both[1] == "Sep 20 2026 / 21:00:30", " (%r)" % (both[1],))

        one = carried("rev-only.elf", "AK_REV 34bc690-dirty\n")
        expect("an image carrying one of the two reads as one of the two",
               one == ("34bc690-dirty", None), " (%r)" % (one,))

        neither = carried("bare.elf", "an image with no identity in it\n")
        expect("an image carrying neither reads as neither, not as something",
               neither == (None, None), " (%r)" % (neither,))

        unknown = carried("unknown.elf",
                          "AK_REV unknown\nAK_STAMP_VALUE unknown\n")
        expect("and a build that really says 'unknown' is reported saying it",
               unknown == ("unknown", "unknown"), " (%r)" % (unknown,))
    finally:
        shutil.rmtree(work, ignore_errors=True)


def check_the_whole_command_line():
    """End to end, with a stand-in for the toolchain and the real fixtures.

    The four checks above hold the arithmetic. This holds everything wrapped
    around it - the NAME=ELF arguments, the map found from the ELF's name, the
    exit code - and it does that on a host with no cross compiler, because the
    two tools it needs are two files this writes into a temporary directory.
    A check that only runs where the toolchain is installed is a check that
    runs on the machine somebody remembered to install it on.
    """
    import shutil
    import tempfile

    work = tempfile.mkdtemp(prefix="image-report-check.")
    try:
        def stub(name, path, body):
            with open(path, "w") as handle:
                handle.write("#!/bin/sh\n" + body)
            os.chmod(path, 0o755)

        stub("readelf", os.path.join(work, "readelf"), "cat %s\n" % SEGMENTS)
        stub("nm", os.path.join(work, "nm"),
             "echo '0000000000000000 0000000000001234 b a_ring_buffer'\n"
             "echo '0000000000000000 0000000000000004 r not_in_ram'\n")

        def image(name, identity):
            """An "ELF" both stubs can read: the fixture's map, and the strings."""
            elf = os.path.join(work, name + ".elf")
            with open(elf, "w") as handle:
                handle.write(identity +
                             "not an ELF, and does not need to be one\n")
            shutil.copyfile(MAP, os.path.join(work, name + ".map"))
            return elf

        def measure(name, elf):
            """One run: the exit code, the report, and the JSON beside it."""
            out = os.path.join(work, name + ".txt")
            as_json = os.path.join(work, name + ".json")
            code = image_report.main([
                "--readelf", os.path.join(work, "readelf"),
                "--nm", os.path.join(work, "nm"),
                "--out", out, "--json", as_json,
                "%s=%s" % (name, elf),
            ])
            return (code, open(out).read(),
                    json.load(open(as_json))["profiles"][0])

        def table(text):
            """(header, rule, data rows) - the table, and nothing after it."""
            lines = text.splitlines()
            rule = [i for i, line in enumerate(lines) if line and not line.strip("-")][0]
            rows = []
            for line in lines[rule + 1:]:
                if not line.strip():
                    break
                rows.append(line)
            return lines[rule - 1], lines[rule], rows

        # One revision, two builds, and the revision is the same string in both
        # because that is what `git describe --always --dirty` is: it records
        # that something was uncommitted and never what.
        revision = "AK_REV 34bc690-dirty\n"
        built_a = "AK_STAMP_VALUE Sep 20 2026 / 08:38:52\n"
        built_b = "AK_STAMP_VALUE Sep 20 2026 / 21:00:30\n"

        code, text, profile = measure("profile", image("profile", ""))
        expect("a profile inside its part exits zero", code == 0, " (%s)" % code)
        expect("and its flash is the linker's number",
               "104,500" in text, " (%s)" % text.strip().splitlines()[3:4])
        expect("and its RAM is the linker's number", "78,452" in text)

        # Read with `.get`, so a tool too old to write these fields fails the
        # assertions below with a line saying what is missing rather than
        # raising KeyError out of the middle of the check.
        expect("the JSON the stage consumes carries both identity fields",
               "revision" in profile and "built" in profile,
               " (%s)" % ", ".join(sorted(profile)))
        # The image carries no revision and the report says so. `unknown` is a
        # string this firmware really can carry - `src/core/ak_version.h`
        # defines the macros to it - so a tool that printed it here would be
        # claiming the image said something it does not say.
        expect("an image with no identity in it reports none, and invents none",
               profile.get("revision") is None and profile.get("built") is None and
               "unknown" not in text,
               " (%r, %r)" % (profile.get("revision"), profile.get("built")))

        header, rule, rows = table(text)
        expect("the header and the rule and every row are the same width",
               len(header) == len(rule) and
               all(len(row) == len(header) for row in rows),
               " (%d, %d, %s)" % (len(header), len(rule),
                                  sorted({len(r) for r in rows})))
        expect("and the unread identity is the '-' the row actually printed",
               rows[0][header.index("built")] == "-", " (%r)" % rows[0])

        # And the two builds of that one revision, which is the check this
        # whole section exists for: a report that carried the revision alone
        # would produce two identical rows for two different images.
        code_a, text_a, profile_a = measure(
            "build-a", image("build-a", revision + built_a))
        code_b, text_b, profile_b = measure(
            "build-b", image("build-b", revision + built_b))

        expect("and both of those runs passed",
               code_a == 0 and code_b == 0, " (%s, %s)" % (code_a, code_b))
        expect("both builds carry the same revision",
               profile_a.get("revision") == profile_b.get("revision") == "34bc690-dirty",
               " (%r, %r)" % (profile_a.get("revision"), profile_b.get("revision")))
        expect("and are told apart by the stamp each one carries",
               profile_a.get("built") == "Sep 20 2026 / 08:38:52" and
               profile_b.get("built") == "Sep 20 2026 / 21:00:30",
               " (%r, %r)" % (profile_a.get("built"), profile_b.get("built")))
        # The names are blanked first, because they are different in the two
        # reports whatever else is: comparing the texts as they come would be a
        # check that passes for the name and says nothing about the stamp.
        expect("so two builds of one revision, differing only in when they "
               "were built, do not produce one report",
               text_a.replace("build-a", "one") != text_b.replace("build-b", "one"))
        expect("and each report carries its own stamp in the column named for it",
               table(text_a)[0].index("built") >= 0 and
               built_a.split(" ", 1)[1].strip() in
               table(text_a)[2][0][table(text_a)[0].index("built"):] and
               built_b.split(" ", 1)[1].strip() in
               table(text_b)[2][0][table(text_b)[0].index("built"):])
        expect("and the stamp fits between its column and the next one",
               len("Sep 20 2026 / 21:00:30") <=
               table(text_b)[0].index("region") - table(text_b)[0].index("built"))

        # A map whose RAM is a hundred bytes must make the same command fail,
        # and say which region - the one judgement this tool makes.
        small = os.path.join(work, "small.map")
        open(small, "w").write(
            "Memory Configuration\n\n"
            "Name             Origin             Length             Attributes\n"
            "FLASH            0x08000000         0x00100000         xr\n"
            "RAM              0x20000000         0x00000064         xrw\n")
        small_elf = os.path.join(work, "small.elf")
        open(small_elf, "w").write("x\n")
        code = image_report.main([
            "--readelf", os.path.join(work, "readelf"),
            "--nm", os.path.join(work, "nm"),
            "--out", os.path.join(work, "small.txt"),
            "small=%s" % small_elf,
        ])
        expect("an image over its part exits non-zero", code == 1, " (%s)" % code)
    finally:
        shutil.rmtree(work, ignore_errors=True)


def main():
    print("image report: the arithmetic, against the linker's own")
    check_regions_from_the_fixture()
    check_the_block_ends_without_a_default_row()
    check_segments_from_the_fixture()
    check_the_numbers_are_the_linkers()
    check_a_profile_that_does_not_fit_fails()
    check_the_identity_comes_out_of_the_image()
    check_the_whole_command_line()
    print("image report: %s" % ("ok" if failures == 0 else "FAILED"))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
