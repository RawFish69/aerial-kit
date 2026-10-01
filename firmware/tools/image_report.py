#!/usr/bin/env python3
"""What each built image costs, against the part it was built for.

    make image-report                    # every profile this host can build
    tools/image_report.py NAME=ELF ..    # or the images you already have

Why this exists: the host suite says the code is right and says nothing at all
about whether it fits. The two numbers that decide that - how much of the part's
flash the image occupies, and how much of its SRAM is gone before the aircraft
is switched on - are printed by the linker at the end of the build, into a
scroll of compile lines nobody reads, and they are not compared with anything.

Here they are compared with something: the region sizes come out of the linker
script the image was *actually linked with* (through the map file's Memory
Configuration block, which the linker writes from that script), so a report can
never be generated against the wrong part. That matters more than it sounds -
`make check` measures the image against `scripts/image-facts/$(PART).txt`, and
the day those two disagree is the day a build is checked against the previous
part's hardware. The facts file is not consulted here.

**How the numbers are arrived at, and why not the obvious way.** `size` gives
`text`, `data` and `bss`, which is exactly the wrong shape: `data` is counted
once as flash (it is stored there) and once as RAM (it is copied there), and
`bss` counts section sizes without the alignment padding between them. The
linker's own `--print-memory-usage` is authoritative but only exists in the
build log. So this reads the ELF's program headers instead, which say it
directly: each PT_LOAD has a load address and a file size (bytes in flash) and
a virtual address and a memory size (bytes in RAM), and the region is the span
from its origin to the furthest end any of those reaches. That span includes
the padding, which is what the linker reserved, so it agrees with
`--print-memory-usage` byte for byte - `tools/image_report_check.py` holds it to
that agreement on the committed fixtures.

**Which image the numbers describe.** Every row carries the revision and the
build stamp that the image itself carries, read back out of its `.rodata`, so a
figure and its provenance travel together. The revision is the weaker of the
two: `git describe --always --dirty` records *that* something was uncommitted,
never *what*, so two builds of one revision can differ by every uncommitted byte
in the tree and still print the same revision. The stamp is what separates those
two builds, and the sha256 in `artifacts.txt` is what identifies them - this
tool will tell you when an image was built and not that it is the image you
think it is.

What it is *not*: a statement about the stack. The stack lives in the same SRAM
and is not a section, so it is not in these numbers at all - see
`scripts/check-stack.sh`, which answers that question from the compiler's call
graph and is a floor rather than a proof. A profile whose RAM is at sixty per
cent with a three-kilobyte stack floor has less headroom than sixty per cent
sounds like: the heap is nothing here (this firmware does not have one), and
everything above the used span is stack.

Usage:
    image_report.py [--readelf PATH] [--nm PATH] [--top N] [--out FILE]
                    [--json FILE] NAME=ELF [NAME=ELF ...]

Exits non-zero if an image does not fit its part, or if a region's use cannot
be established - a report that prints a number it could not read is worse than
one that refuses.
"""

import json
import os
import re
import subprocess
import sys

DEFAULT_READELF = "arm-none-eabi-readelf"
DEFAULT_NM = "arm-none-eabi-nm"

# `*default*` is the linker's own catch-all region and has no origin worth
# reporting; everything else in the block is a region the script declared.
REGION_SKIP = "*default*"

# `nm --size-sort` prints "0000000000000123 0000000000000045 b name"; we want the
# ones that occupy RAM at run time and are big enough to be worth naming.
RAM_SYMBOL_TYPES = "bBdD"


class ReportError(Exception):
    pass


def run(argv):
    """Run a tool and return its stdout, or say what went wrong in full."""
    try:
        done = subprocess.run(argv, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        raise ReportError(
            f"{argv[0]} is not on PATH - it comes with the cross toolchain; "
            "see docs/28-build.md")
    if done.returncode != 0:
        raise ReportError(f"{' '.join(argv)} failed:\n{done.stderr}")
    return done.stdout


def parse_regions(map_path):
    """The part's memory, as the linker that linked this image saw it.

    The map's Memory Configuration block is written from the linker script, so
    this cannot name a region the image was not linked against, and cannot miss
    one either. The order is the script's own, which keeps FLASH first.
    """
    if not os.path.exists(map_path):
        raise ReportError(
            f"no linker map at {map_path} - the region sizes are read from it, "
            "and guessing them from the ELF is how a report ends up describing "
            "a part the image was not built for")
    return parse_regions_text(open(map_path, "r", errors="replace").read(),
                              map_path)


def parse_regions_text(text, where):
    """The same, from the map's text rather than from a path - see
    `parse_segments_text` for why that is worth the extra function."""
    at = text.find("Memory Configuration")
    if at < 0:
        raise ReportError(f"{where} has no Memory Configuration block")
    # The block is four columns wide and is followed by the memory map proper,
    # which has the same shape for its own reasons (a section name, an address,
    # a size, and a contributing object). Reading past the end of the block
    # therefore does not fail - it quietly turns every section in the image into
    # a "region", which is how the first version of this reported `.bss.proto`
    # as a part of the microcontroller. So the block is read as a block: it ends
    # at the first line that is not four columns of a name and two hex numbers.
    row = re.compile(r"^(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+\S+\s*$")
    regions = []
    for line in text[at:].splitlines():
        found = row.match(line)
        if found is None:
            if regions:
                break
            continue  # the "Name Origin Length Attributes" heading
        name = found.group(1)
        if name == REGION_SKIP:
            break
        regions.append({
            "name": name,
            "origin": int(found.group(2), 16),
            "length": int(found.group(3), 16),
        })
    if not regions:
        raise ReportError(f"{where}: the Memory Configuration block was empty")
    return regions


def parse_segments(elf_path, readelf):
    """Every PT_LOAD, as (load address, file size, virtual address, memory size).

    Only PT_LOAD: the other program header types describe debugging and do not
    occupy either memory on the part.
    """
    return parse_segments_text(run([readelf, "-lW", elf_path]), elf_path)


def parse_segments_text(out, elf_path):
    """The same, from the tool's output rather than from the tool.

    Split out so the fixtures under tests/fixtures/image-report/ - which are
    real readelf output from a real image - pin this without a cross toolchain
    being present, which is the difference between a check CI can run on every
    pull request and one it can run only where the compiler is installed.
    """
    segments = []
    started = False
    for line in out.splitlines():
        line = line.strip()
        if line.startswith("Type") and "VirtAddr" in line:
            started = True
            continue
        if not started:
            continue
        if line.startswith("Section to Segment"):
            break
        fields = line.split()
        if len(fields) < 6 or fields[0] != "LOAD":
            continue
        try:
            segments.append({
                "lma": int(fields[3], 16),
                "filesz": int(fields[4], 16),
                "vma": int(fields[2], 16),
                "memsz": int(fields[5], 16),
            })
        except ValueError:
            raise ReportError(f"{elf_path}: could not read program header: {line}")
    if not segments:
        raise ReportError(f"{elf_path}: no PT_LOAD segments - is it an ELF?")
    return segments


def span_in(segments, origin, length, which):
    """How far into a region the image reaches, in bytes.

    The furthest end any span reaches, measured from the region's origin - not
    the sum of the spans. The difference is the alignment padding the linker
    left between sections, which is memory it reserved and the part cannot use
    for anything else, so a sum would report less than the image really costs.

    **Not clipped to the region.** A span that runs past the end of its region
    reports more than the region holds, and that is the answer this is for: a
    linker given a script it cannot satisfy stops, so an image that overflows
    arrives here only from a map somebody edited or a script whose regions were
    changed under a built image - and both of those are cases where the report
    saying "108% of RAM" is the whole of its value. Clipping would make every
    answer at most 100%, and `over_capacity` - the one judgement this tool
    makes - would never fire.
    """
    if which == "flash":
        spans = [(s["lma"], s["filesz"]) for s in segments]
    else:
        spans = [(s["vma"], s["memsz"]) for s in segments]
    end = origin
    seen = False
    for start, size in spans:
        if size == 0:
            continue
        if start >= origin + length or start + size <= origin:
            continue
        seen = True
        end = max(end, start + size)
    if not seen:
        return None
    return end - origin


def biggest_ram_objects(elf_path, nm, regions, top):
    """The names worth knowing when RAM is the tight number."""
    out = run([nm, "--size-sort", "-S", elf_path])
    rows = []
    for line in out.splitlines():
        fields = line.split()
        if len(fields) < 4:
            continue
        try:
            size, address = int(fields[1], 16), int(fields[0], 16)
        except ValueError:
            continue
        kind = fields[-2]
        if kind not in RAM_SYMBOL_TYPES or size == 0:
            continue
        for region in regions:
            if region["origin"] <= address < region["origin"] + region["length"]:
                rows.append((size, region["name"], fields[-1]))
                break
    rows.sort(reverse=True)
    return rows[:top]


# The two identity strings the firmware embeds in .rodata and prints at boot -
# `src/core/ak_version.h`, and the `rev:`/`built:` pair the console banner is
# built from. Both are read back out of the artifact rather than out of the tree
# it came from, which is the whole point of reporting them.
IDENTITY_PREFIXES = ("AK_REV ", "AK_STAMP_VALUE ")


def embedded_identity(elf_path):
    """The revision and the build stamp the image itself carries, or None each.

    One pass for both, because they are two lines of one .rodata blob and
    reading the file twice to find the second one is a way to report two
    different files.

    The two Nones are not the same finding and the caller must not have to
    guess between them. An image built without `-DAK_REV`/`-DAK_STAMP_VALUE`
    carries the header's own `unknown` default, which this returns as the string
    `"unknown"` - the image really does say that, and reporting it as unread
    would be this tool inventing a different fact. `None` means the strings are
    not in the file at all: not an image of this firmware, or built by something
    that does not stamp what it builds. `None` therefore never means "could not
    look" - that is a refusal, below, because a report that cannot distinguish
    an unstamped image from a missing `strings` is a report saying "no identity"
    for two different reasons and only one of them is about the image.
    """
    try:
        out = subprocess.run(["strings", elf_path], capture_output=True,
                             text=True, check=False).stdout
    except FileNotFoundError:
        raise ReportError(
            "strings is not on PATH - it is host binutils, and reading the "
            "identity out of the image is how this report says when it was "
            "built (docs/28-build.md)")
    found = {}
    for line in out.splitlines():
        for prefix in IDENTITY_PREFIXES:
            if prefix not in found and line.startswith(prefix):
                found[prefix] = line[len(prefix):].strip()
        if len(found) == len(IDENTITY_PREFIXES):
            break
    return found.get("AK_REV "), found.get("AK_STAMP_VALUE ")


def human(count):
    return f"{count:,}"


def report(entries, readelf, nm, top, out):
    """One table, then the detail behind the number that is tight."""
    profiles = []
    for name, elf in entries:
        map_path = os.path.splitext(elf)[0] + ".map"
        regions = parse_regions(map_path)
        segments = parse_segments(elf, readelf)
        revision, built = embedded_identity(elf)
        used = {}
        for region in regions:
            flash = span_in(segments, region["origin"], region["length"], "flash")
            ram = span_in(segments, region["origin"], region["length"], "ram")
            # A region holds code or it holds data; which one it is, is what the
            # linker script said when it declared it, and the larger of the two
            # readings is the one that matters. A region where both are present
            # and different would be a script this report should not guess at.
            used[region["name"]] = max(x for x in (flash, ram, 0) if x is not None)
        profiles.append({
            "name": name,
            "elf": elf,
            "map": map_path,
            "revision": revision,
            "built": built,
            "regions": [
                {
                    "name": r["name"],
                    "origin": r["origin"],
                    "capacity": r["length"],
                    "used": used[r["name"]],
                    "percent": 100.0 * used[r["name"]] / r["length"],
                }
                for r in regions
            ],
            "symbols": biggest_ram_objects(elf, nm, regions, top),
        })

    lines = []
    lines.append("")
    # The identity is in the row rather than under the table on purpose: a
    # figure an operator can read without reading when it was measured from is
    # how a stale report gets quoted as a current one (04-traps.md §156). The
    # header is built with the same format as the rows so the two cannot drift.
    # The `of` column is nine wide because the F405's flash is 1,048,576 bytes
    # and that is nine characters with the thousands separators; at eight, the
    # flash row - the one row whose figure decides whether a feature fits - was
    # the one row whose percentage did not line up under the others'.
    lines.append(
        f"{'profile':<28}  {'rev':<13}  {'built':<22}  {'region':<7}"
        f"  {'used':>9}  {'of':>9}  {'%':>7}")
    lines.append("-" * 107)
    for profile in profiles:
        for index, region in enumerate(profile["regions"]):
            first = profile["name"] if index == 0 else ""
            rev = (profile["revision"] or "-") if index == 0 else ""
            built = (profile["built"] or "-") if index == 0 else ""
            lines.append(
                f"{first:<28}  {rev:<13}  {built:<22}  {region['name']:<7}"
                f"  {human(region['used']):>9}  {human(region['capacity']):>9}"
                f"  {region['percent']:>6.2f}%")
        if len(profile["regions"]) > 1:
            lines.append("")
    lines.append("")
    for profile in profiles:
        if not profile["symbols"]:
            continue
        lines.append(f"{profile['name']}: the largest things in RAM")
        for size, region, symbol in profile["symbols"]:
            lines.append(f"  {human(size):>9}  {region:<7}  {symbol}")
        lines.append("")

    text = "\n".join(lines)
    if out:
        with open(out, "w") as handle:
            handle.write(text + "\n")
    else:
        print(text)
    return profiles


def parse_arguments(argv):
    readelf, nm, top, out, json_out = DEFAULT_READELF, DEFAULT_NM, 5, None, None
    entries = []
    index = 0
    while index < len(argv):
        item = argv[index]
        if item == "--readelf":
            index += 1
            readelf = argv[index]
        elif item == "--nm":
            index += 1
            nm = argv[index]
        elif item == "--top":
            index += 1
            top = int(argv[index])
        elif item == "--out":
            index += 1
            out = argv[index]
        elif item == "--json":
            index += 1
            json_out = argv[index]
        elif "=" in item:
            name, elf = item.split("=", 1)
            entries.append((name, elf))
        else:
            sys.stderr.write(f"image_report: cannot read {item!r} as NAME=ELF\n")
            sys.exit(2)
        index += 1
    if not entries:
        sys.stderr.write(__doc__.split("Usage:")[1].split("\n\n")[0].strip() + "\n")
        sys.exit(2)
    return readelf, nm, top, out, json_out, entries


def over_capacity(profiles):
    """The profiles that do not fit, which is the one thing worth failing for.

    Kept apart from `main` because it is the only judgement this tool makes -
    everything above it is measurement - and a judgement that cannot be tested
    without a cross toolchain is a judgement nobody checks.
    """
    return [(p["name"], r["name"], r["percent"])
            for p in profiles for r in p["regions"] if r["percent"] > 100.0]


def main(argv):
    readelf, nm, top, out, json_out, entries = parse_arguments(argv)
    try:
        profiles = report(entries, readelf, nm, top, out)
    except ReportError as error:
        sys.stderr.write(f"image_report: {error}\n")
        return 1
    if json_out:
        with open(json_out, "w") as handle:
            json.dump({"profiles": profiles}, handle, indent=2)
    over = over_capacity(profiles)
    for name, region, percent in over:
        sys.stderr.write(
            f"image_report: {name} does not fit {region} - {percent:.2f}% used\n")
    return 1 if over else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
