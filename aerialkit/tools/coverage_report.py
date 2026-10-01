#!/usr/bin/env python3
"""Which lines of the firmware this machine can reach, and which it never does.

    make coverage            # builds with --coverage, runs everything, reports
    tools/coverage_report.py build-cov [where the sources are]

The second argument defaults to the current directory, which is right when this
is run from the repository; naming it is how the report can be run from
anywhere, and gcov is run from a scratch directory either way so that nothing
it writes lands in the tree.

The host suite, the sessions and the fuzzer between them run a *lot* of the
firmware, and nothing had ever said which parts. This walks the `.gcno`/`.gcda`
files GCC leaves in the build directory, asks `gcov` for each source, and prints
the result three ways: by directory, worst file first, and unattributed.

What it is not: a target for a number to be hit. The parts it cannot reach are
the port registers (a mapped page is not a chip), the radios, and anything that
needs the part to act on a write - and a firmware with no unreached lines is
usually a firmware whose tests are describing themselves. The report is a map of
where a test would be worth writing.
"""

import collections
import pathlib
import subprocess
import sys
import tempfile


def sources(build_dir, src_root):
    """`source -> the directory of its object`, from every .gcno under a build.

    The keys are the tree-relative paths the report prints; the lookup is
    against `src_root`, which is where the sources actually are. The two are
    the same thing when this is run from the repository, and they are not when
    it is run from somewhere else - which is the point: `gcov` writes an
    annotated copy of every file it is asked about into the *current*
    directory, and running this from the repository root used to leave a
    `.gcov` per source sitting in the tree (04-traps.md §23, on the NAS).
    """
    found = {}
    root = pathlib.Path(build_dir).resolve()
    for gcno in sorted(root.rglob("*.gcno")):
        # The build directory mirrors the tree, so the source is the object's
        # own relative path with the suffix changed - not the bare file name,
        # which is what a first draft of this used and found nothing.
        relative = gcno.relative_to(root).parent
        for suffix in (".c", ".cpp"):
            name = relative / (gcno.stem + suffix)
            candidate = src_root / name
            if candidate.exists():
                found[str(name)] = str(gcno.parent)
                break
    return found


def unattributed(build_dir, src_root, found):
    """The .gcno files no source could be found for, so nothing is quiet.

    A source compiled more than once cannot be keyed by its object's path, and
    this build does it twice: everything under `pic/` - the copies the ABI
    library `libaerialkit-control.so` is linked from, which is what
    tools/akcontrol_check.py exists to drive - and the ESP32 ADC, whose second
    object is built from adc.c under renamed symbols and lands as
    adc_curve.gcno. Every one of those is left out of the map, and a map that
    silently leaves out a fifth of the objects reads as more complete than it
    is.
    """
    root = pathlib.Path(build_dir).resolve()
    out = []
    for gcno in sorted(root.rglob("*.gcno")):
        relative = gcno.relative_to(root).parent
        if all(str(relative / (gcno.stem + suffix)) not in found
               for suffix in (".c", ".cpp")):
            out.append(str(gcno.relative_to(root)))
    return out


# Every gcov complaint, kept rather than discarded. `measure` used to read
# only the subprocess's stdout, so a run whose profile data was missing, whose
# stamp did not match, or whose source could not be opened published a number
# with no sign that anything had been wrong.
WARNINGS = []


def note(source, stderr):
    """Keep gcov's diagnostics for a source, so the run can report them."""
    for line in stderr.splitlines():
        if line.strip():
            WARNINGS.append((source, line.strip()))


def cell(scratch, src_root):
    """A directory where the sources' relative paths resolve.

    The .gcno records each source path *relative to the directory it was
    compiled in*, so gcov only annotates a file when it is run somewhere that
    relative path exists. Run it anywhere else and it says "cannot open source
    file" and writes an annotated copy holding nothing but the header - whose
    only "line" is line 0, so a count taken from it is silently zero. Symlinks
    to each top-level directory are enough, and the annotated copies land in
    the cell rather than in the tree.
    """
    path = pathlib.Path(scratch) / "cell"
    path.mkdir()
    for entry in sorted(src_root.iterdir()):
        if entry.is_dir():
            (path / entry.name).symlink_to(entry)
    return path


def measure(source, objdir, src_root, scratch):
    """(executed, total) executable lines for one source, counted exactly.

    Not from gcov's summary line. That line is `Lines executed:%.2f%% of N`,
    and recovering a count from it - `int(round(percent * count / 100.0))` -
    is lossy, because the percentage is rounded to two decimals. Measured
    against the annotated copy on this tree the two agree for all 138 files
    (23047 of 23798 either way), so the reconstruction is not currently wrong;
    it is only fragile, and a count read from the per-line data cannot be.
    """
    proc = subprocess.run(["gcov", "-o", objdir, source],
                          capture_output=True, text=True, cwd=scratch)
    note(source, proc.stderr)
    annotated = pathlib.Path(scratch) / (pathlib.Path(source).name + ".gcov")
    if not annotated.exists():
        return 0, 0

    executed = total = 0
    for row in annotated.read_text(errors="replace").splitlines():
        # count:lineno:source, so the line number is what comes before the
        # second colon and the source text after it may contain colons itself.
        head, _, rest = row.partition(":")
        if not rest.partition(":")[0].strip().isdigit():
            continue
        field = head.strip()
        if field == "-":
            continue
        total += 1
        # gcov marks a line whose count is non-zero but which has blocks that
        # never ran with a trailing '*': `2*`. It is an executed line - gcov's
        # own summary counts it - and reading it as a digit run would lose it.
        digits = field.rstrip("*")
        if digits.isdigit() and int(digits) > 0:
            executed += 1
    return executed, total


def group_of(source):
    parts = pathlib.Path(source).parts
    if len(parts) >= 3 and parts[0] == "src":
        return "/".join(parts[:3]) if parts[1] == "core" else "/".join(parts[:2])
    return "other"


def main():
    build_dir = sys.argv[1] if len(sys.argv) > 1 else "build-cov"
    src_root = pathlib.Path(sys.argv[2] if len(sys.argv) > 2 else ".").resolve()
    found = sources(build_dir, src_root)
    if not found:
        print("coverage: no .gcno files under %s - build with --coverage"
              % build_dir)
        return 1

    rows = []
    harness = []
    with tempfile.TemporaryDirectory(prefix="ak-coverage-") as scratch:
        scratch = cell(scratch, src_root)
        for source, objdir in sorted(found.items()):
            executed, total = measure(source, objdir, src_root, scratch)
            if total:
                (rows if source.startswith("src/") else harness).append(
                    (source, executed, total))

    by_dir = collections.defaultdict(lambda: [0, 0])
    for source, executed, total in rows:
        slot = by_dir[group_of(source)]
        slot[0] += executed
        slot[1] += total

    ran = sum(r[1] for r in rows)
    all_lines = sum(r[2] for r in rows)
    h_ran = sum(r[1] for r in harness)
    h_lines = sum(r[2] for r in harness)

    print("coverage: the firmware, %d files, %d of %d lines executed (%.1f%%)"
          % (len(rows), ran, all_lines, 100.0 * ran / all_lines))
    lost = unattributed(build_dir, src_root, found)
    if lost:
        print("          (%d of %d objects are not in this map: a source "
              "compiled more than once cannot be keyed by its object's path)"
              % (len(lost), len(lost) + len(found)))
    if WARNINGS:
        print()
        print("  gcov reported %d problem(s); this number is suspect"
              % len(WARNINGS))
        for source, line in WARNINGS[:10]:
            print("    %-44s %s" % (source, line))
    if h_lines:
        print("          (and the harness beside it: %d of %d lines, %.1f%%)"
              % (h_ran, h_lines, 100.0 * h_ran / h_lines))
    print()
    print("  by directory")
    for name in sorted(by_dir):
        executed, total = by_dir[name]
        print("    %-28s %5d / %5d  %5.1f%%"
              % (name, executed, total, 100.0 * executed / total))

    print()
    print("  the files with the most lines nothing reaches")
    worst = sorted(rows, key=lambda r: (r[2] - r[1]), reverse=True)[:15]
    for source, executed, total in worst:
        missed = total - executed
        if missed == 0:
            continue
        print("    %-48s %4d of %4d never run" % (source, missed, total))
    return 0


if __name__ == "__main__":
    sys.exit(main())
