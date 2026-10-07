#!/usr/bin/env python3
"""Check public firmware scope and real source bytes; no Linguist overrides.

Run from any directory. Counts tracked and nonignored untracked files, including
legacy sources, once each; ignored build outputs/dependencies are excluded by
Git. Headers with .h are conservatively counted as C. This is an explicit
extension-based measurement, not a prediction of GitHub Linguist percentages.
"""
from collections import Counter
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
LANGUAGES = {
    ".py": "Python", ".c": "C", ".h": "C",
    ".cpp": "C++", ".cc": "C++", ".cxx": "C++", ".hpp": "C++",
    ".ts": "TypeScript", ".tsx": "TypeScript",
    ".js": "JavaScript", ".jsx": "JavaScript", ".mjs": "JavaScript",
    ".sh": "Shell", ".rs": "Rust", ".go": "Go", ".S": "Assembly",
}
ALLOWED_RUNTIME = (
    "src/core/", "src/arch/arm/cortex-m4/", "src/arch/stm32f405/",
    "src/arch/esp32/", "src/boards/FEATHER_F405/", "src/boards/ESP32DEV/",
)


def main():
    names = set(subprocess.check_output(
        ["git", "-C", str(ROOT), "ls-files", "-co", "--exclude-standard", "-z"]
    ).decode().split("\0"))
    totals = Counter()
    extensions = Counter()
    failures = []
    for name in sorted(names):
        path = ROOT / name
        if not path.is_file():
            continue
        if path.suffix in LANGUAGES:
            size = path.stat().st_size
            totals[LANGUAGES[path.suffix]] += size
            extensions[path.suffix] += size
        if not name.startswith("firmware/") or name.startswith("firmware/legacy/"):
            continue
        rel = name[len("firmware/"):]
        if rel.startswith("src/") and not rel.startswith(ALLOWED_RUNTIME):
            failures.append("runtime outside public scope: " + name)
        if (rel.startswith(("tests/", "tools/")) and
                path.suffix in {".c", ".h", ".cpp"}):
            failures.append("C host suite/simulator outside public scope: " + name)
        if rel.startswith("linker/") and rel != "linker/stm32f405rg.ld":
            failures.append("linker outside public scope: " + name)
        if re.search(r"sdkconfig\.(c3|s2|s3)$", rel):
            failures.append("ESP32 variant outside public scope: " + name)

    firmware = ROOT / "firmware"
    boards = {p.name for p in (firmware / "src/boards").iterdir() if p.is_dir()}
    if boards != {"FEATHER_F405", "ESP32DEV"}:
        failures.append("board directories must be exactly FEATHER_F405 and ESP32DEV")
    cmake = (firmware / "ports/esp32/main/CMakeLists.txt").read_text()
    listed = set(re.findall(r'"\$\{AK_ROOT\}/(src/core/[^"]+\.c)"', cmake))
    expected = {p.relative_to(firmware).as_posix()
                for p in (firmware / "src/core").rglob("*.c")
                if p.relative_to(firmware).as_posix() != "src/core/time.c"}
    for name in sorted(expected - listed):
        failures.append("missing ESP32 core source: " + name)
    for name in sorted(listed - expected):
        failures.append("invalid ESP32 core source: " + name)
    if 'NOT AK_BOARD STREQUAL "ESP32DEV"' not in cmake:
        failures.append("ESP32DEV guard missing")
    make = (firmware / "Makefile").read_text()
    for variable, value in [("BOARD", "FEATHER_F405"), ("ARCH", "stm32f405"),
                            ("PART", "stm32f405rg")]:
        if "ifneq ($(" + variable + ")," + value + ")" not in make:
            failures.append(variable + " Makefile guard missing")
    if totals["Python"] <= max((v for k, v in totals.items() if k != "Python"),
                              default=0):
        failures.append("Python is no longer the largest source language")
    print("Source bytes (tracked + nonignored untracked, no generated builds):")
    for language, size in totals.most_common():
        print(f"  {language}: {size:,}")
    print("Extensions: " + ", ".join(f"{k}={v}" for k, v in sorted(extensions.items())))
    for failure in failures:
        print("FAIL: " + failure)
    print(f"Scope/source-size check: {len(failures)} failure(s)")
    return bool(failures)


if __name__ == "__main__":
    sys.exit(main())
