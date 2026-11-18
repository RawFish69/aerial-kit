#!/usr/bin/env python3
"""Build the AK flight core as a host shared library for software-in-the-loop.

Compiles the firmware's own, unmodified sources (``firmware/src/core/flight``,
``filter`` and ``sensors``) plus ``akfc_shim.c`` into
``sim_py/fc_sil/build/libakfc.so``. Rebuilds only when a source is newer
than the library. Needs a C compiler (``cc``/``gcc``) and libm.

    python3 sim_py/fc_sil/build.py [--force]
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CORE = REPO / "firmware" / "src" / "core"
BUILD = HERE / "build"
LIB = BUILD / "libakfc.so"
SOURCE_DIRS = ("flight", "filter", "sensors")
CORE_HELPERS = ("ak_params.c", "ak_ring.c", "ak_text.c")
CFLAGS = ["-std=c11", "-O2", "-fPIC", "-Wall", "-Wno-unused-function"]


def sources() -> list[Path]:
    files = [HERE / "akfc_shim.c"]
    for d in SOURCE_DIRS:
        files += sorted((CORE / d).glob("*.c"))
    # Board-independent helpers the flight core links against.
    files += [CORE / name for name in CORE_HELPERS]
    return files


def include_dirs() -> list[str]:
    return [f"-I{CORE}"] + [f"-I{CORE / d}" for d in SOURCE_DIRS]


def fingerprint() -> str:
    h = hashlib.sha256()
    for f in sources():
        h.update(f.read_bytes())
    for d in SOURCE_DIRS + ("",):
        for header in sorted((CORE / d).glob("*.h")):
            h.update(header.read_bytes())
    return h.hexdigest()


def build(force: bool = False) -> Path:
    cc = os.environ.get("CC") or shutil.which("cc") or shutil.which("gcc")
    if cc is None:
        raise RuntimeError("no C compiler found (install gcc, or set CC)")
    stamp = BUILD / "fingerprint.txt"
    digest = fingerprint()
    if not force and LIB.is_file() and stamp.is_file() and stamp.read_text() == digest:
        return LIB
    BUILD.mkdir(parents=True, exist_ok=True)
    cmd = [cc, *CFLAGS, "-shared", "-o", str(LIB), *include_dirs(), *map(str, sources()), "-lm"]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f"building the AK flight core failed:\n{result.stderr}")
    stamp.write_text(digest)
    return LIB


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    print(build(args.force))
    return 0


if __name__ == "__main__":
    sys.exit(main())
