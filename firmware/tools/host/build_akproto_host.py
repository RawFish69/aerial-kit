#!/usr/bin/env python3
"""Build the read-only AKP host harness from firmware core sources."""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path

HERE = Path(__file__).resolve().parent
FIRMWARE = HERE.parents[1]
CORE = FIRMWARE / "src" / "core"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=HERE / "build" / "aerialkit-akproto-host",
    )
    args = parser.parse_args()
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        raise SystemExit("no C compiler found")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    command = [
        compiler,
        "-std=c11",
        "-O2",
        "-Wall",
        "-Wextra",
        "-DAK_PRODUCT=aerialkit-akp-host",
        f"-I{CORE}",
        f"-I{CORE / 'flight'}",
        str(CORE / "ak_proto.c"),
        str(CORE / "ak_params.c"),
        str(CORE / "ak_text.c"),
        str(HERE / "akproto_host.c"),
        "-o",
        str(args.output),
    ]
    subprocess.run(command, check=True)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
