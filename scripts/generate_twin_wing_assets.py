#!/usr/bin/env python3
"""Generate the Gazebo twin-wing model from the shared airframe YAML.

    python3 scripts/generate_twin_wing_assets.py            # write files
    python3 scripts/generate_twin_wing_assets.py --check    # fail on drift
    python3 scripts/generate_twin_wing_assets.py --mjcf out.xml  # dump MJCF

The MuJoCo model needs no generated files: the backend builds its MJCF from
the same YAML at runtime. ``--mjcf`` writes that text out for inspection or
for ``mujoco.viewer`` / ``simulate``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from aerial_kit.airframes.twin_wing import load_twin_wing_spec  # noqa: E402
from aerial_kit.airframes.twin_wing.sdf import MESH_DIR, twin_wing_gazebo_files  # noqa: E402

DEFAULT_GAZEBO_DIR = REPO / "ros2_ws" / "src" / "sim_gazebo" / "models" / "twin_wing_visual"


def check(out_dir: Path, files: dict[str, str]) -> list[str]:
    problems = []
    for rel, text in files.items():
        path = out_dir / rel
        if not path.is_file():
            problems.append(f"missing: {path}")
        elif path.read_text(encoding="utf-8") != text:
            problems.append(f"differs from the YAML: {path}")
    mesh_dir = out_dir / MESH_DIR
    if mesh_dir.is_dir():
        wanted = {Path(rel).name for rel in files if rel.startswith(MESH_DIR + "/")}
        for path in sorted(mesh_dir.glob("*.obj")):
            if path.name not in wanted:
                problems.append(f"stale mesh not produced by the YAML: {path}")
    return problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--spec", type=Path, default=None, help="airframe YAML (default: bundled)")
    parser.add_argument("--out", type=Path, default=DEFAULT_GAZEBO_DIR)
    parser.add_argument("--check", action="store_true", help="verify instead of writing")
    parser.add_argument("--mjcf", type=Path, default=None, help="also write the MJCF here")
    args = parser.parse_args()

    spec = load_twin_wing_spec(args.spec)
    files = twin_wing_gazebo_files(spec)
    if args.check:
        problems = check(args.out, files)
        for line in problems:
            print(line, file=sys.stderr)
        if problems:
            print("run scripts/generate_twin_wing_assets.py to regenerate", file=sys.stderr)
            return 1
        print(f"{args.out}: {len(files)} files match {spec.source}")
        return 0

    (args.out / MESH_DIR).mkdir(parents=True, exist_ok=True)
    wanted = {Path(rel).name for rel in files if rel.startswith(MESH_DIR + "/")}
    for path in (args.out / MESH_DIR).glob("*.obj"):
        if path.name not in wanted:
            path.unlink()
    for rel, text in files.items():
        (args.out / rel).write_text(text, encoding="utf-8")
    print(f"wrote {len(files)} files to {args.out}")
    if args.mjcf is not None:
        from aerial_kit.airframes.twin_wing.mjcf import twin_wing_mjcf

        args.mjcf.write_text(twin_wing_mjcf(spec), encoding="utf-8")
        print(f"wrote {args.mjcf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
