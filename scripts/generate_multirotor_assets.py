#!/usr/bin/env python3
"""Generate kinematic Gazebo visuals for the bundled multirotor specs.

    python3 scripts/generate_multirotor_assets.py
    python3 scripts/generate_multirotor_assets.py --airframe research_hex
    python3 scripts/generate_multirotor_assets.py --check

The generated models are static and are moved at runtime by
gazebo_multirotor_pose_node. MuJoCo remains the dynamics plant.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from aerial_kit.airframes.multirotor_specs import (  # noqa: E402
    available_airframes,
    load_multirotor_spec,
)
from aerial_kit.airframes.multirotor_specs.sdf import (  # noqa: E402
    MESH_DIR,
    multirotor_gazebo_files,
)

DEFAULT_MODELS_DIR = REPO / "ros2_ws" / "src" / "sim_gazebo" / "models"


def model_directory(models_dir: Path, airframe: str) -> Path:
    return models_dir / f"{airframe}_visual"


def check(model_dir: Path, files: dict[str, str]) -> list[str]:
    problems: list[str] = []
    for relative, expected in files.items():
        path = model_dir / relative
        if not path.is_file():
            problems.append(f"missing: {path}")
        elif path.read_text(encoding="utf-8") != expected:
            problems.append(f"differs from the spec: {path}")

    mesh_dir = model_dir / MESH_DIR
    if mesh_dir.is_dir():
        wanted = {
            Path(relative).name
            for relative in files
            if relative.startswith(f"{MESH_DIR}/")
        }
        for path in sorted(mesh_dir.iterdir()):
            if path.is_file() and path.name not in wanted:
                problems.append(f"stale mesh not produced by the spec: {path}")
    return problems


def write_model(model_dir: Path, files: dict[str, str]) -> None:
    mesh_dir = model_dir / MESH_DIR
    mesh_dir.mkdir(parents=True, exist_ok=True)
    wanted = {
        Path(relative).name
        for relative in files
        if relative.startswith(f"{MESH_DIR}/")
    }
    for path in sorted(mesh_dir.iterdir()):
        if path.is_file() and path.name not in wanted:
            path.unlink()
    for relative, text in files.items():
        path = model_dir / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--airframe",
        choices=available_airframes(),
        help="generate one bundled airframe (default: all)",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=DEFAULT_MODELS_DIR,
        help=f"Gazebo models directory (default: {DEFAULT_MODELS_DIR})",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify generated files instead of writing them",
    )
    args = parser.parse_args()

    airframes = [args.airframe] if args.airframe else available_airframes()
    if args.check:
        problems: list[str] = []
        for airframe in airframes:
            spec = load_multirotor_spec(airframe)
            directory = model_directory(args.out, airframe)
            files = multirotor_gazebo_files(spec)
            current = check(directory, files)
            problems.extend(current)
            if not current:
                print(f"{directory}: {len(files)} files match {spec.source}")
        for problem in problems:
            print(problem, file=sys.stderr)
        if problems:
            print(
                "run scripts/generate_multirotor_assets.py to regenerate",
                file=sys.stderr,
            )
            return 1
        return 0

    for airframe in airframes:
        spec = load_multirotor_spec(airframe)
        directory = model_directory(args.out, airframe)
        files = multirotor_gazebo_files(spec)
        write_model(directory, files)
        print(f"wrote {len(files)} files to {directory}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
