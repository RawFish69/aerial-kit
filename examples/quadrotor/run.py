"""Runnable quadrotor example using the public aerial_kit API."""

from __future__ import annotations

import argparse
from pathlib import Path

from aerial_kit.sim import load_config, run_simulation
from aerial_kit.visualization import plot_simulation


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the aerial-kit quadrotor example.")
    parser.add_argument("--config", type=Path, default=Path(__file__).with_name("config.yaml"))
    parser.add_argument("--save", type=Path, help="Save the plot as PNG/SVG/PDF.")
    parser.add_argument("--no-show", action="store_true", help="Run without opening a window.")
    args = parser.parse_args()

    config = load_config(args.config)
    result = run_simulation(config)
    figure = plot_simulation(result, show=not args.no_show)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160, bbox_inches="tight")

    print(f"Final position: {result.trajectory[-1].round(2).tolist()} m")
    print(f"Goal error: {result.distance_to_goal:.2f} m")
    print(f"Collisions: {result.collisions_detected}")


if __name__ == "__main__":
    main()
