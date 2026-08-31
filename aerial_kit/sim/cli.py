"""Command-line entry point for the optional aerial-kit simulator."""

from __future__ import annotations

import argparse
from dataclasses import replace
from importlib.resources import as_file, files
from pathlib import Path

from aerial_kit.visualization import plot_simulation

from .api import load_config, run_simulation


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run an aerial-kit simulation.")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--config", type=Path, help="Simulation YAML file.")
    source.add_argument(
        "--example",
        choices=["quadrotor", "fixed-wing"],
        default="quadrotor",
        help="Bundled scenario to run when --config is not supplied.",
    )
    parser.add_argument(
        "--planner",
        choices=["straight", "astar", "rrt", "rrtstar", "rrt*", "dubins"],
        help="Override path planner (default: path.planner_type from the config).",
    )
    parser.add_argument("--controller", help="Override controller type.")
    parser.add_argument("--backend", help="Override dynamics backend.")
    parser.add_argument("--airframe", help="Override airframe.")
    parser.add_argument("--terrain", choices=["forest", "mountains", "plains"])
    parser.add_argument("--sim-time", type=float, help="Override duration in seconds.")
    parser.add_argument("--dt", type=float, help="Override integration timestep.")
    parser.add_argument("--save", type=Path, help="Save the resulting plot.")
    parser.add_argument("--no-show", action="store_true", help="Do not open a plot window.")
    parser.add_argument("--teleop", action="store_true", help="Fly interactively with the keyboard.")
    return parser


def main() -> None:
    args = _parser().parse_args()
    example_file = "fixed_wing.yaml" if args.example == "fixed-wing" else "quadrotor.yaml"
    default_resource = files("aerial_kit.sim.defaults").joinpath(example_file)
    with as_file(default_resource) as default_path:
        config = load_config(args.config or default_path)

    # The planner is read from path_cfg at plan time, so override it in place
    # rather than adding a competing top-level field.
    path_cfg = config.path_cfg
    if args.planner:
        path_cfg = {**path_cfg, "planner_type": args.planner}

    config = replace(
        config,
        path_cfg=path_cfg,
        controller_name=args.controller or config.controller_name,
        backend_name=args.backend or config.backend_name,
        airframe_name=args.airframe or config.airframe_name,
        terrain_override=args.terrain or config.terrain_override,
        sim_time=(
            args.sim_time
            if args.sim_time is not None
            else (0.0 if args.teleop else config.sim_time)
        ),
        dt=args.dt if args.dt is not None else config.dt,
    )
    if args.teleop:
        import logging

        from sim_py.teleop.session import run_teleop_cli

        logging.basicConfig(level=logging.INFO, format="%(message)s")
        run_teleop_cli(config)
        return

    result = run_simulation(config)
    figure = plot_simulation(result, show=not args.no_show)
    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.save, dpi=160, bbox_inches="tight")

    final = result.trajectory[-1]
    print(
        f"final_position=[{final[0]:.2f}, {final[1]:.2f}, {final[2]:.2f}] "
        f"goal_error={result.distance_to_goal:.2f}m "
        f"collisions={result.collisions_detected}"
    )


if __name__ == "__main__":
    main()
