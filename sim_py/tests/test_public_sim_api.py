from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from aerial_kit.sim import load_config, run_simulation
from aerial_kit.visualization import plot_follow_view, plot_simulation
from aerial_kit.sim.cli import _parser


REPO_ROOT = Path(__file__).resolve().parents[2]


def test_public_cli_has_short_teleop_mode() -> None:
    args = _parser().parse_args(["--teleop"])
    assert args.teleop is True
    assert args.example == "quadrotor"


def test_public_cli_planner_flag_overrides_the_config() -> None:
    """`--planner` must reach path_cfg, which is where the runner reads it."""
    from dataclasses import replace

    config = load_config(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    assert config.path_cfg["planner_type"] == "straight"

    args = _parser().parse_args(["--planner", "rrtstar"])
    assert args.planner == "rrtstar"

    overridden = replace(config, path_cfg={**config.path_cfg, "planner_type": args.planner})
    assert overridden.path_cfg["planner_type"] == "rrtstar"
    # The original config must not be mutated by the override.
    assert config.path_cfg["planner_type"] == "straight"


def test_public_cli_planner_flag_defaults_to_none() -> None:
    assert _parser().parse_args([]).planner is None


def test_planner_override_changes_the_planner_the_runner_reports() -> None:
    from dataclasses import replace

    config = load_config(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    result = run_simulation(
        replace(config, path_cfg={**config.path_cfg, "planner_type": "rrtstar"})
    )

    assert result.planner_type == "rrtstar"
    assert len(result.planned_waypoints) > 2


def test_legacy_cli_planner_flag_reaches_path_cfg() -> None:
    from sim_py.app.cli import parse_args
    from sim_py.core.config import normalize_sim_config

    import sys

    config_path = str(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    argv = sys.argv
    try:
        sys.argv = ["run_sim", "--sim-config", config_path, "--planner", "dubins"]
        cfg = normalize_sim_config(parse_args())
    finally:
        sys.argv = argv

    assert cfg.path_cfg["planner_type"] == "dubins"


def test_legacy_cli_without_planner_flag_keeps_the_yaml_value() -> None:
    from sim_py.app.cli import parse_args
    from sim_py.core.config import normalize_sim_config

    import sys

    config_path = str(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    argv = sys.argv
    try:
        sys.argv = ["run_sim", "--sim-config", config_path]
        cfg = normalize_sim_config(parse_args())
    finally:
        sys.argv = argv

    assert cfg.path_cfg["planner_type"] == "straight"


def test_quadrotor_example_runs_and_reaches_goal() -> None:
    config = load_config(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    result = run_simulation(config)

    assert config.airframe_name == "quad"
    assert config.backend_name == "multirotor"
    assert result.distance_to_goal <= 3.0
    assert result.collisions_detected == 0


def test_twin_wing_example_runs_from_cruise_and_reaches_goal() -> None:
    config = load_config(REPO_ROOT / "examples" / "fixed_wing" / "config.yaml")
    result = run_simulation(config)

    assert config.airframe_name == "twin_wing"
    assert config.backend_name == "fixedwing"
    assert config.initial_state_cfg["velocity_mps"] != [0.0, 0.0, 0.0]
    assert result.distance_to_goal <= 6.1
    assert result.collisions_detected == 0


def test_public_visualizer_accepts_a_simulation_result() -> None:
    config = load_config(REPO_ROOT / "examples" / "quadrotor" / "config.yaml")
    result = run_simulation(config)
    figure = plot_simulation(result, show=False)

    assert figure.axes


def test_follow_view_zooms_onto_the_aircraft() -> None:
    config = load_config(REPO_ROOT / "examples" / "fixed_wing" / "config.yaml")
    result = run_simulation(config)
    figure = plot_follow_view(result, show=False, follow_radius=11.0)
    ax = figure.axes[0]
    xlim = ax.get_xlim()
    ylim = ax.get_ylim()

    assert (xlim[1] - xlim[0]) < 0.5 * float(result.space_dim[0])
    assert (ylim[1] - ylim[0]) < 0.5 * float(result.space_dim[1])
    assert result.attitude_quats is not None
    assert len(result.attitude_quats) == len(result.trajectory)
