#!/bin/bash
# Run the standalone Python simulator (no ROS) with a chosen controller.
#
# Usage: ./run_sim.sh <controller> [headless] [extra aerial_kit.sim.cli args...]
#   controller: any registered name - pid, lqr, mpc, mppi, constrained_mpc,
#               warm_mppi (python3 -m sim_py.run_sim --help lists them)
#   headless:   'headless', 'true' or '1' to skip the plot window and save
#               the figure to results/<controller>.png instead
#
# Environment: TERRAIN (default forest), PLANNER (default rrtstar),
#              SIM_CONFIG (default sim_py/sim_config.yaml), PYTHON (python3)
#
# These scripts used to launch a ROS 2 'sim_dyn' package and C++ controllers
# that were removed when the workspace was consolidated; the ROS 2 stack now
# lives under ros2_ws/ (see ros2_ws/README.md and docs/SOFTWARE_GUIDE.md).

set -e

CONTROLLER="${1:?usage: run_sim.sh <controller> [headless] [extra args...]}"
shift
HEADLESS="false"
if [ "${1:-}" = "headless" ] || [ "${1:-}" = "true" ] || [ "${1:-}" = "1" ]; then
    HEADLESS="true"
    shift
fi

cd "$(dirname "$0")/.."
PYTHON="${PYTHON:-python3}"
ARGS=(
    --config "${SIM_CONFIG:-sim_py/sim_config.yaml}"
    --controller "$CONTROLLER"
    --terrain "${TERRAIN:-forest}"
    --planner "${PLANNER:-rrtstar}"
)
if [ "$HEADLESS" = "true" ]; then
    mkdir -p results
    ARGS+=(--no-show --save "results/${CONTROLLER}.png")
fi

echo "=== aerial-kit sim: controller=$CONTROLLER terrain=${TERRAIN:-forest} planner=${PLANNER:-rrtstar} headless=$HEADLESS ==="
exec "$PYTHON" -m aerial_kit.sim.cli "${ARGS[@]}" "$@"
