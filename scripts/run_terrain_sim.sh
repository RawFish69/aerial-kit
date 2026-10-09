#!/bin/bash
# Standalone Python sim over generated terrain.
# Usage: ./run_terrain_sim.sh [controller] [terrain_type] [headless]
#   controller:   pid | lqr | mpc (-> constrained_mpc) | mppi (-> warm_mppi) | any registered name
#   terrain_type: forest | mountains | plains (default: forest)
#   headless:     true/false (default: false)
set -e
CONTROLLER="${1:-pid}"
case "$CONTROLLER" in
    mpc) CONTROLLER=constrained_mpc ;;
    mppi) CONTROLLER=warm_mppi ;;
esac
HEADLESS="${3:-false}"
MODE=""
if [ "$HEADLESS" = "true" ] || [ "$HEADLESS" = "1" ] || [ "$HEADLESS" = "headless" ]; then
    MODE="headless"
fi
TERRAIN="${2:-forest}" exec "$(dirname "$0")/run_sim.sh" "$CONTROLLER" $MODE
