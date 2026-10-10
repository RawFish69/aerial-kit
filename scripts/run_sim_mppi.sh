#!/bin/bash
# Standalone Python sim with the warm-started MPPI. Usage: ./run_sim_mppi.sh [headless] [extra args...]
exec "$(dirname "$0")/run_sim.sh" warm_mppi "$@"
