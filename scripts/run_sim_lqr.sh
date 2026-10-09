#!/bin/bash
# Standalone Python sim with the LQR controller. Usage: ./run_sim_lqr.sh [headless] [extra args...]
exec "$(dirname "$0")/run_sim.sh" lqr "$@"
