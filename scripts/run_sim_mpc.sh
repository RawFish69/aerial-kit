#!/bin/bash
# Standalone Python sim with the constrained (QP) MPC. Usage: ./run_sim_mpc.sh [headless] [extra args...]
# The unconstrained finite-horizon LQ is still available as: ./run_sim.sh mpc
exec "$(dirname "$0")/run_sim.sh" constrained_mpc "$@"
