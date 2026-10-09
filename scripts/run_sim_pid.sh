#!/bin/bash
# Standalone Python sim with the PID controller. Usage: ./run_sim_pid.sh [headless] [extra args...]
exec "$(dirname "$0")/run_sim.sh" pid "$@"
