#!/usr/bin/env bash
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
airframe="${1:-fpv_quad_5in}"
loss_pct="${2:-0.0}"
latency_ms="${3:-8.0}"
rate_hz="${4:-150.0}"
tag="${5:-${airframe}-loss${loss_pct}}"

ros_setup="${ROS_SETUP:-$HOME/.local/share/ros/jazzy/setup.bash}"
if [[ ! -f "$ros_setup" ]]; then
    echo "ROS setup not found: $ros_setup" >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "$ros_setup"
source "$repo/ros2_ws/install/setup.bash"
set -u
export PYTHONPATH="$repo${PYTHONPATH:+:$PYTHONPATH}"

stamp="$(date +%Y%m%d-%H%M%S)"
base="$repo/docs/evidence/radio-sil-${tag}-${stamp}"
log="$base.log"
summary="$base.json"

setsid ros2 launch sim_fast bringup.launch.py \
    start_demo:=true \
    start_monitor:=false \
    sim_backend:=mujoco \
    airframe:="$airframe" \
    radio_sim:=true \
    radio_rate_hz:="$rate_hz" \
    radio_latency_ms:="$latency_ms" \
    radio_packet_loss_percent:="$loss_pct" \
    >"$log" 2>&1 &
launch_pid=$!

cleanup() {
    kill -INT -- -"$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep 1
    done
    kill -TERM -- -"$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 5); do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep 1
    done
    kill -KILL -- -"$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "$repo/scripts/multirotor_mission_monitor.py" \
    --out "$summary" --log "$log" --timeout "${RADIO_SIL_TIMEOUT_SEC:-300}"

echo "log=$log"
echo "summary=$summary"
