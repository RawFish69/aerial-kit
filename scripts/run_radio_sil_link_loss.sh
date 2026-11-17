#!/usr/bin/env bash
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
airframe="${1:-fpv_quad_5in}"
tag="${2:-${airframe}-link-loss}"

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
    radio_rate_hz:=150.0 \
    radio_latency_ms:=8.0 \
    radio_packet_loss_percent:=0.0 \
    >"$log" 2>&1 &
launch_pid=$!

cleanup() {
    kill -INT -- -"$launch_pid" 2>/dev/null || true
    for _ in $(seq 1 20); do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep 1
    done
    kill -TERM -- -"$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 "$repo/scripts/radio_sil_link_loss_probe.py" \
    --out "$summary" --timeout "${RADIO_SIL_LINK_LOSS_TIMEOUT_SEC:-180}"

echo "log=$log"
echo "summary=$summary"
