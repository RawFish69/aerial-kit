#!/usr/bin/env bash
set -euo pipefail

repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ros_setup="${ROS_SETUP:-$HOME/.local/share/ros/jazzy/setup.bash}"

if [[ ! -f "$ros_setup" ]]; then
    echo "test_ros2: ROS setup not found: $ros_setup" >&2
    echo "Set ROS_SETUP to the underlay's setup.bash." >&2
    exit 1
fi

# shellcheck disable=SC1090
set +u
source "$ros_setup"
set -u
cd "$repo/ros2_ws"
colcon build --symlink-install
# shellcheck disable=SC1091
set +u
source install/setup.bash
set -u

# This workspace uses pytest-style functions. The legacy setup.py test entries
# do not discover them, so run the package test directories explicitly.
if (( $# > 0 )); then
    python3 -m pytest -q "$@"
else
    python3 -m pytest -q \
        src/air_unit/test \
        src/hw_bridge/test \
        src/mavlink_bridge/test \
        src/uav_algorithms/test \
        src/sim_bridge/test \
        src/uav_control/test \
        src/sim_fast/test \
        src/sim_gazebo/test
fi
