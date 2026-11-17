#!/bin/bash
# Build script for aerial-kit workspace

set -e

echo "=== Building aerial-kit Workspace ==="

# Navigate to workspace
cd "$(dirname "$0")/../ros2_ws"

# Source ROS 2. A Conda/RoboStack Jazzy environment is already exported when
# `colcon` and `AMENT_PREFIX_PATH` are present; otherwise prefer an explicit
# ROS_SETUP, then this repo's installation guide's local Jazzy prefix, then the
# conventional system installs.
REQUESTED_ROS_SETUP="${ROS_SETUP:-}"
ROS_SETUP=""
if ! command -v colcon >/dev/null 2>&1 || [ -z "${AMENT_PREFIX_PATH:-}" ]; then
    for candidate in \
        "$REQUESTED_ROS_SETUP" \
        "$HOME/.local/share/ros/jazzy/setup.bash" \
        "/opt/ros/jazzy/setup.bash" \
        "/opt/ros/humble/setup.bash"; do
        if [ -n "$candidate" ] && [ -f "$candidate" ]; then
            ROS_SETUP="$candidate"
            break
        fi
    done
    if [ -z "$ROS_SETUP" ]; then
        echo "Error: no active ROS 2 environment; set ROS_SETUP to the underlay setup.bash."
        exit 1
    fi
fi
[ -n "$ROS_SETUP" ] && source "$ROS_SETUP"

# Build with colcon
echo "Building packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "=== Build complete! ==="
echo "To use the workspace, run: source ros2_ws/install/setup.bash"

