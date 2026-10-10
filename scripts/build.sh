#!/bin/bash
# Build script for aerial-kit workspace

set -e

echo "=== Building aerial-kit Workspace ==="

# Navigate to workspace
cd "$(dirname "$0")/../ros2_ws"

# Source ROS 2 (Jazzy on Ubuntu 24.04, Humble on 22.04)
ROS_SETUP=""
for distro in jazzy humble; do
    if [ -f "/opt/ros/$distro/setup.bash" ]; then ROS_SETUP="/opt/ros/$distro/setup.bash"; break; fi
done
if [ -z "$ROS_SETUP" ]; then
    echo "Error: no ROS 2 install found under /opt/ros (tried jazzy, humble)."
    exit 1
fi
source "$ROS_SETUP"

# Build with colcon
echo "Building packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "=== Build complete! ==="
echo "To use the workspace, run: source ros2_ws/install/setup.bash"

