#!/bin/bash
# Build script for aerial-kit workspace

set -e

echo "=== Building aerial-kit Workspace ==="

# Navigate to workspace
cd "$(dirname "$0")/../ros2_ws"

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found. Please install ROS 2 Humble."
    exit 1
fi

# Build with colcon
echo "Building packages..."
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

echo "=== Build complete! ==="
echo "To use the workspace, run: source ros2_ws/install/setup.bash"

