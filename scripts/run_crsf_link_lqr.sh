#!/bin/bash
set -e

cd "$(dirname "$0")/../ros2_ws"
for distro in jazzy humble foxy; do if [ -f "/opt/ros/$distro/setup.bash" ]; then source "/opt/ros/$distro/setup.bash"; break; fi; done
[ -d install ] || colcon build --symlink-install
source install/setup.bash
ros2 launch hw_bridge hw_crsf.launch.py "$@"

