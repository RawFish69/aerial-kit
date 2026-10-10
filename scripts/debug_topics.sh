#!/bin/bash
# Follow one command through the ROS 2 stack, hop by hop:
#   /uav/command -> mission follower -> /uav/internal/mission_cmd_vel
#   -> command_manager -> /uav/backend/cmd_twist (+ /uav/backend/enable)
#   -> backend -> /uav/backend/odom -> telemetry_adapter -> /uav/backend/telemetry_raw
#   -> command_manager -> /uav/telemetry
# The first hop that prints nothing is where the chain is broken.

hop() {
    echo "--- $1 ($2)"
    timeout 3s ros2 topic echo "$1" --once 2>/dev/null | head -${3:-20} || echo "    (nothing within 3 s)"
    ros2 topic info "$1" 2>/dev/null | sed 's/^/    /'
    echo
}

hop /uav/command "uav_msgs/Command, from the ground station" 12
hop /uav/mission_status "uav_msgs/MissionStatus, from the mission follower" 12
hop /uav/internal/mission_cmd_vel "geometry_msgs/Twist, follower -> command manager" 10
hop /uav/backend/enable "std_msgs/Bool, armed state to the backend" 3
hop /uav/backend/cmd_twist "geometry_msgs/Twist, to the backend" 10
hop /uav/backend/odom "nav_msgs/Odometry, from the backend" 20
hop /uav/telemetry "uav_msgs/Telemetry, enriched" 20
