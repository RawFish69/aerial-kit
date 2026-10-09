#!/bin/bash
# Quick diagnostic for the ROS 2 simulation stack (sim_fast or sim_gazebo).
# Run in a terminal with the workspace sourced, while a bringup is running.

echo "=== Nodes ==="
ros2 node list
echo

echo "=== Mission follower (one of these should be running) ==="
ros2 node list | grep -E "mission_executor_node|mpc_tracker_node" || echo "  none running"
ros2 param get /mpc_tracker_node controller 2>/dev/null && \
    ros2 param get /mpc_tracker_node velocity_loop_tau_s 2>/dev/null
echo

echo "=== Command manager ==="
timeout 3s ros2 topic echo /uav/telemetry --once --field status_text 2>/dev/null || echo "  no /uav/telemetry"
echo

echo "=== Topic samples (/uav/* contract) ==="
exec bash "$(dirname "$0")/check_ros2_v2_topics.sh"
