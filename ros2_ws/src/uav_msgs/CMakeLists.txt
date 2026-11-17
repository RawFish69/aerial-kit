cmake_minimum_required(VERSION 3.8)
project(uav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManualControl.msg"
  "msg/Waypoint.msg"
  "msg/Trajectory.msg"
  "msg/Command.msg"
  "msg/Telemetry.msg"
  "msg/MissionStatus.msg"
  "msg/AirspeedNav.msg"
  "msg/ActuatorCommand.msg"
  "msg/MultirotorManualCommand.msg"
  "srv/PlanPath.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
