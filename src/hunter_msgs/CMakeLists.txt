cmake_minimum_required(VERSION 3.16)
project(hunter_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Waypoint.msg"
  "msg/Trajectory.msg"
  "msg/EpisodeEvent.msg"
  "msg/TrackingDiagnostics.msg"
  DEPENDENCIES std_msgs
)

ament_package()
