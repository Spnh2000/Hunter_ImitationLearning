cmake_minimum_required(VERSION 3.16)
project(hunter_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY description launch config rviz meshes
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
