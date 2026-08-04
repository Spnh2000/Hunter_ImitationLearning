cmake_minimum_required(VERSION 3.16)
project(hunter_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hunter_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_toolbox REQUIRED)

add_library(hunter_control_common src/hunter_control_common.cpp)
target_include_directories(hunter_control_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hunter_control_common hunter_msgs nav_msgs)

add_library(pid_tracker_lib src/pid_tracker.cpp)
target_include_directories(pid_tracker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pid_tracker_lib hunter_control_common)
ament_target_dependencies(pid_tracker_lib rclcpp hunter_msgs nav_msgs geometry_msgs control_toolbox)

add_executable(pid_tracker src/pid_tracker_node.cpp)
target_link_libraries(pid_tracker pid_tracker_lib)
ament_target_dependencies(pid_tracker rclcpp)

install(TARGETS hunter_control_common pid_tracker_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS pid_tracker
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
