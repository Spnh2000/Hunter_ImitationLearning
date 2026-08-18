cmake_minimum_required(VERSION 3.8)
project(hunter_localization)

find_package(ament_cmake REQUIRED)

# Config/launch only -- no targets to build. maps/ is installed so a
# serialized pose graph produced in mapping mode is found by localization
# mode through the normal package share path.
install(
  DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
