cmake_minimum_required(VERSION 3.16)
project(hunter_benchmark)

find_package(ament_cmake REQUIRED)

# Python scripts, not a full ament_python package -- stats.py is installed
# alongside the two executables so they can `import stats` from the same
# directory post-install (see each script's sys.path.insert()).
install(PROGRAMS
  scripts/metrics_logger.py
  scripts/summarize_run.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  scripts/stats.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
