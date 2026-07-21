cmake_minimum_required(VERSION 3.8)
project(vacuum_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 1. Install directories to share folder
install(DIRECTORY
  urdf
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install Python scripts as ROS 2 executables in lib folder
install(PROGRAMS
  vacuum_bot/lidar_scanner_node.py
  vacuum_bot/path_planner_node.py
  vacuum_bot/line_follower_node.py  # 👈 Add here
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()