cmake_minimum_required(VERSION 3.8)
project(compa_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install all static assets (launch files, configs, sim sensor/world, rviz).
# `maps/` and `bags/` are runtime working dirs (git-ignored) and are NOT installed.
install(
  DIRECTORY launch config urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
