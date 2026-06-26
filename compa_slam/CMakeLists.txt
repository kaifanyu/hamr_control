cmake_minimum_required(VERSION 3.8)
project(compa_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install all static assets (launch files, configs, sim sensor/world, rviz).
install(
  DIRECTORY launch config urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Map-conversion / sim-setup and bag-recording tools.
install(
  PROGRAMS
    scripts/map_to_sim.py
    scripts/record_compa_slam_bag
    scripts/analyze_rosbag.py
  DESTINATION lib/${PROJECT_NAME}
)

# Generated sim maps (heightmap PNG + .yaml + .sdf) -- SMALL, and what
# replay_map_sim.launch.py reads. Heavy capture artifacts (.db/.ply/.pcd) and raw
# bags stay in maps/ + bags/ as runtime working dirs and are excluded from install.
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
  PATTERN "*.db" EXCLUDE
  PATTERN "*.ply" EXCLUDE
  PATTERN "*.pcd" EXCLUDE
  PATTERN "bags" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
