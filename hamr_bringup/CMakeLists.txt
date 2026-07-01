cmake_minimum_required(VERSION 3.8)
project(hamr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds terrain_assets
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/live_mag_plot
    scripts/measure_encoder_resolution
    scripts/open_loop_wheel_calibration
    scripts/record_hamr_vicon_bag
    scripts/record_hamr_test_bag
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
