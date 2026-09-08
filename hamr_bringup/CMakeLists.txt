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
    scripts/wheel_response_test
    scripts/closed_loop_turn_test
    scripts/record_hamr_vicon_bag
    scripts/record_hamr_test_bag
    scripts/run_waypoint_test
    scripts/run_waypoint_stack
    scripts/run_waypoint_reference
    scripts/analyze_waypoint_test
    scripts/run_study_stack
    scripts/run_study_reference
    scripts/analyze_study_test
    scripts/create_study_plan
  DESTINATION lib/${PROJECT_NAME}
)

# Keep the generic offline analyzers beside the concise operator wrapper so
# `ros2 run` works from an installed workspace without relying on source paths.
install(
  PROGRAMS
    ../rosbags/analyze_hamr_vicon_straight.py
    ../rosbags/plot_hamr_actuators.py
    ../rosbags/analyze_hamr_study.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(closed_loop_turn_test_math
    test/test_closed_loop_turn_test.py
  )
  ament_add_pytest_test(vicon_delivery_config
    test/test_vicon_delivery_config.py
  )
  ament_add_pytest_test(wheel_response_test_safety
    test/test_wheel_response_test.py
  )
  ament_add_pytest_test(run_waypoint_test_wrapper
    test/test_run_waypoint_test.py
  )
  ament_add_pytest_test(split_waypoint_commands
    test/test_split_waypoint_commands.py
  )
  ament_add_pytest_test(analyze_waypoint_test_wrapper
    test/test_analyze_waypoint_test.py
  )
  ament_add_pytest_test(study_commands
    test/test_study_commands.py
  )
  ament_add_pytest_test(study_analyzer
    test/test_analyze_study_test.py
  )
  ament_add_pytest_test(study_plan_generator
    test/test_create_study_plan.py
  )
  ament_add_pytest_test(actuator_plot_contract
    test/test_plot_hamr_actuators.py
  )
endif()

ament_package()
