cmake_minimum_required(VERSION 3.8)
project(hamr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(hamr_topic_monitor src/hamr_topic_monitor.cpp)
target_compile_features(hamr_topic_monitor PRIVATE cxx_std_17)
ament_target_dependencies(hamr_topic_monitor rclcpp diagnostic_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(hamr_topic_monitor yaml-cpp nlohmann_json::nlohmann_json)
install(TARGETS hamr_topic_monitor DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch config worlds
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
    scripts/caster_encoder_bridge
    scripts/ordinary_caster_bridge
    scripts/analyze_ordinary_caster
    scripts/replay_ordinary_caster_joints
    scripts/render_ordinary_caster_recording
    scripts/caster_recording
    scripts/start_encoder_recording
    scripts/stop_encoder_recording
    scripts/caster_center_angle
    scripts/calibrate_caster_center
    scripts/center_turn_check
    scripts/record_caster_rotation.py
    scripts/replay_caster_encoder_joints
    scripts/render_caster_encoder_video.py
    scripts/render_caster_recording.sh
    scripts/render_caster_simulation.sh
    scripts/render_caster_recording.py
    scripts/record_camera_clip
    scripts/record_dual_cameras
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

install(FILES scripts/c920_controls.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_encoder_protocol.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_replay_model.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_encoder_selection.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_encoder_plots.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_recording_client.py scripts/caster_recording_format.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_recording_session.py scripts/caster_recording_bag.py DESTINATION lib/${PROJECT_NAME})
install(FILES ../docs/LOCAL_CASTER_RECORDING.md DESTINATION share/${PROJECT_NAME}/docs)
install(FILES scripts/caster_center_calibration.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/caster_center_workflow.py DESTINATION lib/${PROJECT_NAME})
install(FILES ../docs/CENTER_CALIBRATION.md DESTINATION share/${PROJECT_NAME}/docs)
install(FILES ../docs/FOXGLOVE.md DESTINATION share/${PROJECT_NAME}/docs)
install(FILES ../docs/assets/foxglove_validation.json DESTINATION share/${PROJECT_NAME}/docs/assets)
install(DIRECTORY ordinary_caster DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES ../docs/ORDINARY_CASTER_RECORDING.md DESTINATION share/${PROJECT_NAME}/docs)

# Keep the generic offline analyzers beside the concise operator wrapper so
# `ros2 run` works from an installed workspace without relying on source paths.
install(
  PROGRAMS
    ../rosbags/analyze_hamr_vicon_straight.py
    ../rosbags/plot_hamr_actuators.py
    ../rosbags/analyze_caster_encoders.py
    ../rosbags/analyze_hamr_study.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(topic_health test/test_topic_health.cpp)
  target_compile_features(topic_health PRIVATE cxx_std_17)
  ament_target_dependencies(topic_health rclcpp diagnostic_msgs nav_msgs sensor_msgs std_msgs)
  target_link_libraries(topic_health yaml-cpp nlohmann_json::nlohmann_json)
  ament_add_pytest_test(foxglove_monitor test/test_foxglove_monitor.py)
  ament_add_pytest_test(ordinary_live test/test_ordinary_live.py)
  ament_add_pytest_test(ordinary_split test/test_ordinary_split.py TIMEOUT 90)
  ament_add_pytest_test(ordinary_integration test/test_ordinary_integration.py)
  ament_add_pytest_test(ordinary_protocol test/test_ordinary_protocol.py)
  ament_add_pytest_test(ordinary_recording_format test/test_ordinary_recording_format.py)
  ament_add_pytest_test(ordinary_recording_client test/test_ordinary_recording_client.py TIMEOUT 90)
  ament_add_pytest_test(ordinary_history test/test_ordinary_history.py)
  ament_add_pytest_test(ordinary_analysis test/test_ordinary_analysis.py)
  ament_add_pytest_test(ordinary_replay test/test_ordinary_replay.py)
  ament_add_pytest_test(caster_center_calibration
    test/test_caster_center_calibration.py
  )
  ament_add_pytest_test(caster_center_workflow
    test/test_caster_center_workflow.py
  )
  ament_add_pytest_test(center_turn_check test/test_center_turn_check.py)
  ament_add_pytest_test(right_center_reconstruction test/test_right_center_reconstruction.py)
  ament_add_pytest_test(caster_recording_client
    test/test_caster_recording_client.py
    TIMEOUT 90
  )
  ament_add_pytest_test(caster_recording_format
    test/test_caster_recording_format.py
  )
  ament_add_pytest_test(caster_recording_session test/test_caster_recording_session.py TIMEOUT 90)
  ament_add_pytest_test(caster_encoder_bridge
    test/test_caster_encoder_bridge.py
  )
  ament_add_pytest_test(record_caster_rotation
    test/test_record_caster_rotation.py
  )
  ament_add_pytest_test(caster_encoder_analysis
    test/test_analyze_caster_encoders.py
  )
  ament_add_pytest_test(caster_encoder_recording
    test/test_caster_encoder_recording.py
  )
  ament_add_pytest_test(dual_caster_pipeline
    test/test_dual_caster_pipeline.py
    TIMEOUT 90
  )
  ament_add_pytest_test(caster_encoder_replay
    test/test_caster_encoder_replay.py
  )
  ament_add_pytest_test(caster_encoder_video
    test/test_caster_encoder_video.py
  )
  ament_add_pytest_test(render_caster_recording
    test/test_render_caster_recording.py
  )
  ament_add_pytest_test(c920_controls
    test/test_c920_controls.py
  )
  ament_add_pytest_test(camera_recording_launch
    test/test_camera_recording_launch.py
  )
  ament_add_pytest_test(continuous_hardware_launch
    test/test_continuous_hardware_launch.py
  )
  ament_add_pytest_test(record_dual_cameras
    test/test_record_dual_cameras.py
  )
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
