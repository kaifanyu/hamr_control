cmake_minimum_required(VERSION 3.8)
project(hamr_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

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
    scripts/caster_recording
    scripts/caster_center_angle
    scripts/record_caster_rotation.py
    scripts/replay_caster_encoder_joints
    scripts/render_caster_encoder_video.py
    scripts/render_caster_recording.sh
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
install(FILES scripts/caster_center_calibration.py DESTINATION lib/${PROJECT_NAME})

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
  ament_add_pytest_test(caster_center_calibration
    test/test_caster_center_calibration.py
  )
  ament_add_pytest_test(caster_recording_client
    test/test_caster_recording_client.py
    TIMEOUT 90
  )
  ament_add_pytest_test(caster_recording_format
    test/test_caster_recording_format.py
  )
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
