cmake_minimum_required(VERSION 3.8)
project(hamr_ball_caster)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch config worlds assets docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/run_physics_checks.py scripts/run_physics_suite.py scripts/run_compa_checks.py
  scripts/run_waypoint_sim.py scripts/record_simulation.py scripts/record_waypoint_run.py
  scripts/run_ball_caster.py scripts/finalize_ball_comparison.py
  scripts/run_ball_comparison.py scripts/build_caster_comparison.py
  scripts/analyze_ball_stress.py scripts/compare_caster_runs.py
  scripts/summarize_caster_comparison.py scripts/analyze_ball_comparison.py
  scripts/active_ball_gazebo.py scripts/render_active_ball_demo.py
  tools/evaluate_waypoint_profile.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/feasible_waypoint.py scripts/continuous_waypoint.py
  scripts/startup_alignment.py scripts/prepare_caster_world.py
  scripts/active_ball_trajectory.py
  tools/analyze_corner_tracking.py tools/analyze_continuous_tracking.py
  tools/prepare_continuous_world.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(caster_geometry test/test_geometry.py)
  ament_add_pytest_test(compa_retrofit test/test_compa_retrofit.py)
  ament_add_pytest_test(waypoint_sim test/test_waypoint_sim.py)
  ament_add_pytest_test(recording_model test/test_recording_model.py)
  ament_add_pytest_test(feasible_waypoint test/test_feasible_waypoint.py)
  ament_add_pytest_test(corner_tracking test/test_corner_tracking.py)
  ament_add_pytest_test(waypoint_settling test/test_waypoint_settling.py)
  ament_add_pytest_test(startup_alignment test/test_startup_alignment.py)
  ament_add_pytest_test(ball_run_harness test/test_run_ball_caster.py)
  ament_add_pytest_test(ball_video_validation test/test_ball_video_validation.py)
  ament_add_pytest_test(ball_comparison test/test_ball_comparison.py)
  ament_add_pytest_test(ball_comparison_batch test/test_ball_comparison_batch.py)
  ament_add_pytest_test(comparison_suite test/test_comparison_suite.py)
  ament_add_pytest_test(comparison_page test/test_comparison_page.py)
  ament_add_pytest_test(comparison_summary test/test_comparison_summary.py)
  ament_add_pytest_test(continuous_waypoint test/test_continuous_waypoint.py)
  ament_add_pytest_test(continuous_tracking test/test_continuous_tracking.py)
  ament_add_pytest_test(recording_quality test/test_recording_quality.py)
  ament_add_pytest_test(active_ball_trajectory test/test_active_ball_trajectory.py)
endif()
ament_package()
