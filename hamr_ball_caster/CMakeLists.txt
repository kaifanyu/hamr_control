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
  tools/evaluate_waypoint_profile.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/feasible_waypoint.py scripts/continuous_waypoint.py
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
  ament_add_pytest_test(continuous_waypoint test/test_continuous_waypoint.py)
  ament_add_pytest_test(continuous_tracking test/test_continuous_tracking.py)
  ament_add_pytest_test(recording_quality test/test_recording_quality.py)
endif()
ament_package()
