cmake_minimum_required(VERSION 3.8)
project(hamr_swivel_caster)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf config launch worlds docs DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/run_swivel_caster.py scripts/analyze_swivel_run.py scripts/annotate_overview.py
  scripts/run_caster_sweep.py scripts/analyze_caster_stress.py
  scripts/build_stress_report.py
  scripts/finalize_caster_sweep.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/prepare_swivel_world.py DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(swivel_geometry test/test_geometry.py)
  ament_add_pytest_test(swivel_run_harness test/test_run_harness.py)
  ament_add_pytest_test(caster_stress_analysis test/test_caster_stress.py)
  ament_add_pytest_test(caster_sweep test/test_sweep.py)
  ament_add_pytest_test(caster_stress_report test/test_stress_report.py)
endif()
ament_package()
