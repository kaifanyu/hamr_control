cmake_minimum_required(VERSION 3.8)
project(hamr_terrain)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch docs DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
# Install the canonical map folders, not the source compatibility symlinks.
install(DIRECTORY
  "${CMAKE_CURRENT_SOURCE_DIR}/../maps/config"
  "${CMAKE_CURRENT_SOURCE_DIR}/../maps/meshes"
  "${CMAKE_CURRENT_SOURCE_DIR}/../maps/worlds"
  "${CMAKE_CURRENT_SOURCE_DIR}/../maps/assets"
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/generate_terrains.py scripts/terrain_simulation_check.py
  scripts/build_terrain_gallery.py
  scripts/generate_adtg_terrains.py scripts/build_adtg_gallery.py
  scripts/sample_adtg.py scripts/setup_adtg.sh
  scripts/terrain_replay_paths.py scripts/run_terrain_replay.py
  scripts/build_terrain_replays.py scripts/run_terrain_replay_suite.py
  scripts/audit_terrain_replays.py
  scripts/generate_full_uneven_terrains.py
  scripts/build_full_uneven_report.py
  scripts/generate_verti_arena.py
  scripts/partition_terrain_collision.py
  scripts/build_verti_arena_report.py
  scripts/generate_caster_stress_terrains.py
  scripts/generate_rocky_terrains.py
  scripts/analyze_rocky_return.py
  scripts/finalize_rocky_media.py
  scripts/analyze_caster_stress.py
  scripts/plot_caster_stress_pair.py
  scripts/build_caster_stress_comparison.py
  scripts/render_caster_pair.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(terrain_geometry test/test_geometry.py TIMEOUT 120)
  ament_add_pytest_test(terrain_launch test/test_launch_contract.py)
  ament_add_pytest_test(terrain_checks test/test_simulation_check.py)
  ament_add_pytest_test(adtg_import test/test_adtg_import.py)
  ament_add_pytest_test(adtg_sampling test/test_adtg_sampling.py)
  ament_add_pytest_test(terrain_replay_paths test/test_replay_paths.py)
  ament_add_pytest_test(terrain_replay_suite test/test_replay_suite.py)
  ament_add_pytest_test(full_uneven_terrain test/test_full_uneven.py TIMEOUT 120)
  ament_add_pytest_test(verti_arena_terrain test/test_verti_arena.py TIMEOUT 120)
  ament_add_pytest_test(caster_stress_analysis test/test_caster_stress_analysis.py TIMEOUT 120)
  ament_add_pytest_test(caster_stress_geometry test/test_caster_stress_geometry.py TIMEOUT 120)
  ament_add_pytest_test(rocky_terrain_geometry test/test_rocky_terrains.py TIMEOUT 120)
  ament_add_pytest_test(rocky_return_analysis test/test_rocky_return_analysis.py TIMEOUT 120)
  ament_add_pytest_test(caster_stress_pair_plot test/test_caster_stress_pair_plot.py TIMEOUT 120)
  ament_add_pytest_test(caster_stress_comparison test/test_caster_stress_comparison.py TIMEOUT 120)
  ament_add_pytest_test(caster_stress_pair_video test/test_render_caster_pair.py TIMEOUT 120)
endif()
ament_package()
