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
  PROGRAMS scripts/record_hamr_vicon_bag scripts/record_webcam.py
  DESTINATION lib/${PROJECT_NAME}
  # Explicit permissions also make ament copy these programs during symlink
  # installs, so Windows checkouts without Unix executable bits still work.
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(record_webcam test/test_record_webcam.py)
endif()

ament_package()
