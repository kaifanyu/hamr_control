The files in this directory are unmodified copies of the BSD-licensed
robot_localization EKF, filter base, and filter utilities from:

- Repository: https://github.com/cra-ros-pkg/robot_localization
- Branch: jazzy-devel
- Commit: 3efa714fb9c1ff40966327b7bed7053b2570be4d
- Retrieved: 2026-09-20

Only the numerical filter and its direct headers are vendored. The wrapper in
../engine.cpp creates the ROS filter's planar measurements without running ROS
transport. See ../engine_README.md for equivalence and limitations.
