#!/usr/bin/env bash
# Shared launcher checks. All changes live in the launcher's child shell.
hamr_load_runtime() {
  local repository="$1" mode="$2"
  if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
    printf '[HAMR] ROS 2 Jazzy is missing: /opt/ros/jazzy/setup.bash. Install ROS 2 Jazzy and the dependencies in %s/hamr_sim_ui/README.md, then run bash hamr_terrain/scripts/build.sh.\n' "$repository" >&2
    return 1
  fi
  source "$repository/hamr_terrain/scripts/env.sh"
  /usr/bin/python3 - "$repository" "$mode" <<'PY'
import importlib.util
from pathlib import Path
import sys

root, mode = Path(sys.argv[1]), sys.argv[2]
modules = {"rclpy": "ros-jazzy-rclpy", "rosbag2_py": "ros-jazzy-rosbag2-py",
           "ament_index_python": "ros-jazzy-ament-index-python", "numpy": "python3-numpy"}
if mode == "ui":
    modules["PyQt5"] = "python3-pyqt5"
missing = [package for module, package in modules.items() if importlib.util.find_spec(module) is None]
if missing:
    print("[HAMR] Missing Python dependencies. On Ubuntu with ROS Jazzy configured, run:\n"
          "  sudo apt install " + " ".join(missing), file=sys.stderr)
    sys.exit(1)
from ament_index_python.packages import get_package_prefix, PackageNotFoundError
missing_ros, missing_local = [], []
for name in ("ros_gz_sim", "ros_gz_bridge", "robot_state_publisher", "xacro",
             "hamr_terrain", "hamr_ball_caster", "hamr_swivel_caster", "hamr_interfaces",
             "reference_trajectory"):
    try:
        get_package_prefix(name)
    except PackageNotFoundError:
        (missing_local if name.startswith("hamr_") or name == "reference_trajectory" else missing_ros).append(name)
if missing_ros:
    print("[HAMR] Missing ROS packages: " + ", ".join(missing_ros) +
          ". Install the ROS/Gazebo dependencies in " + str(root / "hamr_sim_ui/README.md") + ".", file=sys.stderr)
if missing_local:
    print("[HAMR] Workspace packages are not built/sourced: " + ", ".join(missing_local) +
          ". From the repository run: bash hamr_terrain/scripts/build.sh\n"
          "[HAMR] To use a different workspace, set HAMR_SWIVEL_CASTER_WS to its directory.", file=sys.stderr)
sys.exit(bool(missing_ros or missing_local))
PY
}
