#!/usr/bin/env bash
set -euo pipefail
engine_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ros_prefix="${ROS_PREFIX:-/opt/ros/jazzy}"
includes=(-I"${engine_dir}/engine_vendor/include" -I/usr/include/eigen3)
for include_dir in "${ros_prefix}"/include/*; do
  if [[ -d "${include_dir}" ]]; then includes+=(-I"${include_dir}"); fi
done
if [[ "${1:-}" == "--packaged" ]]; then
  local_prefix="${engine_dir}/engine_ros/root/opt/ros/jazzy"
  "${CXX:-g++}" -O3 -DNDEBUG -std=c++17 -fPIC -shared \
    -I"${local_prefix}/include/robot_localization" "${includes[@]}" \
    "${engine_dir}/engine.cpp" \
    -L"${local_prefix}/lib" -L"${ros_prefix}/lib" \
    -Wl,--disable-new-dtags,-rpath,"${local_prefix}/lib:${engine_dir}/engine_ros/root/usr/lib/x86_64-linux-gnu:${ros_prefix}/lib" \
    -lrl_lib -lrclcpp -lrcutils -lrcl -o "${engine_dir}/engine_installed_core.so"
  exit 0
fi
"${CXX:-g++}" -O3 -DNDEBUG -std=c++17 -fPIC -shared \
  "${includes[@]}" \
  "${engine_dir}/engine.cpp" \
  "${engine_dir}/engine_vendor/src/ekf.cpp" \
  "${engine_dir}/engine_vendor/src/filter_base.cpp" \
  "${engine_dir}/engine_vendor/src/filter_utilities.cpp" \
  -L"${ros_prefix}/lib" -Wl,--disable-new-dtags,-rpath,"${ros_prefix}/lib" -lrclcpp -lrcutils -lrcl \
  -o "${engine_dir}/engine_core.so"
