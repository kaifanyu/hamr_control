#!/usr/bin/env bash
# Obtain the packaged node for validation without changing the system install.
set -euo pipefail
engine_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "${engine_dir}/engine_ros/packages"
cd "${engine_dir}/engine_ros/packages"
apt download ros-jazzy-robot-localization libgeographiclib26 \
  ros-jazzy-geographic-msgs ros-jazzy-diagnostic-updater
for package_file in ./*.deb; do
  dpkg-deb -x "$package_file" "${engine_dir}/engine_ros/root"
done
