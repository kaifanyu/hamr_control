#!/usr/bin/env bash
set -euo pipefail
swivel_package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
swivel_repository="$(dirname "$swivel_package")"
swivel_workspace="${HAMR_SWIVEL_CASTER_WS:-$HOME/hamr_swivel_caster_ws}"
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -d /opt/ros/jazzy/share/ros_gz_sim && -f "$HOME/.cache/hamr-gazebo-root/env.sh" ]]; then
  source "$HOME/.cache/hamr-gazebo-root/env.sh"
fi
set -u
export PATH="/opt/ros/jazzy/bin:/usr/bin:/bin:$PATH"
swivel_colcon="$(command -v colcon || true)"
if [[ -z "$swivel_colcon" && -x "$HOME/.cache/hamr-ball-caster-venv/bin/colcon" ]]; then
  swivel_colcon="$HOME/.cache/hamr-ball-caster-venv/bin/colcon"
fi
if [[ -z "$swivel_colcon" ]] || ! command -v xacro >/dev/null; then
  echo "Install python3-colcon-common-extensions and ros-jazzy-xacro first." >&2
  exit 1
fi
mkdir -p "$swivel_workspace"
cd "$swivel_workspace"
swivel_packages=("$swivel_package" "$swivel_repository/hamr_ball_caster"
  "$swivel_repository/reference_trajectory" "$swivel_repository/hamr_interfaces")
"$swivel_colcon" build --symlink-install --paths "${swivel_packages[@]}" \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
set +u
source "$swivel_workspace/install/setup.bash"
set -u
"$swivel_colcon" test --paths "${swivel_packages[@]}" \
  --packages-select hamr_swivel_caster hamr_ball_caster --event-handlers console_direct+
"$swivel_colcon" test-result --verbose
