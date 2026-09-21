#!/usr/bin/env bash
set -euo pipefail
caster_package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
caster_repository="$(dirname "$caster_package")"
caster_workspace="${HAMR_BALL_CASTER_WS:-$HOME/hamr_ball_caster_ws}"
# Do not source an old overlay while rebuilding its source.
set +u
source /opt/ros/jazzy/setup.bash
if [[ ! -d /opt/ros/jazzy/share/ros_gz_sim && -f "$HOME/.cache/hamr-gazebo-root/env.sh" ]]; then
  source "$HOME/.cache/hamr-gazebo-root/env.sh"
fi
set -u
export PATH="/opt/ros/jazzy/bin:/usr/bin:/bin:$PATH"
caster_colcon="$(command -v colcon || true)"
if [[ -z "$caster_colcon" && -x "$HOME/.cache/hamr-ball-caster-venv/bin/colcon" ]]; then
  caster_colcon="$HOME/.cache/hamr-ball-caster-venv/bin/colcon"
fi
if [[ -z "$caster_colcon" ]] || ! command -v xacro >/dev/null; then
  echo "Install python3-colcon-common-extensions and ros-jazzy-xacro first." >&2
  exit 1
fi
mkdir -p "$caster_workspace"
cd "$caster_workspace"
caster_packages=("$caster_package" "$caster_repository/reference_trajectory" "$caster_repository/hamr_interfaces")
"$caster_colcon" build --symlink-install --paths "${caster_packages[@]}" \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
set +u
source "$caster_workspace/install/setup.bash"
set -u
"$caster_colcon" test --paths "$caster_package" --packages-select hamr_ball_caster --event-handlers console_direct+
"$caster_colcon" test-result --verbose
