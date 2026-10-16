#!/usr/bin/env bash
set -euo pipefail
terrain_package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
terrain_repository="$(dirname "$terrain_package")"
terrain_workspace="${HAMR_SWIVEL_CASTER_WS:-$HOME/hamr_swivel_caster_ws}"
set +u
source "$terrain_package/scripts/env.sh"
set -u
terrain_colcon="$(command -v colcon || true)"
if [[ -z "$terrain_colcon" && -x "$HOME/.cache/hamr-ball-caster-venv/bin/colcon" ]]; then
  terrain_colcon="$HOME/.cache/hamr-ball-caster-venv/bin/colcon"
fi
if [[ -z "$terrain_colcon" ]]; then
  echo "Install python3-colcon-common-extensions first." >&2
  exit 1
fi
mkdir -p "$terrain_workspace"
cd "$terrain_workspace"
terrain_packages=("$terrain_package" "$terrain_repository/hamr_swivel_caster"
  "$terrain_repository/hamr_ball_caster" "$terrain_repository/reference_trajectory"
  "$terrain_repository/hamr_interfaces")
"$terrain_colcon" build --symlink-install --paths "${terrain_packages[@]}" \
  --cmake-args -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3
set +u
source "$terrain_workspace/install/setup.bash"
set -u
"$terrain_colcon" test --paths "${terrain_packages[@]}" \
  --packages-select hamr_terrain --event-handlers console_direct+
"$terrain_colcon" test-result --verbose
