#!/usr/bin/env bash
set -eo pipefail
ui_package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ui_repository="$(dirname "$ui_package")"
# Help and the terrain catalogue are useful before dependencies are installed.
ui_metadata_only=false
for ui_argument in "$@"; do
  case "$ui_argument" in -h|--help|--list-terrains) ui_metadata_only=true ;; esac
done
if [[ "$ui_metadata_only" == false ]]; then
  source "$ui_package/scripts/runtime_env.sh"
  hamr_load_runtime "$ui_repository" runner
fi
source "$ui_package/scripts/graphics_env.sh"
export PYTHONPATH="$ui_package${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m hamr_sim_ui.runner --root "$ui_repository" "$@"
