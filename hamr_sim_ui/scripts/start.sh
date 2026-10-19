#!/usr/bin/env bash
set -eo pipefail
ui_package="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ui_repository="$(dirname "$ui_package")"
source "$ui_package/scripts/runtime_env.sh"
hamr_load_runtime "$ui_repository" ui
source "$ui_package/scripts/graphics_env.sh"
export PYTHONPATH="$ui_package${PYTHONPATH:+:$PYTHONPATH}"
# Qt must use the system Python and its matching libraries, not Conda's Qt.
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
exec /usr/bin/python3 -m hamr_sim_ui --root "$ui_repository" "$@"
