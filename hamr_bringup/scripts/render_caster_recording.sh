#!/usr/bin/env bash
# Ubuntu workstation entry point; use the ROS-compatible system Python.
set -eo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# Help and argument errors do not require a ROS installation.
if (( $# == 0 )); then
  exec /usr/bin/python3 "$script_dir/render_caster_recording.py" "$@"
fi
for argument in "$@"; do
  if [[ "$argument" == --help || "$argument" == -h ]]; then
    exec /usr/bin/python3 "$script_dir/render_caster_recording.py" "$@"
  fi
done

ros_distro="${ROS_DISTRO:-}"
if [[ -z "$ros_distro" ]]; then
  for candidate in jazzy humble; do
    if [[ -f "/opt/ros/$candidate/setup.bash" ]]; then
      ros_distro="$candidate"
      break
    fi
  done
fi
ros_setup="/opt/ros/${ros_distro:-jazzy}/setup.bash"
if [[ ! -f "$ros_setup" ]]; then
  printf 'ROS setup not found: %s\nInstall ROS 2 Jazzy or Humble, or select an installed distro with ROS_DISTRO.\n' "$ros_setup" >&2
  exit 2
fi
# ROS setup scripts may reference unset variables; enable nounset afterwards.
source "$ros_setup"
set -u

exec /usr/bin/python3 "$script_dir/render_caster_recording.py" "$@"
