#!/usr/bin/env bash
# Share the existing caster workspace, ROS installation and Gazebo runtime.
terrain_script_directory="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$terrain_script_directory/../../hamr_swivel_caster/scripts/env.sh"
unset terrain_script_directory
