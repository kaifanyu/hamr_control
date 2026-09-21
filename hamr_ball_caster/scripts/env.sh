#!/usr/bin/env bash
# Source this file in every simulation terminal. ROS Jazzy uses Ubuntu Python,
# not a Conda interpreter (this workstation starts with Conda on PATH).
export HAMR_BALL_CASTER_WS="${HAMR_BALL_CASTER_WS:-$HOME/hamr_ball_caster_ws}"
source /opt/ros/jazzy/setup.bash
# This machine has a validated, isolated Gazebo installation prepared during
# development. Prefer the regular apt installation when it exists.
if [[ ! -d /opt/ros/jazzy/share/ros_gz_sim && -f "$HOME/.cache/hamr-gazebo-root/env.sh" ]]; then
  source "$HOME/.cache/hamr-gazebo-root/env.sh"
fi
if [[ -f "$HAMR_BALL_CASTER_WS/install/setup.bash" ]]; then
  source "$HAMR_BALL_CASTER_WS/install/setup.bash"
fi
export PATH="/opt/ros/jazzy/bin:/usr/bin:/bin:$PATH"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-88}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-LOCALHOST}"
export GZ_PARTITION="${GZ_PARTITION:-hamr_ball_caster_$USER}"
if [[ -d /mnt/wslg ]]; then
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi
hash -r
