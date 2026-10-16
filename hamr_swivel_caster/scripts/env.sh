#!/usr/bin/env bash
# Source in a simulation terminal; use Ubuntu Python for ROS rather than Conda.
export HAMR_SWIVEL_CASTER_WS="${HAMR_SWIVEL_CASTER_WS:-$HOME/hamr_swivel_caster_ws}"
source /opt/ros/jazzy/setup.bash
if [[ ! -d /opt/ros/jazzy/share/ros_gz_sim && -f "$HOME/.cache/hamr-gazebo-root/env.sh" ]]; then
  source "$HOME/.cache/hamr-gazebo-root/env.sh"
fi
if [[ -f "$HAMR_SWIVEL_CASTER_WS/install/setup.bash" ]]; then
  source "$HAMR_SWIVEL_CASTER_WS/install/setup.bash"
fi
export PATH="/opt/ros/jazzy/bin:/usr/bin:/bin:$PATH"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-89}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST"
export GZ_PARTITION="${GZ_PARTITION:-hamr_swivel_caster_$USER}"
if [[ -d /mnt/wslg ]]; then
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi
hash -r
