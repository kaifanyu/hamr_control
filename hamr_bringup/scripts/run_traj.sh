#!/bin/bash
source ~/Documents/hamr_control/install/setup.bash

BAG=~/hamr_runs/run_$(date +%Y%m%d_%H%M%S)
mkdir -p ~/hamr_runs

echo "Recording bag to: $BAG"
ros2 bag record /hamr/odom /reference_trajectory /state_error /tf -o "$BAG" &
BAG_PID=$!

echo "Running trajectory... (Ctrl+C to stop)"
ros2 run reference_trajectory waypoint_traj_simple --ros-args -p use_sim_time:=true

kill $BAG_PID
wait $BAG_PID 2>/dev/null

echo "Generating plot..."
PLOT="${BAG}.png"
ros2 run hamr_control plot_traj "$BAG" -o "$PLOT"
echo "Plot saved to: $PLOT"
