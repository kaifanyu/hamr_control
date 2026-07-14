#!/bin/bash
# One-command VSLAM data-collection run.
#
#   ./run_slam_mapping.sh          (from anywhere; env is self-contained)
#
# Launches Gazebo + RViz + RTAB-Map (hamr_slam.launch.xml); the robot drives
# its waypoint loop and maps as it goes. Records an evaluation bag alongside.
# When you Ctrl+C the launch, everything is archived to
# ~/hamr_runs/slam_<timestamp>/ : the bag, the RTAB-Map database, and a
# textured terrain mesh exported from it.
# (no `set -u` here: ROS setup.bash scripts reference unset variables)
source /opt/ros/jazzy/setup.bash
source ~/Documents/hamr_control/install/setup.bash

RUN=~/hamr_runs/slam_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN"
echo "Run directory: $RUN"

# Evaluation topics (same set run_traj.sh records, for plot_traj etc.)
ros2 bag record -o "$RUN/bag" /hamr/odom /reference_trajectory /state_error /tf &
BAG_PID=$!

echo "Mapping... drive as long as you like, Ctrl+C to finish."
ros2 launch hamr_bringup hamr_slam.launch.xml

kill $BAG_PID 2>/dev/null
wait $BAG_PID 2>/dev/null

# The gz server can survive launch teardown; kill leftovers so the next run
# doesn't end up with two simulators merged on gz-transport.
pkill -9 -f "gz si[m] .*2\.5D_maze_vslam" 2>/dev/null

echo "Archiving SLAM database and exporting meshes..."
cp ~/.ros/rtabmap.db "$RUN/rtabmap.db"
# .obj: textured (best-looking, open in Blender - MeshLab's OBJ importer
#       has an assertion bug that crashes on these files)
rtabmap-export --mesh --texture --output_dir "$RUN" "$RUN/rtabmap.db"
mv "$RUN/rtabmap_mesh.obj" "$RUN/rtabmap_mesh_textured.obj" 2>/dev/null
# .ply: vertex-colored (opens fine in MeshLab)
rtabmap-export --mesh --output_dir "$RUN" "$RUN/rtabmap.db"

echo ""
echo "Done. Artifacts in $RUN :"
ls -lh "$RUN"
echo ""
echo "View mesh:      meshlab $RUN/rtabmap_mesh.ply   (run from a normal"
echo "                terminal, NOT the VSCode one - snap env breaks GUIs;"
echo "                the textured .obj is for Blender)"
echo "Inspect graph:  rtabmap-databaseViewer $RUN/rtabmap.db"
echo "Plot tracking:  ros2 run hamr_control plot_traj $RUN/bag -o $RUN/traj.png"
