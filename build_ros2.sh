#!/bin/bash
# build_ros2.sh
# -------------
# Builds the uav_controller ROS2 package inside multi_uav_sim/ros2/
# Run this once after cloning, and again after any code changes.
#
# Usage:
#   bash build_ros2.sh
#
# After building, every new terminal needs:
#   source ~/FYP/multi_uav_sim/ros2/install/setup.bash

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_DIR="$SCRIPT_DIR/ros2"

echo "=== Sourcing ROS2 Humble ==="
source /opt/ros/humble/setup.bash

echo "=== Sourcing ardu_ws (for ardupilot_msgs) ==="
if [ -f "$HOME/ardu_ws/install/setup.bash" ]; then
    source "$HOME/ardu_ws/install/setup.bash"
    echo "    ardu_ws sourced OK"
else
    echo "    WARNING: ~/ardu_ws/install/setup.bash not found"
    echo "    Make sure ardu_ws is built first"
fi

echo "=== Building uav_controller ==="
cd "$ROS2_DIR"
colcon build --packages-select uav_controller --symlink-install

echo ""
echo "=== Build complete! ==="
echo ""
echo "Add this to ~/.bashrc so every terminal has it:"
echo "   source $ROS2_DIR/install/setup.bash"
echo ""
echo "Or source it manually in each terminal:"
echo "   source $ROS2_DIR/install/setup.bash"
echo ""
echo "Then run:"
echo "   ros2 run uav_controller drone_bridge"
echo "   ros2 run uav_controller takeoff_mission"
