#!/bin/bash
# Portable setup script for multi_uav_sim
# Source this before running anything: source setup.sh

# ============================================================
# CONFIGURE THIS FOR YOUR MACHINE:
# Examples:
#   export ARDUPILOT_HOME="$HOME/ardupilot"
#   export ARDUPILOT_HOME="/opt/ardupilot"
#   export ARDUPILOT_HOME="/media/user/drive-uuid/ardupilot"
export ARDUPILOT_HOME="/media/randilsk/eeca64c8-e2c1-4af0-b6c4-f55dd0394558/ubuntu/ardupilot"
# ============================================================

export PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ":${GAZEBO_MODEL_PATH:-}:" != *":$PROJECT_DIR/models:"* ]]; then
  export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
fi

export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

if [[ ! -d "$ARDUPILOT_HOME" ]]; then
  echo "ERROR: ARDUPILOT_HOME not found at: $ARDUPILOT_HOME"
  echo "Edit setup.sh and set ARDUPILOT_HOME to your ardupilot path."
  exit 1
fi

echo "ArduPilot found at: $ARDUPILOT_HOME"
echo "Project dir   : $PROJECT_DIR"
echo "Gazebo models : $GAZEBO_MODEL_PATH"

# ROS 2
set +u
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  source "$HOME/ros2_ws/install/setup.bash"
fi
set -u

if [[ -f "$HOME/ros2_ws/install/setup.bash" ]]; then
  set +u  # colcon setup scripts use unbound variables
  source "$HOME/ros2_ws/install/setup.bash"
  set -u
fi