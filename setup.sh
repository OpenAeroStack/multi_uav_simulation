#!/bin/bash
# Portable setup script for multi_uav_sim
# Source this before running anything: source setup.sh

# ============================================================
# CONFIGURE THIS FOR YOUR MACHINE:
# Examples:
#   export ARDUPILOT_HOME="$HOME/ardupilot"
#   export ARDUPILOT_HOME="/opt/ardupilot"
#   export ARDUPILOT_HOME="/media/user/drive-uuid/ardupilot"
# REMOVED: pointed at a directory that does not exist on this machine:
# export ARDUPILOT_HOME="$HOME/ardu_ws/src/ardupilot"
# ADDED: the built ArduPilot tree actually lives here:
export ARDUPILOT_HOME="$HOME/ardu_ws/src/ardupilot"
# ============================================================

export PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ":${GAZEBO_MODEL_PATH:-}:" != *":$PROJECT_DIR/models:"* ]]; then
  export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
fi

export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:$PROJECT_DIR:$PROJECT_DIR/worlds:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
# Stock plugins, then ours: the obstacle raycaster (built into install/) and the
# ArduPilot FDM plugin that every iris_*_netns model loads.
export GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:/opt/ros/humble/lib:$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH:-}
export LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}

if [[ ! -d "$ARDUPILOT_HOME" ]]; then
  echo "ERROR: ARDUPILOT_HOME not found at: $ARDUPILOT_HOME"
  echo "Edit setup.sh and set ARDUPILOT_HOME to your ardupilot path."
  exit 1
fi

echo "ArduPilot found at: $ARDUPILOT_HOME"
echo "Project dir   : $PROJECT_DIR"
echo "Gazebo models : $GAZEBO_MODEL_PATH"

