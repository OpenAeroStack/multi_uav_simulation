#!/bin/bash
# Portable setup script for multi_uav_sim
# Source this before running anything: source setup.sh

# ============================================================
# CONFIGURE THIS FOR YOUR MACHINE:
# Examples:
#   export ARDUPILOT_HOME="$HOME/ardupilot"
#   export ARDUPILOT_HOME="/opt/ardupilot"
#   export ARDUPILOT_HOME="/media/user/drive-uuid/ardupilot"

if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  if [[ -d "$HOME/ardu_ws/src/ardupilot" ]]; then
    ARDUPILOT_HOME="$HOME/ardu_ws/src/ardupilot"
  elif [[ -d "$HOME/ardupilot" ]]; then
    ARDUPILOT_HOME="$HOME/ardupilot"
  fi
fi

export ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardu_ws/src/ardupilot}"

export PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ":${GAZEBO_MODEL_PATH:-}:" != *":$PROJECT_DIR/models:"* ]]; then
  export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
fi

export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=~/simulation/small_city_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=~/simulation/small_city_gazebo:$GAZEBO_RESOURCE_PATH
if [[ ! -d "$ARDUPILOT_HOME" ]]; then
  echo "ERROR: ARDUPILOT_HOME not found at: $ARDUPILOT_HOME"
  echo "Set ARDUPILOT_HOME to your ArduPilot checkout path (e.g. export ARDUPILOT_HOME=\"$HOME/ardupilot\")"
  exit 1
fi

echo "ArduPilot found at: $ARDUPILOT_HOME"
echo "Project dir   : $PROJECT_DIR"
echo "Gazebo models : $GAZEBO_MODEL_PATH"