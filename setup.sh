#!/bin/bash
# Portable setup script for multi_uav_sim
# Source this before running anything: source setup.sh

# ============================================================
# ARDUPILOT_HOME selection order:
# 1) Existing ARDUPILOT_HOME from your shell, if set
# 2) Fallback to "$HOME/ardupilot"
#
# Override example:
#   export ARDUPILOT_HOME="/opt/ardupilot"
#   source setup.sh
# ============================================================
export ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"

export PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ":${GAZEBO_MODEL_PATH:-}:" != *":$PROJECT_DIR/models:"* ]]; then
  export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
fi

export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""

if [[ ! -d "$ARDUPILOT_HOME" ]]; then
  echo "ERROR: ARDUPILOT_HOME not found at: $ARDUPILOT_HOME"
  echo "Set ARDUPILOT_HOME to your ArduPilot path, then source setup.sh again."
  echo "Example: export ARDUPILOT_HOME=\"$HOME/ardupilot\""
  exit 1
fi

echo "ArduPilot found at: $ARDUPILOT_HOME"
echo "Project dir   : $PROJECT_DIR"
echo "Gazebo models : $GAZEBO_MODEL_PATH"
echo "Tip           : For namespace mode, run scripts/setup_netns_tap.sh"