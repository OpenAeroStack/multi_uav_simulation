#!/bin/bash
# Portable setup script for multi_uav_sim
# Source this before running anything: source setup.sh

# ============================================================
# CONFIGURE THIS FOR YOUR MACHINE:
# ============================================================
# CONFIGURE THIS FOR YOUR MACHINE:
# Examples:
#   export ARDUPILOT_HOME="$HOME/ardupilot"                  # if installed in home directory
#   export ARDUPILOT_HOME="/opt/ardupilot"                   # if installed in /opt
#   export ARDUPILOT_HOME="/media/user/drive-uuid/ardupilot" # if on external drive (not recommended)

export ARDUPILOT_HOME="/media/randilsk/eeca64c8-e2c1-4af0-b6c4-f55dd0394558/ubuntu/ardupilot"

# ============================================================
export PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Only add to GAZEBO_MODEL_PATH if not already there
if [[ ":${GAZEBO_MODEL_PATH:-}:" != *":$PROJECT_DIR/models:"* ]]; then
  export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:${GAZEBO_MODEL_PATH:-}"
fi

export GAZEBO_MODEL_DATABASE_URI=""  #disable online model database to speed up Gazebo startup

# ArduPilot - set ARDUPILOT_HOME to wherever it is on this machine
if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  # Try common locations
  for candidate in \
    "$HOME/ardupilot" \
    "$HOME/src/ardupilot" \
    "/opt/ardupilot"; do
    if [[ -d "$candidate" ]]; then
      export ARDUPILOT_HOME="$candidate"
      break
    fi
  done
fi

if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  echo "WARNING: ARDUPILOT_HOME not found. Set it manually:"
  echo "  export ARDUPILOT_HOME=/path/to/ardupilot"
else
  echo "ArduPilot found at: $ARDUPILOT_HOME"
fi

echo "Project dir   : $PROJECT_DIR"
echo "Gazebo models : $GAZEBO_MODEL_PATH"