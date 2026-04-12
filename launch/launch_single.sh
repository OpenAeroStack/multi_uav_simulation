#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source portable setup
source "$PROJECT_DIR/setup.sh"

echo "=== Killing any previous SITL/Gazebo instances ==="
pkill -f arducopter 2>/dev/null || true
pkill -f gzserver  2>/dev/null || true
pkill -f gzclient  2>/dev/null || true
sleep 5
echo "=== Cleanup done ==="

if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  echo "ERROR: ARDUPILOT_HOME is not set."
  echo "Edit setup.sh and set ARDUPILOT_HOME to your ardupilot path."
  exit 1
fi

WORLD_PATH="$PROJECT_DIR/worlds/single_uav.world"
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
GAZEBO_WAIT_SECONDS="${GAZEBO_WAIT_SECONDS:-20}"
UAV1_SIM_ADDRESS="${UAV1_SIM_ADDRESS:-127.0.0.1}"
UAV1_MAVLINK_HOST="${UAV1_MAVLINK_HOST:-127.0.0.1}"
SITL_PID=""

cleanup_child() {
  if [[ -n "$SITL_PID" ]]; then
    echo "=== Stopping SITL child process ==="
    kill "$SITL_PID" 2>/dev/null || true
  fi
}
trap cleanup_child EXIT INT TERM

if ! command -v gazebo >/dev/null 2>&1; then
  echo "ERROR: gazebo not found."
  exit 1
fi

if [[ ! -f "$WORLD_PATH" ]]; then
  echo "ERROR: world file not found at $WORLD_PATH"
  exit 1
fi

echo "=== Building ArduCopter binary ==="
cd "$ARDUPILOT_HOME"
python3 modules/waf/waf-light build --target bin/arducopter
echo "=== Build done ==="

if [[ ! -x "$BINARY" ]]; then
  echo "ERROR: arducopter binary not found at $BINARY"
  exit 1
fi

echo "=== Launching Gazebo with 1 UAV ==="
gazebo --verbose "$WORLD_PATH" &
GAZEBO_PID=$!

echo "=== Waiting for Gazebo to fully load (${GAZEBO_WAIT_SECONDS}s) ==="
sleep "$GAZEBO_WAIT_SECONDS"

echo "=== Launching ArduCopter SITL instance 0 (UAV1) ==="
cd ~/
"$BINARY" --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults "$DEFAULTS" \
  --sim-address="$UAV1_SIM_ADDRESS" -I0 &
SITL_PID="$!"

echo ""
echo "=== SITL instance running ==="
echo "Connect to UAV1: cd ~ && mavproxy.py --master=tcp:${UAV1_MAVLINK_HOST}:5760 --logfile=~/mav1.tlog"

wait $GAZEBO_PID