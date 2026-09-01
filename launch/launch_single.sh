#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source portable setup
source "$PROJECT_DIR/setup.sh"
source /opt/ros/humble/setup.bash

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
gazebo --verbose "$WORLD_PATH" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!

echo "=== Waiting for Gazebo to fully load (20 seconds) ==="
sleep 20

echo "=== Launching ArduCopter SITL instance 0 (UAV1) ==="
cd ~/
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I0 &

echo ""
echo "=== SITL instance running ==="
echo "Connect to UAV1: cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog"

wait $GAZEBO_PID

#after running the launch script run the below command in new terminal to connect to STIL instance 
# mavproxy.py --master=tcp:127.0.0.1:5760 