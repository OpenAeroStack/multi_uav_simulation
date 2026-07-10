#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source portable setup
source "$PROJECT_DIR/setup.sh"

# ADDED: the obstacle raycast plugin (libgazebo_obstacle_plugin.so) lives in
# the colcon install tree, so Gazebo needs it on the plugin path
WS_INSTALL="$(cd "$PROJECT_DIR/../.." && pwd)/install"
export GAZEBO_PLUGIN_PATH="$WS_INSTALL/multi_uav_gazebo_plugins/lib:$GAZEBO_PLUGIN_PATH"

echo "=== Killing any previous SITL/Gazebo instances ==="
pkill -f arducopter 2>/dev/null || true
pkill -f gzserver  2>/dev/null || true
pkill -f gzclient  2>/dev/null || true
sleep 5
echo "=== Cleanup done ==="

if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  echo "ERROR: ARDUPILOT_HOME is not set."
  echo "Run: export ARDUPILOT_HOME=/path/to/ardupilot"
  exit 1
fi

# REMOVED: plain world without the obstacle raycast plugin / wall:
# WORLD_PATH="$PROJECT_DIR/worlds/multi_uav.world"
# ADDED: world with the obstacle raycast plugin and the wall obstacle
WORLD_PATH="$PROJECT_DIR/worlds/multi_uav_plugin.world"
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

echo "=== Launching Gazebo with 3 UAVs ==="
gazebo --verbose "$WORLD_PATH" &
GAZEBO_PID=$!

echo "=== Waiting for Gazebo to fully load (20 seconds) ==="
sleep 20

# REMOVED: all 3 SITL instances ran from the same directory (~), fighting
# over a single eeprom.bin — which is also root-owned on this machine —
# causing "Arm: Param storage failed" on every arm attempt:
# cd ~/  # run from home so tlog files go to ~/
# ADDED: each instance gets its own working directory so eeprom.bin and
# logs are per-UAV
echo "=== Launching ArduCopter SITL instance 0 (UAV1) ==="
mkdir -p /tmp/sitl_uav1
(cd /tmp/sitl_uav1 && $BINARY --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I0 &)
sleep 5

echo "=== Launching ArduCopter SITL instance 1 (UAV2) ==="
mkdir -p /tmp/sitl_uav2
(cd /tmp/sitl_uav2 && $BINARY --model gazebo-iris --speedup 1 --sysid 2 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I1 &)
sleep 5

echo "=== Launching ArduCopter SITL instance 2 (UAV3) ==="
mkdir -p /tmp/sitl_uav3
(cd /tmp/sitl_uav3 && $BINARY --model gazebo-iris --speedup 1 --sysid 3 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I2 &)

echo ""
echo "=== All 3 SITL instances running ==="
echo "Connect to UAV1: cd ~ && mavproxy.py --master=tcp:127.0.0.1:5760 --logfile=~/mav1.tlog"
echo "Connect to UAV2: cd ~ && mavproxy.py --master=tcp:127.0.0.1:5770 --logfile=~/mav2.tlog"
echo "Connect to UAV3: cd ~ && mavproxy.py --master=tcp:127.0.0.1:5780 --logfile=~/mav3.tlog"

wait $GAZEBO_PID


#after running the launch script run the below command in new terminal to connect to STIL instance 
# mavproxy.py --master=tcp:127.0.0.1:5760 
# mavproxy.py --master=tcp:127.0.0.1:5770 
# mavproxy.py --master=tcp:127.0.0.1:5780 