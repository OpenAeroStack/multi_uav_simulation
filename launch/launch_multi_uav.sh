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
  echo "Run: export ARDUPILOT_HOME=/path/to/ardupilot"
  exit 1
fi

WORLD_PATH="$PROJECT_DIR/worlds/multi_uav.world"
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
GAZEBO_WAIT_SECONDS="${GAZEBO_WAIT_SECONDS:-20}"

UAV1_SIM_ADDRESS="${UAV1_SIM_ADDRESS:-127.0.0.1}"
UAV2_SIM_ADDRESS="${UAV2_SIM_ADDRESS:-127.0.0.1}"
UAV3_SIM_ADDRESS="${UAV3_SIM_ADDRESS:-127.0.0.1}"

UAV1_MAVLINK_HOST="${UAV1_MAVLINK_HOST:-127.0.0.1}"
UAV2_MAVLINK_HOST="${UAV2_MAVLINK_HOST:-127.0.0.1}"
UAV3_MAVLINK_HOST="${UAV3_MAVLINK_HOST:-127.0.0.1}"

UAV_SIM_ADDRS=("$UAV1_SIM_ADDRESS" "$UAV2_SIM_ADDRESS" "$UAV3_SIM_ADDRESS")
UAV_MAV_HOSTS=("$UAV1_MAVLINK_HOST" "$UAV2_MAVLINK_HOST" "$UAV3_MAVLINK_HOST")
UAV_SYSIDS=(1 2 3)
UAV_INSTANCES=(0 1 2)
UAV_MAV_PORTS=(5760 5770 5780)
SITL_PIDS=()

cleanup_children() {
  if [[ ${#SITL_PIDS[@]} -gt 0 ]]; then
    echo "=== Stopping SITL child processes ==="
    kill "${SITL_PIDS[@]}" 2>/dev/null || true
  fi
}
trap cleanup_children EXIT INT TERM

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

echo "=== Waiting for Gazebo to fully load (${GAZEBO_WAIT_SECONDS}s) ==="
sleep "$GAZEBO_WAIT_SECONDS"

cd ~/
for idx in 0 1 2; do
  uav_num=$((idx + 1))
  echo "=== Launching ArduCopter SITL instance ${UAV_INSTANCES[$idx]} (UAV${uav_num}) ==="
  "$BINARY" --model gazebo-iris --speedup 1 --sysid "${UAV_SYSIDS[$idx]}" \
    --defaults "$DEFAULTS" \
    --sim-address="${UAV_SIM_ADDRS[$idx]}" -I"${UAV_INSTANCES[$idx]}" &
  SITL_PIDS+=("$!")
  sleep 5
done

echo ""
echo "=== All 3 SITL instances running ==="
echo "Connect to UAV1: cd ~ && mavproxy.py --master=tcp:${UAV_MAV_HOSTS[0]}:${UAV_MAV_PORTS[0]} --logfile=~/mav1.tlog"
echo "Connect to UAV2: cd ~ && mavproxy.py --master=tcp:${UAV_MAV_HOSTS[1]}:${UAV_MAV_PORTS[1]} --logfile=~/mav2.tlog"
echo "Connect to UAV3: cd ~ && mavproxy.py --master=tcp:${UAV_MAV_HOSTS[2]}:${UAV_MAV_PORTS[2]} --logfile=~/mav3.tlog"

wait $GAZEBO_PID