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

UAV1_SIM_ADDRESS="10.42.1.1"
UAV2_SIM_ADDRESS="10.42.2.1"
UAV3_SIM_ADDRESS="10.42.3.1"

UAV1_MAVLINK_HOST="${UAV1_MAVLINK_HOST:-10.42.1.2}"
UAV2_MAVLINK_HOST="${UAV2_MAVLINK_HOST:-10.42.2.2}"
UAV3_MAVLINK_HOST="${UAV3_MAVLINK_HOST:-10.42.3.2}"

UAV_SIM_ADDRS=("$UAV1_SIM_ADDRESS" "$UAV2_SIM_ADDRESS" "$UAV3_SIM_ADDRESS")
UAV_MAV_HOSTS=("$UAV1_MAVLINK_HOST" "$UAV2_MAVLINK_HOST" "$UAV3_MAVLINK_HOST")
UAV_SYSIDS=(1 2 3)
UAV_INSTANCES=(0 1 2)
UAV_MAV_PORTS=(5760 5770 5780)
SITL_PIDS=()

cleanup_children() {
  if [[ ${#SITL_PIDS[@]} -gt 0 ]]; then
    echo "=== Stopping SITL and Bridge child processes ==="
    kill "${SITL_PIDS[@]}" 2>/dev/null || true
  fi
  
  echo "=== Stopping NS-3 ==="
  # Force kill the ns-3 process with sudo. 
  # IMPORTANT: Change 'your_ns3_script_name' to the actual name of your compiled ns-3 file!
  sudo pkill -f three_uav_tapbridge_wifi 2>/dev/null || true
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
  
  # Calculate dynamic IPs and Ports for this specific drone
  BRIDGE_IP="10.42.${uav_num}.1"
  NS_IP="10.42.${uav_num}.2"
  FDM_PORT_IN="90${idx}2"   # 9002, 9012, 9022
  FDM_PORT_OUT="90${idx}3"  # 9003, 9013, 9023

  echo "=== Launching Two-Way Mirrors for UAV${uav_num} ==="
  # 1. Host Relay (Catches motors from bridge, hands to Gazebo)
  socat UDP4-LISTEN:${FDM_PORT_IN},bind=${BRIDGE_IP},reuseaddr,fork UDP4:127.0.0.1:${FDM_PORT_IN} &
  SITL_PIDS+=("$!")
  
  # 2. Namespace Relay (Catches physics from bridge, hands to ArduPilot)
  sudo ip netns exec "uav${uav_num}" socat UDP4-LISTEN:${FDM_PORT_OUT},bind=${NS_IP},reuseaddr,fork UDP4:127.0.0.1:${FDM_PORT_OUT} &
  SITL_PIDS+=("$!")

  echo "=== Launching ArduCopter SITL for UAV${uav_num} ==="
  # Notice --sim-address is now pointing to the Bridge IP so it hits the mirror!
  sudo ip netns exec "uav${uav_num}" "$BINARY" --model gazebo-iris --speedup 1 --sysid "${UAV_SYSIDS[$idx]}" \
    --defaults "$DEFAULTS" \
    --sim-address="${BRIDGE_IP}" -I"${UAV_INSTANCES[$idx]}" &
  SITL_PIDS+=("$!")
  sleep 5
  
  echo "=== Launching MAVProxy Router for UAV${uav_num} ==="
  sudo ip netns exec "uav${uav_num}" sudo -u ubuntu /home/ubuntu/.local/bin/mavproxy.py \
    --master=tcp:127.0.0.1:${UAV_MAV_PORTS[$idx]} \
    --out=tcpin:0.0.0.0:14550 \
    --daemon > /home/ubuntu/mavproxy_uav${uav_num}.log 2>&1 &
  SITL_PIDS+=("$!")
done

echo ""
echo "=== All 3 SITL instances running ==="
# Notice we hardcoded port 14550 here because all three daemons use it!
echo "Manual Connection to UAV1: mavproxy.py --master=tcp:${UAV_MAV_HOSTS[0]}:14550"
echo "Manual Connection to UAV2: mavproxy.py --master=tcp:${UAV_MAV_HOSTS[1]}:14550"
echo "Manual Connection to UAV3: mavproxy.py --master=tcp:${UAV_MAV_HOSTS[2]}:14550"

echo "=== Launching ROS 2 to NS-3 UDP Bridge ==="
# Adjust the path to wherever you saved penalty_bridge.py
# If it's a built ROS 2 node, use: ros2 run your_package penalty_bridge &
python3 "$PROJECT_DIR/scripts/penalty_bridge.py" &
BRIDGE_PID=$!
SITL_PIDS+=("$BRIDGE_PID") # Add to cleanup array

# Give the bridge a second to open port 5555
sleep 2 

echo "=== Launching NS-3 Simulation ==="
# Adjust the path to your ns-3 installation directory
NS3_DIR="/home/ubuntu/ns3-workspace/ns-3-dev"
cd "$NS3_DIR"

# Launch NS-3. It requires sudo to bind to the TAP interfaces
# Replace "your_ns3_script_name" with the actual name of your compiled C++ file
./ns3 run "scratch/three_uav_tapbridge_wifi" --enable-sudo &
NS3_PID=$!

# We don't add NS3_PID to SITL_PIDS because sudo processes often ignore 
# standard kill signals from a non-root script. We'll handle it in cleanup.

###################################################################
###################################################################
###################################################################

# Autonomous missions could be changed by changing location of the python script file






wait $GAZEBO_PID
