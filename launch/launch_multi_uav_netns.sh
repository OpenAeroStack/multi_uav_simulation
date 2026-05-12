#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

AUTO_SETUP_NETNS="${AUTO_SETUP_NETNS:-1}"
NS3_HOME="${NS3_HOME:-}"
NS3_PID_FILE="/tmp/ns3_pid.txt"
SYNC_NS3_SCRATCH="${SYNC_NS3_SCRATCH:-1}"

# NS-3 scenario runtime knobs (used when starting the three-uav TapBridge program)
NS3_SIM_DURATION_SEC="${NS3_SIM_DURATION_SEC:-0}"
NS3_ANIM_FILE="${NS3_ANIM_FILE:-/tmp/three_uav_anim.xml}"
NS3_FLOWMON_XML="${NS3_FLOWMON_XML:-/tmp/three_uav_flowmon.xml}"
NS3_DELAY_MS="${NS3_DELAY_MS:-20}"
NS3_LOSS_RATE="${NS3_LOSS_RATE:-0}"
NS3_ENABLE_FLOWMON="${NS3_ENABLE_FLOWMON:-1}"

resolve_ns3_home() {
  if [[ -n "$NS3_HOME" ]]; then
    return 0
  fi

  # Default to the newest matching ns-3 allinone install under $HOME.
  # Example: $HOME/ns-allinone-3.38/ns-3.38
  local candidate
  candidate=$(ls -d "$HOME"/ns-allinone-3.*/ns-3.* 2>/dev/null | sort -V | tail -n 1 || true)
  if [[ -z "$candidate" ]]; then
    echo "ERROR: NS3_HOME is not set and no default ns-3 install found under $HOME/ns-allinone-3.xx/ns-3.xx"
    exit 1
  fi
  NS3_HOME="$candidate"
}

sync_ns3_scratch_program() {
  if [[ "$SYNC_NS3_SCRATCH" != "1" ]]; then
    return 0
  fi

  echo "=== Syncing three-uav scratch program into NS-3 tree ==="
  mkdir -p "$NS3_HOME/scratch/three-uav"
  cp "$PROJECT_DIR/ns3/CMakeLists.txt" "$NS3_HOME/scratch/three-uav/CMakeLists.txt"
  cp "$PROJECT_DIR/ns3/three_uav_tapbridge_rt.cc" "$NS3_HOME/scratch/three-uav/three_uav_tapbridge_rt.cc"

  echo "=== Building NS-3 target: three-uav ==="
  (cd "$NS3_HOME" && ./ns3 build three-uav)
}

wait_for_tcp_port() {
  local host="$1"
  local port="$2"
  local timeout_sec="$3"
  local start_ts
  start_ts=$(date +%s)
  while true; do
    # /dev/tcp is supported by bash.
    if (echo >"/dev/tcp/$host/$port") >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "ERROR: Timed out waiting for $host:$port"
      return 1
    fi
    sleep 1
  done
}

cleanup_ns3() {
  if [[ -f "$NS3_PID_FILE" ]]; then
    local pid
    pid=$(cat "$NS3_PID_FILE" 2>/dev/null || true)
    if [[ "$pid" =~ ^[0-9]+$ ]]; then
      echo "=== Stopping NS-3 (PID $pid) ==="
      sudo kill "$pid" 2>/dev/null || true
    fi
    rm -f "$NS3_PID_FILE" || true
  fi
}
trap cleanup_ns3 EXIT INT TERM

# ─────────────────────────────────────────────────────────────────────────────
# Orchestration order (required):
#   1) setup netns + TAP (SHARED_SUBNET=1)
#   2) start NS-3 TapBridge scenario (background)
#   3) wait 3s for TAP devices
#   4) launch Gazebo (background) via ros2 launch
#   5) start arducopter in each namespace
#   6) start micro_ros_agent for each port
#   7) launch ROS2 mission node
#   8) wait on NS-3 PID
# ─────────────────────────────────────────────────────────────────────────────

# 1) Namespace/TAP setup (shared /24 so NS-3 can bridge UAVs together)
echo "=== [1/8] Setting up namespaces and TAP devices (SHARED_SUBNET=1) ==="
sudo -E SHARED_SUBNET=1 "$PROJECT_DIR/scripts/setup_netns_tap.sh"

# 2) Start NS-3 TapBridge real-time scenario in background (with sudo)
echo "=== [2/8] Starting NS-3 TapBridge scenario in background ==="
resolve_ns3_home
if [[ ! -d "$NS3_HOME" ]]; then
  echo "ERROR: NS3_HOME does not exist: $NS3_HOME"
  exit 1
fi

sync_ns3_scratch_program

echo "NS-3 outputs: anim=/tmp/three_uav_anim.xml flowmon=/tmp/three_uav_flowmon.xml duration=120s"
(
  set -euo pipefail
  cd "$NS3_HOME"
  ./ns3 run --enable-sudo "three-uav \
    --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
    --simDurationSec=120 \
    --animFile=/tmp/three_uav_anim.xml \
    --flowmonXml=/tmp/three_uav_flowmon.xml"
) &
NS3_PID=$!
echo "$NS3_PID" > "$NS3_PID_FILE"
echo "NS-3 PID: $NS3_PID (saved to $NS3_PID_FILE)"

# 3) Wait briefly for TAP devices
echo "=== [3/8] Waiting 3 seconds for TAP devices to be ready ==="
sleep 3

# ── Source ROS2 ─────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source "$PROJECT_DIR/ros2/install/setup.bash" 2>/dev/null || {
  echo "ERROR: ROS2 package not built. Run: bash build_ros2.sh"
  exit 1
}

# 4) Launch Gazebo via ros2 launch
echo "=== [4/8] Launching Gazebo via ardupilot_gz_bringup (background) ==="
ros2 launch ardupilot_gz_bringup iris_runway.launch.py &
GAZEBO_PID=$!
echo "Sleeping 6 seconds for Gazebo to start..."
sleep 6

# 5) Start arducopter in each namespace
echo "=== [5/8] Starting 3 ArduCopter SITL instances inside namespaces ==="
if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  echo "ERROR: ARDUPILOT_HOME not set. Check setup.sh"
  exit 1
fi
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
if [[ ! -x "$BINARY" ]]; then
  echo "ERROR: arducopter binary not found/executable: $BINARY"
  exit 1
fi

for i in 1 2 3; do
  echo "  - [UAV${i}] Starting arducopter in netns uav${i} (instance $((i-1)), uartC port $((2018+i)))"
  ip netns exec "uav${i}" "$BINARY" \
    --model gazebo-iris \
    --instance $((i-1)) \
    --uartC udp:10.42.0.1:$((2018+i)) \
    --home -35.363261,149.165230,584,0 &
  sleep 1
done

# 6) Start micro_ros_agent UDP4 on per-UAV ports
echo "=== [6/8] Starting 3 micro_ros_agent instances (UDP4 ports 2019-2021) ==="
for i in 1 2 3; do
  echo "  - micro_ros_agent udp4 --port $((2018+i))"
  ros2 run micro_ros_agent micro_ros_agent udp4 --port $((2018+i)) &
  sleep 1
done

# 7) Launch ROS2 mission node
echo "=== [7/8] Launching ROS2 mission node (multi_mission) ==="
sleep 3
ros2 run uav_controller multi_mission &
MISSION_PID=$!
echo "Mission PID: $MISSION_PID"

# 8) Wait for NS-3 to finish
echo "=== [8/8] Waiting for NS-3 (PID $NS3_PID) ==="
wait "$NS3_PID"
