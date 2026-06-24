#!/bin/bash
set -euo pipefail
set -x

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

echo "=== Pre-flight cleanup: stopping old Gazebo/SITL/bridges ==="
pkill -9 -f gzserver        2>/dev/null || true
pkill -9 -f gzclient        2>/dev/null || true
pkill -9 -f gazebo          2>/dev/null || true

# SITL may be running under sudo (root-owned) when using netns.
sudo pkill -9 -f 'ip netns exec' 2>/dev/null || true
sudo pkill -9 -f arducopter      2>/dev/null || true

# Kill any micro_ros_agent running inside gcsns
sudo ip netns exec gcsns pkill -9 -f micro_ros_agent 2>/dev/null || true
pkill -9 -f micro_ros_agent 2>/dev/null || true
pkill -9 -f drone_bridge    2>/dev/null || true

# Clean up root-owned SITL scratch dirs from any previous netns run
for i in 1 2 3; do
  sudo rm -rf "/tmp/sitl_uav${i}"
done
sleep 3

AUTO_SETUP_NETNS="${AUTO_SETUP_NETNS:-1}"
NS3_HOME="${NS3_HOME:-}"
NS3_PID_FILE="/tmp/ns3_pid.txt"
SYNC_NS3_SCRATCH="${SYNC_NS3_SCRATCH:-1}"

# NS-3 scenario runtime knobs
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
  local newly_created=0
  if [[ ! -d "$NS3_HOME/scratch/three-uav" ]]; then
    newly_created=1
  fi
  mkdir -p "$NS3_HOME/scratch/three-uav"
  cp "$PROJECT_DIR/ns3/CMakeLists.txt" "$NS3_HOME/scratch/three-uav/CMakeLists.txt"
  cp "$PROJECT_DIR/ns3/three_uav_tapbridge_rt.cc" "$NS3_HOME/scratch/three-uav/three_uav_tapbridge_rt.cc"

  if [[ "$newly_created" == "1" ]]; then
    echo "=== New scratch directory — reconfiguring NS-3 ==="
    ( cd "$NS3_HOME" && ./ns3 configure --enable-examples --enable-tests 2>&1 | tail -5 )
  fi

  echo "=== Building NS-3 target: three-uav ==="
  ( cd "$NS3_HOME" && ./ns3 build three-uav )
}

wait_for_tcp_port() {
  local host="$1"
  local port="$2"
  local timeout_sec="$3"
  local start_ts
  start_ts=$(date +%s)
  while true; do
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

safe_source() {
  local file="$1"
  if [[ ! -f "$file" ]]; then
    return 1
  fi
  set +u
  source "$file"
  set -u
}

# 1) Namespace/TAP setup
echo "=== [1/8] Setting up namespaces and TAP devices (SHARED_SUBNET=1) ==="
sudo -E SHARED_SUBNET=1 "$PROJECT_DIR/scripts/setup_netns_tap.sh"

# 2) Start NS-3 TapBridge real-time scenario in background
echo "=== [2/8] Starting NS-3 TapBridge scenario in background ==="
resolve_ns3_home
if [[ ! -d "$NS3_HOME" ]]; then
  echo "ERROR: NS3_HOME does not exist: $NS3_HOME"
  exit 1
fi

sync_ns3_scratch_program

echo "NS-3 outputs: anim=$NS3_ANIM_FILE flowmon=$NS3_FLOWMON_XML duration=${NS3_SIM_DURATION_SEC}s"
(
  set +x
  set -euo pipefail
  cd "$NS3_HOME"
  exec sudo ./build/scratch/three-uav/ns3.38-three-uav-default \
    --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
    --simDurationSec=$NS3_SIM_DURATION_SEC \
    --delayMs=$NS3_DELAY_MS \
    --lossRate=$NS3_LOSS_RATE \
    --enableFlowMonitor=$NS3_ENABLE_FLOWMON \
    --animFile=$NS3_ANIM_FILE \
    --flowmonXml=$NS3_FLOWMON_XML
) >/tmp/ns3_stdout.log 2>&1 &
NS3_PID=$!
echo "$NS3_PID" > "$NS3_PID_FILE"
echo "NS-3 PID: $NS3_PID (saved to $NS3_PID_FILE)"

# 3) Wait for NS-3 to bring TAP devices UP
echo "=== [3/8] Waiting for NS-3 TAP devices to come UP ==="
for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
  echo -n "  Waiting for $tap..."
  deadline=$(( $(date +%s) + 60 ))
  while true; do
    if ip link show "$tap" 2>/dev/null | grep -q 'state UP'; then
      echo " UP"
      break
    fi
    if (( $(date +%s) >= deadline )); then
      echo " TIMEOUT — NS-3 may have crashed"
      echo "  Check /tmp/ns3_stdout.log for errors"
      exit 1
    fi
    sleep 0.5
  done
done

# ── Source ROS2 ─────────────────────────────────────────────────────────────
safe_source /opt/ros/humble/setup.bash || {
  echo "ERROR: ROS2 Humble setup not found at /opt/ros/humble/setup.bash"
  exit 1
}

if [[ -f "$HOME/ardu_ws/install/setup.bash" ]]; then
  safe_source "$HOME/ardu_ws/install/setup.bash"
else
  echo "WARNING: $HOME/ardu_ws/install/setup.bash not found; skipping ardu_ws overlay"
fi

safe_source "$PROJECT_DIR/ros2/install/setup.bash" 2>/dev/null || {
  echo "ERROR: ROS2 package not built. Run: bash build_ros2.sh"
  exit 1
}

# 4) Launch Gazebo Classic
echo "=== [4/8] Launching Gazebo Classic with Airport World ==="
WORLD_PATH="$PROJECT_DIR/worlds/airport_3uav.world"
if ! command -v gazebo >/dev/null 2>&1; then
  echo "ERROR: gazebo command not found. Install Gazebo Classic 11."
  exit 1
fi
if [[ ! -f "$WORLD_PATH" ]]; then
  echo "ERROR: world file not found: $WORLD_PATH"
  exit 1
fi

gazebo --verbose "$WORLD_PATH" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Sleeping 15 seconds for Gazebo to start..."
sleep 15

# 5) Start micro_ros_agent inside gcsns
echo "=== [5/8] Starting 3 micro_ros_agent instances inside gcsns ==="
if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "ERROR: ROS2 package 'micro_ros_agent' not found."
  exit 1
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

for i in 1 2 3; do
  PORT=$((2018+i))
  echo "  - micro_ros_agent udp4 port ${PORT} for uav${i} — inside gcsns"
  sudo -E ip netns exec gcsns \
    su - "${SUDO_USER:-$USER}" -s /bin/bash -c "
      source /opt/ros/humble/setup.bash
      source \"$HOME/ardu_ws/install/setup.bash\" 2>/dev/null || true
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      ros2 run micro_ros_agent micro_ros_agent udp4 \
        --port ${PORT} \
        --ros-args -r __ns:=/uav${i}
    " &
  sleep 1
done
echo "  Agents started inside gcsns. Waiting 3s..."
sleep 3

# 6) Start arducopter in ROOT namespace
echo "=== [6/8] Starting 3 ArduCopter SITL instances (root namespace) ==="
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
BASE_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"

for i in 1 2 3; do
  echo "  - [UAV${i}] Starting arducopter (root ns, instance $((i-1)))"
  UAV_DEFAULTS="$BASE_DEFAULTS,$PROJECT_DIR/params/uav${i}_dds.parm"
  SITL_LOG="/tmp/uav${i}_sitl.log"
  SITL_DIR="/tmp/sitl_uav${i}"
  mkdir -p "$SITL_DIR"
  sudo rm -f "$SITL_DIR/eeprom.bin"
  sudo rm -f "$SITL_DIR"/*.bin 2>/dev/null || true
  rm -f "$SITL_LOG" 2>/dev/null || true

  ( cd "$SITL_DIR" && exec "$BINARY" \
      --model gazebo-iris \
      --speedup 1 \
      --sysid "${i}" \
      --instance $((i-1)) \
      --defaults "$UAV_DEFAULTS" \
      --sim-address 127.0.0.1 \
      --serial1 "udpclient:10.42.0.10:$((2018+i))" \
      --home 37.523640,-122.255122,1.7,0 \
      ${SITL_EXTRA_ARGS:-} \
  ) >"$SITL_LOG" 2>&1 &
  SITL_PID=$!
  echo "    SITL PID: $SITL_PID (dir: $SITL_DIR, log: $SITL_LOG)"
  sleep 3

  if ! kill -0 "$SITL_PID" 2>/dev/null; then
    echo "ERROR: UAV${i} SITL exited immediately. Last 120 lines of $SITL_LOG:"
    tail -n 120 "$SITL_LOG" || true
    exit 1
  fi
done

echo "=== [6.5/8] Waiting for MAVLink TCP ports to be reachable ==="
for i in 1 2 3; do
  mavlink_host="127.0.0.1"
  mavlink_port=$((5760 + 10 * (i - 1)))
  echo "  - [UAV${i}] waiting for tcp:${mavlink_host}:${mavlink_port}"
  wait_for_tcp_port "$mavlink_host" "$mavlink_port" 90
done

# 7) Start drone_bridge nodes, then launch airport mission
echo "=== [7/8] Starting 3 drone_bridge nodes, then airport_mission ==="

# UAV1 Alt: 40m, UAV2 Alt: 35m, UAV3 Alt: 40m
declare -A ALTS
ALTS[1]=40.0
ALTS[2]=35.0
ALTS[3]=40.0

for i in 1 2 3; do
  mavlink_host="127.0.0.1"
  mavlink_port=$((5760 + 10 * (i - 1)))
  alt=${ALTS[$i]}
  echo "  - [UAV${i}] drone_bridge mavlink_host=${mavlink_host} mavlink_port=${mavlink_port} alt=${alt}"
  ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=${i} \
    -p mavlink_host:=${mavlink_host} \
    -p mavlink_port:=${mavlink_port} \
    -p takeoff_altitude:=${alt} &
  sleep 1
done

sleep 3
ros2 run uav_controller airport_mission &
MISSION_PID=$!
echo "Airport Mission PID: $MISSION_PID"

# 8) Wait for NS-3 to finish
echo "=== [8/8] Waiting for NS-3 (PID $NS3_PID) ==="
wait "$NS3_PID"
