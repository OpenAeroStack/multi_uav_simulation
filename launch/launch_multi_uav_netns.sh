#!/bin/bash
set -euo pipefail
set -x

safe_source() {
  # ROS 2 setup scripts commonly reference variables that may be unset.
  # This launcher uses 'set -u', so temporarily disable nounset while sourcing.
  local file="$1"
  if [[ ! -f "$file" ]]; then
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$file"
  set -u
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

safe_source "$PROJECT_DIR/setup.sh" || {
  echo "ERROR: Failed to source setup.sh at $PROJECT_DIR/setup.sh"
  exit 1
}

NS3_PID_FILE="/tmp/ns3_pid.txt"

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

# Call cleanup_ns3 at the very top to reap any stale process from last time
cleanup_ns3

# Set trap to clean up NS-3 on exit
trap cleanup_ns3 EXIT INT TERM

echo "=== Pre-flight cleanup: stopping old Gazebo/SITL/bridges ==="
pkill -9 -f gzserver        2>/dev/null || true
pkill -9 -f gzclient        2>/dev/null || true
pkill -9 -f gazebo          2>/dev/null || true

# SITL may be running under sudo (root-owned) when using netns.
sudo pkill -9 -f 'ip netns exec' 2>/dev/null || true
sudo pkill -9 -f arducopter      2>/dev/null || true
sudo pkill -9 -f 'ns3.38-three-uav-default' 2>/dev/null || true

# Kill any micro_ros_agent running inside gcsns
sudo ip netns exec gcsns pkill -9 -f micro_ros_agent 2>/dev/null || true
sudo ip netns exec gcsns pkill -9 -f fastdds 2>/dev/null || true
pkill -9 -f micro_ros_agent 2>/dev/null || true
pkill -9 -f fastdds 2>/dev/null || true
pkill -9 -f drone_bridge    2>/dev/null || true
sudo pkill -9 -f socat           2>/dev/null || true

# Clean up root-owned SITL scratch dirs from any previous netns run
for i in 1 2 3; do
  sudo rm -rf "/tmp/sitl_uav${i}"
done
sleep 3

AUTO_SETUP_NETNS="${AUTO_SETUP_NETNS:-1}"
NS3_HOME="${NS3_HOME:-}"
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

# NS3_SIM_DURATION_SEC=0 means "run forever" (no Simulator::Stop in the .cc).
# Do NOT override it to 120: SITL missions last longer than 2 minutes and
# killing NS-3 mid-flight tears down the TAP bridges for all UAVs.
# Set NS3_SIM_DURATION_SEC to a positive value only for bounded test runs.
echo "NS-3 outputs: anim=$NS3_ANIM_FILE flowmon=$NS3_FLOWMON_XML duration=${NS3_SIM_DURATION_SEC}s"
(
  set +x   # suppress xtrace — prevents "cd ..." noise bleeding into main script
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
# Use multi_uav.world (regular iris models with fdm_addr=127.0.0.1)
# because SITL runs in root namespace alongside Gazebo.
echo "=== [4/8] Launching Gazebo Classic (background) ==="
WORLD_PATH="$PROJECT_DIR/worlds/multi_uav.world"
if ! command -v gazebo >/dev/null 2>&1; then
  echo "ERROR: gazebo command not found. Install Gazebo Classic 11 (gazebo11)."
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

if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
  echo "ERROR: Gazebo exited immediately or failed to start. Check Gazebo logs/output."
  exit 1
fi


# 5) Start micro_ros_agent inside gcsns
#    gcsns has IP 10.42.0.10 on the NS-3 WiFi channel.
#    Running agents here means XRCE-DDS UDP frames FROM the SITL instances
#    (which use --serial1 udpclient:10.42.0.10:PORT) traverse the NS-3
#    simulated WiFi channel before reaching the agent.
echo "=== [5/8] Starting 3 micro_ros_agent instances inside gcsns (UDP4 ports 2019-2021) ==="
if ! ros2 pkg prefix micro_ros_agent >/dev/null 2>&1; then
  echo "ERROR: ROS2 package 'micro_ros_agent' not found in the current environment."
  echo "- If you built it in ~/ardu_ws, ensure ~/ardu_ws/install/setup.bash exists."
  echo "- Otherwise install it: sudo apt install ros-humble-micro-ros-agent"
  exit 1
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Verify gcsns exists (created by setup_netns_tap.sh)
if ! ip netns list | grep -q '^gcsns'; then
  echo "ERROR: namespace 'gcsns' not found — did setup_netns_tap.sh run successfully?"
  exit 1
fi

# Start Fast-DDS Discovery Server inside gcsns
echo "  - Starting Fast-DDS Discovery Server inside gcsns on 10.42.0.10:11811..."
sudo -E ip netns exec gcsns \
  su - "${SUDO_USER:-$USER}" -s /bin/bash -c "
    source /opt/ros/humble/setup.bash
    fastdds discovery -i 0 -l 10.42.0.10 -p 11811
  " >/tmp/discovery_server.log 2>&1 &

echo "  Waiting for Discovery Server to bind port 11811..."
deadline=$(( $(date +%s) + 30 ))
while ! sudo ip netns exec gcsns ss -ulnp | grep -q ":11811"; do
  (( $(date +%s) >= deadline )) && { echo "TIMEOUT waiting for discovery server port 11811"; exit 1; }
  sleep 0.5
done
echo "  Discovery Server ready"

for i in 1 2 3; do
  PORT=$((2018+i))
  echo "  - micro_ros_agent udp4 port ${PORT} for uav${i} (ns=/uav${i}) — inside gcsns"
  # Run agent inside gcsns so it binds to 10.42.0.10 on the NS-3 WiFi channel.
  # We use 'sudo ip netns exec gcsns' then drop back to OWNER_USER for the agent.
  sudo -E ip netns exec gcsns \
    su - "${SUDO_USER:-$USER}" -s /bin/bash -c "
      source /opt/ros/humble/setup.bash
      source \"$HOME/ardu_ws/install/setup.bash\" 2>/dev/null || true
      export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
      export ROS_DISCOVERY_SERVER=10.42.0.10:11811
      ros2 run micro_ros_agent micro_ros_agent udp4 \
        --port ${PORT} \
        --ros-args -r __ns:=/uav${i}
    " &
  sleep 3
done
echo "  Agents started inside gcsns. Waiting for UDP ports to be bound..."
for i in 1 2 3; do
  PORT=$((2018+i))
  echo "  Waiting for micro_ros_agent on port ${PORT}..."
  deadline=$(( $(date +%s) + 30 ))
  while ! sudo ip netns exec gcsns ss -ulnp | grep -q ":${PORT}"; do
    (( $(date +%s) >= deadline )) && { echo "TIMEOUT waiting for agent port ${PORT}"; exit 1; }
    sleep 0.5
  done
  echo "  Agent port ${PORT} ready"
done

# 6) Start 3 ArduCopter SITL instances (inside isolated namespaces)
echo "=== [6/8] Starting 3 ArduCopter SITL instances (inside isolated namespaces) ==="
if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  echo "ERROR: ARDUPILOT_HOME not set. Check setup.sh"
  exit 1
fi
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
if [[ ! -x "$BINARY" ]]; then
  echo "ERROR: arducopter binary not found/executable: $BINARY"
  exit 1
fi

BASE_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"

for i in 1 2 3; do
  NS="uav${i}"                        # Matches the patched setup script
  NS_IP="10.42.0.$((10 + i))"         # The IP inside the namespace (e.g., .11)
  ROOT_VETH_IP="10.42.0.$((200 + i))" # The IP on the root side (e.g., .201)
  
  echo "  - [UAV${i}] Starting arducopter (inside ${NS}, instance $((i-1)))"
  UAV_DEFAULTS="$BASE_DEFAULTS,$PROJECT_DIR/params/uav${i}_dds.parm"
  SITL_LOG="/tmp/uav${i}_sitl.log"
  SITL_DIR="/tmp/sitl_uav${i}"
  mkdir -p "$SITL_DIR"
  sudo rm -f "$SITL_DIR/eeprom.bin"
  sudo rm -f "$SITL_DIR"/*.bin 2>/dev/null || true
  rm -f "$SITL_LOG" 2>/dev/null || true

  SITL_IN_PORT=$((9003 + 10 * (i - 1)))
  MAV_PORT=$((5760 + 10 * (i - 1)))

  # 1. MAVLink Tunnel (TCP) - Host 5760 -> Namespace 10.42.0.11:5760 (Removed - bridge connects directly)
  # socat TCP-LISTEN:${MAV_PORT},fork,reuseaddr TCP:${NS_IP}:${MAV_PORT} &

  # 2. Gazebo servo tunnel NOT needed — ArduPilot plugin uses UDP (not TCP).
  #    SITL sends servo commands via UDP to --sim-address (root veth IP).
  #    Gazebo plugin now listens on 0.0.0.0:fdm_port_in so it accepts them.

  # 3. Gazebo FDM Tunnel (UDP) - Root 127.0.0.1:PORT -> Namespace NS_IP:PORT
  #    Gazebo plugin sends FDM data to 127.0.0.1:<fdm_port_out> in root ns.
  #    This socat relays those packets into the namespace where SITL listens
  #    on 0.0.0.0:<port_in>.  No reverse tunnel needed: SITL sends servo
  #    data back to Gazebo via --sim-address (root-side veth IP) directly.
  socat UDP-LISTEN:${SITL_IN_PORT},fork,reuseaddr UDP-SENDTO:${NS_IP}:${SITL_IN_PORT} &

  sleep 1

  # 3. Launch SITL inside the namespace
  sudo -E ip netns exec "${NS}" \
    su - "${SUDO_USER:-$USER}" -s /bin/bash -c "
      cd \"$SITL_DIR\" && exec \"$BINARY\" \
        --model gazebo-iris \
        --speedup 1 \
        --sysid \"${i}\" \
        --instance $((i-1)) \
        --defaults \"$UAV_DEFAULTS\" \
        --sim-address ${ROOT_VETH_IP} \
        --serial0 tcp:${MAV_PORT} \
        --home -35.363261,149.165230,584,0 \
        ${SITL_EXTRA_ARGS:-}
    " >"$SITL_LOG" 2>&1 &
  SITL_PID=$!
  echo "    SITL PID: $SITL_PID (dir: $SITL_DIR, log: $SITL_LOG)"
  sleep 3

  if ! sudo ip netns exec "${NS}" kill -0 "$SITL_PID" 2>/dev/null; then
    echo "ERROR: UAV${i} SITL exited immediately. Last 120 lines of $SITL_LOG:"
    tail -n 120 "$SITL_LOG" || true
    exit 1
  fi
done

echo "=== [6.5/8] Waiting for MAVLink TCP ports to be reachable inside namespaces ==="
for i in 1 2 3; do
  NS="uav${i}"
  MAV_PORT=$((5760 + 10 * (i - 1)))
  
  echo "  - [UAV${i}] waiting for MAVLink port ${MAV_PORT} inside ${NS}..."
  
  deadline=$(( $(date +%s) + 90 ))
  while true; do
    if sudo ip netns exec "${NS}" ss -tulpn | grep -q ":${MAV_PORT}"; then
      echo "    [UAV${i}] MAVLink port ${MAV_PORT} is ACTIVE"
      break
    fi
    
    if (( $(date +%s) >= deadline )); then
      echo "ERROR: Timed out waiting for UAV${i} MAVLink on port ${MAV_PORT}"
      exit 1
    fi
    sleep 1
  done
done



# 7) Start drone_bridge nodes, then launch mission
echo "=== [7/8] Starting 3 drone_bridge nodes, then multi_mission ==="

# drone_bridge runs in ROOT namespace — it connects directly to the namespace IP
for i in 1 2 3; do
  mavlink_host="10.42.0.$((10 + i))"
  mavlink_port=$((5760 + 10 * (i - 1)))
  echo "  - [UAV${i}] drone_bridge mavlink_host=${mavlink_host} mavlink_port=${mavlink_port}"
  
  # Set discovery server so all nodes can find each other
  export ROS_DISCOVERY_SERVER=10.42.0.10:11811

  # Launch bridge
  ros2 run uav_controller drone_bridge \
    --ros-args \
    -p uav_id:=${i} \
    -p mavlink_host:=${mavlink_host} \
    -p mavlink_port:=${mavlink_port} \
    -p takeoff_altitude:=20.0 &
  BRIDGE_PID=$!
  sleep 1
  if ! kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "ERROR: drone_bridge for UAV${i} failed to start or exited immediately."
    exit 1
  fi
done

sleep 3
ros2 run uav_controller multi_mission &
MISSION_PID=$!
echo "Mission PID: $MISSION_PID"
sleep 2
if ! kill -0 "$MISSION_PID" 2>/dev/null; then
  echo "ERROR: multi_mission node failed to start or exited immediately."
  exit 1
fi

# 8) Wait for NS-3 to finish
echo "=== [8/8] Waiting for NS-3 (PID $NS3_PID) ==="

 

# CHANGE: Use this block to monitor the processes instead of just waiting
wait $NS3_PID || echo "WARNING: NS-3 exited with error $?"

# Keep the script alive for 60 seconds to inspect processes if mission crashes
echo "Script finished, waiting 60s for inspection..."
sleep 60