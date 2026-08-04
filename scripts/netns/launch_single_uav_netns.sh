#!/bin/bash
# Single-UAV (+ GCS) netns/NS-3 launch — reduced from the 3-UAV pipeline.
# Reuses the unmodified 4-node ns-3 binary (three_uav_tapbridge_integrated);
# UAV2/UAV3 get bare, unbridged dummy TAPs so the binary's required tap2/tap3
# arguments are satisfied, but no namespace/SITL/traffic exists behind them.
#
# Does NOT auto-launch the mission, detector, or metrics_logger — run those
# manually afterward, in separate terminals, so edge/ground/nav-only runs
# stay independently controllable and debuggable.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"

# Provides ARDUPILOT_HOME and other project-wide env vars
source "$PROJECT_DIR/setup.sh"

NS3_ROOT="$HOME/ns-allinone-3.38/ns-3.38"
NS3_LOG="/tmp/ns3_single.log"
GAZEBO_LOG="/tmp/gazebo_netns.log"
AGENT_LOG="/tmp/agent_netns.log"
BRIDGE_LOG="/tmp/bridge_netns.log"
SITL_LOG_DIR="/tmp/sitl_netns_uav1"

HOME_GPS="6.0790684,80.1915283,0.00,0"
WORLD_PATH="$PROJECT_DIR/worlds/small_city_single_uav_netns.world"
DDS_PARM="$PROJECT_DIR/params/uav1_dds_netns.parm"

TAP_READY_TIMEOUT=30
AGENT_READY_TIMEOUT=20
DDS_GPS_TIMEOUT=90
SITL_TCP_TIMEOUT=90
GAZEBO_STARTUP_SEC=30

NS3_PID=""
GAZEBO_PID=""
SITL_PID=""
AGENT_PID=""
BRIDGE_PID=""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 0 — Cleanup
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [0] Pre-flight cleanup ==="
for pattern in drone_bridge micro_ros_agent '/build/sitl/bin/arducopter' \
               'three_uav_tapbridge_integrated' gzserver gzclient; do
    sudo pkill -9 -f -- "$pattern" 2>/dev/null && echo "  killed: $pattern" || true
done
for ns in gcsns uav1ns; do
    sudo ip netns del "$ns" 2>/dev/null || true
done
for br in br-gcs br-uav1; do
    sudo ip link del "$br" type bridge 2>/dev/null || true
done
for link in tap-gcs tap-uav1 tap-uav2 tap-uav3 veth0h veth1h sim1h; do
    sudo ip link del "$link" 2>/dev/null || true
done
sleep 2
echo "=== Cleanup done ==="
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 1 — Real wireless topology: gcsns + uav1ns only
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [1/8] Wireless topology: gcsns + uav1ns ==="
setup_ns() {
  local NS=$1 TAP=$2 BR=$3 VETH_H=$4 VETH_NS=$5 NS_IP=$6
  sudo ip netns add "$NS" 2>/dev/null || true
  sudo ip tuntap add dev "$TAP" mode tap user "$RUN_USER" 2>/dev/null || true
  sudo ip link set "$TAP" up
  sudo ip link add name "$BR" type bridge 2>/dev/null || true
  sudo ip link set "$TAP" master "$BR"
  sudo ip link set "$BR" up
  sudo ip link add "$VETH_H" type veth peer name "$VETH_NS" 2>/dev/null || true
  sudo ip link set "$VETH_H" master "$BR"
  sudo ip link set "$VETH_H" up
  sudo ip link set "$VETH_NS" netns "$NS"
  sudo ip netns exec "$NS" ip link set "$VETH_NS" up
  sudo ip netns exec "$NS" ip addr add "$NS_IP" dev "$VETH_NS" 2>/dev/null || true
  sudo ip netns exec "$NS" ip link set lo up
  echo "  $NS ready: $NS_IP on $VETH_NS"
}
setup_ns gcsns  tap-gcs  br-gcs  veth0h veth0n 10.42.0.10/24
setup_ns uav1ns tap-uav1 br-uav1 veth1h veth1n 10.42.0.11/24

echo "=== [1b] Dummy TAPs for unused UAV2/UAV3 slots (ns-3 binary requires them) ==="
for tap in tap-uav2 tap-uav3; do
    sudo ip tuntap add dev "$tap" mode tap user "$RUN_USER" 2>/dev/null || true
    sudo ip link set "$tap" up
    echo "  $tap up (bare, unbridged, unused)"
done
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 2 — Management link: uav1ns <-> root (FDM physics, bypasses NS-3)
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [2/8] Management link (SITL <-> Gazebo physics) ==="
sudo ip link add sim1h type veth peer name sim1n 2>/dev/null || true
sudo ip addr add 172.31.1.1/30 dev sim1h 2>/dev/null || true
sudo ip link set sim1h up
sudo ip link set sim1n netns uav1ns
sudo ip netns exec uav1ns ip addr add 172.31.1.2/30 dev sim1n 2>/dev/null || true
sudo ip netns exec uav1ns ip link set sim1n up
sudo ethtool -K sim1h rx off tx off sg off tso off gso off gro off 2>/dev/null || true
sudo ip netns exec uav1ns ethtool -K sim1n rx off tx off sg off tso off gso off gro off 2>/dev/null || true
echo "  root=172.31.1.1 <-> uav1ns=172.31.1.2"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 3 — NS-3 wireless simulation (unmodified 4-node binary)
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [3/8] Building + starting ns-3 (three_uav_tapbridge_integrated) ==="
(cd "$NS3_ROOT" && ./ns3 build three_uav_tapbridge_integrated)

: > "$NS3_LOG"
(
    cd "$NS3_ROOT"
    exec ./ns3 run "three_uav_tapbridge_integrated \
        --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
        --simTime=0 --uavAltitude=30"
) > "$NS3_LOG" 2>&1 &
NS3_PID=$!
echo "  ns-3 PID=$NS3_PID  log=$NS3_LOG"

echo "  Waiting for TAP attachment..."
deadline=$((SECONDS + TAP_READY_TIMEOUT))
while true; do
    if ! kill -0 "$NS3_PID" 2>/dev/null; then
        echo "ERROR: ns-3 exited early. Log:" >&2
        cat "$NS3_LOG" >&2
        exit 1
    fi
    all_up=true
    for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
        carrier_file="/sys/class/net/$tap/carrier"
        [[ -r "$carrier_file" && "$(<"$carrier_file")" == 1 ]] || all_up=false
    done
    $all_up && break
    if (( SECONDS >= deadline )); then
        echo "ERROR: Timed out waiting for TAP attachment." >&2
        cat "$NS3_LOG" >&2
        exit 1
    fi
    sleep 0.2
done
echo "  All TAPs attached."

echo "  Verifying gcsns <-> uav1ns connectivity..."
sudo ip netns exec gcsns ping -c 2 -W 2 10.42.0.11 || {
    echo "ERROR: gcsns cannot reach uav1ns over the wireless channel." >&2
    exit 1
}
echo "  Wireless link confirmed working."
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 4 — Gazebo (root namespace)
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [4/8] Starting Gazebo ==="
CITY="$HOME/FYP/small_city_gazebo_world"   # external city-world assets repo (models + terrain)
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$CITY/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$PROJECT_DIR:$PROJECT_DIR/worlds:$CITY:${GAZEBO_RESOURCE_PATH:-}"
# Project + ArduPilot Gazebo plugins (obstacle raycaster + gazebo-iris FDM) — not set by setup.sh
export GAZEBO_PLUGIN_PATH="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH:-}"

[[ -f "$WORLD_PATH" ]] || { echo "ERROR: world file not found: $WORLD_PATH" >&2; exit 1; }

: > "$GAZEBO_LOG"
# HITL: pin Gazebo's DDS to the wired sensor link (10.0.0.x) so the Pi edge node
# can subscribe to the camera. Scoped to THIS process only -- exporting it
# globally would whitelist 10.0.0.x for the gcsns/uav1ns participants too, which
# live on 10.42.0.x, and would break the netns DDS path. Harmless when no Pi is
# attached (the address simply is not matched). See config/fastdds_hitl_eth.xml.
FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml" \
gzserver --verbose "$WORLD_PATH" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > "$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!
echo "  Waiting ${GAZEBO_STARTUP_SEC}s for Gazebo..."
sleep "$GAZEBO_STARTUP_SEC"
kill -0 "$GAZEBO_PID" 2>/dev/null || {
    echo "ERROR: Gazebo exited during startup." >&2
    cat "$GAZEBO_LOG" >&2
    exit 1
}
echo "  Gazebo running: PID=$GAZEBO_PID"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 5 — micro_ros_agent inside gcsns
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [5/8] micro_ros_agent inside gcsns ==="
: > "$AGENT_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
' agent-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$AGENT_LOG" 2>&1 &
AGENT_PID=$!

deadline=$((SECONDS + AGENT_READY_TIMEOUT))
while ! sudo ip netns exec gcsns ss -H -lun "sport = :2019" | grep -q .; do
    sudo kill -0 "$AGENT_PID" 2>/dev/null || {
        echo "ERROR: micro_ros_agent exited early." >&2
        cat "$AGENT_LOG" >&2
        exit 1
    }
    (( SECONDS >= deadline )) && {
        echo "ERROR: agent port 2019 never opened." >&2
        cat "$AGENT_LOG" >&2
        exit 1
    }
    sleep 0.2
done
echo "  agent ready on UDP 2019 inside gcsns"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 6 — SITL inside uav1ns
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [6/8] SITL inside uav1ns ==="
mkdir -p "$SITL_LOG_DIR"
chown "$RUN_USER":"$(id -gn "$RUN_USER")" "$SITL_LOG_DIR"

# Run arducopter UNDER strace (ptrace). REQUIRED workaround: without it, SITL enters
# an endless reboot loop once the Gazebo FDM connects with DDS enabled inside the
# netns (SITL keeps re-loading defaults and re-execing, dropping the FDM link ->
# "Broken ArduPilot connection"). Being ptrace-traced suppresses that auto-reboot, so
# SITL settles and stays up. `-e trace=none` traces NO syscalls (near-zero overhead);
# it is here purely for the ptrace side-effect. Matches the working setup on the other
# laptop. Needs `strace` installed (sudo apt install -y strace).
sudo ip netns exec uav1ns sudo -H -u "$RUN_USER" bash -c '
    cd "$1"
    shift
    exec strace -f -e trace=none -o /dev/null "$@"
' sitl-shell \
    "$SITL_LOG_DIR" \
    "$ARDUPILOT_HOME/build/sitl/bin/arducopter" \
    --wipe --model gazebo-iris --speedup 1 --sysid 1 --instance 0 \
    --defaults "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm,$DDS_PARM" \
    --sim-address "172.31.1.1" \
    --home "$HOME_GPS" \
    --serial0=tcp:0.0.0.0:5760 \
    > "$SITL_LOG_DIR/arducopter.log" 2>&1 &
SITL_PID=$!



echo "  Waiting for SITL UDP 9003 inside uav1ns..."
deadline=$((SECONDS + 60))
while ! sudo ip netns exec uav1ns ss -H -lun "sport = :9003" | grep -q .; do
    if ! sudo kill -0 "$SITL_PID" 2>/tmp/killcheck_err.log; then
        echo "ERROR: SITL liveness check failed. Diagnostics:" >&2
        echo "--- sudo kill -0 stderr ---" >&2
        cat /tmp/killcheck_err.log >&2
        echo "--- ps -p \$SITL_PID ---" >&2
        ps -p "$SITL_PID" -o pid,ppid,stat,cmd >&2 || echo "  no such process" >&2
        echo "--- processes inside uav1ns ---" >&2
        sudo ip netns pids uav1ns 2>/dev/null | xargs -r ps -o pid,ppid,stat,cmd -p >&2 || echo "  none" >&2
        echo "--- arducopter log ---" >&2
        cat "$SITL_LOG_DIR/arducopter.log" >&2
        exit 1
    fi
    (( SECONDS >= deadline )) && {
        echo "ERROR: SITL FDM port never opened." >&2
        cat "$SITL_LOG_DIR/arducopter.log" >&2
        exit 1
    }
    sleep 0.2
done
echo "  SITL alive, FDM port open"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 7 — Verify DDS GPS + SITL TCP reachable from gcsns
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [7/8] Verifying DDS GPS + MAVLink reachability from gcsns ==="

echo "  Waiting for /ap/v1/navsat publisher + valid GPS..."
deadline=$((SECONDS + DDS_GPS_TIMEOUT))
while true; do
    info=$(sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
        source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
        source "$1"
        ros2 topic info /ap/v1/navsat 2>/dev/null
    ' t-shell "$PROJECT_DIR/ros2/install/setup.bash" || true)
    if echo "$info" | grep -q "Publisher count: [1-9]"; then
        echo "  /ap/v1/navsat has a live publisher."
        break
    fi
    (( SECONDS >= deadline )) && {
        echo "ERROR: No publisher on /ap/v1/navsat within ${DDS_GPS_TIMEOUT}s." >&2
        exit 1
    }
    sleep 1
done

echo "  Waiting for SITL TCP 5760 reachable from gcsns..."
deadline=$((SECONDS + SITL_TCP_TIMEOUT))
until sudo ip netns exec gcsns sudo -H -u "$RUN_USER" \
    timeout 1 bash -c 'exec 3<>/dev/tcp/10.42.0.11/5760' 2>/dev/null; do
    (( SECONDS >= deadline )) && {
        echo "ERROR: SITL TCP 5760 not reachable from gcsns." >&2
        exit 1
    }
    sleep 0.5
done
echo "  SITL reachable from gcsns at 10.42.0.11:5760"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 8 — drone_bridge inside gcsns
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [8/8] Starting drone_bridge inside gcsns ==="
: > "$BRIDGE_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run uav_controller drone_bridge --ros-args \
        -p uav_id:=1 -p mavlink_host:=10.42.0.11 -p mavlink_port:=5760
' bridge-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!
sleep 3
sudo kill -0 "$BRIDGE_PID" 2>/dev/null || {
    echo "ERROR: drone_bridge exited during startup." >&2
    cat "$BRIDGE_LOG" >&2
    exit 1
}
echo "  drone_bridge running: PID=$BRIDGE_PID"
echo ""

echo "════════════════════════════════════════════════════════════"
echo " PIPELINE READY — nothing auto-launched beyond this point."
echo "════════════════════════════════════════════════════════════"
echo "  Run mission/detector/metrics_logger manually, INSIDE gcsns, e.g.:"
echo ""
echo "  sudo ip netns exec gcsns sudo -H -u $RUN_USER bash -lc '"
echo "      source /opt/ros/humble/setup.bash"
echo "      source $PROJECT_DIR/ros2/install/setup.bash"
echo "      python3 $PROJECT_DIR/ros2/uav_controller/uav_controller/uav1_patrol_mission.py"
echo "  '"
echo ""
echo "  PIDs: ns3=$NS3_PID gazebo=$GAZEBO_PID sitl=$SITL_PID agent=$AGENT_PID bridge=$BRIDGE_PID"
echo "  Logs: $NS3_LOG  $GAZEBO_LOG  $SITL_LOG_DIR/arducopter.log  $AGENT_LOG  $BRIDGE_LOG"
echo "  Ctrl+C to shut down everything."
echo ""

cleanup() {
    echo "Shutting down..."
    kill "$BRIDGE_PID" "$AGENT_PID" "$SITL_PID" "$GAZEBO_PID" "$NS3_PID" 2>/dev/null || true
    sleep 1
    sudo ip netns del gcsns 2>/dev/null || true
    sudo ip netns del uav1ns 2>/dev/null || true
    sudo ip link del br-gcs type bridge 2>/dev/null || true
    sudo ip link del br-uav1 type bridge 2>/dev/null || true
    for l in tap-gcs tap-uav1 tap-uav2 tap-uav3 veth0h veth1h sim1h; do
        sudo ip link del "$l" 2>/dev/null || true
    done
    exit
}
trap cleanup INT TERM
wait
