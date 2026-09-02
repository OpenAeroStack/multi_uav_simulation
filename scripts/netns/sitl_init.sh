#!/bin/bash
# Step 1 of 4: cold-start the pipeline — netns, ns-3, Gazebo, SITL x2,
# micro-ROS agents x2, drone_bridge x2. Ends at PIPELINE READY and flies nothing.
#
#   ./scripts/netns/rpi_init.sh                 # 0 - verify both Raspberry Pi boards
#   ./scripts/netns/sitl_init.sh --gui --view   # 1 - host pipeline, leave running
#   ./scripts/netns/detector_start.sh           # 2 - Pi detectors + receivers
#   ./scripts/netns/run_missions.sh             # 3 - fly
#
# Missions are deliberately separate: initialisation must be fully verified
# before anything arms, and each half can then be debugged on its own.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"

# --gui opens the Gazebo 3D viewer; --view opens a camera window per aircraft.
WITH_GUI=0
WITH_VIEW=0
for arg in "$@"; do
    case "$arg" in
        --gui)  WITH_GUI=1 ;;
        --view) WITH_VIEW=1 ;;
        -h|--help)
            echo "usage: sitl_init.sh [--gui] [--view]"
            echo "  --gui   open the Gazebo 3D viewer (gzclient)"
            echo "  --view  open a camera + detections window per aircraft"
            exit 0 ;;
        *) echo "ERROR: unknown option: $arg" >&2; exit 2 ;;
    esac
done

# SITL children turn off ONLCR; save the tty settings and restore them on exit.
TTY_SAVED=""
[[ -t 0 ]] && TTY_SAVED="$(stty -g 2>/dev/null || true)"
restore_tty() { [[ -n "$TTY_SAVED" ]] && stty "$TTY_SAVED" 2>/dev/null || true; }

# Provides ARDUPILOT_HOME and other project-wide env vars
source "$PROJECT_DIR/setup.sh"

# Logs
NS3_ROOT="$HOME/ns-allinone-3.38/ns-3.38"
NS3_LOG="/tmp/ns3_2uav.log"
GAZEBO_LOG="/tmp/gazebo_2uav.log"
AGENT_LOG="/tmp/agent_2uav_uav1.log"
BRIDGE_LOG="/tmp/bridge_2uav_uav1.log"
POSPUB_LOG="/tmp/pospub_2uav.log"
SITL_LOG_DIR="/tmp/sitl_netns_uav1"
SITL_LOG_DIR2="/tmp/sitl_netns_uav2"
AGENT_LOG2="/tmp/agent_2uav_uav2.log"
BRIDGE_LOG2="/tmp/bridge_2uav_uav2.log"


# One --home per aircraft, matching its <pose>. See docs/COORDINATE_FRAMES.pdf.

# UAV1
HOME_GPS="6.0790684,80.1915283,0.00,0"   
# UAV2 
HOME_GPS2="6.0790684,80.1916186,0.00,0"   

WORLD_PATH="$PROJECT_DIR/worlds/small_city_2uav_netns.world"
DDS_PARM="$PROJECT_DIR/params/uav1_dds_netns.parm"
DDS_PARM2="$PROJECT_DIR/params/uav2_dds_netns.parm"


# ── HITL: the Pi edge node's two links over one cable (see STEP 1c) ─────────
# Auto-detect the Pi-facing NIC; override by exporting PI_LINK_IF beforehand.
PI_LINK_IF="${PI_LINK_IF:-$(ls /sys/class/net 2>/dev/null | grep -m1 '^enx' || true)}"
CAM_VLAN=10          # unimpaired camera link  (behaves like a camera cable)
RF_VLAN=42           # impaired radio link     (goes through ns-3)
RF_VLAN2=43          # board 2's radio link 
CAM_VLAN_IF="eth-cam"
RF_VLAN_IF="eth-rf"
RF_VLAN_IF2="eth-rf2"
CAM_HOST_IP="10.0.0.1"
CAM_PI_IP="10.0.0.2"
RF_PI_IP="10.42.0.12"   # the Pi is UAV2 on the wireless subnet

TAP_READY_TIMEOUT=30
AGENT_READY_TIMEOUT=20
DDS_GPS_TIMEOUT=90
SITL_SETTLE_SEC=15      # AP_DDS handshake headroom; see STEP 6 for why
SITL_TCP_TIMEOUT=90
GAZEBO_STARTUP_SEC=30

NS3_PID=""
GAZEBO_PID=""
SITL_PID=""
SITL_PID2=""
AGENT_PID=""
BRIDGE_PID=""
AGENT_PID2=""
BRIDGE_PID2=""
POSPUB_PID=""

# ── STEP 0 — Cleanup ─────────────────────────────────────────────────────────
restore_tty
echo "=== [0] Pre-flight cleanup ==="
# `[g]zserver` stops pkill matching its own sudo. Stale procs corrupt the next run.
KILL_PATTERNS=(
    drone_bridge
    micro_ros_agent
    '/build/sitl/bin/arducopter'
    'three_uav_tapbridge_integrated'
    gzserver
    gzclient
    'world_pos_publisher.py'
    'two_drone_mission.py'
    'uav1_patrol_mission.py'
    'uav2_road_patrol.py'
    'detection_viewer.py'
    gcs_receiver
    metrics_logger
)
for pattern in "${KILL_PATTERNS[@]}"; do
    safe="[${pattern:0:1}]${pattern:1}"
    sudo pkill -9 -f -- "$safe" 2>/dev/null && echo "  killed: $pattern" || true
done

# Second pass: SIGKILL is not instant. Warn only; pgrep -f has false positives.
sleep 1
for pattern in "${KILL_PATTERNS[@]}"; do
    safe="[${pattern:0:1}]${pattern:1}"
    if pgrep -f -- "$safe" >/dev/null 2>&1; then
        sudo pkill -9 -f -- "$safe" 2>/dev/null || true
        sleep 0.3
        pgrep -f -- "$safe" >/dev/null 2>&1 &&
            echo "  WARNING: '$pattern' still matches a process — check with: pgrep -af '$pattern'" >&2
    fi
done
for ns in gcsns uav1ns uav2ns; do
    sudo ip netns del "$ns" 2>/dev/null || true
done
for br in br-gcs br-uav1 br-uav2 br-uav3 br-uav4; do
    sudo ip link del "$br" type bridge 2>/dev/null || true
done
# eth-cam is NOT deleted: it is the persistent sensor cable, owned by NetworkManager.
for link in tap-gcs tap-uav1 tap-uav2 tap-uav3 tap-uav4 veth0h veth1h veth2h \
            sim1h sim2h eth-rf eth-rf2; do
    sudo ip link del "$link" 2>/dev/null || true
done
sleep 2
echo "=== Cleanup done ==="
echo ""

# ── STEP 1 — Real wireless topology: gcsns + uav1ns only ─────────────────────
restore_tty
echo "=== [1/8] Wireless topology: gcsns + uav1ns + uav2ns ==="
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
setup_ns uav2ns tap-uav3 br-uav3 veth2h veth2n 10.42.0.13/24



restore_tty
echo "=== [1b] TAPs for the Pi edge nodes ==="
for tap in tap-uav2 tap-uav4; do
    sudo ip tuntap add dev "$tap" mode tap user "$RUN_USER" 2>/dev/null || true
    sudo ip link set "$tap" up
done
echo "  tap-uav2 up (node 2 - Pi 1 attaches here, see 1c)"
echo "  tap-uav4 up (node 4 - Pi 2 attaches here, see 1c)"
echo ""

# ── STEP 1c — HITL: attach the Raspberry Pi as UAV2 on the IMPAIRED link ─────
# One cable, two VLANs: 10 = unimpaired camera in, 42 = detections out via ns-3.
restore_tty
echo "=== [1c] HITL: Pi edge node as UAV2 on the impaired link ==="
if [[ -n "$PI_LINK_IF" ]] && [[ -d "/sys/class/net/$PI_LINK_IF" ]]; then
    sudo modprobe 8021q 2>/dev/null || true

    # br-uav2 joins the Pi's radio VLAN to ns-3's UAV2 node.
    sudo ip link add name br-uav2 type bridge 2>/dev/null || true
    sudo ip link set tap-uav2 master br-uav2
    sudo ip link set br-uav2 up

    # Address must sit on the VLAN, not the untagged parent. All idempotent.
    if ip link show "$CAM_VLAN_IF" >/dev/null 2>&1; then
        echo "  $CAM_VLAN_IF already present (persistent) — left alone"
    else
        echo "  $CAM_VLAN_IF missing — creating it for this run only."
        echo "  To make it permanent so the sensor link survives reboots:"
        echo "    sudo nmcli connection modify '<wired-profile>' ipv4.method disabled"
        echo "    sudo nmcli connection add type vlan con-name $CAM_VLAN_IF \\"
        echo "         ifname $CAM_VLAN_IF dev $PI_LINK_IF id $CAM_VLAN ip4 $CAM_HOST_IP/24"
        echo "    sudo nmcli connection modify $CAM_VLAN_IF ipv4.never-default yes"
    fi
    sudo ip addr del "$CAM_HOST_IP/24" dev "$PI_LINK_IF" 2>/dev/null || true
    sudo ip link add link "$PI_LINK_IF" name "$CAM_VLAN_IF" type vlan id "$CAM_VLAN" 2>/dev/null || true
    sudo ip addr add "$CAM_HOST_IP/24" dev "$CAM_VLAN_IF" 2>/dev/null || true
    sudo ip link set "$CAM_VLAN_IF" up

    # Radio VLAN carries no host IP — it is a pure L2 leg into the bridge.
    sudo ip link add link "$PI_LINK_IF" name "$RF_VLAN_IF" type vlan id "$RF_VLAN" 2>/dev/null || true
    sudo ip link set "$RF_VLAN_IF" up
    sudo ip link set "$RF_VLAN_IF" master br-uav2

    # Board 2: VLAN 43 -> br-uav4 -> tap-uav4 -> ns-3 node 4.
    sudo ip link add name br-uav4 type bridge 2>/dev/null || true
    sudo ip link set tap-uav4 master br-uav4
    sudo ip link set br-uav4 up


    sudo ip link add link "$PI_LINK_IF" name "$RF_VLAN_IF2" type vlan id "$RF_VLAN2" 2>/dev/null || true
    sudo ip link set "$RF_VLAN_IF2" up
    sudo ip link set "$RF_VLAN_IF2" master br-uav4


    sudo ip link set "$PI_LINK_IF" up
    echo "  $PI_LINK_IF: VLAN $CAM_VLAN -> $CAM_VLAN_IF ($CAM_HOST_IP) unimpaired camera link"
    echo "  $PI_LINK_IF: VLAN $RF_VLAN -> $RF_VLAN_IF -> br-uav2 -> tap-uav2 -> ns-3"
    echo "  Pi should hold ${CAM_PI_IP}/24 on VLAN $CAM_VLAN and ${RF_PI_IP}/24 on VLAN $RF_VLAN"
else
    echo "  skipped — no Pi-facing NIC (PI_LINK_IF='$PI_LINK_IF'); host-only run"
fi
echo ""

# ── STEP 2 — Management link: uav1ns <-> root (FDM physics, bypasses NS-3) ────
restore_tty
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

sudo ip link add sim2h type veth peer name sim2n 2>/dev/null || true
sudo ip addr add 172.31.2.1/30 dev sim2h 2>/dev/null || true
sudo ip link set sim2h up
sudo ip link set sim2n netns uav2ns
sudo ip netns exec uav2ns ip addr add 172.31.2.2/30 dev sim2n 2>/dev/null || true
sudo ip netns exec uav2ns ip link set sim2n up
sudo ethtool -K sim2h rx off tx off sg off tso off gso off gro off 2>/dev/null || true
sudo ip netns exec uav2ns ethtool -K sim2n rx off tx off sg off tso off gso off gro off 2>/dev/null || true
echo "  root=172.31.2.1 <-> uav2ns=172.31.2.2"

echo ""

# ── STEP 3 — NS-3 wireless simulation (unmodified 4-node binary) ─────────────
restore_tty
echo "=== [3/8] Building + starting ns-3 (three_uav_tapbridge_integrated) ==="
(cd "$NS3_ROOT" && ./ns3 build three_uav_tapbridge_integrated)

: > "$NS3_LOG"
(
    cd "$NS3_ROOT"
    exec ./ns3 run "three_uav_tapbridge_integrated \
        --nRadios=4 \
        --simTime=0 --uavAltitude=30"
) > "$NS3_LOG" 2>&1 < /dev/null &
NS3_PID=$!
restore_tty
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
    for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3 tap-uav4; do
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

# ── STEP 4 — Gazebo (root namespace) ─────────────────────────────────────────
restore_tty
echo "=== [4/8] Starting Gazebo ==="
CITY="$HOME/FYP/small_city_gazebo_world"   # external city-world assets repo (models + terrain)
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$CITY/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:$PROJECT_DIR:$PROJECT_DIR/worlds:$CITY:${GAZEBO_RESOURCE_PATH:-}"
# Project + ArduPilot Gazebo plugins (obstacle raycaster + gazebo-iris FDM) — not set by setup.sh
export GAZEBO_PLUGIN_PATH="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH:-}"

[[ -f "$WORLD_PATH" ]] || { echo "ERROR: world file not found: $WORLD_PATH" >&2; exit 1; }

: > "$GAZEBO_LOG"
# Pin Gazebo's DDS to the sensor link so the Pi can subscribe to the camera.
# Scoped to THIS process: exporting it globally would break the netns DDS path.
FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml" \
gzserver --verbose "$WORLD_PATH" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > "$GAZEBO_LOG" 2>&1 < /dev/null &
GAZEBO_PID=$!
restore_tty
echo "  Waiting ${GAZEBO_STARTUP_SEC}s for Gazebo..."
sleep "$GAZEBO_STARTUP_SEC"
kill -0 "$GAZEBO_PID" 2>/dev/null || {
    echo "ERROR: Gazebo exited during startup." >&2
    cat "$GAZEBO_LOG" >&2
    exit 1
}
echo "  Gazebo running: PID=$GAZEBO_PID"
echo ""

# ── STEP 4b — Position publisher (ns-3 mobility) ─────────────────────────────
# Without this every ns-3 node stays frozen and the link numbers are fiction.
restore_tty
echo "=== [4b] Position publisher (feeds real UAV position to ns-3) ==="
if [[ -f "$PROJECT_DIR/scripts/world_pos_publisher.py" ]]; then
    : > "$POSPUB_LOG"
    # set +u: ROS setup.bash reads an unset var and would abort under -u.
    ( set +u
      source /opt/ros/humble/setup.bash
      export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml"
      # --ros-args MUST come first or rclpy silently ignores every -p flag.
      # Never put a comment after a trailing backslash: it eats the arguments.
      exec python3 "$PROJECT_DIR/scripts/world_pos_publisher.py" \
                --ros-args -p n_uavs:=1 -p mirror:=2:1

    ) > "$POSPUB_LOG" 2>&1 < /dev/null &
    POSPUB_PID=$!
restore_tty
    sleep 3
    if kill -0 "$POSPUB_PID" 2>/dev/null; then
        echo "  running: PID=$POSPUB_PID"
    else
        echo "  WARNING: position publisher exited — ns-3 nodes will stay at their" >&2
        echo "           default positions and path loss will NOT track the flight." >&2
        cat "$POSPUB_LOG" >&2
    fi
else
    echo "  WARNING: world_pos_publisher.py not found — ns-3 mobility will be static." >&2
fi
echo ""

# ── STEP 5 — micro_ros_agent inside gcsns ────────────────────────────────────
restore_tty
echo "=== [5/8] micro_ros_agent inside gcsns ==="
: > "$AGENT_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
' agent-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$AGENT_LOG" 2>&1 < /dev/null &
AGENT_PID=$!
restore_tty

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

restore_tty
echo "=== [5b/8] micro_ros_agent for UAV2 inside gcsns ==="
: > "$AGENT_LOG2"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 2020
' agent-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$AGENT_LOG2" 2>&1 < /dev/null &
AGENT_PID2=$!
restore_tty

deadline=$((SECONDS + AGENT_READY_TIMEOUT))
while ! sudo ip netns exec gcsns ss -H -lun "sport = :2020" | grep -q .; do
    sudo kill -0 "$AGENT_PID2" 2>/dev/null || {
        echo "ERROR: micro_ros_agent (UAV2) exited early." >&2
        cat "$AGENT_LOG2" >&2
        exit 1
    }
    (( SECONDS >= deadline )) && {
        echo "ERROR: agent port 2020 never opened." >&2
        cat "$AGENT_LOG2" >&2
        exit 1
    }
    sleep 0.2
done
echo "  agent ready on UDP 2020 inside gcsns"
echo ""

# ── STEP 6 — SITL inside uav1ns ──────────────────────────────────────────────
restore_tty
echo "=== [6a/8] SITL inside uav1ns ==="
mkdir -p "$SITL_LOG_DIR"
chown "$RUN_USER":"$(id -gn "$RUN_USER")" "$SITL_LOG_DIR"

# strace REQUIRED: being ptrace-traced suppresses SITL's netns+DDS reboot loop.
# `-e trace=none` traces nothing; it is here purely for that side-effect.
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
    > "$SITL_LOG_DIR/arducopter.log" 2>&1 < /dev/null &
SITL_PID=$!
restore_tty


restore_tty
echo "=== [6b/8] SITL inside uav2ns ==="
mkdir -p "$SITL_LOG_DIR2"
chown "$RUN_USER":"$(id -gn "$RUN_USER")" "$SITL_LOG_DIR2"
sudo ip netns exec uav2ns sudo -H -u "$RUN_USER" bash -c '
    cd "$1"
    shift
    exec strace -f -e trace=none -o /dev/null "$@"
' sitl-shell \
    "$SITL_LOG_DIR2" \
    "$ARDUPILOT_HOME/build/sitl/bin/arducopter" \
    --wipe --model gazebo-iris --speedup 1 --sysid 2 --instance 1 \
    --defaults "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm,$DDS_PARM2" \
    --sim-address "172.31.2.1" \
    --home "$HOME_GPS2" \
    --serial0=tcp:0.0.0.0:5770 \
    > "$SITL_LOG_DIR2/arducopter.log" 2>&1 < /dev/null &
SITL_PID2=$!
restore_tty
echo "  SITL2 PID=$SITL_PID2  log=$SITL_LOG_DIR2/arducopter.log"



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

# AP_DDS needs to finish its XRCE handshake before the bridges pull MAVLink.
echo "  Letting AP_DDS settle (${SITL_SETTLE_SEC}s)..."
sleep "$SITL_SETTLE_SEC"
echo ""

# ── STEP 7 — Verify DDS GPS + SITL TCP reachable from gcsns ──────────────────
restore_tty
echo "=== [7/8] Verifying DDS GPS + MAVLink reachability from gcsns ==="

# A publisher count is not a working feed: wait for a real message, on both.
wait_for_navsat() {
    local uav=$1 topic="/ap/v$1/navsat"
    echo "  Waiting for a message on $topic ..."
    local deadline=$((SECONDS + DDS_GPS_TIMEOUT))
    while true; do
        if sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
                source /opt/ros/humble/setup.bash
                source "$HOME/ardu_ws/install/setup.bash"
                source "$1"
                timeout 5 ros2 topic echo --once "$2" >/dev/null 2>&1
            ' t-shell "$PROJECT_DIR/ros2/install/setup.bash" "$topic"; then
            echo "  $topic is delivering messages."
            return 0
        fi
        (( SECONDS >= deadline )) && {
            echo "ERROR: no data on $topic within ${DDS_GPS_TIMEOUT}s." >&2
            echo "       AP_DDS is up but silent. Check $AGENT_LOG for an" >&2
            echo "       'establish_session' later than the last 'create_topic'." >&2
            exit 1
        }
        sleep 1
    done
}

wait_for_navsat 1
wait_for_navsat 2

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

# ── STEP 8 — drone_bridge inside gcsns ───────────────────────────────────────
restore_tty
    # -r __node:= is REQUIRED: duplicate node names make the second displace
    # the first, and UAV1 silently loses its topics every run.
echo "=== [8/8] Starting drone_bridge inside gcsns ==="
: > "$BRIDGE_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run uav_controller drone_bridge --ros-args \
        -r __node:=drone_bridge_uav1 \
        -p uav_id:=1 -p mavlink_host:=10.42.0.11 -p mavlink_port:=5760
' bridge-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$BRIDGE_LOG" 2>&1 < /dev/null &
BRIDGE_PID=$!
restore_tty
sleep 3
sudo kill -0 "$BRIDGE_PID" 2>/dev/null || {
    echo "ERROR: drone_bridge exited during startup." >&2
    cat "$BRIDGE_LOG" >&2
    exit 1
}
echo "  drone_bridge running: PID=$BRIDGE_PID"
echo ""

restore_tty
echo "=== [8b/8] drone_bridge for UAV2 inside gcsns ==="
: > "$BRIDGE_LOG2"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run uav_controller drone_bridge --ros-args \
        -r __node:=drone_bridge_uav2 \
        -p uav_id:=2 -p mavlink_host:=10.42.0.13 -p mavlink_port:=5770 \
        -p takeoff_altitude:=30.0
' bridge-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$BRIDGE_LOG2" 2>&1 < /dev/null &
BRIDGE_PID2=$!
restore_tty
sleep 3
sudo kill -0 "$BRIDGE_PID2" 2>/dev/null || {
    echo "ERROR: drone_bridge (UAV2) exited during startup." >&2
    cat "$BRIDGE_LOG2" >&2
    exit 1
}
echo "  drone_bridge UAV2 running: PID=$BRIDGE_PID2"
echo ""


echo ""

# ── Viewers ──────────────────────────────────────────────────────────────────
# gzclient attaches to the running gzserver, so it can be opened or closed at
# any time. Both cost host CPU; close them before a timing run.
GZCLIENT_PID=""
VIEWER_PIDS=()
if (( WITH_GUI )); then
    gzclient > /tmp/gzclient_2uav.log 2>&1 &
    GZCLIENT_PID=$!
    sleep 2
    if kill -0 "$GZCLIENT_PID" 2>/dev/null; then
        echo "  Gazebo viewer opened (pid $GZCLIENT_PID)"
    else
        echo "  WARNING: gzclient exited — see /tmp/gzclient_2uav.log" >&2
    fi
fi
if (( WITH_VIEW )); then
    for i in 1 2; do
        FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml" \
        bash -lc "source /opt/ros/humble/setup.bash && \
                  exec python3 '$PROJECT_DIR/scripts/detection_viewer.py' --ros-args \
                       -p image_topic:=/uav$i/camera/image_raw \
                       -p detection_topic:=/detections/uav$i" \
            > "/tmp/viewer_2uav_uav$i.log" 2>&1 &
        VIEWER_PIDS+=("$!")
        echo "  uav$i camera window opened"
    done
fi
echo ""

restore_tty

# say(): leading \r forces column 0. SITL children clear ONLCR, so a plain echo
# starts where the previous line ended and the whole banner prints skewed.
say() { printf '\r%s\n' "$*"; }

say ""
say "════════════════════════════════════════════════════════════"
say " PIPELINE READY — nothing auto-launched beyond this point."
say "════════════════════════════════════════════════════════════"
say ""
say "  Initialisation is complete and verified. Fly in a SECOND terminal:"
say ""
say "      ./scripts/netns/run_missions.sh"
say ""
say "  It re-checks every stage before arming, so a half-built stack fails"
say "  immediately instead of thirty seconds into a flight."
say ""
say "  PIDs   ns3=$NS3_PID  gazebo=$GAZEBO_PID  pospub=$POSPUB_PID"
say "$(printf '         uav1: sitl=%-8s agent=%-8s bridge=%s' "$SITL_PID"  "$AGENT_PID"  "$BRIDGE_PID")"
say "$(printf '         uav2: sitl=%-8s agent=%-8s bridge=%s' "$SITL_PID2" "$AGENT_PID2" "$BRIDGE_PID2")"
say ""
say "  Logs   shared: $NS3_LOG  $GAZEBO_LOG  $POSPUB_LOG"
say "         uav1:   $SITL_LOG_DIR/arducopter.log  $AGENT_LOG  $BRIDGE_LOG"
say "         uav2:   $SITL_LOG_DIR2/arducopter.log  $AGENT_LOG2  $BRIDGE_LOG2"
say ""
say "  Ctrl+C here shuts down everything."
echo ""

cleanup() {
    restore_tty
    echo "Shutting down..."
    kill "$BRIDGE_PID" "$BRIDGE_PID2" "$AGENT_PID" "$AGENT_PID2" "$SITL_PID" "$SITL_PID2" "$POSPUB_PID" "$GAZEBO_PID" \
         "$NS3_PID" ${GZCLIENT_PID:-} ${VIEWER_PIDS[@]:-} 2>/dev/null || true
    sleep 1
    sudo ip netns del gcsns 2>/dev/null || true
    sudo ip netns del uav1ns 2>/dev/null || true
    sudo ip netns del uav2ns 2>/dev/null || true

    sudo ip link del br-gcs type bridge 2>/dev/null || true
    sudo ip link del br-uav1 type bridge 2>/dev/null || true
    sudo ip link del br-uav2 type bridge 2>/dev/null || true
    sudo ip link del br-uav3 type bridge 2>/dev/null || true
    sudo ip link del br-uav4 type bridge 2>/dev/null || true

    # eth-cam left up on purpose: it is the sensor cable, not the simulation.
    for l in tap-gcs tap-uav1 tap-uav2 tap-uav3 tap-uav4 veth0h veth1h veth2h \
             sim1h sim2h eth-rf eth-rf2; do
        sudo ip link del "$l" 2>/dev/null || true
    done
    exit
}
trap cleanup INT TERM
trap restore_tty EXIT
wait
