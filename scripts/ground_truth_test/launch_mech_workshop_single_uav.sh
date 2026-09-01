#!/bin/bash
# Single-UAV (+ GCS) netns/NS-3 launch for the mech_workshop_validation world.
# Reuses the unmodified 4-node ns-3 binary (three_uav_tapbridge_integrated);
# UAV2/UAV3 get bare, unbridged dummy TAPs so the binary's required tap2/tap3
# arguments are satisfied, but no namespace/SITL/traffic exists behind them.
# Mirrors scripts/netns/launch_single_uav_netns.sh, retargeted at the
# mech_workshop world and its real spherical_coordinates origin.
#
# Auto-launches drone_bridge so the /uav1 topics and command services are
# ready, but leaves mission launch under manual control.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
RNG_RUN="${RNG_RUN:-1}"

# Provides ARDUPILOT_HOME and other project-wide env vars
source "$PROJECT_DIR/setup.sh"

ARDU_WS_INSTALL="$HOME/ardu_ws/install/setup.bash"

NS3_ROOT="$HOME/ns-allinone-3.38/ns-3.38"
NS3_LOG="/tmp/ns3_single_mw.log"
GAZEBO_LOG="/tmp/gazebo_netns_mw.log"
AGENT_LOG="/tmp/agent_netns_mw.log"
BRIDGE_LOG="/tmp/drone_bridge_mw_uav1.log"
POSPUB_LOG="/tmp/pospub_netns_mw.log"
SITL_LOG_DIR="/tmp/sitl_netns_mw_uav1"
SNR_LOG="/tmp/ns3_snr_mw.csv"

# This world's actual spherical_coordinates origin (worlds/mech_workshop_validation.world).
HOME_GPS="6.0773722,80.1907552,0,0"
WORLD_PATH="$PROJECT_DIR/worlds/mech_workshop_validation.world"
DDS_PARM="$PROJECT_DIR/params/uav1_dds_netns.parm"

TAP_READY_TIMEOUT=30
AGENT_READY_TIMEOUT=20
DDS_GPS_TIMEOUT=90
SITL_TCP_TIMEOUT=90
GAZEBO_FDM_TIMEOUT=60
BRIDGE_READY_TIMEOUT=45

NS3_PID=""
GAZEBO_PID=""
SITL_PID=""
AGENT_PID=""
BRIDGE_PID=""
POSPUB_PID=""

[[ -f "$WORLD_PATH" ]] || { echo "ERROR: world file not found: $WORLD_PATH" >&2; exit 1; }
[[ -f "$DDS_PARM" ]] || { echo "ERROR: DDS parm file not found: $DDS_PARM" >&2; exit 1; }
[[ -f "$ARDU_WS_INSTALL" ]] || { echo "ERROR: ardu_ws overlay not found: $ARDU_WS_INSTALL" >&2; exit 1; }

echo "NS-3 RNG run : $RNG_RUN"

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
        --simTime=0 --uavAltitude=30 \
        --snrLogFile=$SNR_LOG --posLogPeriod=2.0 --rngRun=${RNG_RUN}"
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
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_PLUGIN_PATH="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_RESOURCE_PATH="$PROJECT_DIR:$PROJECT_DIR/worlds:${GAZEBO_RESOURCE_PATH:-}"

: > "$GAZEBO_LOG"
gazebo --verbose "$WORLD_PATH" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > "$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!

echo "  Waiting for Gazebo ArduPilot FDM port 9002..."
deadline=$((SECONDS + GAZEBO_FDM_TIMEOUT))
while ! ss -H -lun "sport = :9002" | grep -q .; do
    kill -0 "$GAZEBO_PID" 2>/dev/null || {
        echo "ERROR: Gazebo exited during startup." >&2
        cat "$GAZEBO_LOG" >&2
        exit 1
    }
    (( SECONDS >= deadline )) && {
        echo "ERROR: Gazebo FDM port 9002 never opened." >&2
        cat "$GAZEBO_LOG" >&2
        exit 1
    }
    sleep 0.25
done
echo "  Gazebo running: PID=$GAZEBO_PID  FDM port 9002 open"
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 4b — world_pos_publisher.py (feeds real Gazebo positions into ns-3's
# mobility model; without this, ns-3 nodes stay frozen at static placeholder
# coordinates regardless of where the drone actually flies)
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [4b/8] Starting world_pos_publisher.py (root namespace) ==="
: > "$POSPUB_LOG"
python3 "$PROJECT_DIR/scripts/world_pos_publisher.py" --ros-args -p n_uavs:=1 \
    > "$POSPUB_LOG" 2>&1 &
POSPUB_PID=$!
sleep 3
if ! kill -0 "$POSPUB_PID" 2>/dev/null; then
    echo "ERROR: world_pos_publisher.py exited during startup." >&2
    cat "$POSPUB_LOG" >&2
    exit 1
fi
if grep -q "Relaying /gazebo/model_states" "$POSPUB_LOG"; then
    echo "  world_pos_publisher.py running: PID=$POSPUB_PID"
else
    echo "  WARNING: world_pos_publisher.py started but did not confirm relay startup."
    echo "           Check $POSPUB_LOG"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════════════
# STEP 5 — micro_ros_agent inside gcsns
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [5/8] micro_ros_agent inside gcsns ==="
: > "$AGENT_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$2"
    source "$1"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
' agent-shell "$PROJECT_DIR/ros2/install/setup.bash" "$ARDU_WS_INSTALL" > "$AGENT_LOG" 2>&1 &
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

sudo ip netns exec uav1ns sudo -H -u "$RUN_USER" bash -c '
    cd "$1"
    shift
    exec "$@"
' sitl-shell \
    "$SITL_LOG_DIR" \
    strace -f -e trace=none -o /dev/null \
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
    if ! sudo kill -0 "$SITL_PID" 2>/dev/null; then
        echo "ERROR: SITL exited during startup." >&2
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
        source "$2"
        source "$1"
        ros2 topic info /ap/v1/navsat 2>/dev/null
    ' t-shell "$PROJECT_DIR/ros2/install/setup.bash" "$ARDU_WS_INSTALL" || true)
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
    source "$2"
    source "$1"
    exec ros2 run uav_controller drone_bridge --ros-args \
        -p uav_id:=1 -p mavlink_host:=10.42.0.11 -p mavlink_port:=5760 \
        -p takeoff_altitude:=3.09
' bridge-shell "$PROJECT_DIR/ros2/install/setup.bash" "$ARDU_WS_INSTALL" \
    > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

sleep 3
if ! sudo kill -0 "$BRIDGE_PID" 2>/dev/null; then
    echo "ERROR: drone_bridge exited during startup." >&2
    cat "$BRIDGE_LOG" >&2
    exit 1
fi

echo "  Waiting for a bridged GPS message on /uav1/gps..."
if ! sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$2"
    source "$1"
    timeout "$3" ros2 topic echo /uav1/gps sensor_msgs/msg/NavSatFix --once \
        >/dev/null
' gps-shell "$PROJECT_DIR/ros2/install/setup.bash" "$ARDU_WS_INSTALL" \
    "$BRIDGE_READY_TIMEOUT"; then
    echo "ERROR: drone_bridge did not publish /uav1/gps within ${BRIDGE_READY_TIMEOUT}s." >&2
    cat "$BRIDGE_LOG" >&2
    exit 1
fi
echo "  drone_bridge ready: /uav1/gps and /uav1 command services available"
echo ""

echo "════════════════════════════════════════════════════════════"
echo " PIPELINE READY — /uav1 GPS and command services available."
echo " The mission remains manually launched."
echo "════════════════════════════════════════════════════════════"
echo "  Run a mission script manually, INSIDE gcsns, e.g.:"
echo ""
echo "  sudo ip netns exec gcsns sudo -H -u $RUN_USER bash -lc '"
echo "      source /opt/ros/humble/setup.bash"
echo "      source $PROJECT_DIR/ros2/install/setup.bash"
echo "      source $ARDU_WS_INSTALL"
echo "      # your mission script here"
echo "  '"
echo ""
echo "  PIDs: ns3=$NS3_PID gazebo=$GAZEBO_PID pospub=$POSPUB_PID sitl=$SITL_PID agent=$AGENT_PID bridge=$BRIDGE_PID"
echo "  Logs: $NS3_LOG  $GAZEBO_LOG  $POSPUB_LOG  $SITL_LOG_DIR/arducopter.log  $AGENT_LOG  $BRIDGE_LOG"
echo "  Ctrl+C to shut down everything."
echo ""

cleanup() {
    echo "Shutting down..."
    kill "$BRIDGE_PID" "$AGENT_PID" "$SITL_PID" "$GAZEBO_PID" "$NS3_PID" "$POSPUB_PID" 2>/dev/null || true
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
