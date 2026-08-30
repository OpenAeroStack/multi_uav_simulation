#!/usr/bin/env bash
# Three-UAV + GCS namespace/NS-3/Gazebo launcher.
#
# Mirrors scripts/netns/launch_single_uav_netns.sh, but provisions real
# wireless and management paths, DDS agents, SITL instances, and bridges for
# UAV1/UAV2/UAV3. Missions and vision workloads remain manual.

set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
RNG_RUN="${RNG_RUN:-1}"
NS3_ROOT="${NS3_ROOT:-$HOME/ns-allinone-3.38/ns-3.38}"

# setup.sh predates strict nounset handling for these two variables.
export GAZEBO_PLUGIN_PATH="${GAZEBO_PLUGIN_PATH:-}"
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}"
source "$PROJECT_DIR/setup.sh"

WORLD_PATH="$PROJECT_DIR/worlds/small_city_base_netns_3uav.world"
HOME_GPS="${HOME_GPS:-6.0790684,80.1915283,0.00,0}"
LOG_ROOT="${LOG_ROOT:-/tmp/three_uav_netns}"

TAP_READY_TIMEOUT=30
AGENT_READY_TIMEOUT=30
SITL_READY_TIMEOUT=90
DDS_READY_TIMEOUT=120
MAVLINK_READY_TIMEOUT=90
GAZEBO_STARTUP_SECONDS=30
DISCOVERY_SERVER_ADDRESS="10.42.0.10"
DISCOVERY_SERVER_PORT=11811

NS3_PID=""; GAZEBO_PID=""; POSPUB_PID=""; DISCOVERY_SERVER_PID=""
DISCOVERY_SERVER_WRAPPER_PID=""
declare -a AGENT_PIDS=() SITL_PIDS=() BRIDGE_PIDS=()

usage() {
    echo "Usage: $0"
    echo "Environment: RNG_RUN=1 NS3_ROOT=<ns-3.38> LOG_ROOT=/tmp/three_uav_netns"
}
[[ "${1:-}" == "-h" || "${1:-}" == "--help" ]] && { usage; exit 0; }
[[ $# -eq 0 ]] || { usage >&2; exit 2; }

require_file() {
    [[ -f "$1" ]] || { echo "ERROR: required file not found: $1" >&2; exit 1; }
}
require_command() {
    command -v "$1" >/dev/null || { echo "ERROR: required command not found: $1" >&2; exit 1; }
}

for command_name in gazebo ros2 python3 ip ethtool timeout fastdds ss; do
    require_command "$command_name"
done
require_file "$WORLD_PATH"
require_file "$ARDUPILOT_HOME/build/sitl/bin/arducopter"
for uav in 1 2 3; do require_file "$PROJECT_DIR/params/uav${uav}_dds.parm"; done
[[ -x "$NS3_ROOT/ns3" ]] || { echo "ERROR: NS-3 wrapper not found: $NS3_ROOT/ns3" >&2; exit 1; }

sudo -v
sudo -n true
mkdir -p "$LOG_ROOT"

cleanup_owned_processes() {
    local status=$?
    trap - EXIT INT TERM
    echo "Shutting down three-UAV pipeline..."
    for pid in "${BRIDGE_PIDS[@]}" "${SITL_PIDS[@]}" "${AGENT_PIDS[@]}" \
               "$POSPUB_PID" "$GAZEBO_PID" "$NS3_PID" "$DISCOVERY_SERVER_PID"; do
        [[ -n "$pid" ]] || continue
        sudo -n kill -INT "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${BRIDGE_PIDS[@]}" "${SITL_PIDS[@]}" "${AGENT_PIDS[@]}" \
               "$POSPUB_PID" "$GAZEBO_PID" "$NS3_PID" "$DISCOVERY_SERVER_PID"; do
        [[ -n "$pid" ]] || continue
        sudo -n kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 1
    for ns in gcsns uav1ns uav2ns uav3ns; do
        sudo -n ip netns del "$ns" 2>/dev/null || true
    done
    for br in br-gcs br-uav1 br-uav2 br-uav3; do
        sudo -n ip link del "$br" type bridge 2>/dev/null || true
    done
    for link in tap-gcs tap-uav1 tap-uav2 tap-uav3 veth0h veth1h veth2h veth3h \
                sim1h sim2h sim3h; do
        sudo -n ip link del "$link" 2>/dev/null || true
    done
    exit "$status"
}
trap cleanup_owned_processes EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "=== [0/9] Scoped pre-flight cleanup ==="
for pattern in drone_bridge micro_ros_agent '/build/sitl/bin/arducopter' \
               three_uav_tapbridge_integrated fast-discovery-server gzserver gzclient; do
    sudo -n pkill -9 -f -- "$pattern" 2>/dev/null || true
done
for ns in gcsns uav1ns uav2ns uav3ns; do sudo -n ip netns del "$ns" 2>/dev/null || true; done
for br in br-gcs br-uav1 br-uav2 br-uav3; do sudo -n ip link del "$br" type bridge 2>/dev/null || true; done
for link in tap-gcs tap-uav1 tap-uav2 tap-uav3 veth0h veth1h veth2h veth3h \
            sim1h sim2h sim3h; do
    sudo -n ip link del "$link" 2>/dev/null || true
done
sleep 2

echo "=== [1/9] Four-node wireless namespace/TAP topology ==="
setup_ns() {
    local ns="$1" tap="$2" bridge="$3" host_veth="$4" ns_veth="$5" address="$6"
    sudo -n ip netns add "$ns"
    sudo -n ip tuntap add dev "$tap" mode tap user "$RUN_USER"
    sudo -n ip link set "$tap" up
    sudo -n ip link add "$bridge" type bridge
    sudo -n ip link set "$tap" master "$bridge"
    sudo -n ip link set "$bridge" up
    sudo -n ip link add "$host_veth" type veth peer name "$ns_veth"
    sudo -n ip link set "$host_veth" master "$bridge"
    sudo -n ip link set "$host_veth" up
    sudo -n ip link set "$ns_veth" netns "$ns"
    sudo -n ip netns exec "$ns" ip link set lo up
    sudo -n ip netns exec "$ns" ip link set "$ns_veth" up
    sudo -n ip netns exec "$ns" ip addr add "$address" dev "$ns_veth"
    sudo -n ethtool -K "$host_veth" rx off tx off sg off tso off gso off gro off 2>/dev/null || true
    sudo -n ip netns exec "$ns" ethtool -K "$ns_veth" \
        rx off tx off sg off tso off gso off gro off 2>/dev/null || true
    echo "  $ns: $address via $tap/$bridge"
}
setup_ns gcsns  tap-gcs  br-gcs  veth0h veth0n 10.42.0.10/24
sudo -n ip addr add 10.42.0.1/24 dev br-gcs
setup_ns uav1ns tap-uav1 br-uav1 veth1h veth1n 10.42.0.11/24
setup_ns uav2ns tap-uav2 br-uav2 veth2h veth2n 10.42.0.12/24
setup_ns uav3ns tap-uav3 br-uav3 veth3h veth3n 10.42.0.13/24

echo "=== [2/9] Three private SITL/Gazebo management links ==="
setup_management_link() {
    local uav="$1" ns="uav${1}ns" host="sim${1}h" peer="sim${1}n"
    sudo -n ip link add "$host" type veth peer name "$peer"
    sudo -n ip addr add "172.31.${uav}.1/30" dev "$host"
    sudo -n ip link set "$host" up
    sudo -n ip link set "$peer" netns "$ns"
    sudo -n ip netns exec "$ns" ip addr add "172.31.${uav}.2/30" dev "$peer"
    sudo -n ip netns exec "$ns" ip link set "$peer" up
    sudo -n ethtool -K "$host" rx off tx off sg off tso off gso off gro off 2>/dev/null || true
    sudo -n ip netns exec "$ns" ethtool -K "$peer" \
        rx off tx off sg off tso off gso off gro off 2>/dev/null || true
    echo "  UAV$uav: root 172.31.${uav}.1 <-> $ns 172.31.${uav}.2"
}
for uav in 1 2 3; do setup_management_link "$uav"; done

echo "=== [3/9] NS-3 shared wireless channel ==="
(cd "$NS3_ROOT" && ./ns3 build three_uav_tapbridge_integrated)
NS3_LOG="$LOG_ROOT/ns3.log"; SNR_LOG="$LOG_ROOT/ns3_snr.csv"
: >"$NS3_LOG"
(
    cd "$NS3_ROOT"
    exec ./ns3 run "three_uav_tapbridge_integrated \
        --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
        --enableTap=true --simTime=0 --uavAltitude=30 \
        --snrLogFile=$SNR_LOG --posLogPeriod=2.0 --rngRun=$RNG_RUN"
) >"$NS3_LOG" 2>&1 &
NS3_PID=$!
deadline=$((SECONDS + TAP_READY_TIMEOUT))
while true; do
    kill -0 "$NS3_PID" 2>/dev/null || { cat "$NS3_LOG" >&2; echo "ERROR: NS-3 exited" >&2; exit 1; }
    all_attached=true
    for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
        [[ -r "/sys/class/net/$tap/carrier" && "$(<"/sys/class/net/$tap/carrier")" == 1 ]] || all_attached=false
    done
    $all_attached && break
    (( SECONDS < deadline )) || { cat "$NS3_LOG" >&2; echo "ERROR: TAP attachment timeout" >&2; exit 1; }
    sleep 0.2
done
for uav in 1 2 3; do
    sudo -n ip netns exec gcsns ping -c 2 -W 2 "10.42.0.1$uav" >/dev/null || {
        echo "ERROR: GCS cannot reach UAV$uav through NS-3" >&2; exit 1; }
done
echo "  All three GCS/UAV wireless paths respond."

echo "=== [3b/9] Fast DDS Discovery Server in gcsns ==="
DISCOVERY_SERVER_LOG="$LOG_ROOT/fastdds_discovery_server.log"; : >"$DISCOVERY_SERVER_LOG"
sudo -n setsid ip netns exec gcsns runuser -u "$RUN_USER" -- bash -lc '
    source /opt/ros/humble/setup.bash
    exec fastdds discovery -i 0 -l "$1" -p "$2"
' discovery-server-shell "$DISCOVERY_SERVER_ADDRESS" "$DISCOVERY_SERVER_PORT" \
    >"$DISCOVERY_SERVER_LOG" 2>&1 &
DISCOVERY_SERVER_WRAPPER_PID=$!
deadline=$((SECONDS + 10))
while (( SECONDS < deadline )); do
    listener="$(sudo -n ip netns exec gcsns ss -H -lunp \
        "sport = :$DISCOVERY_SERVER_PORT")"
    candidate_pid="$(sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' <<<"$listener" | head -1)"
    if [[ "$candidate_pid" =~ ^[1-9][0-9]*$ ]] &&
       sudo -n kill -0 "$candidate_pid" 2>/dev/null &&
       sudo -n tr '\0' ' ' <"/proc/$candidate_pid/cmdline" | \
           grep -Fq fast-discovery-server; then
        DISCOVERY_SERVER_PID="$candidate_pid"
        break
    fi
    sleep 0.2
done
[[ "$DISCOVERY_SERVER_PID" =~ ^[1-9][0-9]*$ ]] || {
    cat "$DISCOVERY_SERVER_LOG" >&2
    echo "ERROR: Fast DDS Discovery Server startup timeout" >&2
    exit 1
}
echo "  Discovery Server ready: $DISCOVERY_SERVER_ADDRESS:$DISCOVERY_SERVER_PORT"

echo "=== [4/9] Gazebo three-UAV small-city world ==="
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$HOME/FYP/small_city_gazebo_world/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_RESOURCE_PATH="$PROJECT_DIR:$PROJECT_DIR/worlds:${GAZEBO_RESOURCE_PATH:-}"
GAZEBO_LOG="$LOG_ROOT/gazebo.log"; : >"$GAZEBO_LOG"
env -u ROS_DISCOVERY_SERVER -u ROS_SUPER_CLIENT \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
    gazebo --verbose "$WORLD_PATH" -s libgazebo_ros_init.so -s libgazebo_ros_factory.so \
    >"$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!
echo "  Waiting ${GAZEBO_STARTUP_SECONDS}s for Gazebo..."
sleep "$GAZEBO_STARTUP_SECONDS"
kill -0 "$GAZEBO_PID" 2>/dev/null || { cat "$GAZEBO_LOG" >&2; echo "ERROR: Gazebo exited" >&2; exit 1; }
if grep -q 'failed to bind with 172\.31\.' "$GAZEBO_LOG"; then
    grep 'failed to bind with 172\.31\.' "$GAZEBO_LOG" >&2
    echo "ERROR: an ArduPilot Gazebo plugin failed to bind" >&2
    exit 1
fi
for uav in 1 2 3; do
    sudo -n ss -H -lun "sport = :$((8992 + 10*uav))" | grep -q . || {
        echo "ERROR: Gazebo FDM listener for UAV$uav is absent" >&2; exit 1; }
done
echo "  Gazebo has all three FDM listeners."

echo "=== [5/9] Gazebo position feed to NS-3 ==="
POSPUB_LOG="$LOG_ROOT/world_positions.log"; : >"$POSPUB_LOG"
env -u ROS_DISCOVERY_SERVER -u ROS_SUPER_CLIENT \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
    python3 "$PROJECT_DIR/scripts/world_pos_publisher.py" >"$POSPUB_LOG" 2>&1 &
POSPUB_PID=$!
sleep 3
kill -0 "$POSPUB_PID" 2>/dev/null || { cat "$POSPUB_LOG" >&2; echo "ERROR: position publisher exited" >&2; exit 1; }

echo "=== [6/9] Three micro-ROS agents inside gcsns ==="
for uav in 1 2 3; do
    port=$((2018 + uav)); log="$LOG_ROOT/agent_uav${uav}.log"; : >"$log"
    sudo -n ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
        source /opt/ros/humble/setup.bash
        source "$HOME/FYP/ardu_ws/install/setup.bash"
        source "$1"
        exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$2"
    ' agent-shell "$PROJECT_DIR/ros2/install/setup.bash" "$port" >"$log" 2>&1 &
    AGENT_PIDS+=("$!")
done
deadline=$((SECONDS + AGENT_READY_TIMEOUT))
while true; do
    all_ready=true
    for uav in 1 2 3; do
        pid="${AGENT_PIDS[$((uav-1))]}"; port=$((2018 + uav))
        sudo -n kill -0 "$pid" 2>/dev/null || { cat "$LOG_ROOT/agent_uav${uav}.log" >&2; exit 1; }
        sudo -n ip netns exec gcsns ss -H -lun "sport = :$port" | grep -q . || all_ready=false
    done
    $all_ready && break
    (( SECONDS < deadline )) || { echo "ERROR: DDS agent readiness timeout" >&2; exit 1; }
    sleep 0.2
done
echo "  DDS agents listening on 2019/2020/2021."

echo "=== [7/9] Three isolated ArduPilot SITL instances ==="
BASE_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
for uav in 1 2 3; do
    instance=$((uav-1)); mavport=$((5750 + 10*uav)); fdmout=$((8993 + 10*uav))
    sitl_dir="$LOG_ROOT/sitl_uav${uav}"; mkdir -p "$sitl_dir"
    sudo -n chown "$RUN_USER":"$(id -gn "$RUN_USER")" "$sitl_dir"
    sudo -n ip netns exec "uav${uav}ns" sudo -H -u "$RUN_USER" bash -c '
        cd "$1"; shift
        exec strace -f -e trace=none -o /dev/null "$@"
    ' sitl-shell "$sitl_dir" "$ARDUPILOT_HOME/build/sitl/bin/arducopter" \
        --wipe --model gazebo-iris --speedup 1 --sysid "$uav" --instance "$instance" \
        --defaults "$BASE_DEFAULTS,$PROJECT_DIR/params/uav${uav}_dds.parm" \
        --sim-address "172.31.${uav}.1" --home "$HOME_GPS" \
        --serial0="tcp:0.0.0.0:$mavport" >"$sitl_dir/arducopter.log" 2>&1 &
    SITL_PIDS+=("$!")
done
deadline=$((SECONDS + SITL_READY_TIMEOUT))
while true; do
    all_ready=true
    for uav in 1 2 3; do
        pid="${SITL_PIDS[$((uav-1))]}"; fdmout=$((8993 + 10*uav))
        if ! sudo -n kill -0 "$pid" 2>/dev/null; then
            cat "$LOG_ROOT/sitl_uav${uav}/arducopter.log" >&2
            echo "ERROR: UAV$uav SITL exited" >&2; exit 1
        fi
        sudo -n ip netns exec "uav${uav}ns" ss -H -lun "sport = :$fdmout" | grep -q . || all_ready=false
    done
    $all_ready && break
    (( SECONDS < deadline )) || { echo "ERROR: SITL FDM readiness timeout" >&2; exit 1; }
    sleep 0.3
done
echo "  SITL FDM ports 9003/9013/9023 are ready."

echo "=== [8/9] DDS and MAVLink verification ==="
for uav in 1 2 3; do
    topic="/ap/v${uav}/navsat"
    deadline=$((SECONDS + DDS_READY_TIMEOUT))
    while true; do
        info="$(sudo -n ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
            source /opt/ros/humble/setup.bash
            source "$HOME/FYP/ardu_ws/install/setup.bash"
            source "$1"
            ros2 topic info "$2" 2>/dev/null
        ' topic-shell "$PROJECT_DIR/ros2/install/setup.bash" "$topic" || true)"
        echo "$info" | grep -q 'Publisher count: [1-9]' && break
        (( SECONDS < deadline )) || { echo "ERROR: no publisher on $topic" >&2; exit 1; }
        sleep 1
    done
    mavport=$((5750 + 10*uav)); deadline=$((SECONDS + MAVLINK_READY_TIMEOUT))
    until sudo -n ip netns exec gcsns timeout 1 bash -c \
        "exec 3<>/dev/tcp/10.42.0.1${uav}/${mavport}" 2>/dev/null; do
        (( SECONDS < deadline )) || { echo "ERROR: UAV$uav MAVLink TCP $mavport unreachable" >&2; exit 1; }
        sleep 0.5
    done
    echo "  UAV$uav: $topic and MAVLink TCP $mavport ready."
done

echo "=== [9/9] Three GCS-side drone bridges ==="
for uav in 1 2 3; do
    mavport=$((5750 + 10*uav)); log="$LOG_ROOT/bridge_uav${uav}.log"; : >"$log"
    sudo -n ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
        source /opt/ros/humble/setup.bash
        source "$HOME/FYP/ardu_ws/install/setup.bash"
        source "$1"
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export ROS_DOMAIN_ID=0
        export ROS_DISCOVERY_SERVER=10.42.0.10:11811
        exec ros2 run uav_controller drone_bridge --ros-args \
            -p uav_id:="$2" -p mavlink_host:="$3" -p mavlink_port:="$4"
    ' bridge-shell "$PROJECT_DIR/ros2/install/setup.bash" "$uav" \
        "10.42.0.1${uav}" "$mavport" >"$log" 2>&1 &
    BRIDGE_PIDS+=("$!")
done
sleep 5
for uav in 1 2 3; do
    pid="${BRIDGE_PIDS[$((uav-1))]}"
    sudo -n kill -0 "$pid" 2>/dev/null || {
        cat "$LOG_ROOT/bridge_uav${uav}.log" >&2
        echo "ERROR: UAV$uav bridge exited" >&2; exit 1; }
done

echo ""
echo "============================================================"
echo " THREE-UAV PIPELINE READY"
echo "============================================================"
echo " World : $WORLD_PATH"
echo " RNG   : $RNG_RUN"
echo " Logs  : $LOG_ROOT"
echo " ROS   : /ap/v1, /ap/v2, /ap/v3 and /uav1, /uav2, /uav3"
echo " Missions and vision workloads are intentionally not started."
echo " Press Ctrl+C to stop runner-owned processes."
echo ""

wait
