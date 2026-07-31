#!/bin/bash
# launch_netns_v2.sh — single UAV + GCS, following the PROVEN startup order:
#   topology -> Gazebo -> agent -> SITL -> ns-3 -> verify -> drone_bridge
#
# Rationale for ordering: SITL's early init appears timing-sensitive. Starting
# ns-3 (a CPU-hungry real-time discrete-event sim) BEFORE SITL reliably caused
# SITL to exit(1) silently during startup. The proven manual sequence starts
# ns-3 last, after SITL is already stable. Do not reorder without testing.
#
# Stops after bring-up. Mission/detector/metrics are launched manually.

set -uo pipefail   # NOTE: no -e; we handle failures explicitly per step

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"

source "$PROJECT_DIR/setup.sh"

NS3_ROOT="$HOME/ns-allinone-3.38/ns-3.38"
WORLD_PATH="$PROJECT_DIR/worlds/small_city_single_uav_netns.world"
DDS_PARM="$PROJECT_DIR/params/uav1_dds_netns.parm"
HOME_GPS="6.0790684,80.1915283,0.00,0"

GAZEBO_LOG="/tmp/gazebo_netns.log"
AGENT_LOG="/tmp/agent_netns.log"
NS3_LOG="/tmp/ns3_single.log"
BRIDGE_LOG="/tmp/bridge_netns.log"
POSPUB_LOG="/tmp/pospub_netns.log"
SITL_DIR="/tmp/sitl_netns_uav1"
SITL_LOG="$SITL_DIR/arducopter.log"

GAZEBO_PID=""; AGENT_PID=""; SITL_PID=""; NS3_PID=""; BRIDGE_PID=""; POSPUB_PID=""

die() { echo ""; echo "FATAL: $*" >&2; echo "  (leaving processes up for inspection; run kill_all_netns.sh to clean)" >&2; exit 1; }

# Liveness check that works through the sudo/netns wrapper chain: match the
# real binary inside the namespace rather than trusting $! (which can be a
# short-lived sudo wrapper).
sitl_alive() { sudo ip netns exec uav1ns pgrep -f "sitl/bin/arducopter" >/dev/null 2>&1; }

# ══════════════════════════════════════════════════════════════════════════
echo "=== [0/8] Cleanup ==="
bash "$SCRIPT_DIR/kill_all_netns.sh" >/dev/null 2>&1
sleep 2
rm -rf "$SITL_DIR"; mkdir -p "$SITL_DIR"
echo "  clean"
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [1/8] Wireless topology (gcsns + uav1ns) ==="
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
  echo "  $NS -> $NS_IP"
}
setup_ns gcsns  tap-gcs  br-gcs  veth0h veth0n 10.42.0.10/24
setup_ns uav1ns tap-uav1 br-uav1 veth1h veth1n 10.42.0.11/24

# ns-3 binary hardcodes 4 nodes and requires all 4 tap args; UAV2/3 get bare
# unbridged TAPs so those simulated nodes exist but carry no real traffic.
for tap in tap-uav2 tap-uav3; do
    sudo ip tuntap add dev "$tap" mode tap user "$RUN_USER" 2>/dev/null || true
    sudo ip link set "$tap" up
done
echo "  tap-uav2/tap-uav3 up (inert placeholders)"

echo "--- Management link (FDM physics, bypasses ns-3) ---"
sudo ip link add sim1h type veth peer name sim1n 2>/dev/null || true
sudo ip addr add 172.31.1.1/30 dev sim1h 2>/dev/null || true
sudo ip link set sim1h up
sudo ip link set sim1n netns uav1ns
sudo ip netns exec uav1ns ip addr add 172.31.1.2/30 dev sim1n 2>/dev/null || true
sudo ip netns exec uav1ns ip link set sim1n up
sudo ethtool -K sim1h rx off tx off sg off tso off gso off gro off 2>/dev/null || true
sudo ip netns exec uav1ns ethtool -K sim1n rx off tx off sg off tso off gso off gro off 2>/dev/null || true
echo "  172.31.1.1 <-> 172.31.1.2"
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [2/8] Gazebo (headless) ==="
[[ -f "$WORLD_PATH" ]] || die "world file missing: $WORLD_PATH"

export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$HOME/FYP/small_city_gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_RESOURCE_PATH="$PROJECT_DIR:$PROJECT_DIR/worlds:${GAZEBO_RESOURCE_PATH:-}"

: > "$GAZEBO_LOG"
gzserver --verbose "$WORLD_PATH" > "$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!
echo "  PID=$GAZEBO_PID, waiting 30s..."
sleep 30
kill -0 "$GAZEBO_PID" 2>/dev/null || { cat "$GAZEBO_LOG" >&2; die "Gazebo died during startup"; }

if grep -qi "Unable to find\|Error Code" "$GAZEBO_LOG"; then
    echo "  WARNING: Gazebo logged missing-asset/errors — check $GAZEBO_LOG"
    echo "           (human/vehicle models may not have rendered)"
fi
echo "  Gazebo up"
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [3/8] micro_ros_agent inside gcsns ==="
: > "$AGENT_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    exec ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
' agent-shell > "$AGENT_LOG" 2>&1 &
AGENT_PID=$!

deadline=$((SECONDS + 25))
until sudo ip netns exec gcsns ss -H -lun "sport = :2019" | grep -q .; do
    (( SECONDS >= deadline )) && { cat "$AGENT_LOG" >&2; die "agent port 2019 never opened"; }
    sleep 0.3
done
echo "  agent listening on 2019"
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [4/8] SITL inside uav1ns  (BEFORE ns-3 — this ordering matters) ==="
sudo ip netns exec uav1ns sudo -H -u "$RUN_USER" bash -c '
    cd "$1"
    exec "$2" --model gazebo-iris --speedup 1 --sysid 1 --instance 0 \
        --defaults "$3" --sim-address "$4" --home "$5" --serial0=tcp:0.0.0.0:5760
' sitl-shell \
    "$SITL_DIR" \
    "$ARDUPILOT_HOME/build/sitl/bin/arducopter" \
    "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm,$DDS_PARM" \
    "172.31.1.1" "$HOME_GPS" \
    > "$SITL_LOG" 2>&1 &
SITL_PID=$!

echo "  waiting for SITL FDM port 9003 (up to 90s)..."
deadline=$((SECONDS + 90))
until sudo ip netns exec uav1ns ss -H -lun "sport = :9003" | grep -q .; do
    if ! sitl_alive; then
        echo "" >&2
        echo "--- SITL is gone. Log: ---" >&2
        cat "$SITL_LOG" >&2
        echo "--------------------------" >&2
        echo "If the log stops right after the first 'Loaded defaults' line," >&2
        echo "this is the early-exit issue. FALLBACK: run SITL manually in its" >&2
        echo "own terminal (it survives reliably when attached to a TTY):" >&2
        echo "" >&2
        echo "  source $PROJECT_DIR/setup.sh" >&2
        echo "  sudo ip netns exec uav1ns sudo -H -u $RUN_USER bash -c 'cd $SITL_DIR && exec \"\$ARDUPILOT_HOME/build/sitl/bin/arducopter\" --model gazebo-iris --speedup 1 --sysid 1 --instance 0 --defaults \"...\" --sim-address 172.31.1.1 --home $HOME_GPS --serial0=tcp:0.0.0.0:5760'" >&2
        echo "" >&2
        die "SITL exited during startup"
    fi
    (( SECONDS >= deadline )) && { cat "$SITL_LOG" >&2; die "SITL alive but FDM port never opened"; }
    sleep 0.5
done
echo "  SITL alive, FDM port open"

# Confirm Gazebo actually latched onto it (physics link, not just port open)
sleep 5
if grep -q "ArduPilot controller online detected" "$GAZEBO_LOG"; then
    echo "  Gazebo <-> SITL physics link confirmed"
else
    echo "  WARNING: Gazebo hasn't reported 'controller online detected' yet."
    echo "           If GPS never appears later, suspect the FDM link."
fi
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [5/8] ns-3 wireless channel (LAST — after SITL is stable) ==="
(cd "$NS3_ROOT" && ./ns3 build three_uav_tapbridge_integrated) || die "ns-3 build failed"

: > "$NS3_LOG"
( cd "$NS3_ROOT"
  exec ./ns3 run "three_uav_tapbridge_integrated \
      --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
      --enableTap=true --simTime=0 --uavAltitude=30 --posLogPeriod=10"
) > "$NS3_LOG" 2>&1 &
NS3_PID=$!

echo "  waiting for TAP attachment (up to 40s)..."
deadline=$((SECONDS + 40))
while true; do
    kill -0 "$NS3_PID" 2>/dev/null || { cat "$NS3_LOG" >&2; die "ns-3 exited early"; }
    all_up=true
    for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
        cf="/sys/class/net/$tap/carrier"
        [[ -r "$cf" && "$(<"$cf")" == 1 ]] || all_up=false
    done
    $all_up && break
    (( SECONDS >= deadline )) && { cat "$NS3_LOG" >&2; die "TAPs never attached"; }
    sleep 0.3
done
echo "  TAPs attached"

# SITL must have survived ns-3 starting up (CPU contention spike)
sitl_alive || { cat "$SITL_LOG" >&2; die "SITL died when ns-3 started (CPU contention)"; }
echo "  SITL survived ns-3 startup"

echo "  verifying wireless path gcsns -> uav1ns..."
sudo ip netns exec gcsns ping -c 3 -W 2 10.42.0.11 >/dev/null 2>&1 \
    || die "no wireless connectivity gcsns -> uav1ns"
echo "  wireless link OK"
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [6/8] Position publisher (feeds real UAV position to ns-3 mobility) ==="
if [[ -f "$PROJECT_DIR/scripts/world_pos_publisher.py" ]]; then
    : > "$POSPUB_LOG"
    ( source /opt/ros/humble/setup.bash
      exec python3 "$PROJECT_DIR/scripts/world_pos_publisher.py"
    ) > "$POSPUB_LOG" 2>&1 &
    POSPUB_PID=$!
    sleep 3
    if kill -0 "$POSPUB_PID" 2>/dev/null; then
        echo "  running (PID=$POSPUB_PID)"
    else
        echo "  WARNING: position publisher exited — ns-3 nodes will stay at"
        echo "           default positions, so path loss won't track real flight."
        cat "$POSPUB_LOG"
    fi
else
    echo "  WARNING: world_pos_publisher.py not found — ns-3 mobility will be static."
fi
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [7/8] Verify AP_DDS GPS reaches gcsns ==="
echo "  waiting for a publisher on /ap/v1/navsat (up to 120s)..."
deadline=$((SECONDS + 120))
got_pub=false
while (( SECONDS < deadline )); do
    sitl_alive || { cat "$SITL_LOG" >&2; die "SITL died while waiting for DDS"; }
    info=$(sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
        source /opt/ros/humble/setup.bash
        source "$HOME/ardu_ws/install/setup.bash"
        ros2 topic info /ap/v1/navsat 2>/dev/null
    ' t-shell 2>/dev/null || true)
    if echo "$info" | grep -q "Publisher count: [1-9]"; then got_pub=true; break; fi
    sleep 2
done

if ! $got_pub; then
    echo "--- agent log ---" >&2; tail -30 "$AGENT_LOG" >&2
    echo "--- SITL log ---"  >&2; tail -30 "$SITL_LOG"  >&2
    echo "" >&2
    echo "No DDS publisher. Check in order:" >&2
    echo "  1. agent log for 'session established' (did SITL's DDS client connect?)" >&2
    echo "  2. DDS_IP0-3 in $DDS_PARM must be 10/42/0/10 (gcsns address)" >&2
    echo "  3. sudo ip netns exec uav1ns ping -c2 10.42.0.10" >&2
    die "AP_DDS GPS never appeared in gcsns"
fi
echo "  /ap/v1/navsat has a live publisher"
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "=== [8/8] drone_bridge inside gcsns ==="
[[ -f "$PROJECT_DIR/ros2/install/setup.bash" ]] \
    || die "ROS 2 workspace not built. Run: cd $PROJECT_DIR/ros2 && colcon build"

echo "  waiting for SITL MAVLink TCP 5760 reachable from gcsns..."
deadline=$((SECONDS + 60))
until sudo ip netns exec gcsns sudo -H -u "$RUN_USER" \
        timeout 1 bash -c 'exec 3<>/dev/tcp/10.42.0.11/5760' 2>/dev/null; do
    (( SECONDS >= deadline )) && die "SITL MAVLink port not reachable from gcsns"
    sleep 0.5
done
echo "  MAVLink reachable at 10.42.0.11:5760"

: > "$BRIDGE_LOG"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc '
    source /opt/ros/humble/setup.bash
    source "$HOME/ardu_ws/install/setup.bash"
    source "$1"
    exec ros2 run uav_controller drone_bridge --ros-args \
        -p uav_id:=1 -p mavlink_host:=10.42.0.11 -p mavlink_port:=5760 \
        -p takeoff_altitude:=30.0
' bridge-shell "$PROJECT_DIR/ros2/install/setup.bash" > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

sleep 8
sudo kill -0 "$BRIDGE_PID" 2>/dev/null || { cat "$BRIDGE_LOG" >&2; die "drone_bridge exited during startup"; }

if grep -q "GPS flowing" "$BRIDGE_LOG"; then
    echo "  drone_bridge: DDS GPS confirmed flowing"
else
    echo "  drone_bridge started; GPS confirmation not logged yet (watch $BRIDGE_LOG)"
fi
echo ""

# ══════════════════════════════════════════════════════════════════════════
echo "════════════════════════════════════════════════════════════════"
echo " PIPELINE READY"
echo "════════════════════════════════════════════════════════════════"
echo " Logs:  $GAZEBO_LOG"
echo "        $SITL_LOG"
echo "        $AGENT_LOG"
echo "        $NS3_LOG"
echo "        $BRIDGE_LOG"
echo "        $POSPUB_LOG"
echo ""
echo " Health checks:"
echo "   gz stats                                  # RTF (expect ~0.7 under load)"
echo "   sudo ip netns exec gcsns sudo -H -u $RUN_USER bash -lc '"
echo "       source /opt/ros/humble/setup.bash"
echo "       source \$HOME/ardu_ws/install/setup.bash"
echo "       ros2 topic hz /ap/v1/navsat'"
echo ""
echo " Next (each in its own terminal, all INSIDE gcsns):"
echo "   1. Mission:   python3 $PROJECT_DIR/ros2/uav_controller/uav_controller/uav1_patrol_mission.py"
echo "   2. Detector:  (~/yolo_env active) python3 $PROJECT_DIR/ros2/uav_vision/uav_vision/detector.py \\"
echo "                    --ros-args -p uav_id:=1 -p processing_mode:=edge -p target_classes:=0"
echo "   3. Metrics:   python3 $PROJECT_DIR/ros2/uav_vision/uav_vision/metrics_logger.py \\"
echo "                    --ros-args -p processing_mode:=edge -p run_id:=edge_run1"
echo ""
echo " Ctrl+C to tear everything down."
echo ""

cleanup() {
    echo ""; echo "Shutting down..."
    bash "$SCRIPT_DIR/kill_all_netns.sh"
    exit 0
}
trap cleanup INT TERM
wait
