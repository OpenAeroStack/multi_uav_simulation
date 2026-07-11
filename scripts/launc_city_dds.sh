
#!/bin/bash
# launch_city_dds.sh
# ------------------
# Launches small_city world with 3 UAVs, DDS, and drone_bridge nodes,
# with GCS<->UAV ROS 2 traffic routed through the ns-3 Nakagami/log-distance
# wireless channel model (see setup_tap_only.sh + three_uav_tapbridge_rt.cc).
#
# UAV roles:
#   UAV1 (port 5760) → CLUSTER HEAD — circles city then hovers centre
#   UAV2 (port 5770) → Member — pond side patrol
#   UAV3 (port 5780) → Member — mountain side patrol
#
# Takeoff altitudes (vertically separated for collision avoidance):
#   UAV1 → 60m (cluster head, highest — best relay position)
#   UAV2 → 40m (pond side — flat terrain, lower altitude fine)
#   UAV3 → 50m (mountain side — mid altitude)
#
# Then in terminal 2:
#   bash scripts/run_city_mission.sh
#   (this replaces `ros2 run uav_controller city_mission` directly — it also
#    pins city_mission's DDS traffic to the tap-gcs IP)
 
set -eo pipefail
 
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
 
# EDIT THIS to your actual ns-3 checkout path
NS3_ROOT="${NS3_ROOT:-$HOME/ns-3-dev}"
 
source "$PROJECT_DIR/setup.sh"
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source "$PROJECT_DIR/ros2/install/setup.bash" 2>/dev/null || {
    echo "ERROR: ROS2 package not built. Run: bash build_ros2.sh"
    exit 1
}
 
# ── Small city world model and resource paths ─────────────────────────────────
export GAZEBO_MODEL_PATH=~/simulation/small_city_gazebo_world/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_RESOURCE_PATH=~/simulation/small_city_gazebo_world:$GAZEBO_RESOURCE_PATH
 
echo "=== Killing previous instances ==="
pkill -f arducopter      2>/dev/null || true
pkill -f gzserver        2>/dev/null || true
pkill -f gzclient        2>/dev/null || true
pkill -f micro_ros_agent 2>/dev/null || true
pkill -f drone_bridge    2>/dev/null || true
pkill -f three-uav       2>/dev/null || true   # ns-3 realtime channel sim
sleep 3
 
if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
    echo "ERROR: ARDUPILOT_HOME not set. Check setup.sh"
    exit 1
fi
 
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
BASE="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
UAV1_DEFAULTS="$BASE,$PROJECT_DIR/params/uav1_dds.parm"
UAV2_DEFAULTS="$BASE,$PROJECT_DIR/params/uav2_dds.parm"
UAV3_DEFAULTS="$BASE,$PROJECT_DIR/params/uav3_dds.parm"
 
# GPS home: San Jose CA — matches spherical_coordinates in city_3uav.world
# All 3 SITL instances use same home (spawn offset handled by Gazebo pose)
HOME_GPS="37.3382,-121.8863,0,0"
 
# ── 0. TAP devices + ns-3 realistic wireless channel ─────────────────────────
echo ""
echo "=== [0/5] Setting up TAP devices + ns-3 realistic wireless channel ==="
sudo "$SCRIPT_DIR/setup_tap_only.sh"
 
pushd "$NS3_ROOT" >/dev/null
./ns3 run "three-uav \
  --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
  --simDurationSec=0 \
  --nakagamiM=1.5 --shadowingStdDb=4.0 --txPowerDbm=20 --uavAltitude=50 \
  --enableFlowMonitor=true --snrLogFile=/tmp/snr_log.csv \
  --animFile=/tmp/three_uav_anim.xml" &
NS3_PID=$!
popd >/dev/null
 
cleanup() {
  echo ""
  echo "=== Cleaning up ns-3 + TAP devices ==="
  kill "$NS3_PID" 2>/dev/null || true
  sudo ip link del tap-gcs  2>/dev/null || true
  sudo ip link del tap-uav1 2>/dev/null || true
  sudo ip link del tap-uav2 2>/dev/null || true
  sudo ip link del tap-uav3 2>/dev/null || true
}
trap cleanup EXIT
 
echo "Waiting for ns-3 to attach TAP devices (5s)..."
sleep 5
 
# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [1/5] Launching Gazebo with city world ==="
gazebo --verbose "$PROJECT_DIR/worlds/city_3uav.world" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Waiting for Gazebo to load (20s)..."
sleep 20
 
# ── 2. micro_ros_agent × 3 ───────────────────────────────────────────────────
echo ""
echo "=== [2/5] Starting 3 micro_ros_agent instances ==="
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 &
sleep 1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2020 &
sleep 1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2021 &
sleep 2
echo "micro_ros_agents running on ports 2019-2021"
 
# ── 3. ArduPilot SITL × 3 ────────────────────────────────────────────────────
echo ""
echo "=== [3/5] Launching 3 SITL instances ==="
cd ~/
 
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address=127.0.0.1 -I0 \
    --home $HOME_GPS \ --out udp:127.0.0.1:14550  &
sleep 2
 
$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS --sim-address=127.0.0.1 -I1 \
    --home $HOME_GPS &
sleep 2
 
$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS --sim-address=127.0.0.1 -I2 \
    --home $HOME_GPS &
 
sleep 3
echo "Waiting for SITL instances to boot (15s)..."
sleep 15
 
# ── 4. drone_bridge × 3 — DDS traffic pinned to the ns-3 tap link ───────────
echo ""
echo "=== [4/5] Starting 3 drone_bridge nodes (DDS routed via ns-3 taps) ==="
 
# Builds a per-node CycloneDDS profile: bind to this node's tap IP, disable
# multicast (it won't cross the emulated link), and list explicit peers.
# Requires: sudo apt install ros-humble-rmw-cyclonedds-cpp
dds_uri_for() {
  local self_ip="$1"; shift
  local peers=""
  for p in "$@"; do peers+="<Peer address=\"$p\"/>"; done
  cat <<EOF
<CycloneDDS><Domain><General><NetworkInterfaceAddress>${self_ip}</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers>${peers}</Peers></Discovery></Domain></CycloneDDS>
EOF
}
 
GCS_IP="10.42.0.10"
UAV1_IP="10.42.0.11"
UAV2_IP="10.42.0.12"
UAV3_IP="10.42.0.13"
 
# UAV1 — CLUSTER HEAD — highest altitude for best relay coverage
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI="$(dds_uri_for "$UAV1_IP" "$GCS_IP")" \
  ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=60.0 &
sleep 1
 
# UAV2 — Member pond side — lower altitude, flat terrain
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI="$(dds_uri_for "$UAV2_IP" "$GCS_IP")" \
  ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 -p mavlink_port:=5770 -p takeoff_altitude:=40.0 &
sleep 1
 
# UAV3 — Member mountain side — mid altitude for terrain clearance
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
CYCLONEDDS_URI="$(dds_uri_for "$UAV3_IP" "$GCS_IP")" \
  ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=3 -p mavlink_port:=5780 -p takeoff_altitude:=50.0 &
 
echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║         City Surveillance Mission — 3 UAVs              ║"
echo "║                                                          ║"
echo "║  UAV1 → CLUSTER HEAD (city circle then hover at 60m)    ║"
echo "║  UAV2 → Member: pond side patrol      (40m)             ║"
echo "║  UAV3 → Member: mountain side patrol  (50m)             ║"
echo "║                                                          ║"
echo "║  DDS traffic to/from GCS now crosses the ns-3 Nakagami / ║"
echo "║  log-distance channel via tap-gcs/tap-uav1/2/3.          ║"
echo "║  SNR log: /tmp/snr_log.csv                                ║"
echo "║                                                          ║"
echo "║  Wait for all 3 bridges to show:                        ║"
echo "║    ✓ DDS GPS flowing                                     ║"
echo "║                                                          ║"
echo "║  Then run in terminal 2:                                 ║"
echo "║    bash scripts/run_city_mission.sh                      ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
 
wait $GAZEBO_PID