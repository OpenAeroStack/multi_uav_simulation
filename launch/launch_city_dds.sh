#!/bin/bash
# launch_city_dds.sh
# ------------------
# Launches small_city world with 3 UAVs, DDS, and drone_bridge nodes.
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
#   ros2 run uav_controller city_mission

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source "$PROJECT_DIR/ros2/install/setup.bash" 2>/dev/null || {
    echo "ERROR: ROS2 package not built. Run: bash build_ros2.sh"
    exit 1
}

# ── Small city world model and resource paths ─────────────────────────────────
export GAZEBO_MODEL_PATH="$HOME/simulation/small_city_gazebo_world/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_RESOURCE_PATH="$HOME/simulation/small_city_gazebo_world:${GAZEBO_RESOURCE_PATH:-}"

echo "=== Killing previous instances ==="
pkill -f arducopter      2>/dev/null || true
pkill -f gzserver        2>/dev/null || true
pkill -f gzclient        2>/dev/null || true
pkill -f micro_ros_agent 2>/dev/null || true
pkill -f drone_bridge    2>/dev/null || true
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
echo "=== [0/5] Setting up TAP devices for ns-3 wireless channel ==="
sudo "$PROJECT_DIR/scripts/setup_tap_only.sh"

echo "=== Starting ns-3 realistic wifi channel ==="
pushd "$HOME/ns-3-dev" >/dev/null   # <-- your actual ns-3 root
sudo ./ns3 run --enable-sudo "three-uav \
  --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
  --simDurationSec=0 \
  --nakagamiM=1.5 --shadowingStdDb=4.0 --txPowerDbm=20 --uavAltitude=50 \
  --enableFlowMonitor=true --snrLogFile=/tmp/snr_log.csv \
  --animFile=/tmp/three_uav_anim.xml" &
NS3_PID=$!
popd >/dev/null
sleep 5   # let ns-3 attach the TAPs before anything tries to use them
# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [1/4] Launching Gazebo with city world ==="
gazebo --verbose "$PROJECT_DIR/worlds/city_3uav.world" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Waiting for Gazebo to load (20s)..."
sleep 20

# ── 2. micro_ros_agent × 3 ───────────────────────────────────────────────────
echo ""
echo "=== [2/4] Starting 3 micro_ros_agent instances ==="
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 &
sleep 1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2020 &
sleep 1
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2021 &
sleep 2
echo "micro_ros_agents running on ports 2019-2021"

# ── 3. ArduPilot SITL × 3 ────────────────────────────────────────────────────
echo ""
echo "=== [3/4] Launching 3 SITL instances ==="
cd "$HOME"

"$BINARY" --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults "$UAV1_DEFAULTS" --sim-address=127.0.0.1 -I0 \
    --home "$HOME_GPS" --out udp:127.0.0.1:14550 &
sleep 2

"$BINARY" --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults "$UAV2_DEFAULTS" --sim-address=127.0.0.1 -I1 \
    --home "$HOME_GPS" &
sleep 2

"$BINARY" --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults "$UAV3_DEFAULTS" --sim-address=127.0.0.1 -I2 \
    --home "$HOME_GPS" &

sleep 3
echo "Waiting for SITL instances to boot (15s)..."
sleep 15

# ── 4. drone_bridge × 3 ──────────────────────────────────────────────────────
echo ""
echo "=== [4/4] Starting 3 drone_bridge nodes ==="

# UAV1 — CLUSTER HEAD — highest altitude for best relay coverage
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=60.0 &
sleep 1

# UAV2 — Member pond side — lower altitude, flat terrain
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 -p mavlink_port:=5770 -p takeoff_altitude:=40.0 &
sleep 1

# UAV3 — Member mountain side — mid altitude for terrain clearance
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
echo "║  Wait for all 3 bridges to show:                        ║"
echo "║    ✓ DDS GPS flowing                                     ║"
echo "║                                                          ║"
echo "║  Then run in terminal 2:                                 ║"
echo "║    ros2 run uav_controller city_mission                  ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID
