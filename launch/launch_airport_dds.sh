#!/bin/bash
# launch_airport_dds.sh
# ---------------------
# Launches ksql_airport world with 3 UAVs, DDS, and drone_bridge nodes.
#
# UAV roles:
#   UAV1 (port 5760) → CLUSTER HEAD — hovers center at 50m
#   UAV2 (port 5770) → Member — South sector patrol (takeoff alt 35m)
#   UAV3 (port 5780) → Member — North sector patrol (takeoff alt 40m)
#
# Collision avoidance: UAV2 takeoff=35m, UAV3 takeoff=40m
# They are vertically separated from liftoff — no mid-air crossing.
#
# Then in terminal 2:
#   ros2 run uav_controller airport_mission

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

# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [1/4] Launching Gazebo with airport world ==="
gazebo --verbose "$PROJECT_DIR/worlds/airport_3uav.world" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Waiting for Gazebo to load (15s)..."
sleep 15

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
cd ~/

$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address=127.0.0.1 -I0 \
    --home 37.523640,-122.255122,1.7,0 &
sleep 2

$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS --sim-address=127.0.0.1 -I1 \
    --home 37.523640,-122.255122,1.7,0 &
sleep 2

$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS --sim-address=127.0.0.1 -I2 \
    --home 37.523640,-122.255122,1.7,0 &

sleep 3

echo "Waiting for SITL instances to boot (15s)..."
sleep 15

# ── 4. drone_bridge × 3 ──────────────────────────────────────────────────────
echo ""
echo "=== [4/4] Starting 3 drone_bridge nodes ==="

# UAV1 — CLUSTER HEAD — takeoff to 40m
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=40.0 &
sleep 1

# UAV2 — Member South — takeoff to 35m (lower than UAV3 for collision avoidance)
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 -p mavlink_port:=5770 -p takeoff_altitude:=35.0 &
sleep 1

# UAV3 — Member North — takeoff to 40m
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=3 -p mavlink_port:=5780 -p takeoff_altitude:=40.0 &

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║       Airport Clustering Mission — 3 UAVs               ║"
echo "║                                                          ║"
echo "║  UAV1 → CLUSTER HEAD (hovers center at 50m)             ║"
echo "║  UAV2 → Member: South sector patrol (takeoff 35m)       ║"
echo "║  UAV3 → Member: North sector patrol (takeoff 40m)       ║"
echo "║                                                          ║"
echo "║  Wait for all 3 bridges to show:                        ║"
echo "║    ✓ DDS GPS flowing                                     ║"
echo "║                                                          ║"
echo "║  Then run in terminal 2:                                 ║"
echo "║    ros2 run uav_controller airport_mission               ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID