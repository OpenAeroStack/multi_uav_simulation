#!/bin/bash
# launch_faculty_dds.sh
# ---------------------
# Launches Faculty of Engineering world with 3 UAVs, DDS, and drone_bridge nodes.
#
# GPS Origin: 6.0792673°N, 80.1921607°E
#   (Front entrance, Administration Building, University of Ruhuna)
#
# UAV roles:
#   UAV1 (port 5760) → CLUSTER HEAD — hovers at 60m above campus center
#   UAV2 (port 5770) → Member North — patrols Main Gate / Guard Room area
#   UAV3 (port 5780) → Member South — patrols Electrical Dept / Civil Dept area
#
# Collision avoidance via takeoff altitude separation:
#   UAV1 → 45m   UAV2 → 35m   UAV3 → 40m
#   (UAV1 climbs further to 60m head position after barrier sync)
#
# GAZEBO_MODEL_PATH includes faculty_gazebo/models so campus_world model
# (terrain + buildings mesh) is found without copying assets.
#
# Then in terminal 2:
#   ros2 run uav_controller faculty_mission

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FACULTY_GAZEBO_DIR="$HOME/FYP/faculty_gazebo"

source "$PROJECT_DIR/setup.sh"
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source "$PROJECT_DIR/ros2/install/setup.bash" 2>/dev/null || {
    echo "ERROR: ROS2 package not built. Run: bash build_ros2.sh"
    exit 1
}

# Add faculty_gazebo models to path so campus_world model is found
export GAZEBO_MODEL_PATH="$FACULTY_GAZEBO_DIR/models:$PROJECT_DIR/models:$GAZEBO_MODEL_PATH"

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

# Faculty GPS origin — must match spherical_coordinates in faculty_3uav.world
FACULTY_HOME="6.0792673,80.1921607,31.0,0"

# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [1/4] Launching Gazebo with faculty world ==="
gazebo --verbose "$PROJECT_DIR/worlds/faculty_3uav.world" \
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
cd ~/

# UAV1 — CLUSTER HEAD
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address=127.0.0.1 -I0 \
    --home $FACULTY_HOME &
sleep 2

# UAV2 — Member North
$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS --sim-address=127.0.0.1 -I1 \
    --home $FACULTY_HOME &
sleep 2

# UAV3 — Member South
$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS --sim-address=127.0.0.1 -I2 \
    --home $FACULTY_HOME &

sleep 3
echo "Waiting for SITL instances to boot (20s)..."
sleep 20

# ── 4. drone_bridge × 3 ──────────────────────────────────────────────────────
echo ""
echo "=== [4/4] Starting 3 drone_bridge nodes ==="

# UAV1 — CLUSTER HEAD — takeoff to 45m (climbs to 60m in mission)
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=45.0 &
sleep 1

# UAV2 — Member North — takeoff to 35m
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 -p mavlink_port:=5770 -p takeoff_altitude:=35.0 &
sleep 1

# UAV3 — Member South — takeoff to 40m
ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=3 -p mavlink_port:=5780 -p takeoff_altitude:=40.0 &

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║     Faculty of Engineering — Campus Surveillance Mission    ║"
echo "║                                                              ║"
echo "║  UAV1 → CLUSTER HEAD (hovers at 60m above campus center)   ║"
echo "║  UAV2 → Member North: Main Gate / Guard Room patrol         ║"
echo "║  UAV3 → Member South: Electrical Dept / Civil Dept patrol   ║"
echo "║                                                              ║"
echo "║  Takeoff separation: UAV1=45m  UAV2=35m  UAV3=40m          ║"
echo "║                                                              ║"
echo "║  Wait for all 3 bridges to show:                            ║"
echo "║    ✓ DDS GPS flowing                                         ║"
echo "║                                                              ║"
echo "║  Then run in terminal 2:                                     ║"
echo "║    ros2 run uav_controller faculty_mission                   ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID