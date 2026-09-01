#!/bin/bash
# launch_faculty_single.sh
# ------------------------
# Launches Faculty of Engineering world with 1 UAV, DDS, and drone_bridge node.
#
# GPS Origin: 6.0792673°N, 80.1921607°E
#   (Front entrance, Administration Building, University of Ruhuna)
#
# UAV1 (port 5760) → takes off, flies 40m North, turns left, flies 40m West, RTL
#
# Then in terminal 2:
#   ros2 run uav_controller single_uav_mission

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

# ── Gazebo paths — MUST come after all source calls so setup.sh cannot overwrite ──
# MODEL_PATH: finds campus_world/model.sdf
export GAZEBO_MODEL_PATH="$FACULTY_GAZEBO_DIR/models:$PROJECT_DIR/models:$GAZEBO_MODEL_PATH"
# RESOURCE_PATH: unset so Gazebo resolves textures naturally via model path,
# matching the behaviour of running gazebo directly from the faculty_gazebo repo.
# setup.sh sets this to /usr/share/gazebo-11 which interferes with OBJ texture lookup.
unset GAZEBO_RESOURCE_PATH

echo "GAZEBO_MODEL_PATH    : $GAZEBO_MODEL_PATH"
echo "GAZEBO_RESOURCE_PATH : ${GAZEBO_RESOURCE_PATH:-(unset)}"

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

# Faculty GPS origin — must match spherical_coordinates in world file
FACULTY_HOME="6.0792673,80.1921607,31.0,0"

# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [1/4] Launching Gazebo with faculty world ==="
gazebo --verbose "$PROJECT_DIR/worlds/faculty_1uav.world" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Waiting for Gazebo to load (20s)..."
sleep 20

# ── 2. micro_ros_agent ───────────────────────────────────────────────────────
echo ""
echo "=== [2/4] Starting micro_ros_agent ==="
# NOTE: must match DDS_UDP_PORT in params/uav1_dds.parm
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 &
sleep 2
echo "micro_ros_agent running on port 2019"

# ── 3. ArduPilot SITL ────────────────────────────────────────────────────────
echo ""
echo "=== [3/4] Launching SITL instance ==="
cd ~/

# UAV1 — sysid 1, instance 0 (Gazebo ports 9002/9003, MAVLink 5760)
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address=127.0.0.1 -I0 \
    --home $FACULTY_HOME &

echo "Waiting for SITL to boot (20s)..."
sleep 20

# ── 4. drone_bridge ──────────────────────────────────────────────────────────
echo ""
echo "=== [4/4] Starting drone_bridge node ==="

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=20.0 &

echo ""
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║     Faculty of Engineering — Single UAV L-Shape Mission     ║"
echo "║                                                              ║"
echo "║  UAV1 → Takeoff 20m → Fly 40m North → Fly 40m West → RTL  ║"
echo "║                                                              ║"
echo "║  Wait for bridge to show:                                    ║"
echo "║    ✓ DDS GPS flowing                                         ║"
echo "║                                                              ║"
echo "║  Then run in terminal 2:                                     ║"
echo "║    ros2 run uav_controller single_uav_mission                ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID