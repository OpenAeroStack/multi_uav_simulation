#!/bin/bash
# launch_multi_dds.sh
# -------------------
# Launches everything needed for 3-drone DDS mission in one terminal:
#   - Gazebo with multi_uav.world
#   - 3 ArduCopter SITL instances with DDS enabled
#   - 3 micro_ros_agent instances
#   - 3 drone_bridge ROS2 nodes
#
# Then in a second terminal run:
#   ros2 run uav_controller multi_mission

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

# ── Source ROS2 ───────────────────────────────────────────────────────────────
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
echo "=== [1/4] Launching Gazebo ==="
gazebo --verbose "$PROJECT_DIR/worlds/multi_uav.world" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Waiting for Gazebo (15s)..."
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
echo "micro_ros_agents running on ports 2019, 2020, 2021"

# ── 3. ArduPilot SITL × 3 ────────────────────────────────────────────────────
echo ""
echo "=== [3/4] Launching 3 SITL instances ==="
cd ~/
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address 127.0.0.1 -I0 &
sleep 2

$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS --sim-address 127.0.0.1 -I1 &
sleep 2

$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS --sim-address 127.0.0.1 -I2 &
sleep 3

echo "Waiting for SITL instances to boot (15s)..."
sleep 15

# ── 4. drone_bridge × 3 ──────────────────────────────────────────────────────
echo ""
echo "=== [4/4] Starting 3 drone_bridge nodes ==="

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 \
    -p mavlink_port:=5760 \
    -p takeoff_altitude:=20.0 &
BRIDGE1_PID=$!
sleep 1

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 \
    -p mavlink_port:=5770 \
    -p takeoff_altitude:=20.0 &
BRIDGE2_PID=$!
sleep 1

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=3 \
    -p mavlink_port:=5780 \
    -p takeoff_altitude:=20.0 &
BRIDGE3_PID=$!

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║           All processes launched!                    ║"
echo "║                                                      ║"
echo "║  Waiting for all 3 bridges to get DDS GPS...        ║"
echo "║  Watch for: ✓ DDS GPS flowing                       ║"
echo "║                                                      ║"
echo "║  Once all 3 show GPS flowing, run in terminal 2:    ║"
echo "║    ros2 run uav_controller multi_mission             ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID
