#!/bin/bash
# launch_faculty_dds.sh
# ---------------------
# Launches the Faculty of Engineering world with 3 UAVs, DDS, and drone_bridge nodes.
#
# UAV roles for faculty_mission:
#   UAV1 (port 5760) → Dept Electrical + Mechanical sweep
#   UAV2 (port 5770) → CLUSTER HEAD — hovers over Admin Building
#   UAV3 (port 5780) → Dept Civil + Lecture Halls sweep
#
# Then in terminal 2:
#   ros2 run uav_controller faculty_mission

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

# Add faculty meshes to Gazebo model path
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$PROJECT_DIR/models/faculty_meshes:${GAZEBO_MODEL_PATH}"

# ── 1. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [1/4] Launching Gazebo with faculty world ==="
gazebo --verbose "$PROJECT_DIR/worlds/faculty_uav.world" \
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
echo "micro_ros_agents running on ports 2019, 2020, 2021"

# ── 3. ArduPilot SITL × 3 ────────────────────────────────────────────────────
echo ""
echo "=== [3/4] Launching 3 SITL instances ==="
cd ~/
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address=127.0.0.1 -I0 &
sleep 2

$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS --sim-address=127.0.0.1 -I1 &
sleep 2

$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS --sim-address=127.0.0.1 -I2 &
sleep 3

echo "Waiting for SITL instances to boot (15s)..."
sleep 15

# ── 4. drone_bridge × 3 ──────────────────────────────────────────────────────
echo ""
echo "=== [4/4] Starting 3 drone_bridge nodes ==="

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=30.0 &
sleep 1

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 -p mavlink_port:=5770 -p takeoff_altitude:=30.0 &
sleep 1

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=3 -p mavlink_port:=5780 -p takeoff_altitude:=30.0 &

echo ""
echo "╔══════════════════════════════════════════════════════════╗"
echo "║     Faculty of Engineering — UAV Surveillance Demo      ║"
echo "║                                                          ║"
echo "║  UAV1 → Dept Electrical + Mechanical sweep              ║"
echo "║  UAV2 → CLUSTER HEAD (hovers over Admin Building)       ║"
echo "║  UAV3 → Dept Civil + Lecture Halls sweep                ║"
echo "║                                                          ║"
echo "║  Wait for all 3 bridges to show:                        ║"
echo "║    ✓ DDS GPS flowing                                     ║"
echo "║                                                          ║"
echo "║  Then run in terminal 2:                                 ║"
echo "║    ros2 run uav_controller faculty_mission               ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID
