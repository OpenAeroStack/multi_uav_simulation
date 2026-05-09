#!/bin/bash
# launch_multi_dds.sh
# -------------------
# Launches Gazebo + 3 ArduCopter SITL instances with DDS enabled.
# Each UAV gets its own micro_ros_agent instance on a separate port.
#
# Port mapping:
#   UAV1: MAVLink=5760  FDM=9002/9003  DDS=2019
#   UAV2: MAVLink=5770  FDM=9012/9013  DDS=2020
#   UAV3: MAVLink=5780  FDM=9022/9023  DDS=2021
#
# Usage:
#   bash launch/launch_multi_dds.sh

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

echo "=== Killing previous instances ==="
pkill -f arducopter      2>/dev/null || true
pkill -f gzserver        2>/dev/null || true
pkill -f gzclient        2>/dev/null || true
pkill -f micro_ros_agent 2>/dev/null || true
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
echo "=== Launching Gazebo (multi_uav.world) ==="
gazebo --verbose "$PROJECT_DIR/worlds/multi_uav.world" &
GAZEBO_PID=$!
echo "Waiting for Gazebo to load (15s)..."
sleep 15

# ── 2. micro_ros_agent × 3 ───────────────────────────────────────────────────
echo "=== Starting micro_ros_agent for UAV1 (port 2019) ==="
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash

ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 &
AGENT1_PID=$!
sleep 1

echo "=== Starting micro_ros_agent for UAV2 (port 2020) ==="
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2020 &
AGENT2_PID=$!
sleep 1

echo "=== Starting micro_ros_agent for UAV3 (port 2021) ==="
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2021 &
AGENT3_PID=$!
sleep 2

# ── 3. ArduPilot SITL × 3 ────────────────────────────────────────────────────
echo "=== Launching UAV1 SITL (sysid=1, port=5760, DDS=2019) ==="
cd ~/
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS \
    --sim-address=127.0.0.1 -I0 &
UAV1_PID=$!
sleep 2

echo "=== Launching UAV2 SITL (sysid=2, port=5770, DDS=2020) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS \
    --sim-address=127.0.0.1 -I1 &
UAV2_PID=$!
sleep 2

echo "=== Launching UAV3 SITL (sysid=3, port=5780, DDS=2021) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS \
    --sim-address=127.0.0.1 -I2 &
UAV3_PID=$!

echo ""
echo "=== All processes launched ==="
echo ""
echo "Next steps (in separate terminals):"
echo ""
echo "  1. Run 3 drone_bridge nodes:"
echo "     ros2 run uav_controller drone_bridge --ros-args -p uav_id:=1 -p mavlink_port:=5760"
echo "     ros2 run uav_controller drone_bridge --ros-args -p uav_id:=2 -p mavlink_port:=5770"
echo "     ros2 run uav_controller drone_bridge --ros-args -p uav_id:=3 -p mavlink_port:=5780"
echo ""
echo "  2. Wait for all 3 bridges to show:"
echo "     ✓ DDS GPS flowing"
echo ""
echo "  3. Run the multi-drone mission:"
echo "     ros2 run uav_controller multi_mission"
echo ""
echo "  4. Verify topics:"
echo "     ros2 topic list | grep uav"
echo ""

wait $GAZEBO_PID
