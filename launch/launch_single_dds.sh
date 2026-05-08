#!/bin/bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

echo "=== Killing previous instances ==="
pkill -f arducopter 2>/dev/null || true
pkill -f gzserver   2>/dev/null || true
pkill -f gzclient   2>/dev/null || true
pkill -f micro_ros_agent 2>/dev/null || true
sleep 3

if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  echo "ERROR: ARDUPILOT_HOME not set. Check setup.sh"
  exit 1
fi

BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
BASE_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
DDS_PARAMS="$PROJECT_DIR/params/uav1_dds.parm"
UAV1_DEFAULTS="$BASE_DEFAULTS,$DDS_PARAMS"

# ── 1. Gazebo ────────────────────────────────────────────────────────────────
echo "=== Launching Gazebo ==="
gazebo --verbose "$PROJECT_DIR/worlds/single_uav.world" &
GAZEBO_PID=$!
echo "Waiting for Gazebo to load (15s)..."
sleep 15

# ── 2. micro-ROS agent (MUST be running before SITL starts) ──────────────────
echo "=== Starting micro-ROS agent ==="
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 &
AGENT_PID=$!
echo "Waiting for agent to bind port (3s)..."
sleep 3

# ── 3. ArduPilot SITL ────────────────────────────────────────────────────────
echo "=== Launching UAV1 SITL with DDS enabled ==="
cd ~/
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults $UAV1_DEFAULTS \
  --sim-address=127.0.0.1 -I0 &
SITL_PID=$!

echo ""
echo "=== All processes launched ==="
echo ""
echo "Next steps (in separate terminals):"
echo ""
echo "  1. Connect MAVProxy to trigger full SITL boot:"
echo "     mavproxy.py --master=tcp:127.0.0.1:5760"
echo ""
echo "  2. Watch this terminal — micro_ros_agent should print:"
echo "     'session established'"
echo ""
echo "  3. Once session established, verify ROS2 topics:"
echo "     source /opt/ros/humble/setup.bash"
echo "     source ~/ardu_ws/install/setup.bash"
echo "     ros2 topic list"
echo "     ros2 topic echo /ap/navsat"
echo ""

wait $GAZEBO_PID