#!/bin/bash
# Flat-mode waypoint finding launch — UAV1 only.
# NOTE: no mavproxy and no sim_vehicle.py on purpose.
#   - mavproxy exits immediately when backgrounded from a script (stdin EOF)
#   - sim_vehicle.py kills SITL when its mavproxy exits
#   - drone_bridge itself is the MAVLink TCP client that unlocks SITL

echo "=== Cleanup ==="
pkill -9 -f arducopter 2>/dev/null || true
pkill -9 -f sim_vehicle 2>/dev/null || true
pkill -9 -f mavproxy 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f micro_ros_agent 2>/dev/null || true
pkill -9 -f drone_bridge 2>/dev/null || true
sleep 3

source ~/FYP/multi_uav_sim/setup.sh
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source ~/FYP/multi_uav_sim/ros2/install/setup.bash

export GAZEBO_MODEL_PATH=$HOME/FYP/multi_uav_sim/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$HOME/FYP/multi_uav_sim/install/multi_uav_gazebo_plugins/lib:$GAZEBO_PLUGIN_PATH
export GAZEBO_RESOURCE_PATH=$HOME/FYP/multi_uav_sim:$HOME/FYP/multi_uav_sim/worlds:$GAZEBO_RESOURCE_PATH

cleanup() {
  echo "Shutting down..."
  pkill -9 -f arducopter 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f micro_ros_agent 2>/dev/null || true
  pkill -9 -f drone_bridge 2>/dev/null || true
  exit
}
trap cleanup INT

echo "=== [1/4] Gazebo ==="
gazebo --verbose $HOME/FYP/multi_uav_sim/worlds/small_city_single_uav.world \
  > /tmp/gazebo_wp.log 2>&1 &
sleep 20

echo "=== [2/4] SITL (direct arducopter) ==="
mkdir -p /tmp/sitl_wp
cd /tmp/sitl_wp
"$ARDUPILOT_HOME/build/sitl/bin/arducopter" \
  --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm,$HOME/FYP/multi_uav_sim/params/uav1_dds_flat.parm" \
  --sim-address=127.0.0.1 -I0 \
  > /tmp/sitl_wp.log 2>&1 &
sleep 5

echo "=== [3/4] micro_ros_agent (port 2019) ==="
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 \
  > /tmp/agent_wp.log 2>&1 &
sleep 3

echo "=== [4/4] drone_bridge (this unlocks SITL) ==="
ros2 run uav_controller drone_bridge --ros-args -p uav_id:=1 \
  > /tmp/bridge_wp.log 2>&1 &
sleep 20

echo ""
echo "=== Status check ==="
grep -E "DDS: Initialization passed|EKF3 IMU0 origin set" /tmp/sitl_wp.log \
  || echo "  (DDS/EKF not ready yet — give it another 20s)"
echo "--- services ---"
ros2 service list | grep uav1 || echo "  (no /uav1 services yet)"

echo ""
echo "================================================"
echo "  Logs: /tmp/sitl_wp.log /tmp/bridge_wp.log /tmp/agent_wp.log"
echo "  In a new terminal:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ~/FYP/multi_uav_sim/ros2/install/setup.bash"
echo "    python3 $HOME/FYP/multi_uav_sim/scripts/mission/waypoint_finder.py"
echo "================================================"
echo "Ctrl+C to shut down"

wait
