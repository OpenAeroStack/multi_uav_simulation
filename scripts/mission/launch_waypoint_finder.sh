#!/bin/bash
# Flat-mode waypoint finding — UAV1 only

echo "=== Cleanup ==="
pkill -9 -f arducopter 2>/dev/null || true
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
export GAZEBO_MODEL_PATH=$HOME/FYP/small_city_gazebo/models:$GAZEBO_MODEL_PATH

cleanup() {
  echo "Shutting down..."
  pkill -9 -f arducopter 2>/dev/null || true
  pkill -9 -f mavproxy 2>/dev/null || true
  pkill -9 -f gzserver 2>/dev/null || true
  pkill -9 -f gzclient 2>/dev/null || true
  pkill -9 -f micro_ros_agent 2>/dev/null || true
  pkill -9 -f drone_bridge 2>/dev/null || true
  exit
}
trap cleanup INT

echo "=== [1/5] Gazebo ==="
gazebo --verbose \
  $HOME/FYP/multi_uav_sim/worlds/small_city_single_uav.world \
  > /tmp/gazebo_wp.log 2>&1 &
echo "Waiting 25s for Gazebo to load..."
sleep 25

echo "=== [2/5] SITL ==="
mkdir -p /tmp/sitl_wp && cd /tmp/sitl_wp
"$ARDUPILOT_HOME/build/sitl/bin/arducopter" \
  --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm,$HOME/FYP/multi_uav_sim/params/uav1_dds_flat.parm" \
  --sim-address=127.0.0.1 -I0 \
  > /tmp/sitl_wp.log 2>&1 &
sleep 5

echo "=== [3/5] mavproxy (unlocks SITL on port 5760) ==="
nohup mavproxy.py --master=tcp:127.0.0.1:5760 \
  --logfile=/tmp/mav_wp.tlog \
  < /dev/null > /tmp/mavproxy_wp.log 2>&1 &
echo "Waiting 20s for SITL + GPS to initialize..."
sleep 20

echo "=== [4/5] micro_ros_agent ==="
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 \
  > /tmp/agent_wp.log 2>&1 &
sleep 5

echo "=== [5/5] drone_bridge (port 5762 — mavproxy holds 5760) ==="
ros2 run uav_controller drone_bridge --ros-args \
  -p uav_id:=1 \
  -p mavlink_port:=5762 \
  > /tmp/bridge_wp.log 2>&1 &
sleep 10

echo ""
echo "=== Status ==="
grep -E "DDS: Initialization passed|EKF3 IMU0 origin|ArduPilot Ready" \
  /tmp/sitl_wp.log || echo "SITL not ready yet"
grep "Bridge ready\|GPS flowing\|Services" \
  /tmp/bridge_wp.log || echo "Bridge not ready yet"
ros2 topic hz /ap/v1/navsat --window 3 &
HZ_PID=$!
sleep 5
kill $HZ_PID 2>/dev/null

echo ""
echo "================================================"
echo "  In a new terminal:"
echo "    source /opt/ros/humble/setup.bash"
echo "    source ~/FYP/multi_uav_sim/ros2/install/setup.bash"
echo "    python3 $HOME/FYP/multi_uav_sim/scripts/mission/waypoint_finder.py"
echo "================================================"
echo "Ctrl+C to shut down"

wait
