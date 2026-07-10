#!/bin/bash
# multi_uav_simulation/scripts/build_ns3_ros_bridge.sh
source ~/uav_ws/install/setup.bash

NS3_ROOT=~/ns-3.38
SRC_DIR=~/uav_ws/src/multi_uav_simulation/ns3

g++ -std=c++17 \
  -I${NS3_ROOT}/build/include \
  $(pkg-config --cflags rclcpp std_msgs 2>/dev/null) \
  -I$(python3 -c "import os; print(os.environ.get('AMENT_PREFIX_PATH','').split(':')[0])")/include \
  -c ${SRC_DIR}/three_uav_tapbridge_rt.cc -o /tmp/three_uav_tapbridge_rt.o \
  $(pkg-config --libs rclcpp std_msgs 2>/dev/null)

echo "If pkg-config doesn't find rclcpp/std_msgs, fall back to the CMakeLists.txt approach above and run via colcon's CMake instead."