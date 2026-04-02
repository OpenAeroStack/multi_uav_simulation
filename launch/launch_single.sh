#!/bin/bash

source ~/setup_ardupilot.sh

BINARY=/home/ubuntu/ardupilot/build/sitl/bin/arducopter
DEFAULTS=/home/ubuntu/ardupilot/Tools/autotest/default_params/copter.parm,/home/ubuntu/ardupilot/Tools/autotest/default_params/gazebo-iris.parm

echo "=== Building ArduCopter binary ==="
cd ~/ardupilot
python3 modules/waf/waf-light build --target bin/arducopter
echo "=== Build done ==="

echo "=== Launching Gazebo with 1 UAV ==="
gazebo --verbose ~/FYP/multi_uav_sim/worlds/single_uav.world &
GAZEBO_PID=$!

echo "=== Waiting for Gazebo to fully load (20 seconds) ==="
sleep 20

echo "=== Launching ArduCopter SITL instance 0 (UAV1) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I0 &

echo ""
echo "=== SITL instance running ==="
echo "Connect to UAV1: mavproxy.py --master=tcp:127.0.0.1:5760"

wait $GAZEBO_PID

#after running the launch script run the below command in new terminal to connect to STIL instance 
# source ~/setup_ardupilot.sh
# mavproxy.py --master=tcp:127.0.0.1:5760 