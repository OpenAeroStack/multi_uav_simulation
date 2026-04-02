#!/bin/bash

source ~/setup_ardupilot.sh

BINARY=/home/ubuntu/ardupilot/build/sitl/bin/arducopter
DEFAULTS=/home/ubuntu/ardupilot/Tools/autotest/default_params/copter.parm,/home/ubuntu/ardupilot/Tools/autotest/default_params/gazebo-iris.parm
echo "=== Building ArduCopter binary ==="
cd ~/ardupilot
python3 modules/waf/waf-light build --target bin/arducopter
echo "=== Build done ==="

echo "=== Launching Gazebo with 3 UAVs ==="
gazebo --verbose ~/FYP/multi_uav_sim/worlds/multi_uav.world &
GAZEBO_PID=$!

echo "=== Waiting for Gazebo to fully load (20 seconds) ==="
sleep 20

echo "=== Launching ArduCopter SITL instance 0 (UAV1) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 1 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I0 &
sleep 5

echo "=== Launching ArduCopter SITL instance 1 (UAV2) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 2 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I1 &
sleep 5

echo "=== Launching ArduCopter SITL instance 2 (UAV3) ==="
$BINARY --model gazebo-iris --speedup 1 --sysid 3 \
  --defaults $DEFAULTS \
  --sim-address=127.0.0.1 -I2 &

echo ""
echo "=== All 3 SITL instances running ==="
echo "Connect to UAV1: mavproxy.py --master=tcp:127.0.0.1:5760"
echo "Connect to UAV2: mavproxy.py --master=tcp:127.0.0.1:5770"
echo "Connect to UAV3: mavproxy.py --master=tcp:127.0.0.1:5780"

wait $GAZEBO_PID

#after running the launch script run the below command in new terminal to connect to STIL instances
#then you can control it via the mavproxy console for each UAV using the respective ports
# source ~/setup_ardupilot.sh
#mavproxy.py --master=tcp:127.0.0.1:5760
#Terminal for UAV 2:
#mavproxy.py --master=tcp:127.0.0.1:5770
#Terminal for UAV 3:
#mavproxy.py --master=tcp:127.0.0.1:5780