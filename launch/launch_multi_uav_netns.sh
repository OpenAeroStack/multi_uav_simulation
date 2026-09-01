#!/bin/bash
# launch_multi_uav_netns.sh
# Launches Gazebo + 3× ArduPilot SITL using network namespaces.
# NS-3 is launched separately (see below).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../setup.sh"

echo "[launch] Killing previous instances..."
pkill -f arducopter || true
pkill -f gzserver   || true
pkill -f gzclient   || true
sleep 3

echo "[launch] Starting Gazebo..."
gzserver "$PROJECT_DIR/worlds/multi_uav.world" &
sleep 5

echo "[launch] Starting ArduCopter SITL instances..."
for i in 1 2 3; do
  PORT_OFFSET=$(( (i-1) * 10 ))
  SYSID=$i
  cd "$ARDUPILOT_HOME"
  # Run each SITL inside its namespace so its MAVLink traffic goes via NS-3
  ip netns exec "uav${i}ns" \
    build/sitl/bin/arducopter \
      --model gazebo-iris \
      --home "-35.363262,149.165237,584,353" \
      --defaults "$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm" \
      --instance $(( i - 1 )) \
      --uartA "tcp:0" \
      --uartC "udpclient:127.0.0.1:$(( 9002 + PORT_OFFSET ))" \
      --sysid "$SYSID" \
    &
done

echo "[launch] SITL instances started."
echo ""
echo "Now start NS-3 in a SEPARATE terminal:"
echo "  sudo ~/ns-allinone-3.38/ns-3.38/build/scratch/ns3.38-three_uav_tapbridge_rt-default"
echo ""
echo "Then connect MAVProxy or run Python scripts as normal."