cat > ~/finaly_year_project/multi_uav_simulation/launch/launch_multi_dds.sh << 'SCRIPT'
#!/bin/bash
# launch_multi_dds.sh
# -------------------
# Launches everything needed for 3-drone DDS mission with NS-3 Nakagami channel.
#
#   - Gazebo with multi_uav.world
#   - Network namespaces + TAP devices (for NS-3)
#   - 3 micro_ros_agent instances (inside gcsns, traffic goes through NS-3)
#   - 3 ArduCopter SITL instances (root namespace, Gazebo reachable)
#   - 3 drone_bridge ROS2 nodes
#
# Then in Terminal 2:
#   sudo ~/ns-allinone-3.38/ns-3.38/build/scratch/ns3.38-three_uav_tapbridge_rt-default \
#     --tapBase=tap-uav --tapGcs=tap-gcs --distance=50
#
# Then in Terminal 3 (after NS-3 is running and agents show "session established"):
#   source /opt/ros/humble/setup.bash
#   source ~/ardu_ws/install/setup.bash
#   source <project>/ros2/install/setup.bash
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

# ── 0. Full cleanup ───────────────────────────────────────────────────────────
echo "=== Killing previous instances ==="
sudo pkill -9 -f arducopter      2>/dev/null || true
sudo pkill -9 -f gzserver        2>/dev/null || true
sudo pkill -9 -f gzclient        2>/dev/null || true
sudo pkill -9 -f micro_ros_agent 2>/dev/null || true
sudo pkill -9 -f drone_bridge    2>/dev/null || true
sudo pkill -9 -f three_uav       2>/dev/null || true
sleep 3

# Remove old network plumbing
sudo ip link del tap-gcs  2>/dev/null || true
sudo ip link del tap-uav1 2>/dev/null || true
sudo ip link del tap-uav2 2>/dev/null || true
sudo ip link del tap-uav3 2>/dev/null || true
sudo ip link del br-gcs   2>/dev/null || true
sudo ip link del br-uav1  2>/dev/null || true
sudo ip link del br-uav2  2>/dev/null || true
sudo ip link del br-uav3  2>/dev/null || true
sudo ip link del veth0h   2>/dev/null || true
sudo ip link del veth1h   2>/dev/null || true
sudo ip link del veth2h   2>/dev/null || true
sudo ip link del veth3h   2>/dev/null || true
sudo ip netns del gcsns   2>/dev/null || true
sudo ip netns del uav1ns  2>/dev/null || true
sudo ip netns del uav2ns  2>/dev/null || true
sudo ip netns del uav3ns  2>/dev/null || true
sleep 2

if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
    echo "ERROR: ARDUPILOT_HOME not set. Check setup.sh"
    exit 1
fi

BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
BASE="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
UAV1_DEFAULTS="$BASE,$PROJECT_DIR/params/uav1_dds.parm"
UAV2_DEFAULTS="$BASE,$PROJECT_DIR/params/uav2_dds.parm"
UAV3_DEFAULTS="$BASE,$PROJECT_DIR/params/uav3_dds.parm"

# GCS namespace IP where micro_ros_agents will listen
GCS_IP="10.42.0.10"

# ── 1. Network namespaces + TAP devices ──────────────────────────────────────
echo ""
echo "=== [1/5] Creating network namespaces and TAP devices ==="
sudo bash "$PROJECT_DIR/scripts/setup_netns_tap.sh"

# ── 2. Gazebo ─────────────────────────────────────────────────────────────────
echo ""
echo "=== [2/5] Launching Gazebo ==="
gazebo --verbose "$PROJECT_DIR/worlds/multi_uav.world" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &
GAZEBO_PID=$!
echo "Waiting for Gazebo (15s)..."
sleep 15

# ── 3. micro_ros_agent × 3 inside gcsns ──────────────────────────────────────
# Agents run inside the GCS namespace so their UDP traffic goes through NS-3.
# Each agent binds to the gcsns veth IP so SITL connects via 10.42.0.10.
echo ""
echo "=== [3/5] Starting 3 micro_ros_agent instances inside gcsns ==="

sudo ip netns exec gcsns \
    /bin/bash -c "
        source /opt/ros/humble/setup.bash
        source $HOME/ardu_ws/install/setup.bash
        exec ros2 run micro_ros_agent micro_ros_agent \
            udp4 --port 2019 -v4
    " > /tmp/agent1.log 2>&1 &
sleep 2

sudo ip netns exec gcsns \
    /bin/bash -c "
        source /opt/ros/humble/setup.bash
        source $HOME/ardu_ws/install/setup.bash
        exec ros2 run micro_ros_agent micro_ros_agent \
            udp4 --port 2020 -v4
    " > /tmp/agent2.log 2>&1 &
sleep 2

sudo ip netns exec gcsns \
    /bin/bash -c "
        source /opt/ros/humble/setup.bash
        source $HOME/ardu_ws/install/setup.bash
        exec ros2 run micro_ros_agent micro_ros_agent \
            udp4 --port 2021 -v4
    " > /tmp/agent3.log 2>&1 &
sleep 2

echo "micro_ros_agents running in gcsns ($GCS_IP) on ports 2019, 2020, 2021"

# ── 4. ArduPilot SITL × 3 ────────────────────────────────────────────────────
# SITL stays in root namespace so it can reach Gazebo on 127.0.0.1.
# Each SITL's micro-ROS client is pointed at the gcsns IP via DDS_UA_SERVER_IP.
# The UDP packets travel: SITL → root ns → veth → gcsns → TAP → NS-3 → tap-gcs → agent
echo ""
echo "=== [4/5] Launching 3 SITL instances ==="
cd ~/

DDS_UA_SERVER_IP=$GCS_IP DDS_UA_SERVER_PORT=2019 \
setsid $BINARY --model gazebo-iris --speedup 1 --sysid 1 \
    --defaults $UAV1_DEFAULTS --sim-address=127.0.0.1 -I0 \
    > /tmp/sitl1.log 2>&1 &
sleep 3

DDS_UA_SERVER_IP=$GCS_IP DDS_UA_SERVER_PORT=2020 \
setsid $BINARY --model gazebo-iris --speedup 1 --sysid 2 \
    --defaults $UAV2_DEFAULTS --sim-address=127.0.0.1 -I1 \
    > /tmp/sitl2.log 2>&1 &
sleep 3

DDS_UA_SERVER_IP=$GCS_IP DDS_UA_SERVER_PORT=2021 \
setsid $BINARY --model gazebo-iris --speedup 1 --sysid 3 \
    --defaults $UAV3_DEFAULTS --sim-address=127.0.0.1 -I2 \
    > /tmp/sitl3.log 2>&1 &

echo "Waiting for SITL instances to boot (15s)..."
sleep 15

# ── 5. drone_bridge × 3 ──────────────────────────────────────────────────────
echo ""
echo "=== [5/5] Starting 3 drone_bridge nodes ==="

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=1 \
    -p mavlink_port:=5760 \
    -p takeoff_altitude:=20.0 \
    > /tmp/bridge1.log 2>&1 &
sleep 2

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=2 \
    -p mavlink_port:=5770 \
    -p takeoff_altitude:=20.0 \
    > /tmp/bridge2.log 2>&1 &
sleep 2

ros2 run uav_controller drone_bridge --ros-args \
    -p uav_id:=3 \
    -p mavlink_port:=5780 \
    -p takeoff_altitude:=20.0 \
    > /tmp/bridge3.log 2>&1 &

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║           All processes launched!                    ║"
echo "║                                                      ║"
echo "║  NEXT — Terminal 2: start NS-3 Nakagami channel:    ║"
echo "║                                                      ║"
echo "║  sudo ~/ns-allinone-3.38/ns-3.38/build/scratch/     ║"
echo "║    ns3.38-three_uav_tapbridge_rt-default             ║"
echo "║    --tapBase=tap-uav --tapGcs=tap-gcs --distance=50 ║"
echo "║                                                      ║"
echo "║  THEN — Terminal 3: watch agent for DDS session:    ║"
echo "║    tail -f /tmp/agent1.log                           ║"
echo "║    (look for: session established)                   ║"
echo "║                                                      ║"
echo "║  THEN — Terminal 4: run mission:                     ║"
echo "║    source /opt/ros/humble/setup.bash                 ║"
echo "║    source ~/ardu_ws/install/setup.bash               ║"
echo "║    source [project]/ros2/install/setup.bash          ║"
echo "║    ros2 run uav_controller multi_mission             ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

wait $GAZEBO_PID
SCRIPT

chmod +x ~/finaly_year_project/multi_uav_simulation/launch/launch_multi_dds.sh
echo "Done — launch_multi_dds.sh updated"