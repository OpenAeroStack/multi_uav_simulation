#!/bin/bash
# iperf3_channel_test.sh
# ----------------------
# Guaranteed data-plane test of the NS-3 obstacle-loss WiFi channel.
# Pushes real iperf3 UDP traffic from uav1ns -> uav2ns THROUGH the NS-3
# simulated 802.11a link, and shows throughput / packet-loss degrade when an
# obstacle is injected on that link and recover when it is cleared.
#
# Run as a NORMAL user (NOT sudo) so the ROS publishes reach the NS-3 process,
# which runs as your user. sudo is used internally only for `ip netns exec`.
#
# Prereqs:
#   - tap-gcs + tap-uav1..3 and uav1ns/uav2ns/uav3ns exist (setup_netns_tap.sh)
#   - three_uav_tapbridge_integrated is running (default tap names)
#   - NOTE: with the GCS present, NS-3 node 1 = UAV1 = 10.42.0.11 and
#     node 2 = UAV2 = 10.42.0.12, so the addresses below are unchanged.
#
# Usage:  bash iperf3_channel_test.sh      (will prompt once for sudo password)

source /opt/ros/humble/setup.bash
set -o pipefail

SRV_NS=uav2ns;  SRV_IP=10.42.0.12
CLI_NS=uav1ns
PORT=5201
# Offered UDP load MUST stay under the real-time simulator's forwarding
# ceiling (~1.4 Mbps on this host) -- above it, ns-3 can't process packets in
# wall-clock time and drops ~everything, which masks the obstacle effect.
RATE=500K         # offered UDP load
DUR=6             # seconds per phase

# CHANGED (GCS added): ids are now NS-3 node ids -- 0 = GCS, 1..3 = UAV1..UAV3.
# Under the old convention id 0 was UAV1, so these payloads would now move the
# ground station and put the obstacle on a GCS link instead of the one carrying
# the iperf traffic.
pub_positions() {   # GCS on the ground, UAV1 @origin, UAV2 50 m east, UAV3 in formation
  for _ in 1 2 3; do
    ros2 topic pub --once /uav_world_positions std_msgs/msg/Float32MultiArray \
      "{data: [0,0,6,2.9, 1,0,0,10, 2,50,0,10, 3,25,43.3,10]}" >/dev/null 2>&1
  done
}

pub_obstacle() {    # $1 = dB on link 1-2 (UAV1<->UAV2, the iperf path);
                    # publish 25x so the EMA converges
  for _ in $(seq 1 25); do
    ros2 topic pub --once /link_obstacle_loss std_msgs/msg/Float32MultiArray \
      "{data: [1,2,$1]}" >/dev/null 2>&1
  done
}

show_rssi() {
  echo -n "   RSSI now: "
  timeout 3 ros2 topic echo --once /ns3_link_rssi 2>/dev/null | grep -E '^- ' | paste -sd' '
}

run_iperf() {       # one UDP run cli -> srv through the NS-3 channel
  sudo ip netns exec "$SRV_NS" iperf3 -s -1 -p "$PORT" >/tmp/iperf_srv.log 2>&1 &
  sleep 1
  # --connect-timeout guards iperf3's TCP control channel: on a blocked link
  # even that can't connect, so without it the client hangs forever.
  if timeout 25 sudo ip netns exec "$CLI_NS" iperf3 -c "$SRV_IP" -p "$PORT" \
        --connect-timeout 4000 -u -b "$RATE" -t "$DUR" >/tmp/iperf_cli.log 2>&1; then
    grep -E 'receiver' /tmp/iperf_cli.log | sed 's/^/   /'
  else
    echo "   >> link effectively DOWN — client could not complete (0 throughput)"
  fi
  sudo pkill -9 iperf3 2>/dev/null; sleep 0.5
}

echo "############################################################"
echo "# NS-3 obstacle-loss channel — iperf3 data-plane test"
echo "# link under test: uav1ns(10.42.0.11) -> uav2ns(10.42.0.12)"
echo "############################################################"
pub_positions

echo ""
echo "=== PHASE 1: CLEAR channel (0 dB obstacle) ==="
pub_obstacle 0.0; show_rssi
run_iperf

echo ""
echo "=== PHASE 2: BLOCKED (30 dB obstacle on link 0-1) ==="
pub_obstacle 30.0; show_rssi
run_iperf

echo ""
echo "=== PHASE 3: CLEARED again (0 dB) ==="
pub_obstacle 0.0; show_rssi
run_iperf

echo ""
echo "############################################################"
echo "# EXPECT: Phase 1 & 3 = ~500 Kbps received, ~0-2% loss."
echo "#         Phase 2   = link DOWN (or very high % loss)."
echo "# That proves obstacle loss actually gates the data plane."
echo "############################################################"
