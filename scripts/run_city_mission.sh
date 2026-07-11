#!/bin/bash
# run_city_mission.sh
# --------------------
# Run this in "terminal 2" instead of `ros2 run uav_controller city_mission`
# directly. It enters the gcsns network namespace, pins city_mission's DDS
# participant to the tap-gcs IP (10.42.0.10), and sets explicit peers so its
# traffic to/from each drone_bridge crosses the ns-3 channel.
#
# Prereqs: scripts/launch_city_dds.sh must already be running (ns-3 + Gazebo
# + SITL + drone_bridge all up, each showing "✓ DDS GPS flowing").
 
set -eo pipefail
 
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
 
GCS_IP="10.42.0.10"
UAV1_IP="10.42.0.11"
UAV2_IP="10.42.0.12"
UAV3_IP="10.42.0.13"

exec sudo ip netns exec gcsns \
  sudo -H -u "$RUN_USER" \
  env GCS_IP="$GCS_IP" UAV1_IP="$UAV1_IP" UAV2_IP="$UAV2_IP" UAV3_IP="$UAV3_IP" \
  bash -lc '
    set -e
    source /opt/ros/humble/setup.bash
    source ~/ardu_ws/install/setup.bash
    source "$1"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${GCS_IP}</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address=\"${UAV1_IP}\"/><Peer address=\"${UAV2_IP}\"/><Peer address=\"${UAV3_IP}\"/></Peers></Discovery></Domain></CycloneDDS>"
    exec ros2 run uav_controller city_mission
  ' run-city-mission-shell "$PROJECT_DIR/ros2/install/setup.bash"
