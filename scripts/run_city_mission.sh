#!/bin/bash
# run_city_mission.sh
# --------------------
# Run this in "terminal 2" instead of `ros2 run uav_controller city_mission`
# directly. It pins city_mission's DDS participant to the tap-gcs IP
# (10.42.0.10) with explicit peers, so its traffic to/from each drone_bridge
# actually crosses the ns-3 Nakagami/log-distance channel instead of just
# discovering everyone instantly over loopback/multicast.
#
# Prereqs: scripts/launch_city_dds.sh must already be running (ns-3 + Gazebo
# + SITL + drone_bridge all up, each showing "✓ DDS GPS flowing").
 
set -eo pipefail
 
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
 
source /opt/ros/humble/setup.bash
source ~/ardu_ws/install/setup.bash
source "$PROJECT_DIR/ros2/install/setup.bash"
 
GCS_IP="10.42.0.10"
UAV1_IP="10.42.0.11"
UAV2_IP="10.42.0.12"
UAV3_IP="10.42.0.13"
 
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>${GCS_IP}</NetworkInterfaceAddress><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address=\"${UAV1_IP}\"/><Peer address=\"${UAV2_IP}\"/><Peer address=\"${UAV3_IP}\"/></Peers></Discovery></Domain></CycloneDDS>"
 
exec ros2 run uav_controller city_mission
