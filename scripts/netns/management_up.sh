#!/bin/bash
# scripts/netns/management_up.sh
#
# Creates a private point-to-point link between each UAV namespace and the
# root namespace, dedicated to SITL <-> Gazebo physics traffic (FDM).
#
# DELIBERATELY SEPARATE from wireless_up.sh: that link is bridged into
# ns-3's simulated 802.11a channel, and physics traffic must never cross
# it -- a drone's flight-controller-to-airframe link isn't a radio link
# in real life, so routing FDM data through the simulated channel would
# apply fake packet loss/delay to something that should behave like a
# direct wired connection.
#
# No bridge or TAP here -- each UAV only ever talks to one thing (Gazebo,
# in the root namespace), so a simple point-to-point veth pair is enough.
#
# IMPORTANT: run wireless_up.sh FIRST -- this assumes uav1ns/uav2ns/uav3ns
# already exist.
#
# SITL's --sim-address flag must point at the HOST-side IP printed below
# (NOT 127.0.0.1 -- namespaces do not share loopback with the root).

set -euo pipefail

setup_mgmt_link() {
  local NS=$1 VETH_HOST=$2 VETH_NS=$3 HOST_IP=$4 NS_IP=$5
  echo "[mgmt-link] Setting up $NS <-> root ($HOST_IP <-> $NS_IP)"
  ip link add "$VETH_HOST" type veth peer name "$VETH_NS" 2>/dev/null || true
  ip addr add "$HOST_IP" dev "$VETH_HOST"                 2>/dev/null || true
  ip link set "$VETH_HOST" up

  ip link set "$VETH_NS" netns "$NS"
  ip netns exec "$NS" ip addr add "$NS_IP" dev "$VETH_NS" 2>/dev/null || true
  ip netns exec "$NS" ip link set "$VETH_NS" up
}

# UAV1: root=172.31.1.1  <->  uav1ns=172.31.1.2
setup_mgmt_link uav1ns sim1h sim1n 172.31.1.1/30 172.31.1.2/30

# UAV2: root=172.31.2.1  <->  uav2ns=172.31.2.2
setup_mgmt_link uav2ns sim2h sim2n 172.31.2.1/30 172.31.2.2/30

# UAV3: root=172.31.3.1  <->  uav3ns=172.31.3.2
setup_mgmt_link uav3ns sim3h sim3n 172.31.3.1/30 172.31.3.2/30

echo ""
echo "[mgmt-link] Done."
echo "  uav1ns SITL must use --sim-address=172.31.1.1"
echo "  uav2ns SITL must use --sim-address=172.31.2.1"
echo "  uav3ns SITL must use --sim-address=172.31.3.1"