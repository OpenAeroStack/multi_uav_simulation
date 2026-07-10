#!/bin/bash
set -euo pipefail

setup_ns() {
  local NS=$1 TAP=$2 BR=$3 VETH_H=$4 VETH_NS=$5 NS_IP=$6
  echo "[ns-tap] Setting up $NS / $TAP / $BR ($NS_IP)"
  ip netns add "$NS"                                     2>/dev/null || true
  # REMOVED: tap was owned by root, so NS-3's TapBridge (running as a normal
  # user) got EPERM when attaching to it:
  # ip tuntap add dev "$TAP" mode tap                      2>/dev/null || true
  # ADDED: "user" makes the tap owned by the invoking user so NS-3's
  # TapBridge (UseLocal mode) can attach to it without running as root
  ip tuntap add dev "$TAP" mode tap user "${SUDO_USER:-$USER}" 2>/dev/null || true
  ip link set "$TAP" up
  ip link add name "$BR" type bridge                     2>/dev/null || true
  ip link set "$TAP" master "$BR"
  ip link set "$BR" up
  ip link add "$VETH_H" type veth peer name "$VETH_NS"   2>/dev/null || true
  ip link set "$VETH_H" master "$BR"
  ip link set "$VETH_H" up
  ip link set "$VETH_NS" netns "$NS"
  ip netns exec "$NS" ip link set "$VETH_NS" up
  ip netns exec "$NS" ip addr add "$NS_IP" dev "$VETH_NS" 2>/dev/null || true
  ip netns exec "$NS" ip link set lo up
}

# GCS — micro-ROS agents live here
setup_ns gcsns  tap-gcs  br-gcs  veth0h veth0n 10.42.0.10/24

# UAV nodes
setup_ns uav1ns tap-uav1 br-uav1 veth1h veth1n 10.42.0.11/24
setup_ns uav2ns tap-uav2 br-uav2 veth2h veth2n 10.42.0.12/24
setup_ns uav3ns tap-uav3 br-uav3 veth3h veth3n 10.42.0.13/24

echo ""
echo "[ns-tap] Done."
echo "  gcsns  → 10.42.0.10  (tap-gcs,  br-gcs)"
echo "  uav1ns → 10.42.0.11  (tap-uav1, br-uav1)"
echo "  uav2ns → 10.42.0.12  (tap-uav2, br-uav2)"
echo "  uav3ns → 10.42.0.13  (tap-uav3, br-uav3)"
