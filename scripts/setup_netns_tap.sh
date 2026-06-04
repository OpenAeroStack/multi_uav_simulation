#!/bin/bash
# setup_netns_tap.sh
# Creates three network namespaces and TAP-to-bridge plumbing.
# Run once before launching NS-3 and the SITL stack.
# Requires: iproute2 (ip command), bridge-utils

set -euo pipefail

for i in 1 2 3; do
  NS="uav${i}ns"
  TAP="tap-uav${i}"
  BR="br-uav${i}"
  VETH_H="veth${i}h"
  VETH_NS="veth${i}n"
  # IP inside the namespace: 10.42.<i>.2/24
  NS_IP="10.42.${i}.2/24"

  echo "[ns-tap] Setting up $NS / $TAP / $BR"

  # Namespace
  ip netns add "$NS" 2>/dev/null || true

  # TAP device (NS-3 will open this)
  ip tuntap add dev "$TAP" mode tap || true
  ip link set "$TAP" up

  # Bridge
  ip link add name "$BR" type bridge || true
  ip link set "$TAP" master "$BR"
  ip link set "$BR" up

  # veth pair: one end in root, one end in namespace
  ip link add "$VETH_H" type veth peer name "$VETH_NS" || true
  ip link set "$VETH_H" master "$BR"
  ip link set "$VETH_H" up
  ip link set "$VETH_NS" netns "$NS"
  ip netns exec "$NS" ip link set "$VETH_NS" up
  ip netns exec "$NS" ip addr add "$NS_IP" dev "$VETH_NS" 2>/dev/null || true
  ip netns exec "$NS" ip link set lo up

done

echo "[ns-tap] Done. Namespaces: uav1ns uav2ns uav3ns"
echo "         TAP devices:      tap-uav1 tap-uav2 tap-uav3"