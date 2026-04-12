#!/bin/bash
set -euo pipefail

UAV_COUNT="${UAV_COUNT:-3}"
NS_PREFIX="${NS_PREFIX:-uav}"
BR_PREFIX="${BR_PREFIX:-br-uav}"
VETH_PREFIX="${VETH_PREFIX:-veth-uav}"
TAP_PREFIX="${TAP_PREFIX:-tap-uav}"
BASE_SUBNET_A="${BASE_SUBNET_A:-10}"
BASE_SUBNET_B="${BASE_SUBNET_B:-42}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Re-running with sudo for network namespace setup..."
  exec sudo -E "$0" "$@"
fi

owner_user="${SUDO_USER:-$USER}"

echo "=== Creating ${UAV_COUNT} namespace/TAP network slices ==="
for i in $(seq 1 "$UAV_COUNT"); do
  ns_name="${NS_PREFIX}${i}"
  br_name="${BR_PREFIX}${i}"
  veth_root="${VETH_PREFIX}${i}"
  veth_ns="eth0-${NS_PREFIX}${i}"
  tap_name="${TAP_PREFIX}${i}"
  subnet="${BASE_SUBNET_A}.${BASE_SUBNET_B}.${i}"
  ns_ip="${subnet}.2/24"
  br_ip="${subnet}.1/24"

  ip netns del "$ns_name" 2>/dev/null || true
  ip link del "$br_name" 2>/dev/null || true
  ip link del "$veth_root" 2>/dev/null || true
  ip tuntap del dev "$tap_name" mode tap 2>/dev/null || true

  echo "[UAV${i}] namespace=${ns_name} bridge=${br_name} tap=${tap_name}"

  ip netns add "$ns_name"

  ip link add "$veth_root" type veth peer name "$veth_ns"
  ip link set "$veth_ns" netns "$ns_name"

  ip netns exec "$ns_name" ip link set lo up
  ip netns exec "$ns_name" ip link set "$veth_ns" name eth0
  ip netns exec "$ns_name" ip addr add "$ns_ip" dev eth0
  ip netns exec "$ns_name" ip link set eth0 up

  ip link add "$br_name" type bridge
  ip addr add "$br_ip" dev "$br_name"
  ip link set "$br_name" up

  ip tuntap add dev "$tap_name" mode tap user "$owner_user"
  ip link set "$tap_name" up

  ip link set "$veth_root" master "$br_name"
  ip link set "$tap_name" master "$br_name"
  ip link set "$veth_root" up

  echo "  ns address : ${ns_ip}"
  echo "  tap device : ${tap_name}"
  echo "  bridge ip  : ${br_ip}"
  echo
 done

echo "=== Namespace/TAP setup complete ==="
echo "Inspect with: ip netns list && ip -br link | grep -E '${TAP_PREFIX}|${BR_PREFIX}|${VETH_PREFIX}'"
