#!/bin/bash
set -euo pipefail

UAV_COUNT="${UAV_COUNT:-3}"
NS_PREFIX="${NS_PREFIX:-uav}"
BR_PREFIX="${BR_PREFIX:-br-uav}"
VETH_PREFIX="${VETH_PREFIX:-veth-uav}"
TAP_PREFIX="${TAP_PREFIX:-tap-uav}"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Re-running with sudo for network namespace cleanup..."
  exec sudo -E "$0" "$@"
fi

echo "=== Cleaning ${UAV_COUNT} namespace/TAP network slices ==="
for i in $(seq 1 "$UAV_COUNT"); do
  ns_name="${NS_PREFIX}${i}"
  br_name="${BR_PREFIX}${i}"
  veth_root="${VETH_PREFIX}${i}"
  tap_name="${TAP_PREFIX}${i}"

  ip netns del "$ns_name" 2>/dev/null || true
  ip link del "$br_name" 2>/dev/null || true
  ip link del "$veth_root" 2>/dev/null || true
  ip tuntap del dev "$tap_name" mode tap 2>/dev/null || true

done

echo "=== Cleanup complete ==="
