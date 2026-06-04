#!/bin/bash
# cleanup_netns_tap.sh
set -euo pipefail

for i in 1 2 3; do
  ip netns del "uav${i}ns" 2>/dev/null || true
  ip link del "tap-uav${i}" 2>/dev/null || true
  ip link del "br-uav${i}"  2>/dev/null || true
  ip link del "veth${i}h"   2>/dev/null || true
done

echo "[ns-tap] Cleaned up namespaces, TAP devices, and bridges."