#!/bin/bash
set -euo pipefail

sudo pkill -9 -f arducopter      2>/dev/null || true
sudo pkill -9 -f gzserver        2>/dev/null || true
sudo pkill -9 -f gzclient        2>/dev/null || true
sudo pkill -9 -f micro_ros_agent 2>/dev/null || true
sudo pkill -9 -f drone_bridge    2>/dev/null || true
sudo pkill -9 -f three_uav       2>/dev/null || true
sudo pkill -9 -f socat           2>/dev/null || true
sleep 2

for dev in tap-gcs tap-uav1 tap-uav2 tap-uav3 \
           br-gcs  br-uav1  br-uav2  br-uav3  \
           veth0h  veth1h   veth2h   veth3h; do
  sudo ip link del "$dev" 2>/dev/null || true
done

for ns in gcsns uav1ns uav2ns uav3ns; do
  sudo ip netns del "$ns" 2>/dev/null || true
done

echo "[cleanup] Done — all processes and network plumbing removed."
