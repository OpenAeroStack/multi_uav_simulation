#!/bin/bash
# cleanup_nestns_tap.sh — Tear down all processes and network plumbing
set -euo pipefail

# Kill processes inside gcsns first
sudo ip netns exec gcsns pkill -9 -f micro_ros_agent 2>/dev/null || true

sudo pkill -9 -f 'ip netns exec' 2>/dev/null || true
sudo pkill -9 -f arducopter      2>/dev/null || true
sudo pkill -9 -f gzserver        2>/dev/null || true
sudo pkill -9 -f gzclient        2>/dev/null || true
sudo pkill -9 -f micro_ros_agent 2>/dev/null || true
sudo pkill -9 -f drone_bridge    2>/dev/null || true
sudo pkill -9 -f three_uav       2>/dev/null || true
sudo pkill -9 -f socat           2>/dev/null || true
sleep 2

# Remove TAP devices
for dev in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
  sudo ip link del "$dev" 2>/dev/null || true
done

# Remove veth pairs (new naming from setup_netns_tap.sh)
for dev in veth-gcs veth-uav1 veth-uav2 veth-uav3 \
           veth-host1 veth-host2 veth-host3; do
  sudo ip link del "$dev" 2>/dev/null || true
done

# Remove legacy bridge devices (from old setup)
for dev in br-gcs br-uav1 br-uav2 br-uav3 \
           veth0h veth1h veth2h veth3h; do
  sudo ip link del "$dev" 2>/dev/null || true
done

# Remove namespaces
for ns in gcsns uav1 uav2 uav3 uav1ns uav2ns uav3ns; do
  sudo ip netns del "$ns" 2>/dev/null || true
done

# Remove stale host routes
for ip in 10.42.0.10 10.42.0.11 10.42.0.12 10.42.0.13; do
  sudo ip route del "${ip}/32" 2>/dev/null || true
done
sudo ip addr del 10.42.0.1/32 dev lo 2>/dev/null || true

echo "[cleanup] Done — all processes and network plumbing removed."