#!/bin/bash
# kill_all_netns.sh
# Standalone teardown for the single-UAV netns/NS-3 pipeline. Safe to run
# any time — after a clean shutdown, a crashed/interrupted run, or just to
# reset state before a fresh launch. Every step tolerates "already gone".

echo "=== Killing processes ==="
for pattern in \
    'drone_bridge' \
    'micro_ros_agent' \
    '/build/sitl/bin/arducopter' \
    'three_uav_tapbridge_integrated' \
    'two_drone_mission' \
    'uav1_patrol_mission' \
    'uav2_road_patrol' \
    'gcs_receiver' \
    'detection_viewer' \
    'world_pos_publisher' \
    'detector.py' \
    'camera_relay.py' \
    'metrics_logger' \
    'gzserver' \
    'gzclient' \
    'mavproxy' \
    'ros2-daemon'
do
    if sudo pkill -9 -f -- "$pattern" 2>/dev/null; then
        echo "  killed: $pattern"
    fi
done

# Also sweep anything still alive specifically inside the namespaces, in
# case a process was started under a name not covered above.
for ns in gcsns uav1ns; do
    if sudo ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "$ns"; then
        ns_pids="$(sudo ip netns pids "$ns" 2>/dev/null || true)"
        if [[ -n "$ns_pids" ]]; then
            echo "  killing leftover PIDs inside $ns: $ns_pids"
            sudo kill -9 $ns_pids 2>/dev/null || true
        fi
    fi
done

sleep 1

echo ""
echo "=== Removing namespaces ==="
for ns in gcsns uav1ns; do
    if sudo ip netns del "$ns" 2>/dev/null; then
        echo "  removed: $ns"
    fi
done

echo ""
echo "=== Removing bridges ==="
for br in br-gcs br-uav1; do
    sudo ip link set "$br" down 2>/dev/null || true
    if sudo ip link del "$br" type bridge 2>/dev/null; then
        echo "  removed: $br"
    fi
done

echo ""
echo "=== Removing TAPs and veths ==="
for link in tap-gcs tap-uav1 tap-uav2 tap-uav3 veth0h veth1h sim1h; do
    if sudo ip link del "$link" 2>/dev/null; then
        echo "  removed: $link"
    fi
done

echo ""
echo "=== Freeing stale Gazebo master port (11345) if held ==="
if sudo lsof -iTCP:11345 -sTCP:LISTEN >/dev/null 2>&1; then
    sudo fuser -k 11345/tcp 2>/dev/null || true
    echo "  freed port 11345"
else
    echo "  nothing holding it"
fi

echo ""
echo "=== Done — clean slate ==="
echo "  Remaining namespaces (should be empty of gcsns/uav1ns):"
sudo ip netns list 2>/dev/null || echo "  (none)"
