#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_ROOT="${NS3_ROOT:-/home/multi_uav/ns-allinone-3.38/ns-3.38}"
TMP="$(mktemp -d /tmp/network_scaling_preflight.XXXXXX)"
NS3_PGID=""; POS_PGID=""; OBS_PGID=""; SERVER_PGID=""
cleanup() {
    trap - EXIT INT TERM
    for pgid in "$SERVER_PGID" "$POS_PGID" "$OBS_PGID" "$NS3_PGID"; do
        [[ -n "$pgid" ]] && sudo -n kill -TERM -- "-$pgid" 2>/dev/null || true
    done
    rm -rf "$TMP"
}
trap cleanup EXIT INT TERM
sudo -v; sudo -n true

for ns in gcsns uav1ns uav2ns uav3ns; do
    sudo -n ip netns list | awk '{print $1}' | grep -Fxq "$ns" || { echo "FAIL: missing $ns"; exit 1; }
done
for node in gcs uav1 uav2 uav3; do
    tap="tap-$node"; bridge="br-$node"
    ip -o link show "$tap" | grep -q "master $bridge" || { echo "FAIL: $tap is not attached to $bridge"; exit 1; }
done
pgrep -f three_uav_tapbridge_integrated >/dev/null && { echo "FAIL: stop the existing NS-3 process first"; exit 1; }

echo "[1/4] Negative control: NS-3 stopped"
if sudo -n ip netns exec uav1ns ping -c 2 -W 1 10.42.0.10 >/dev/null 2>&1; then
    echo "FAIL: UAV1 reached GCS with NS-3 stopped (bypass exists)"; exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u
setsid ros2 topic pub --rate 10 /uav_world_positions std_msgs/msg/Float32MultiArray \
    '{data: [0, 0, 6, 2.9, 1, 0, 2, 20, 2, 0, -2, 20, 3, 7, 1, 18]}' >"$TMP/positions.log" 2>&1 & POS_PGID=$!
setsid ros2 topic pub --rate 10 /link_obstacle_loss std_msgs/msg/Float32MultiArray \
    '{data: [0,1,0, 0,2,0, 0,3,0, 1,2,0, 1,3,0, 2,3,0]}' >"$TMP/obstacles.log" 2>&1 & OBS_PGID=$!
wall_start="$(date +%s.%N)"
setsid bash -lc 'cd "$1" && exec ./ns3 run "three_uav_tapbridge_integrated --enableTap=true --simTime=0 --rngRun=1 --posLogPeriod=1"' ns3-shell "$NS3_ROOT" >"$TMP/ns3.log" 2>&1 & NS3_PGID=$!

deadline=$((SECONDS + 60))
until grep -q 't=' "$TMP/ns3.log" 2>/dev/null; do
    kill -0 "$NS3_PGID" 2>/dev/null || { cat "$TMP/ns3.log"; echo "FAIL: NS-3 exited"; exit 1; }
    (( SECONDS < deadline )) || { cat "$TMP/ns3.log"; echo "FAIL: NS-3 readiness timeout"; exit 1; }
    sleep 0.25
done
deadline=$((SECONDS + 30))
while ! grep -q '\[integration check\] OK:' "$TMP/ns3.log" 2>/dev/null; do
    grep -q 'INCOMPLETE FEED' "$TMP/ns3.log" 2>/dev/null && {
        cat "$TMP/ns3.log"; echo "FAIL: NS-3 did not receive all positions/links"; exit 1; }
    kill -0 "$NS3_PGID" 2>/dev/null || { cat "$TMP/ns3.log"; echo "FAIL: NS-3 exited"; exit 1; }
    (( SECONDS < deadline )) || { cat "$TMP/ns3.log"; echo "FAIL: integration-check timeout"; exit 1; }
    sleep 0.25
done

echo "[2/4] Connectivity through NS-3"
for uav in 1 2 3; do
    sudo -n ip netns exec "uav${uav}ns" ping -c 2 -W 2 10.42.0.10 >/dev/null || {
        echo "FAIL: UAV$uav cannot reach GCS"; exit 1; }
done

echo "[3/4] One-UAV 500-Kbit/s UDP smoke test"
sudo -n setsid ip netns exec gcsns runuser -u multi_uav -- iperf3 -s -1 -p 5201 >"$TMP/server.log" 2>&1 & SERVER_PGID=$!
sleep 1
timeout 20 sudo -n ip netns exec uav1ns runuser -u multi_uav -- \
    iperf3 -c 10.42.0.10 -p 5201 --connect-timeout 5000 -u -b 500K -t 5 -J >"$TMP/client.json"
python3 - "$TMP/client.json" <<'PY'
import json, sys
d=json.load(open(sys.argv[1])); r=d['end']['sum_received']
if r.get('bits_per_second',0) < 350000:
    raise SystemExit('FAIL: received goodput below 350 Kbit/s')
print(f"  received={r['bits_per_second']/1e6:.3f} Mbps loss={r.get('lost_percent',0):.2f}%")
PY

echo "[4/4] Real-time progress check"
wall_end="$(date +%s.%N)"
first_sim="$(grep -oE 't=[0-9]+\.[0-9]+s' "$TMP/ns3.log" | head -1 | tr -d 'ts=')"
last_sim="$(grep -oE 't=[0-9]+\.[0-9]+s' "$TMP/ns3.log" | tail -1 | tr -d 'ts=')"
python3 - "$wall_start" "$wall_end" "$first_sim" "$last_sim" <<'PY'
import sys
wall=float(sys.argv[2])-float(sys.argv[1]); sim=float(sys.argv[4])-float(sys.argv[3])
ratio=sim/wall if wall else 0
print(f"  simulated_elapsed={sim:.3f}s wall_elapsed={wall:.3f}s ratio={ratio:.3f}")
if ratio < 0.90:
    raise SystemExit('FAIL: NS-3 materially behind wall time (ratio < 0.90)')
PY
echo "PASS: topology, no-bypass control, three GCS paths, 500-Kbit/s flow, and real-time progress verified."
echo "Preflight does not launch official runs."
