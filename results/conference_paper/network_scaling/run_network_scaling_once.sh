#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="/home/multi_uav/FYP/multi_uav_simulation"
ROOT="$PROJECT/results/conference_paper/network_scaling"
BUILDER="$ROOT/build_network_scaling_results.py"
RAW_ROOT="$ROOT/raw"
PROCESSED_ROOT="$ROOT/processed"
RATE="500K"
RATE_MBPS="0.5"
DURATION=30
SETTLE_SECONDS=5
RELEASE_LEAD_SECONDS=3
NS3_ROOT="${NS3_ROOT:-/home/multi_uav/ns-allinone-3.38/ns-3.38}"

usage() {
    echo "Usage: $0 <n_active_uav: 1|2|3> <run_id> <rng_run>"
}
[[ "${1:-}" == -h || "${1:-}" == --help ]] && { usage; exit 0; }
[[ $# -eq 3 ]] || { usage >&2; exit 2; }
N_ACTIVE="$1"; RUN_ID="$2"; RNG_RUN="$3"
[[ "$N_ACTIVE" =~ ^[123]$ ]] || { echo "ERROR: n_active_uav must be 1, 2, or 3" >&2; exit 2; }
[[ "$RUN_ID" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "ERROR: invalid run_id" >&2; exit 2; }
[[ "$RNG_RUN" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: rng_run must be positive" >&2; exit 2; }

for command_name in iperf3 python3 ros2 timeout; do
    command -v "$command_name" >/dev/null || { echo "ERROR: missing $command_name" >&2; exit 1; }
done
[[ -x "$NS3_ROOT/ns3" ]] || { echo "ERROR: NS-3 wrapper unavailable: $NS3_ROOT/ns3" >&2; exit 1; }
[[ -f "$BUILDER" ]] || { echo "ERROR: missing result builder" >&2; exit 1; }
sudo -v; sudo -n true

for namespace in gcsns uav1ns uav2ns uav3ns; do
    sudo -n ip netns list | awk '{print $1}' | grep -Fxq "$namespace" || {
        echo "ERROR: missing namespace $namespace; run setup_network_scaling.sh" >&2; exit 1; }
done
for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
    ip link show "$tap" >/dev/null 2>&1 || { echo "ERROR: missing $tap" >&2; exit 1; }
done
pgrep -f 'three_uav_tapbridge_integrated' >/dev/null 2>&1 && {
    echo "ERROR: an integrated NS-3 process is already running" >&2; exit 1; }

RUN_RAW="$RAW_ROOT/$RUN_ID"
[[ ! -e "$RUN_RAW" ]] || { echo "ERROR: refusing to overwrite $RUN_RAW" >&2; exit 1; }
mkdir -p "$RUN_RAW" "$PROCESSED_ROOT" "$ROOT/final"

NS3_LOG="$RUN_RAW/ns3.log"
POSITION_LOG="$RUN_RAW/fixed_positions.log"
OBSTACLE_LOG="$RUN_RAW/fixed_obstacles.log"
TAP_BEFORE="$RUN_RAW/tap_before.csv"
TAP_AFTER="$RUN_RAW/tap_after.csv"
METADATA="$RUN_RAW/metadata.txt"
NS3_PGID=""; POSITION_PGID=""; OBSTACLE_PGID=""
declare -a SERVER_PGIDS=() CLIENT_PGIDS=()

stop_group() {
    local pgid="${1:-}"; [[ -n "$pgid" ]] || return 0
    sudo -n kill -INT -- "-$pgid" 2>/dev/null || true
    for _ in {1..10}; do sudo -n kill -0 -- "-$pgid" 2>/dev/null || return 0; sleep 0.2; done
    sudo -n kill -TERM -- "-$pgid" 2>/dev/null || true
    sleep 1
    sudo -n kill -KILL -- "-$pgid" 2>/dev/null || true
}
cleanup() {
    local status=$?; trap - EXIT INT TERM
    for pgid in "${CLIENT_PGIDS[@]}" "${SERVER_PGIDS[@]}"; do stop_group "$pgid"; done
    stop_group "$POSITION_PGID"; stop_group "$OBSTACLE_PGID"; stop_group "$NS3_PGID"
    exit "$status"
}
trap cleanup EXIT INT TERM

snapshot_taps() {
    local output="$1"
    echo 'interface,rx_bytes,rx_packets,rx_errors,rx_dropped,tx_bytes,tx_packets,tx_errors,tx_dropped' >"$output"
    for interface in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
        stats="/sys/class/net/$interface/statistics"
        printf '%s,%s,%s,%s,%s,%s,%s,%s,%s\n' "$interface" \
            "$(<"$stats/rx_bytes")" "$(<"$stats/rx_packets")" \
            "$(<"$stats/rx_errors")" "$(<"$stats/rx_dropped")" \
            "$(<"$stats/tx_bytes")" "$(<"$stats/tx_packets")" \
            "$(<"$stats/tx_errors")" "$(<"$stats/tx_dropped")" >>"$output"
    done
}

set +u
source /opt/ros/humble/setup.bash
set -u
setsid ros2 topic pub --rate 10 /uav_world_positions std_msgs/msg/Float32MultiArray \
    '{data: [0, 0, 6, 2.9, 1, 0, 2, 20, 2, 0, -2, 20, 3, 7, 1, 18]}' \
    >"$POSITION_LOG" 2>&1 & POSITION_PGID=$!
setsid ros2 topic pub --rate 10 /link_obstacle_loss std_msgs/msg/Float32MultiArray \
    '{data: [0,1,0, 0,2,0, 0,3,0, 1,2,0, 1,3,0, 2,3,0]}' \
    >"$OBSTACLE_LOG" 2>&1 & OBSTACLE_PGID=$!

NS3_WALL_START="$(date +%s.%N)"
setsid bash -lc 'cd "$1" && exec ./ns3 run "three_uav_tapbridge_integrated --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --enableTap=true --simTime=0 --rngRun=$2 --posLogPeriod=1"' \
    ns3-shell "$NS3_ROOT" "$RNG_RUN" >"$NS3_LOG" 2>&1 & NS3_PGID=$!

deadline=$((SECONDS + 60))
while true; do
    kill -0 "$NS3_PGID" 2>/dev/null || { cat "$NS3_LOG" >&2; echo "ERROR: NS-3 exited" >&2; exit 1; }
    attached=true
    for tap in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
        [[ -r "/sys/class/net/$tap/carrier" && "$(<"/sys/class/net/$tap/carrier")" == 1 ]] || attached=false
    done
    $attached && grep -q 'UAV3' "$NS3_LOG" 2>/dev/null && break
    (( SECONDS < deadline )) || { cat "$NS3_LOG" >&2; echo "ERROR: TAP attachment timeout" >&2; exit 1; }
    sleep 0.25
done
sleep "$SETTLE_SECONDS"
deadline=$((SECONDS + 30))
while ! grep -q '\[integration check\] OK:' "$NS3_LOG" 2>/dev/null; do
    if grep -q 'INCOMPLETE FEED' "$NS3_LOG" 2>/dev/null; then
        cat "$NS3_LOG" >&2; echo "ERROR: incomplete fixed position/link feed" >&2; exit 1
    fi
    kill -0 "$NS3_PGID" 2>/dev/null || { cat "$NS3_LOG" >&2; echo "ERROR: NS-3 exited before integration check" >&2; exit 1; }
    (( SECONDS < deadline )) || { cat "$NS3_LOG" >&2; echo "ERROR: timed out waiting for NS-3 integration check" >&2; exit 1; }
    sleep 0.25
done

for uav in $(seq 1 "$N_ACTIVE"); do
    port=$((5200 + uav))
    server_log="$RUN_RAW/iperf_server_uav${uav}.log"
    sudo -n setsid ip netns exec gcsns runuser -u multi_uav -- \
        iperf3 -s -1 -p "$port" >"$server_log" 2>&1 &
    SERVER_PGIDS+=("$!")
done
sleep 1

RELEASE_TIME="$(python3 -c 'import time; print(f"{time.time()+float(__import__("sys").argv[1]):.9f}")' "$RELEASE_LEAD_SECONDS")"
for uav in $(seq 1 "$N_ACTIVE"); do
    port=$((5200 + uav))
    json="$RUN_RAW/iperf_uav${uav}.json"
    start_file="$RUN_RAW/client_uav${uav}_start.txt"
    end_file="$RUN_RAW/client_uav${uav}_end.txt"
    sudo -n setsid ip netns exec "uav${uav}ns" runuser -u multi_uav -- bash -lc '
        delay=$(python3 -c '\''import sys,time; print(max(0.0,float(sys.argv[1])-time.time()))'\'' "$1")
        sleep "$delay"
        date +%s.%N >"$2"
        iperf3 -c 10.42.0.10 -p "$3" --connect-timeout 5000 -u -b "$4" -t "$5" -J >"$6"
        status=$?
        date +%s.%N >"$7"
        exit "$status"
    ' client-shell "$RELEASE_TIME" "$start_file" "$port" "$RATE" "$DURATION" "$json" "$end_file" &
    CLIENT_PGIDS+=("$!")
done

snapshot_taps "$TAP_BEFORE"
OFFICIAL_START="$RELEASE_TIME"
echo "Official common release boundary: $OFFICIAL_START"
client_failure=0
for pid in "${CLIENT_PGIDS[@]}"; do wait "$pid" || client_failure=1; done
CLIENT_PGIDS=()
(( client_failure == 0 )) || { echo "ERROR: at least one iperf3 client failed" >&2; exit 1; }
OFFICIAL_END="$(python3 -c 'import sys; print(f"{float(sys.argv[1])+30.0:.9f}")' "$OFFICIAL_START")"
snapshot_taps "$TAP_AFTER"
NS3_WALL_END="$(date +%s.%N)"

cat >"$METADATA" <<EOF
run_id=$RUN_ID
n_active_uav=$N_ACTIVE
rng_run=$RNG_RUN
configured_offered_mbps_per_uav=$RATE_MBPS
traffic_duration_seconds=$DURATION
official_start_time_epoch=$OFFICIAL_START
official_end_time_epoch=$OFFICIAL_END
ns3_wall_start_epoch=$NS3_WALL_START
ns3_wall_end_epoch=$NS3_WALL_END
positions=0:(0,6,2.9);1:(0,2,20);2:(0,-2,20);3:(7,1,18)
obstacle_loss_db=all six links 0
ns3_phy=unchanged integrated topology
EOF

NS3_SOURCE="$NS3_ROOT/scratch/three_uav_tapbridge_integrated.cc"
[[ -f "$NS3_SOURCE" ]] || NS3_SOURCE="$PROJECT/ns3/three_uav_tapbridge_integrated.cc"
cp "$NS3_SOURCE" "$RUN_RAW/three_uav_tapbridge_integrated.cc"
sha256sum "$RUN_RAW/three_uav_tapbridge_integrated.cc" \
    >"$RUN_RAW/ns3_source_sha256.txt"
printf '%s\n' "three_uav_tapbridge_integrated --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --enableTap=true --simTime=0 --rngRun=$RNG_RUN --posLogPeriod=1" \
    >"$RUN_RAW/ns3_command.txt"

python3 "$BUILDER" --run-id "$RUN_ID" --n-active-uav "$N_ACTIVE" --rng-run "$RNG_RUN" \
    --raw-dir "$RUN_RAW" --tap-before "$TAP_BEFORE" --tap-after "$TAP_AFTER" \
    --ns3-log "$NS3_LOG" --official-start "$OFFICIAL_START" --official-end "$OFFICIAL_END"
echo "PASS: completed $RUN_ID"
