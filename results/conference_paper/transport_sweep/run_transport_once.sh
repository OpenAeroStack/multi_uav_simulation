#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="/home/multi_uav/FYP/multi_uav_simulation"
SCRIPT_DIR="$PROJECT/results/conference_paper/transport_sweep"
RELAY="$PROJECT/ros2/uav_vision/uav_vision/camera_relay.py"
DETECTOR="$PROJECT/ros2/uav_vision/uav_vision/detector.py"
BUILDER="$SCRIPT_DIR/build_transport_results.py"
RAW_ROOT="$SCRIPT_DIR/raw"
PROCESSED_ROOT="$SCRIPT_DIR/processed"
FRAME_RATE_HZ="1.0"
CONFIDENCE="0.25"
IMAGE_SIZE="960"
SETTLE_SECONDS="${SETTLE_SECONDS:-5}"
POST_WINDOW_GRACE_SECONDS="${POST_WINDOW_GRACE_SECONDS:-3}"
OFFICIAL_PUBLICATIONS=60

usage() {
    cat <<'EOF'
Usage: run_transport_once.sh <run_id> <rng_run> <jpeg_quality>

Ground-only JPEG transport feasibility run.
  jpeg_quality must be one of: 5 10 20 30 40 50

The existing Gazebo/namespace/NS-3 infrastructure must already be running
with the requested RNG run and the UAV at the fixed Phase F test pose.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi
[[ $# -eq 3 ]] || { usage >&2; exit 2; }

RUN_ID="$1"
RNG_RUN="$2"
JPEG_QUALITY="$3"
[[ "$RUN_ID" =~ ^[A-Za-z0-9._-]+$ ]] || {
    echo "ERROR: run_id contains unsupported characters" >&2; exit 2; }
[[ "$RNG_RUN" =~ ^[1-9][0-9]*$ ]] || {
    echo "ERROR: rng_run must be a positive integer" >&2; exit 2; }
case "$JPEG_QUALITY" in
    5|10|20|30|40|50) ;;
    *) echo "ERROR: jpeg_quality must be one of 5 10 20 30 40 50" >&2; exit 2 ;;
esac

RUN_RAW="$RAW_ROOT/$RUN_ID"
TRACE_OUTPUT="$PROCESSED_ROOT/transport_trace_${RUN_ID}.csv"
SUMMARY_OUTPUT="$PROCESSED_ROOT/transport_summary_${RUN_ID}.csv"
TAP_OUTPUT="$PROCESSED_ROOT/tap_deltas_${RUN_ID}.csv"
METADATA="$RUN_RAW/metadata.txt"
RELAY_LOG="$RUN_RAW/camera_relay.log"
DETECTOR_LOG="$RUN_RAW/detector.log"
RELAY_EVENTS="$RUN_RAW/relay_events.jsonl"
DETECTOR_EVENTS="$RUN_RAW/detector_events.jsonl"
TAP_BEFORE="$RUN_RAW/tap_before.txt"
TAP_AFTER="$RUN_RAW/tap_after.txt"
TOPIC_INFO="$RUN_RAW/compressed_topic_info.txt"
NETWORK_INFO="$RUN_RAW/network_configuration.txt"
NS3_ARCHIVE="$RUN_RAW/ns3.log"

[[ ! -e "$RUN_RAW" ]] || { echo "ERROR: run artifact directory exists: $RUN_RAW" >&2; exit 1; }
for output in "$TRACE_OUTPUT" "$SUMMARY_OUTPUT" "$TAP_OUTPUT"; do
    [[ ! -e "$output" ]] || { echo "ERROR: refusing to overwrite $output" >&2; exit 1; }
done
mkdir -p "$RUN_RAW" "$PROCESSED_ROOT"
: >"$RELAY_EVENTS"
: >"$DETECTOR_EVENTS"

sudo -v
for namespace in uav1ns gcsns; do
    sudo -n ip netns list | awk '{print $1}' | grep -Fxq "$namespace" || {
        echo "ERROR: namespace $namespace is unavailable; start infrastructure first" >&2
        exit 1
    }
done
[[ -f /tmp/ns3_single.log ]] || {
    echo "ERROR: /tmp/ns3_single.log is unavailable; NS-3 infrastructure is not verified" >&2
    exit 1
}

ros_in_gcs() {
    sudo -n ip netns exec gcsns runuser -u multi_uav -- bash -lc '
        source /opt/ros/humble/setup.bash
        source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
        exec "$@"
    ' ros-shell "$@"
}

snapshot_taps() {
    local destination="$1"
    {
        echo "--- tap-uav1 ---"
        sudo -n ip -s link show tap-uav1
        echo "--- tap-gcs ---"
        sudo -n ip -s link show tap-gcs
    } >"$destination"
}

RELAY_PGID=""
DETECTOR_PGID=""
stop_group() {
    local pgid="${1:-}"
    [[ -n "$pgid" ]] || return 0
    sudo -n kill -INT -- "-$pgid" 2>/dev/null || true
    for _ in {1..10}; do
        sudo -n kill -0 -- "-$pgid" 2>/dev/null || return 0
        sleep 0.2
    done
    sudo -n kill -TERM -- "-$pgid" 2>/dev/null || true
}
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_group "$RELAY_PGID"
    stop_group "$DETECTOR_PGID"
    exit "$status"
}
trap cleanup EXIT INT TERM

find_python_pid() {
    local path="$1"
    ps -eo pid=,comm=,args= | awk -v path="$path" '
        $2 ~ /^python/ && index($0, path) {print $1}
    '
}

wait_for_node() {
    local path="$1" log="$2" marker="$3" timeout_seconds="$4"
    local pid_variable="$5" pgid_variable="$6"
    local deadline=$((SECONDS + timeout_seconds))
    local -a matches=()
    while (( SECONDS < deadline )); do
        mapfile -t matches < <(find_python_pid "$path")
        if (( ${#matches[@]} > 1 )); then
            echo "ERROR: multiple Python processes found for $path" >&2
            return 1
        fi
        if (( ${#matches[@]} == 1 )) && grep -qF "$marker" "$log" 2>/dev/null; then
            printf -v "$pid_variable" '%s' "${matches[0]}"
            printf -v "$pgid_variable" '%s' "$(ps -o pgid= -p "${matches[0]}" | tr -d ' ')"
            return 0
        fi
        if grep -Eq 'Traceback|ModuleNotFoundError|ImportError|Model load failed' "$log" 2>/dev/null; then
            cat "$log" >&2
            return 1
        fi
        sleep 0.2
    done
    echo "ERROR: timed out waiting for node $path" >&2
    cat "$log" >&2 2>/dev/null || true
    return 1
}

echo "Confirm infrastructure RNG_RUN=$RNG_RUN and the fixed Phase F pose before continuing."
echo "This runner changes no DDS, QoS, MTU, socket-buffer, or NS-3 settings."

sudo -n setsid ip netns exec gcsns runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    source /home/multi_uav/yolo_env/bin/activate
    export PYTHONUNBUFFERED=1
    exec python3 "$1" --ros-args \
        -p uav_id:=1 -p processing_mode:=ground \
        -p conf_threshold:=0.25 -p publish_debug:=false \
        -p experiment_sequence_ids:=true -p transport_trace_path:="$2"
' detector-shell "$DETECTOR" "$DETECTOR_EVENTS" >"$DETECTOR_LOG" 2>&1 &
DETECTOR_WRAPPER_PID=$!
DETECTOR_PID=""
wait_for_node "$DETECTOR" "$DETECTOR_LOG" "Model loaded OK" 120 \
    DETECTOR_PID DETECTOR_PGID

sudo -n setsid ip netns exec uav1ns runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    export PYTHONUNBUFFERED=1
    exec python3 "$1" --ros-args \
        -p uav_id:=1 -p processing_mode:=ground -p jpeg_quality:="$2" \
        -p frame_rate_hz:=1.0 -p experiment_sequence_ids:=true \
        -p transport_trace_path:="$3"
' relay-shell "$RELAY" "$JPEG_QUALITY" "$RELAY_EVENTS" >"$RELAY_LOG" 2>&1 &
RELAY_WRAPPER_PID=$!
RELAY_PID=""
wait_for_node "$RELAY" "$RELAY_LOG" "GROUND output QoS: BEST_EFFORT" 20 \
    RELAY_PID RELAY_PGID

deadline=$((SECONDS + 30))
while true; do
    ros_in_gcs ros2 topic info -v /relay/uav1/compressed >"$TOPIC_INFO" 2>&1 || true
    if grep -Eq 'Publisher count: [1-9][0-9]*' "$TOPIC_INFO" &&
       grep -Eq 'Subscription count: [1-9][0-9]*' "$TOPIC_INFO"; then
        break
    fi
    sudo -n kill -0 -- "-$RELAY_PGID" 2>/dev/null || {
        echo "ERROR: relay exited during endpoint discovery" >&2; cat "$RELAY_LOG" >&2; exit 1; }
    (( SECONDS < deadline )) || {
        echo "ERROR: compressed-image endpoints did not match" >&2; cat "$TOPIC_INFO" >&2; exit 1; }
    sleep 0.2
done

echo "Endpoints matched; fixed settling period: ${SETTLE_SECONDS}s"
sleep "$SETTLE_SECONDS"
OFFICIAL_START="$(wc -l <"$RELAY_EVENTS")"
OFFICIAL_END=$((OFFICIAL_START + OFFICIAL_PUBLICATIONS))
snapshot_taps "$TAP_BEFORE"
OFFICIAL_START_ISO="$(date --iso-8601=ns)"
echo "Official window: sequences $((OFFICIAL_START + 1)) through $OFFICIAL_END"

deadline=$((SECONDS + 180))
while true; do
    CURRENT_COUNT="$(wc -l <"$RELAY_EVENTS")"
    (( CURRENT_COUNT < OFFICIAL_END )) || break
    sudo -n kill -0 -- "-$RELAY_PGID" 2>/dev/null || {
        echo "ERROR: relay exited before 60 official publications" >&2; exit 1; }
    (( SECONDS < deadline )) || {
        echo "ERROR: timed out waiting for 60 official publications" >&2; exit 1; }
    sleep 0.05
done
(( CURRENT_COUNT == OFFICIAL_END )) || {
    echo "ERROR: publication target overshot ($CURRENT_COUNT instead of $OFFICIAL_END)" >&2; exit 1; }

OFFICIAL_END_ISO="$(date --iso-8601=ns)"
snapshot_taps "$TAP_AFTER"
stop_group "$RELAY_PGID"; RELAY_PGID=""
sleep "$POST_WINDOW_GRACE_SECONDS"
stop_group "$DETECTOR_PGID"; DETECTOR_PGID=""
cp /tmp/ns3_single.log "$NS3_ARCHIVE"

{
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
    echo "net.core.wmem_max=$(sysctl -n net.core.wmem_max)"
    echo "net.core.rmem_max=$(sysctl -n net.core.rmem_max)"
    echo "--- root TAP/bridge/veth MTUs ---"
    ip -o link show tap-uav1
    ip -o link show tap-gcs
    ip -o link show br-uav1
    ip -o link show br-gcs
    echo "--- uav1ns interfaces ---"
    sudo -n ip netns exec uav1ns ip -o link show
    echo "--- gcsns interfaces ---"
    sudo -n ip netns exec gcsns ip -o link show
} >"$NETWORK_INFO"

python3 "$BUILDER" \
    --run-id "$RUN_ID" --rng-run "$RNG_RUN" --jpeg-quality "$JPEG_QUALITY" \
    --official-start "$OFFICIAL_START" --official-end "$OFFICIAL_END" \
    --relay-events "$RELAY_EVENTS" --detector-events "$DETECTOR_EVENTS" \
    --tap-before "$TAP_BEFORE" --tap-after "$TAP_AFTER" \
    --trace-output "$TRACE_OUTPUT" --summary-output "$SUMMARY_OUTPUT" \
    --tap-output "$TAP_OUTPUT"

cat >"$METADATA" <<EOF
run_id=$RUN_ID
rng_run=$RNG_RUN
jpeg_quality=$JPEG_QUALITY
frame_rate_hz=$FRAME_RATE_HZ
official_publications=$OFFICIAL_PUBLICATIONS
official_start_sequence=$((OFFICIAL_START + 1))
official_end_sequence=$OFFICIAL_END
official_start_time=$OFFICIAL_START_ISO
official_end_time=$OFFICIAL_END_ISO
yolo_model=YOLOv8n (/home/multi_uav/yolo_env/yolov8n.pt)
yolo_confidence=$CONFIDENCE
yolo_imgsz=$IMAGE_SIZE
yolo_classes=[0] person only
compressed_image_publisher_qos=BEST_EFFORT KEEP_LAST depth=1
ground_detector_subscriber_qos=BEST_EFFORT KEEP_LAST depth=1
rmw_implementation_environment=${RMW_IMPLEMENTATION:-<unset>}
ns3_configuration=three_uav_tapbridge_integrated.cc; 802.11a adhoc; OfdmRate6Mbps; existing DynamicObstacleLoss/LogDistance configuration
git_commit_hash=$(git -C "$PROJECT" rev-parse HEAD 2>/dev/null || echo unavailable)
topic_info_file=$TOPIC_INFO
network_configuration_file=$NETWORK_INFO
relay_log=$RELAY_LOG
detector_log=$DETECTOR_LOG
ns3_log=$NS3_ARCHIVE
transport_trace=$TRACE_OUTPUT
run_summary=$SUMMARY_OUTPUT
tap_deltas=$TAP_OUTPUT
EOF

echo "Run complete: $RUN_ID"
echo "  trace:   $TRACE_OUTPUT"
echo "  summary: $SUMMARY_OUTPUT"
echo "  raw:     $RUN_RAW"
