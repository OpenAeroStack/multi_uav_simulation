#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="/home/multi_uav/FYP/multi_uav_simulation"
EXPERIMENT_DIR="$PROJECT/results/conference_paper/rate_sweep"
RELAY="$PROJECT/ros2/uav_vision/uav_vision/camera_relay.py"
DETECTOR="$PROJECT/ros2/uav_vision/uav_vision/detector.py"
BUILDER="$EXPERIMENT_DIR/build_rate_results.py"
GROUND_OBSERVER="$EXPERIMENT_DIR/ground_result_observer.py"
RAW_ROOT="$EXPERIMENT_DIR/raw"
FAILED_ROOT="$RAW_ROOT/failed"
PROCESSED_ROOT="$EXPERIMENT_DIR/processed"
JPEG_QUALITY=5
SETTLE_SECONDS=5
MEASUREMENT_SECONDS=60
DRAIN_SECONDS=5
DISCOVERY_TIMEOUT_SECONDS=60

usage() {
    cat <<'EOF'
Usage: run_rate_once.sh <mode> <run_id> <rng_run> <frame_rate_hz>

  mode:          edge or ground
  frame_rate_hz: 1, 2, or 5

Infrastructure/NS-3 must already use rng_run, and UAV1 must be holding the
fixed Phase F comparison pose. Ground runs always use JPEG quality 5.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then usage; exit 0; fi
[[ $# -eq 4 ]] || { usage >&2; exit 2; }
MODE="$1"; RUN_ID="$2"; RNG_RUN="$3"; FRAME_RATE_HZ="$4"
[[ "$MODE" == edge || "$MODE" == ground ]] || {
    echo "ERROR: mode must be edge or ground" >&2; exit 2; }
[[ "$RUN_ID" =~ ^[A-Za-z0-9._-]+$ ]] || {
    echo "ERROR: unsupported run_id characters" >&2; exit 2; }
[[ "$RNG_RUN" =~ ^[1-9][0-9]*$ ]] || {
    echo "ERROR: rng_run must be a positive integer" >&2; exit 2; }
case "$FRAME_RATE_HZ" in 1|2|5) ;; *)
    echo "ERROR: frame_rate_hz must be 1, 2, or 5" >&2; exit 2 ;; esac
FRAME_RATE_PARAMETER="${FRAME_RATE_HZ}.0"

for command_name in pidstat python3; do
    command -v "$command_name" >/dev/null || {
        echo "ERROR: required command unavailable: $command_name" >&2; exit 1; }
done
for source_file in "$RELAY" "$DETECTOR" "$BUILDER" "$GROUND_OBSERVER"; do
    [[ -f "$source_file" ]] || { echo "ERROR: missing $source_file" >&2; exit 1; }
done

RUN_RAW="$RAW_ROOT/$RUN_ID"
TRACE_OUTPUT="$PROCESSED_ROOT/frame_trace_${RUN_ID}.csv"
SUMMARY_OUTPUT="$PROCESSED_ROOT/rate_summary_${RUN_ID}.csv"
TAP_OUTPUT="$PROCESSED_ROOT/tap_deltas_${RUN_ID}.csv"
RELAY_LOG="$RUN_RAW/camera_relay.log"
DETECTOR_LOG="$RUN_RAW/detector.log"
GROUND_OBSERVER_LOG="$RUN_RAW/ground_result_observer.log"
RELAY_EVENTS="$RUN_RAW/relay_events.jsonl"
DETECTOR_EVENTS="$RUN_RAW/detector_events.jsonl"
GROUND_RESULT_EVENTS="$RUN_RAW/ground_result_events.jsonl"
PIDSTAT_LOG="$RUN_RAW/pidstat.txt"
TAP_BEFORE="$RUN_RAW/tap_before.txt"
TAP_AFTER="$RUN_RAW/tap_after.txt"
TOPIC_INFO="$RUN_RAW/image_topic_info.txt"
RESULT_TOPIC_INFO="$RUN_RAW/detection_topic_info.txt"
POSE_START="$RUN_RAW/uav_pose_at_official_start.txt"
NETWORK_INFO="$RUN_RAW/network_configuration.txt"
NS3_ARCHIVE="$RUN_RAW/ns3.log"
NS3_SOURCE_ARCHIVE="$RUN_RAW/three_uav_tapbridge_integrated.cc"
METADATA="$RUN_RAW/metadata.txt"

[[ ! -e "$RUN_RAW" ]] || { echo "ERROR: refusing to overwrite $RUN_RAW" >&2; exit 1; }
for output in "$TRACE_OUTPUT" "$SUMMARY_OUTPUT" "$TAP_OUTPUT"; do
    [[ ! -e "$output" ]] || { echo "ERROR: refusing to overwrite $output" >&2; exit 1; }
done
sudo -v
sudo -n true || { echo "ERROR: sudo credentials unavailable" >&2; exit 1; }
for namespace in uav1ns gcsns; do
    sudo -n ip netns list | awk '{print $1}' | grep -Fxq "$namespace" || {
        echo "ERROR: namespace $namespace is unavailable" >&2; exit 1; }
done
[[ -f /tmp/ns3_single.log ]] || {
    echo "ERROR: /tmp/ns3_single.log is unavailable" >&2; exit 1; }

existing_nodes="$(ps -eo comm=,args= | awk -v relay="$RELAY" -v detector="$DETECTOR" \
    -v observer="$GROUND_OBSERVER" '
    $1 ~ /^python/ && (index($0, relay) || index($0, detector) || index($0, observer))')"
[[ -z "$existing_nodes" ]] || {
    echo "ERROR: relay or detector is already running:" >&2
    echo "$existing_nodes" >&2; exit 1; }

ros_in_namespace() {
    local namespace="$1"; shift
    timeout 20s sudo -n ip netns exec "$namespace" runuser -u multi_uav -- bash -lc '
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

RELAY_PGID=""; DETECTOR_PGID=""; GROUND_OBSERVER_PGID=""
stop_group() {
    local pgid="${1:-}"
    [[ -n "$pgid" ]] || return 0
    sudo -n kill -INT -- "-$pgid" 2>/dev/null || true
    for _ in {1..10}; do
        sudo -n kill -0 -- "-$pgid" 2>/dev/null || return 0
        sleep 0.2
    done
    sudo -n kill -TERM -- "-$pgid" 2>/dev/null || true
    for _ in {1..10}; do
        sudo -n kill -0 -- "-$pgid" 2>/dev/null || return 0
        sleep 0.2
    done
    sudo -n kill -KILL -- "-$pgid" 2>/dev/null || true
}
cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_group "$RELAY_PGID"; stop_group "$DETECTOR_PGID"
    stop_group "$GROUND_OBSERVER_PGID"
    if (( status != 0 )) && [[ -d "$RUN_RAW" ]]; then
        local failed_destination
        failed_destination="$FAILED_ROOT/${RUN_ID}_failed_$(date +%Y%m%dT%H%M%S)_pid$$"
        mv "$RUN_RAW" "$failed_destination"
        echo "Failed-run artifacts preserved at: $failed_destination" >&2
        echo "Run ID '$RUN_ID' is available for a clean retry." >&2
    fi
    exit "$status"
}
trap cleanup EXIT INT TERM

# Create artifacts only after infrastructure/process preflight has passed and
# cleanup is active, so any later startup failure is archived automatically.
mkdir -p "$RUN_RAW" "$PROCESSED_ROOT" "$FAILED_ROOT"
: >"$RELAY_EVENTS"; : >"$DETECTOR_EVENTS"; : >"$GROUND_RESULT_EVENTS"

find_python_pid() {
    local path="$1"
    ps -eo pid=,comm=,args= | awk -v path="$path" '
        $2 ~ /^python/ && index($0, path) {print $1}'
}
wait_for_node() {
    local path="$1" log="$2" marker="$3" timeout_seconds="$4"
    local pid_variable="$5" pgid_variable="$6" deadline=$((SECONDS + timeout_seconds))
    local -a matches=()
    while (( SECONDS < deadline )); do
        mapfile -t matches < <(find_python_pid "$path")
        if (( ${#matches[@]} > 1 )); then
            echo "ERROR: multiple processes found for $path" >&2; return 1
        fi
        if (( ${#matches[@]} == 1 )) && grep -qF "$marker" "$log" 2>/dev/null; then
            printf -v "$pid_variable" '%s' "${matches[0]}"
            printf -v "$pgid_variable" '%s' "$(ps -o pgid= -p "${matches[0]}" | tr -d ' ')"
            return 0
        fi
        if grep -Eq 'Traceback|ModuleNotFoundError|ImportError|Model load failed' "$log" 2>/dev/null; then
            cat "$log" >&2; return 1
        fi
        sleep 0.2
    done
    echo "ERROR: timed out waiting for $path" >&2; cat "$log" >&2 || true; return 1
}

echo "Confirm infrastructure RNG_RUN=$RNG_RUN and the fixed Phase F pose."
echo "Configuration remains unchanged: YOLOv8n, conf=0.25, imgsz=960, class 0."
DETECTOR_NS="uav1ns"; [[ "$MODE" == ground ]] && DETECTOR_NS="gcsns"

sudo -n setsid ip netns exec gcsns runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    export PYTHONUNBUFFERED=1
    exec python3 "$1" --ros-args -p uav_id:=1 -p trace_path:="$2"
' observer-shell "$GROUND_OBSERVER" "$GROUND_RESULT_EVENTS" \
    >"$GROUND_OBSERVER_LOG" 2>&1 &
GROUND_OBSERVER_LAUNCHER_PID=$!
wait_for_node "$GROUND_OBSERVER" "$GROUND_OBSERVER_LOG" \
    "[GroundResultObserver] ready" 20 GROUND_OBSERVER_PID GROUND_OBSERVER_PGID

sudo -n setsid ip netns exec "$DETECTOR_NS" runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    source /home/multi_uav/yolo_env/bin/activate
    export PYTHONUNBUFFERED=1
    exec python3 "$1" --ros-args -p uav_id:=1 -p processing_mode:="$2" \
        -p conf_threshold:=0.25 -p publish_debug:=false \
        -p experiment_sequence_ids:=true -p transport_trace_path:="$3"
' detector-shell "$DETECTOR" "$MODE" "$DETECTOR_EVENTS" >"$DETECTOR_LOG" 2>&1 &
DETECTOR_LAUNCHER_PID=$!
wait_for_node "$DETECTOR" "$DETECTOR_LOG" "Model loaded OK" 120 DETECTOR_PID DETECTOR_PGID

relay_parameters=(-p uav_id:=1 -p processing_mode:="$MODE"
    -p frame_rate_hz:="$FRAME_RATE_PARAMETER" -p experiment_sequence_ids:=true
    -p transport_trace_path:="$RELAY_EVENTS")
[[ "$MODE" == ground ]] && relay_parameters+=(-p jpeg_quality:="$JPEG_QUALITY")
sudo -n setsid ip netns exec uav1ns runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    export PYTHONUNBUFFERED=1
    exec python3 "$1" "${@:2}"
' relay-shell "$RELAY" --ros-args "${relay_parameters[@]}" >"$RELAY_LOG" 2>&1 &
RELAY_LAUNCHER_PID=$!
RELAY_MARKER="EDGE output QoS: RELIABLE"
IMAGE_TOPIC="/cluster/cam/uav1"
[[ "$MODE" == ground ]] && {
    RELAY_MARKER="GROUND output QoS: BEST_EFFORT"; IMAGE_TOPIC="/relay/uav1/compressed"; }
wait_for_node "$RELAY" "$RELAY_LOG" "$RELAY_MARKER" 20 RELAY_PID RELAY_PGID

deadline=$((SECONDS + DISCOVERY_TIMEOUT_SECONDS))
while true; do
    ros_in_namespace "$DETECTOR_NS" ros2 topic info -v "$IMAGE_TOPIC" >"$TOPIC_INFO" 2>&1 || true
    if grep -Eq 'Publisher count: [1-9][0-9]*' "$TOPIC_INFO" &&
       grep -Eq 'Subscription count: [1-9][0-9]*' "$TOPIC_INFO"; then break; fi
    sudo -n kill -0 -- "-$RELAY_PGID" 2>/dev/null || {
        echo "ERROR: relay exited during endpoint discovery" >&2; exit 1; }
    sudo -n kill -0 -- "-$DETECTOR_PGID" 2>/dev/null || {
        echo "ERROR: detector exited during image-endpoint discovery" >&2
        cat "$DETECTOR_LOG" >&2; exit 1; }
    sudo -n kill -0 -- "-$GROUND_OBSERVER_PGID" 2>/dev/null || {
        echo "ERROR: ground result observer exited during image-endpoint discovery" >&2
        cat "$GROUND_OBSERVER_LOG" >&2; exit 1; }
    (( SECONDS < deadline )) || {
        echo "ERROR: image endpoints did not match" >&2; cat "$TOPIC_INFO" >&2; exit 1; }
    sleep 0.2
done

deadline=$((SECONDS + DISCOVERY_TIMEOUT_SECONDS))
while true; do
    # For Edge, query from the writer's namespace. DDS discovery across the
    # simulated path can be asymmetric in the GCS graph even while the writer
    # has matched the remote observer and results are reaching its callback.
    ros_in_namespace "$DETECTOR_NS" ros2 topic info -v /detections/uav1 \
        >"$RESULT_TOPIC_INFO" 2>&1 || true
    if grep -Eq 'Publisher count: [1-9][0-9]*' "$RESULT_TOPIC_INFO" &&
       grep -Eq 'Subscription count: [1-9][0-9]*' "$RESULT_TOPIC_INFO"; then break; fi
    sudo -n kill -0 -- "-$DETECTOR_PGID" 2>/dev/null || {
        echo "ERROR: detector exited during result-endpoint discovery" >&2
        cat "$DETECTOR_LOG" >&2; exit 1; }
    sudo -n kill -0 -- "-$GROUND_OBSERVER_PGID" 2>/dev/null || {
        echo "ERROR: ground result observer exited during endpoint discovery" >&2
        cat "$GROUND_OBSERVER_LOG" >&2; exit 1; }
    sudo -n kill -0 -- "-$RELAY_PGID" 2>/dev/null || {
        echo "ERROR: relay exited during result-endpoint discovery" >&2
        cat "$RELAY_LOG" >&2; exit 1; }
    (( SECONDS < deadline )) || {
        echo "ERROR: detection-result endpoints did not match within ${DISCOVERY_TIMEOUT_SECONDS}s" >&2
        echo "--- writer-side /detections/uav1 endpoint state ---" >&2
        cat "$RESULT_TOPIC_INFO" >&2
        echo "--- detector log (last 80 lines) ---" >&2
        tail -n 80 "$DETECTOR_LOG" >&2
        echo "--- observer log (last 80 lines) ---" >&2
        tail -n 80 "$GROUND_OBSERVER_LOG" >&2
        echo "--- relay log (last 40 lines) ---" >&2
        tail -n 40 "$RELAY_LOG" >&2
        exit 1; }
    sleep 0.2
done

echo "Endpoints matched; fixed settling period: ${SETTLE_SECONDS}s"
sleep "$SETTLE_SECONDS"
{
    echo "--- /ap/v1/navsat ---"
    ros_in_namespace gcsns ros2 topic echo --once /ap/v1/navsat || true
    echo "--- /ap/v1/pose/filtered ---"
    ros_in_namespace gcsns ros2 topic echo --once /ap/v1/pose/filtered || true
} >"$POSE_START" 2>&1

snapshot_taps "$TAP_BEFORE"
OFFICIAL_START_TIME="$(date +%s.%N)"
OFFICIAL_START_ISO="$(date --iso-8601=ns)"
echo "Official 60-second window started at $OFFICIAL_START_ISO"
pidstat -h -u -r -p "${RELAY_PID},${DETECTOR_PID}" 1 "$MEASUREMENT_SECONDS" >"$PIDSTAT_LOG"
OFFICIAL_END_TIME="$(date +%s.%N)"
OFFICIAL_END_ISO="$(date --iso-8601=ns)"
sudo -n kill -INT -- "-$RELAY_PGID" 2>/dev/null || true
snapshot_taps "$TAP_AFTER"
stop_group "$RELAY_PGID"; RELAY_PGID=""
echo "Official admission stopped; fixed detector drain: ${DRAIN_SECONDS}s"
sleep "$DRAIN_SECONDS"
stop_group "$DETECTOR_PGID"; DETECTOR_PGID=""
stop_group "$GROUND_OBSERVER_PGID"; GROUND_OBSERVER_PGID=""

cp /tmp/ns3_single.log "$NS3_ARCHIVE"
NS3_SOURCE="/home/multi_uav/ns-allinone-3.38/ns-3.38/scratch/three_uav_tapbridge_integrated.cc"
[[ -f "$NS3_SOURCE" ]] && cp "$NS3_SOURCE" "$NS3_SOURCE_ARCHIVE"
{
    echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
    echo "net.core.wmem_max=$(sysctl -n net.core.wmem_max)"
    echo "net.core.rmem_max=$(sysctl -n net.core.rmem_max)"
    echo "--- root interfaces ---"
    ip -o link show tap-uav1; ip -o link show tap-gcs
    ip -o link show br-uav1; ip -o link show br-gcs
    echo "--- uav1ns interfaces ---"
    sudo -n ip netns exec uav1ns ip -o link show
    echo "--- gcsns interfaces ---"
    sudo -n ip netns exec gcsns ip -o link show
} >"$NETWORK_INFO"

python3 "$BUILDER" --run-id "$RUN_ID" --rng-run "$RNG_RUN" --mode "$MODE" \
    --frame-rate-hz "$FRAME_RATE_HZ" --official-start-time "$OFFICIAL_START_TIME" \
    --official-end-time "$OFFICIAL_END_TIME" --relay-events "$RELAY_EVENTS" \
    --detector-events "$DETECTOR_EVENTS" \
    --ground-result-events "$GROUND_RESULT_EVENTS" --pidstat "$PIDSTAT_LOG" \
    --relay-pid "$RELAY_PID" --detector-pid "$DETECTOR_PID" \
    --tap-before "$TAP_BEFORE" --tap-after "$TAP_AFTER" \
    --trace-output "$TRACE_OUTPUT" --summary-output "$SUMMARY_OUTPUT" \
    --tap-output "$TAP_OUTPUT"

OFFICIAL_PUBLICATIONS="$(python3 -c 'import csv,sys; print(next(csv.DictReader(open(sys.argv[1])))["official_publications"])' "$SUMMARY_OUTPUT")"
JPEG_VALUE="NA"; [[ "$MODE" == ground ]] && JPEG_VALUE=5
cat >"$METADATA" <<EOF
run_id=$RUN_ID
rng_run=$RNG_RUN
mode=$MODE
requested_frame_rate_hz=$FRAME_RATE_HZ
actual_official_publication_count=$OFFICIAL_PUBLICATIONS
official_start_time_epoch=$OFFICIAL_START_TIME
official_end_time_epoch=$OFFICIAL_END_TIME
official_start_time_iso=$OFFICIAL_START_ISO
official_end_time_iso=$OFFICIAL_END_ISO
official_window_seconds=$MEASUREMENT_SECONDS
settling_seconds=$SETTLE_SECONDS
drain_seconds=$DRAIN_SECONDS
ground_jpeg_quality=$JPEG_VALUE
yolo_model=YOLOv8n (/home/multi_uav/yolo_env/yolov8n.pt)
yolo_confidence=0.25
yolo_imgsz=960
yolo_target_classes=[0] person only
camera_input_qos=BEST_EFFORT KEEP_LAST depth=1
edge_image_qos=RELIABLE KEEP_LAST depth=1
ground_compressed_qos=BEST_EFFORT KEEP_LAST depth=1
detection_output_qos=RELIABLE KEEP_LAST depth=10
ground_result_observer_qos=RELIABLE KEEP_LAST depth=10
primary_pipeline_latency=frame admission at relay to beginning of ground result callback
rmw_implementation_environment=${RMW_IMPLEMENTATION:-<unset>}
fixed_phase_f_pose=latitude 6.079430, longitude 80.193085, relative altitude 25.0 m, yaw 102.6 degrees
git_commit_hash=$(git -C "$PROJECT" rev-parse HEAD 2>/dev/null || echo unavailable)
ns3_configuration=existing single-UAV infrastructure; archived log: $NS3_ARCHIVE
ns3_source_archive=$NS3_SOURCE_ARCHIVE
tap_counters=interface traffic including non-image traffic
relay_log=$RELAY_LOG
detector_log=$DETECTOR_LOG
ground_result_observer_log=$GROUND_OBSERVER_LOG
ground_result_events=$GROUND_RESULT_EVENTS
pidstat_log=$PIDSTAT_LOG
tap_before=$TAP_BEFORE
tap_after=$TAP_AFTER
network_configuration=$NETWORK_INFO
pose_at_official_start=$POSE_START
frame_trace=$TRACE_OUTPUT
run_summary=$SUMMARY_OUTPUT
EOF

echo "PASS: completed $MODE $FRAME_RATE_HZ Hz run $RUN_ID"
echo "Summary: $SUMMARY_OUTPUT"
