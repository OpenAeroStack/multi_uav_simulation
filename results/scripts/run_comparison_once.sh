#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="/home/multi_uav/FYP/multi_uav_simulation"
FRAME_RATE_HZ="1.0"
CONF_THRESHOLD="0.25"
GROUND_JPEG_QUALITY="5"
WARMUP_FRAMES="9"
MEASUREMENT_DURATION="${4:-60}"
TARGET_LATITUDE="6.079430"
TARGET_LONGITUDE="80.193085"
TARGET_ALTITUDE_M="25.0"

MODE="${1:-}"
RUN_ID="${2:-}"
RNG_RUN="${3:-}"
RUN_USER="${SUDO_USER:-$USER}"
RAW_DIR="$PROJECT/results/phase_f_comparison/raw"
RESOURCE_DIR="$PROJECT/results/phase_e_resources/raw"
DETECTOR="$PROJECT/ros2/uav_vision/uav_vision/detector.py"
RELAY="$PROJECT/ros2/uav_vision/uav_vision/camera_relay.py"
METRICS="$PROJECT/ros2/uav_vision/uav_vision/metrics_logger.py"

usage() {
    echo "Usage: $0 <edge|ground> <run_id> <rng_run> [measurement_duration_seconds]" >&2
}

[[ $# -ge 3 && $# -le 4 ]] || { usage; exit 2; }
[[ "$MODE" == edge || "$MODE" == ground ]] || { echo "ERROR: mode must be exactly edge or ground." >&2; exit 2; }
[[ -n "$RUN_ID" && "$RUN_ID" =~ ^[A-Za-z0-9._-]+$ ]] || { echo "ERROR: run_id must use only letters, digits, dot, underscore, or hyphen." >&2; exit 2; }
[[ "$RNG_RUN" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: rng_run must be a positive integer." >&2; exit 2; }
[[ "$MEASUREMENT_DURATION" =~ ^[1-9][0-9]*$ ]] || { echo "ERROR: duration must be a positive integer." >&2; exit 2; }

command -v pidstat >/dev/null || { echo "Install sysstat using: sudo apt install sysstat" >&2; exit 1; }
for source_file in "$DETECTOR" "$RELAY" "$METRICS"; do
    [[ -f "$source_file" ]] || { echo "ERROR: source file not found: $source_file" >&2; exit 1; }
done
mkdir -p "$RAW_DIR" "$RESOURCE_DIR"
[[ -w "$RAW_DIR" && -w "$RESOURCE_DIR" ]] || { echo "ERROR: output directories are not writable." >&2; exit 1; }

DETECTOR_LOG="$RAW_DIR/detector_${RUN_ID}.log"
METRICS_LOG="$RAW_DIR/metrics_${RUN_ID}.log"
RELAY_LOG="$RAW_DIR/relay_${RUN_ID}.log"
METADATA="$RAW_DIR/metadata_${RUN_ID}.txt"
RESOURCE_LOG="$RESOURCE_DIR/resources_${RUN_ID}.txt"
for artifact in "$DETECTOR_LOG" "$METRICS_LOG" "$RELAY_LOG" "$METADATA" "$RESOURCE_LOG"; do
    [[ ! -e "$artifact" ]] || { echo "ERROR: $artifact already exists; use a new Run ID." >&2; exit 1; }
done
shopt -s nullglob
old_csvs=("$RAW_DIR"/metrics_uav1_"$MODE"_"$RUN_ID"_*.csv)
shopt -u nullglob
(( ${#old_csvs[@]} == 0 )) || { echo "ERROR: a metrics CSV already exists for Run ID $RUN_ID; use a new Run ID." >&2; exit 1; }

sudo -v
sudo -n true || {
    echo "ERROR: sudo authentication is unavailable. Run sudo -v in this terminal and retry." >&2
    exit 1
}
for ns in gcsns uav1ns; do
    sudo -n ip netns list | awk '{print $1}' | grep -Fxq "$ns" || { echo "ERROR: namespace $ns is not available; infrastructure is not ready." >&2; exit 1; }
done

ros_in_gcs() {
    timeout 15s sudo -n ip netns exec gcsns runuser -u multi_uav -- bash -lc '
        source /opt/ros/humble/setup.bash
        source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
        exec "$@"
    ' ros-shell "$@"
}

ros_in_uav() {
    timeout 15s sudo -n ip netns exec uav1ns runuser -u multi_uav -- bash -lc '
        source /opt/ros/humble/setup.bash
        source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
        exec "$@"
    ' ros-shell "$@"
}

camera_topics="$(ros_in_uav ros2 topic list)" || {
    echo "ERROR: unable to query ROS topics in uav1ns." >&2
    exit 1
}
if ! grep -Fxq "/uav1/camera/image_raw" <<<"$camera_topics"; then
    echo "ERROR: required topic /uav1/camera/image_raw is unavailable in uav1ns." >&2
    echo "Visible topics in uav1ns:" >&2
    printf '%s\n' "$camera_topics" >&2
    exit 1
fi

gcs_topics="$(ros_in_gcs ros2 topic list)" || {
    echo "ERROR: unable to query ROS topics in gcsns." >&2
    exit 1
}
if ! grep -Fxq "/ap/v1/navsat" <<<"$gcs_topics"; then
    echo "ERROR: required topic /ap/v1/navsat is unavailable in gcsns." >&2
    echo "Visible topics in gcsns:" >&2
    printf '%s\n' "$gcs_topics" >&2
    exit 1
fi

existing_nodes="$(ps -eo pid=,comm=,args= | awk -v relay="$RELAY" -v detector="$DETECTOR" -v metrics="$METRICS" '
    $2 ~ /^python/ && (index($0, relay) || index($0, detector) || index($0, metrics))
')"
if [[ -n "$existing_nodes" ]]; then
    echo "ERROR: an experiment Python process is already running; stop it manually or use another session:" >&2
    echo "$existing_nodes" >&2
    exit 1
fi

echo "Confirm that the infrastructure pipeline was started with RNG_RUN=$RNG_RUN."
echo "Expected fixed hover:"
echo "latitude  = $TARGET_LATITUDE"
echo "longitude = $TARGET_LONGITUDE"
echo "altitude  = $TARGET_ALTITUDE_M m"
echo "The runner records GPS but does not control the UAV. Confirm visually that it is hovering at the fixed position with the same camera/yaw direction."

GPS_BEFORE="$(ros_in_gcs ros2 topic echo --once /ap/v1/navsat 2>&1)" || { echo "ERROR: could not capture pre-run GPS message." >&2; exit 1; }
ISO_START="$(date --iso-8601=seconds)"

DETECTOR_PGID=""
METRICS_PGID=""
RELAY_PGID=""
DETECTOR_PY_PID=""
METRICS_PY_PID=""
RELAY_PY_PID=""

stop_group() {
    local pgid="${1:-}"
    [[ -n "$pgid" ]] || return 0
    if sudo -n kill -0 -- "-$pgid" 2>/dev/null; then
        sudo -n kill -INT -- "-$pgid" 2>/dev/null || true
        for _ in {1..10}; do
            sudo -n kill -0 -- "-$pgid" 2>/dev/null || return 0
            sleep 0.5
        done
        sudo -n kill -TERM -- "-$pgid" 2>/dev/null || true
        sleep 1
        sudo -n kill -0 -- "-$pgid" 2>/dev/null && sudo -n kill -KILL -- "-$pgid" 2>/dev/null || true
    fi
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    stop_group "$RELAY_PGID"
    stop_group "$METRICS_PGID"
    stop_group "$DETECTOR_PGID"
    exit "$status"
}
trap cleanup EXIT INT TERM

find_actual_python_pid() {
    local exact_path="$1"
    ps -eo pid=,comm=,args= | awk -v path="$exact_path" '
        $2 ~ /^python/ && index($0, path) {print $1}
    '
}

wait_for_python_readiness() {
    local exact_path="$1" log="$2" marker="$3" timeout_s="$4" label="$5" launcher_pid="$6"
    local pid_var="$7" pgid_var="$8" candidate_pgid=""
    local deadline=$((SECONDS + timeout_s))
    local -a matches=()
    while (( SECONDS < deadline )); do
        mapfile -t matches < <(find_actual_python_pid "$exact_path")
        if (( ${#matches[@]} > 1 )); then
            echo "ERROR: expected exactly one $label Python process, found ${#matches[@]}." >&2
            printf '  PID %s\n' "${matches[@]}" >&2
            return 1
        fi
        if (( ${#matches[@]} == 1 )); then
            candidate_pgid="$(ps -o pgid= -p "${matches[0]}" | tr -d ' ')"
            printf -v "$pid_var" '%s' "${matches[0]}"
            printf -v "$pgid_var" '%s' "$candidate_pgid"
            if grep -qF "$marker" "$log" 2>/dev/null; then
                return 0
            fi
        fi
        if grep -Eq 'Model load failed|Traceback|ModuleNotFoundError|ImportError|No such file' "$log" 2>/dev/null; then
            echo "ERROR: $label reported a startup error:" >&2
            cat "$log" >&2
            return 1
        fi
        sleep 0.2
    done

    echo "ERROR: timed out waiting for $label readiness." >&2
    echo "--- complete $label log ---" >&2
    cat "$log" >&2 2>/dev/null || true
    echo "--- pgrep -af $(basename "$exact_path") ---" >&2
    pgrep -af "$(basename "$exact_path")" >&2 || true
    echo "--- launcher PID $launcher_pid status ---" >&2
    ps -fp "$launcher_pid" >&2 || true
    echo "--- matching actual-Python candidates ---" >&2
    mapfile -t matches < <(find_actual_python_pid "$exact_path")
    if (( ${#matches[@]} == 0 )); then
        echo "none" >&2
    else
        ps -fp "$(IFS=,; echo "${matches[*]}")" >&2 || true
    fi
    return 1
}

# Detector model loading has variable duration and must finish before frame flow starts.
DETECTOR_NS="uav1ns"
[[ "$MODE" == ground ]] && DETECTOR_NS="gcsns"
sudo -n setsid ip netns exec "$DETECTOR_NS" runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    source /home/multi_uav/yolo_env/bin/activate
    export PYTHONUNBUFFERED=1
    exec python3 "$1" --ros-args \
        -p uav_id:=1 -p processing_mode:="$2" \
        -p conf_threshold:=0.25 -p publish_debug:=false
' detector-shell "$DETECTOR" "$MODE" >"$DETECTOR_LOG" 2>&1 &
DETECTOR_LAUNCHER_PID=$!
wait_for_python_readiness "$DETECTOR" "$DETECTOR_LOG" "Model loaded OK" 120 "detector" "$DETECTOR_LAUNCHER_PID" DETECTOR_PY_PID DETECTOR_PGID
DETECTOR_PGID="$(ps -o pgid= -p "$DETECTOR_PY_PID" | tr -d ' ')"
[[ -n "$DETECTOR_PGID" ]] || { echo "ERROR: unable to determine detector process group." >&2; exit 1; }

# The metrics logger is passive and records no frame rows until detection results arrive.
sudo -n setsid ip netns exec gcsns runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    export PYTHONUNBUFFERED=1
    exec python3 "$1" --ros-args \
        -p uav_id:=1 -p processing_mode:="$2" -p run_id:="$3" -p csv_dir:="$4"
' metrics-shell "$METRICS" "$MODE" "$RUN_ID" "$RAW_DIR" >"$METRICS_LOG" 2>&1 &
METRICS_LAUNCHER_PID=$!
wait_for_python_readiness "$METRICS" "$METRICS_LOG" "CSV:" 20 "metrics logger" "$METRICS_LAUNCHER_PID" METRICS_PY_PID METRICS_PGID
METRICS_PGID="$(ps -o pgid= -p "$METRICS_PY_PID" | tr -d ' ')"
[[ -n "$METRICS_PGID" ]] || { echo "ERROR: unable to determine metrics logger process group." >&2; exit 1; }

mapfile -t csv_files < <(find "$RAW_DIR" -maxdepth 1 -type f -name "metrics_uav1_${MODE}_${RUN_ID}_*.csv" -print)
(( ${#csv_files[@]} == 1 )) || { echo "ERROR: expected exactly one metrics CSV for $RUN_ID." >&2; exit 1; }
CSV_PATH="${csv_files[0]}"
head -n 1 "$CSV_PATH" | grep -q 'detection_count' || { echo "ERROR: metrics CSV lacks detection_count." >&2; exit 1; }
head -n 1 "$CSV_PATH" | grep -q 'pipeline_latency_ms' || { echo "ERROR: metrics CSV lacks pipeline_latency_ms." >&2; exit 1; }

# Starting the relay last makes it the gate that begins image processing.
if [[ "$MODE" == edge ]]; then
    relay_parameters=(-p uav_id:=1 -p processing_mode:=edge -p frame_rate_hz:="$FRAME_RATE_HZ")
else
    relay_parameters=(-p uav_id:=1 -p processing_mode:=ground -p jpeg_quality:="$GROUND_JPEG_QUALITY" -p frame_rate_hz:="$FRAME_RATE_HZ")
fi
sudo -n setsid ip netns exec uav1ns runuser -u multi_uav -- bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
    export PYTHONUNBUFFERED=1
    exec python3 "$1" "${@:2}"
' relay-shell "$RELAY" --ros-args "${relay_parameters[@]}" >"$RELAY_LOG" 2>&1 &
RELAY_LAUNCHER_PID=$!
wait_for_python_readiness "$RELAY" "$RELAY_LOG" "frame sent" 20 "camera relay" "$RELAY_LAUNCHER_PID" RELAY_PY_PID RELAY_PGID
RELAY_PGID="$(ps -o pgid= -p "$RELAY_PY_PID" | tr -d ' ')"
[[ -n "$RELAY_PGID" ]] || { echo "ERROR: unable to determine camera relay process group." >&2; exit 1; }

# The first nine processed frames are warm-up frames, based on Phase A4.
# The official resource-monitoring window begins only after warm-up.
# The metrics CSV retains warm-up rows; later analysis will exclude the first nine rows.
while (( $(grep -cF "frame sent" "$RELAY_LOG" 2>/dev/null || true) < WARMUP_FRAMES )); do
    sudo -n kill -0 "$RELAY_PY_PID" 2>/dev/null || { echo "ERROR: camera relay exited during warm-up." >&2; exit 1; }
    sleep 0.2
done
ISO_WARMUP_END="$(date --iso-8601=seconds)"

ps -fp "$RELAY_PY_PID" "$DETECTOR_PY_PID"

pidstat -h -u -r -p "${RELAY_PY_PID},${DETECTOR_PY_PID}" 1 "$MEASUREMENT_DURATION" >"$RESOURCE_LOG"
ISO_MEASUREMENT_END="$(date --iso-8601=seconds)"

# Preserve the required shutdown order while leaving all infrastructure untouched.
stop_group "$RELAY_PGID"; RELAY_PGID=""
stop_group "$METRICS_PGID"; METRICS_PGID=""
stop_group "$DETECTOR_PGID"; DETECTOR_PGID=""

GPS_AFTER="$(ros_in_gcs ros2 topic echo --once /ap/v1/navsat 2>&1)" || { echo "ERROR: could not capture post-run GPS message." >&2; exit 1; }
SENT_FRAMES="$(grep -cF "frame sent" "$RELAY_LOG" || true)"

JPEG_VALUE="n/a"
[[ "$MODE" == ground ]] && JPEG_VALUE="$GROUND_JPEG_QUALITY"
cat >"$METADATA" <<EOF
Run ID: $RUN_ID
Mode: $MODE
Expected NS-3 RNG run: $RNG_RUN
ISO start time: $ISO_START
ISO warm-up-end time: $ISO_WARMUP_END
ISO measurement-end time: $ISO_MEASUREMENT_END
Measurement duration: $MEASUREMENT_DURATION seconds
Warm-up frames: $WARMUP_FRAMES
Frame rate: $FRAME_RATE_HZ Hz
Confidence threshold: $CONF_THRESHOLD
Ground JPEG quality: $JPEG_VALUE
Fixed target latitude: $TARGET_LATITUDE
Fixed target longitude: $TARGET_LONGITUDE
Fixed target altitude: $TARGET_ALTITUDE_M m relative
Detector namespace: $DETECTOR_NS
Relay namespace: uav1ns
Metrics namespace: gcsns
Detector log: $DETECTOR_LOG
Metrics log: $METRICS_LOG
Relay log: $RELAY_LOG
Metrics CSV: $CSV_PATH
Resource log: $RESOURCE_LOG
Metadata: $METADATA

GPS message before the run:
$GPS_BEFORE

GPS message after the run:
$GPS_AFTER
EOF

SUMMARY="$(python3 - "$CSV_PATH" "$WARMUP_FRAMES" "$SENT_FRAMES" "$MODE" <<'PY'
import csv
import math
import statistics
import sys

csv_path, warmup_text, sent_text, mode = sys.argv[1:]
warmup = int(warmup_text)
sent = int(sent_text)
with open(csv_path, newline='') as handle:
    rows = list(csv.DictReader(handle))
analysis = rows[warmup:]

def values(column):
    result = []
    for row in analysis:
        try:
            value = float(row[column])
        except (KeyError, TypeError, ValueError):
            continue
        if value >= 0 and math.isfinite(value):
            result.append(value)
    return result

def mean_text(items):
    return f'{statistics.fmean(items):.2f}' if items else 'n/a'

def percentile(items, fraction):
    if not items:
        return 'n/a'
    ordered = sorted(items)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    value = ordered[lower] if lower == upper else ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)
    return f'{value:.2f}'

detections = values('detection_count')
pipeline = values('pipeline_latency_ms')
inference = values('inference_ms')
compression = values('compression_ms')
decode = values('decode_ms')
ratio = len(rows) / sent if sent else float('nan')
ratio_label = 'complete processed-frame ratio' if mode == 'ground' else 'processed/sent ratio (local edge pipeline)'

print('Run summary')
print(f'  sent frames: {sent}')
print(f'  all CSV data rows: {len(rows)}')
print(f'  analysis rows after excluding first {warmup}: {len(analysis)}')
print(f'  {ratio_label}: {ratio:.4f}' if math.isfinite(ratio) else f'  {ratio_label}: n/a')
print(f'  mean detection_count: {mean_text(detections)}')
print(f'  minimum detection_count: {min(detections):.0f}' if detections else '  minimum detection_count: n/a')
print(f'  maximum detection_count: {max(detections):.0f}' if detections else '  maximum detection_count: n/a')
print(f'  mean pipeline_latency_ms after warm-up: {mean_text(pipeline)}')
print(f'  median pipeline_latency_ms after warm-up: {percentile(pipeline, 0.5)}')
print(f'  p95 pipeline_latency_ms after warm-up: {percentile(pipeline, 0.95)}')
print(f'  mean inference_ms after warm-up: {mean_text(inference)}')
if mode == 'ground':
    print(f'  mean compression_ms after warm-up: {mean_text(compression)}')
    print(f'  mean decode_ms after warm-up: {mean_text(decode)}')
print('Detection count is the number of YOLO detections in each processed frame. It is not precision or recall. Detection accuracy against manually labelled ground truth is reported separately in D4.')
PY
)"
printf '\n%s\n' "$SUMMARY" | tee -a "$METADATA"

cat <<'EOF'

Example matched sessions:
Session 1 (infrastructure RNG_RUN=31):
  bash results/scripts/run_edge_once.sh f_edge_01 31 60
  Keep hovering, wait around 10 seconds, then:
  bash results/scripts/run_ground_once.sh f_ground_q5_01 31 60
Session 2 (infrastructure RNG_RUN=32):
  bash results/scripts/run_ground_once.sh f_ground_q5_02 32 60
  bash results/scripts/run_edge_once.sh f_edge_02 32 60
Session 3 (infrastructure RNG_RUN=33):
  bash results/scripts/run_edge_once.sh f_edge_03 33 60
  bash results/scripts/run_ground_once.sh f_ground_q5_03 33 60
EOF
