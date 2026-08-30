#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT="/home/multi_uav/FYP/multi_uav_simulation"
MODE="${1:-}"
RUN_ID="${2:-}"
DURATION="${3:-60}"
RUN_USER="${SUDO_USER:-$USER}"
ROOT="$PROJECT/results_3uav/$MODE/$RUN_ID"
DETECTOR="$PROJECT/ros2/uav_vision/uav_vision/detector.py"
RELAY="$PROJECT/ros2/uav_vision/uav_vision/camera_relay.py"
METRICS="$PROJECT/ros2/uav_vision/uav_vision/metrics_logger.py"
READINESS_TIMEOUT=60
MODEL_TIMEOUT=180
SETTLE_SECONDS=5
DETECTOR_DDS_WARMUP_SECONDS=15
LOGGER_DDS_SETTLE_SECONDS=10
DISCOVERY_POLL_INTERVAL_SECONDS=2

usage() { echo "Usage: $0 <edge|ground> <run_01> [duration_seconds]" >&2; }
[[ "$MODE" == edge || "$MODE" == ground ]] || { usage; exit 2; }
[[ "$RUN_ID" =~ ^run_[0-9]+$ ]] || { usage; exit 2; }
[[ "$DURATION" =~ ^[1-9][0-9]*$ ]] || { echo "duration must be positive" >&2; exit 2; }
command -v pidstat >/dev/null || { echo "pidstat is required (sysstat)" >&2; exit 1; }
for file in "$DETECTOR" "$RELAY" "$METRICS"; do
    [[ -f "$file" ]] || { echo "missing $file" >&2; exit 1; }
done

sudo -v
sudo -n true || { echo "ERROR: sudo authentication is unavailable" >&2; exit 1; }
for namespace in gcsns uav1ns uav2ns uav3ns; do
    sudo -n ip netns list | awk '{print $1}' | grep -Fxq "$namespace" || {
        echo "ERROR: namespace $namespace is unavailable" >&2
        exit 1
    }
done

[[ ! -e "$ROOT" ]] || { echo "ERROR: refusing to overwrite $ROOT" >&2; exit 1; }
mkdir -p "$ROOT/logs" "$ROOT/.pids"
READINESS_LOG="$ROOT/readiness.log"
: >"$READINESS_LOG"

declare -A DETECTOR_PIDS=() METRICS_PIDS=() RELAY_PIDS=()
declare -a WRAPPER_PIDS=()
SHUTDOWN_COMPLETE=0

pid_alive() {
    local pid="${1:-}"
    [[ -n "$pid" ]] && sudo -n kill -0 "$pid" 2>/dev/null
}

stop_process_set() {
    local array_name="$1" label="$2"
    local -n process_pids="$array_name"
    local uav pid deadline any_alive
    for uav in 1 2 3; do
        pid="${process_pids[$uav]:-}"
        pid_alive "$pid" && sudo -n kill -INT "$pid" 2>/dev/null || true
    done
    deadline=$((SECONDS + 10))
    while (( SECONDS < deadline )); do
        any_alive=0
        for uav in 1 2 3; do
            pid="${process_pids[$uav]:-}"
            pid_alive "$pid" && any_alive=1
        done
        (( any_alive == 0 )) && return 0
        sleep 0.2
    done
    for uav in 1 2 3; do
        pid="${process_pids[$uav]:-}"
        if pid_alive "$pid"; then
            echo "WARNING: $label UAV$uav did not stop after SIGINT; sending SIGTERM" >&2
            sudo -n kill -TERM "$pid" 2>/dev/null || true
        fi
    done
    deadline=$((SECONDS + 5))
    while (( SECONDS < deadline )); do
        any_alive=0
        for uav in 1 2 3; do
            pid="${process_pids[$uav]:-}"
            pid_alive "$pid" && any_alive=1
        done
        (( any_alive == 0 )) && return 0
        sleep 0.2
    done
    echo "WARNING: one or more $label processes did not terminate" >&2
    return 1
}

shutdown_pipeline() {
    (( SHUTDOWN_COMPLETE == 0 )) || return 0
    stop_process_set RELAY_PIDS relay || true
    stop_process_set DETECTOR_PIDS detector || true
    stop_process_set METRICS_PIDS "metrics logger" || true
    for wrapper_pid in "${WRAPPER_PIDS[@]:-}"; do
        wait "$wrapper_pid" 2>/dev/null || true
    done
    SHUTDOWN_COMPLETE=1
}

cleanup() {
    local status=$?
    trap - EXIT INT TERM
    shutdown_pipeline
    exit "$status"
}
trap cleanup EXIT INT TERM

launch_python() {
    local namespace="$1" role="$2" uav="$3" log="$4"; shift 4
    local pid_file="$ROOT/.pids/${role}_uav${uav}.pid"
    # Match the established single-UAV launch pattern: create a runner-owned
    # session before entering the namespace, while the inner exec/PID file
    # still identifies the actual Python process.
    sudo -n setsid ip netns exec "$namespace" runuser -u "$RUN_USER" -- bash -lc '
        source /opt/ros/humble/setup.bash
        source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
        source /home/multi_uav/yolo_env/bin/activate
        export PYTHONUNBUFFERED=1
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        printf "%s\n" "$$" > "$1"
        shift
        exec python3 "$@"
    ' runner-shell "$pid_file" "$@" >"$log" 2>&1 &
    WRAPPER_PIDS+=("$!")

    local deadline=$((SECONDS + 10)) pid=""
    while (( SECONDS < deadline )); do
        [[ -s "$pid_file" ]] && pid="$(<"$pid_file")"
        if [[ "$pid" =~ ^[1-9][0-9]*$ ]] && pid_alive "$pid"; then
            case "$role" in
                detector) DETECTOR_PIDS[$uav]="$pid" ;;
                metrics) METRICS_PIDS[$uav]="$pid" ;;
                relay) RELAY_PIDS[$uav]="$pid" ;;
            esac
            return 0
        fi
        sleep 0.1
    done
    echo "ERROR: UAV$uav $role failed to expose a live Python PID; log: $log" >&2
    [[ -f "$log" ]] && tail -40 "$log" >&2
    return 1
}

wait_for_log_marker() {
    local pid="$1" log="$2" marker="$3" timeout="$4" label="$5"
    local deadline=$((SECONDS + timeout))
    while (( SECONDS < deadline )); do
        grep -qF "$marker" "$log" 2>/dev/null && return 0
        if ! pid_alive "$pid"; then
            echo "ERROR: $label exited during startup" >&2
            tail -80 "$log" >&2 2>/dev/null || true
            return 1
        fi
        if grep -Eq 'Traceback|Model load failed|ModuleNotFoundError|ImportError' "$log" 2>/dev/null; then
            echo "ERROR: $label reported a startup failure" >&2
            tail -80 "$log" >&2
            return 1
        fi
        sleep 0.2
    done
    echo "ERROR: timed out waiting for $label: $marker" >&2
    tail -80 "$log" >&2 2>/dev/null || true
    return 1
}

ros_in_namespace() {
    local namespace="$1"; shift
    timeout 10s sudo -n ip netns exec "$namespace" runuser -u "$RUN_USER" -- bash -lc '
        source /opt/ros/humble/setup.bash
        source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        exec "$@"
    ' ros-shell "$@"
}

require_all_alive() {
    local stage="$1" uav
    for uav in 1 2 3; do
        pid_alive "${DETECTOR_PIDS[$uav]:-}" || {
            echo "ERROR: UAV$uav detector exited during $stage" >&2
            tail -80 "$ROOT/logs/detector_uav${uav}.log" >&2 2>/dev/null || true
            return 1
        }
        pid_alive "${METRICS_PIDS[$uav]:-}" || {
            echo "ERROR: UAV$uav metrics logger exited during $stage" >&2
            tail -80 "$ROOT/logs/metrics_uav${uav}.log" >&2 2>/dev/null || true
            return 1
        }
        if [[ "$stage" != detection_discovery ]]; then
            pid_alive "${RELAY_PIDS[$uav]:-}" || {
                echo "ERROR: UAV$uav relay exited during $stage" >&2
                tail -80 "$ROOT/logs/relay_uav${uav}.log" >&2 2>/dev/null || true
                return 1
            }
        fi
    done
}

wait_fixed_startup_period() {
    local duration="$1" stage="$2" include_metrics="$3"
    local deadline=$((SECONDS + duration)) uav elapsed
    while (( SECONDS < deadline )); do
        for uav in 1 2 3; do
            pid_alive "${DETECTOR_PIDS[$uav]:-}" || {
                echo "ERROR: UAV$uav detector exited during $stage" >&2
                tail -80 "$ROOT/logs/detector_uav${uav}.log" >&2 2>/dev/null || true
                return 1
            }
            if (( include_metrics )); then
                pid_alive "${METRICS_PIDS[$uav]:-}" || {
                    echo "ERROR: UAV$uav metrics logger exited during $stage" >&2
                    tail -80 "$ROOT/logs/metrics_uav${uav}.log" >&2 2>/dev/null || true
                    return 1
                }
            fi
        done
        elapsed=$((duration - (deadline - SECONDS)))
        (( elapsed < 0 )) && elapsed=0
        echo "$stage: ${elapsed}/${duration}s"
        sleep 1
    done
}

wait_for_endpoint_pairs() {
    local kind="$1" deadline=$((SECONDS + READINESS_TIMEOUT))
    local all_ready uav topic namespace info publisher_count subscriber_count state remaining
    while (( SECONDS < deadline )); do
        require_all_alive "$kind" || return 1
        all_ready=1
        [[ "$kind" == detection_discovery ]] && echo "DDS discovery:"
        for uav in 1 2 3; do
            if [[ "$kind" == detection_discovery ]]; then
                topic="/detections/uav${uav}"; namespace=gcsns
            elif [[ "$MODE" == edge ]]; then
                topic="/cluster/cam/uav${uav}"; namespace="uav${uav}ns"
            else
                topic="/relay/uav${uav}/compressed"; namespace=gcsns
            fi
            # Bypass the ROS 2 daemon so readiness reflects a fresh participant
            # discovering the live endpoints in this namespace.
            info="$(ros_in_namespace "$namespace" ros2 topic info \
                --no-daemon --spin-time 3 -v "$topic" 2>&1)" || info=""
            publisher_count="$(awk '/^Publisher count:/ {print $3; exit}' <<<"$info")"
            subscriber_count="$(awk '/^Subscription count:/ {print $3; exit}' <<<"$info")"
            publisher_count="${publisher_count:-0}"
            subscriber_count="${subscriber_count:-0}"
            if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$info" ||
               ! grep -Eq 'Subscription count: [1-9][0-9]*' <<<"$info"; then
                all_ready=0
                state=WAITING
            else
                state=READY
            fi
            if [[ "$kind" == detection_discovery ]]; then
                printf 'UAV%s publisher=%s subscriber=%s %s\n' \
                    "$uav" "$publisher_count" "$subscriber_count" "$state" | \
                    tee -a "$READINESS_LOG"
            fi
        done
        (( all_ready == 1 )) && return 0
        remaining=$((deadline - SECONDS))
        (( remaining > 0 )) && echo "Waiting for all endpoints; up to ${remaining}s remain."
        sleep "$DISCOVERY_POLL_INTERVAL_SECONDS"
    done
    echo "ERROR: timed out waiting for $kind endpoint matching" >&2
    for uav in 1 2 3; do
        if [[ "$kind" == detection_discovery ]]; then
            topic="/detections/uav${uav}"; namespace=gcsns
        elif [[ "$MODE" == edge ]]; then
            topic="/cluster/cam/uav${uav}"; namespace="uav${uav}ns"
        else
            topic="/relay/uav${uav}/compressed"; namespace=gcsns
        fi
        echo "--- UAV$uav $topic ---" >&2
        ros_in_namespace "$namespace" ros2 topic info \
            --no-daemon --spin-time 3 -v "$topic" >&2 || true
    done
    return 1
}

echo "Starting all detectors..."
for uav in 1 2 3; do
    detector_namespace="uav${uav}ns"
    [[ "$MODE" == ground ]] && detector_namespace=gcsns
    launch_python "$detector_namespace" detector "$uav" "$ROOT/logs/detector_uav${uav}.log" \
        "$DETECTOR" --ros-args -r __node:="detector_uav${uav}" \
        -p uav_id:="$uav" -p processing_mode:="$MODE" \
        -p conf_threshold:=0.25 -p publish_debug:=false
done
for uav in 1 2 3; do
    wait_for_log_marker "${DETECTOR_PIDS[$uav]}" "$ROOT/logs/detector_uav${uav}.log" \
        "Model loaded OK" "$MODEL_TIMEOUT" "UAV$uav detector"
    wait_for_log_marker "${DETECTOR_PIDS[$uav]}" "$ROOT/logs/detector_uav${uav}.log" \
        "Publishing detections -> /detections/uav${uav}" 20 \
        "UAV$uav detection publisher"
done

echo "All detector publishers created; DDS discovery warmup: ${DETECTOR_DDS_WARMUP_SECONDS}s"
wait_fixed_startup_period "$DETECTOR_DDS_WARMUP_SECONDS" \
    "Detector DDS warmup" 0

echo "Starting all metrics loggers..."
for uav in 1 2 3; do
    launch_python gcsns metrics "$uav" "$ROOT/logs/metrics_uav${uav}.log" \
        "$METRICS" --ros-args -r __node:="metrics_logger_uav${uav}" \
        -p uav_id:="$uav" -p processing_mode:="$MODE" \
        -p run_id:="${MODE}_${RUN_ID}" -p csv_dir:="$ROOT"
done
for uav in 1 2 3; do
    wait_for_log_marker "${METRICS_PIDS[$uav]}" "$ROOT/logs/metrics_uav${uav}.log" \
        "CSV:" 20 "UAV$uav metrics logger"
done

echo "All metrics loggers created; DDS settling: ${LOGGER_DDS_SETTLE_SECONDS}s"
wait_fixed_startup_period "$LOGGER_DDS_SETTLE_SECONDS" \
    "Detector/logger DDS settling" 1

echo "Waiting for all detection-result endpoints..."
wait_for_endpoint_pairs detection_discovery
for uav in 1 2 3; do
    echo "UAV$uav detections: publisher matched subscriber" | tee -a "$READINESS_LOG"
done

echo "Starting all camera relays..."
for uav in 1 2 3; do
    relay_args=(-p uav_id:="$uav" -p processing_mode:="$MODE" -p frame_rate_hz:=1.0)
    [[ "$MODE" == ground ]] && relay_args+=(-p jpeg_quality:=5)
    launch_python "uav${uav}ns" relay "$uav" "$ROOT/logs/relay_uav${uav}.log" \
        "$RELAY" --ros-args -r __node:="camera_relay_uav${uav}" "${relay_args[@]}"
done
for uav in 1 2 3; do
    wait_for_log_marker "${RELAY_PIDS[$uav]}" "$ROOT/logs/relay_uav${uav}.log" \
        "mode:" 20 "UAV$uav camera relay"
done

echo "Waiting for all relay-to-detector endpoints..."
wait_for_endpoint_pairs relay_discovery
for uav in 1 2 3; do
    echo "UAV$uav camera relay: publisher matched detector subscriber" | tee -a "$READINESS_LOG"
done

echo "All endpoints ready; settling for ${SETTLE_SECONDS}s"
settle_deadline=$((SECONDS + SETTLE_SECONDS))
while (( SECONDS < settle_deadline )); do
    require_all_alive settling
    sleep 0.2
done

printf 'mode=%s\nrun=%s\nduration_seconds=%s\nYOLO=yolov8n\nimgsz=960\nconfidence=0.25\nclass=person\n' \
    "$MODE" "$RUN_ID" "$DURATION" >"$ROOT/notes.txt"
[[ "$MODE" == ground ]] && echo 'jpeg_quality=5' >>"$ROOT/notes.txt" || echo 'jpeg_quality=n/a' >>"$ROOT/notes.txt"
printf 'official_start=%s\n' "$(date --iso-8601=seconds)" >>"$ROOT/notes.txt"

for uav in 1 2 3; do
    pid_alive "${RELAY_PIDS[$uav]}" && pid_alive "${DETECTOR_PIDS[$uav]}" || {
        echo "ERROR: UAV$uav pipeline not alive at official start" >&2
        exit 1
    }
done
pid_list="$(IFS=,; echo "${RELAY_PIDS[*]},${DETECTOR_PIDS[*]}")"
echo "Official ${DURATION}s window started; pidstat Python PIDs: $pid_list"
pidstat -h -u -r -p "$pid_list" 1 "$DURATION" >"$ROOT/system.csv" &
PIDSTAT_PID=$!
while pid_alive "$PIDSTAT_PID"; do
    require_all_alive measurement
    sleep 0.2
done
wait "$PIDSTAT_PID"
printf 'official_end=%s\n' "$(date --iso-8601=seconds)" >>"$ROOT/notes.txt"

echo "Official window complete; stopping relays, detectors, then metrics loggers."
stop_process_set RELAY_PIDS relay
stop_process_set DETECTOR_PIDS detector
stop_process_set METRICS_PIDS "metrics logger"
for wrapper_pid in "${WRAPPER_PIDS[@]}"; do wait "$wrapper_pid" 2>/dev/null || true; done
SHUTDOWN_COMPLETE=1

for uav in 1 2 3; do
    mapfile -t files < <(find "$ROOT" -maxdepth 1 -type f -name "metrics_uav${uav}_*.csv" -print)
    (( ${#files[@]} == 1 )) || {
        echo "ERROR: expected one flushed metrics CSV for UAV$uav, found ${#files[@]}" >&2
        exit 1
    }
    data_rows=$(( $(wc -l <"${files[0]}") - 1 ))
    (( data_rows > 0 )) || { echo "ERROR: UAV$uav metrics CSV has no data rows" >&2; exit 1; }
    cp -- "${files[0]}" "$ROOT/detector_uav${uav}.csv"
done
rm -f "$ROOT/.pids/"*.pid
rmdir "$ROOT/.pids"
echo "PASS: $MODE $RUN_ID results saved in $ROOT"
