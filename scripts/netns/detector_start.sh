#!/bin/bash
# Step 2 of 4: start the edge detectors on the Pi boards and their receivers.
#
#   ./scripts/netns/rpi_init.sh                 # 0 - verify both boards
#   ./scripts/netns/sitl_init.sh --gui --view   # 1 - host pipeline
#   ./scripts/netns/detector_start.sh           # 2 - this script
#   ./scripts/netns/run_missions.sh             # 3 - fly
#
# Ctrl+C stops the remote detectors as well as the local receivers.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"

declare -A PI_HOST=( [1]="anton@10.0.0.2"  [2]="anton@10.0.1.2" )

DDS_PROFILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml"
PI_VENV_PY='$HOME/yolo_env/bin/python'
PI_DETECTOR='$HOME/uav2_ws/install/uav_vision/lib/uav_vision/detector'

MODEL="${MODEL:-/home/anton/models/yolo11n_openvino_model}"
CONF="${CONF:-0.4}"
IMGSZ="${IMGSZ:-640}"
DEBUG="${DEBUG:-false}"
BOARDS="${BOARDS:-1 2}"

DETECTOR_READY_SEC=90     # OpenVINO takes ~12 s to load on a Pi 4B
RECEIVER_SETTLE_SEC=3
PIDS=()

say() { printf '\r%s\n' "$*"; }

# Kill by PATTERN, not by $!. gcs_receiver runs under
# `sudo ip netns exec ... sudo ... bash -lc`, so $! is the OUTER sudo and
# killing it leaves the python child running and holding the topic.
CLEANED=0
cleanup() {
    (( CLEANED )) && return 0
    CLEANED=1
    say ""
    say "=== Shutting down ==="

    # Remote first: a detector outlives this script and blocks the next run.
    for i in $BOARDS; do
        # [u]av_vision stops pkill matching its own ssh command line.
        ssh -o ConnectTimeout=4 "${PI_HOST[$i]}" 'pkill -9 -f "[u]av_vision/detector"' \
            >/dev/null 2>&1 && say "  detector stopped on ${PI_HOST[$i]#*@}" \
            || say "  detector on ${PI_HOST[$i]#*@} already stopped"
    done

    # Local: TERM the whole tree, then KILL whatever ignored it.
    for pid in "${PIDS[@]:-}"; do kill -TERM "$pid" 2>/dev/null || true; done
    sudo pkill -TERM -f -- '[g]cs_receiver' 2>/dev/null || true
    sleep 2
    for pid in "${PIDS[@]:-}"; do kill -KILL "$pid" 2>/dev/null || true; done
    sudo pkill -9 -f -- '[g]cs_receiver' 2>/dev/null || true

    pgrep -f -- '[g]cs_receiver' >/dev/null 2>&1 \
        && say "  WARNING: a gcs_receiver survived — check: pgrep -af gcs_receiver" \
        || say "  receivers stopped"
    say ""
}

# EXIT too: without it a normal or errored exit leaves everything running.
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# ── Preconditions ────────────────────────────────────────────────────────────
# The receivers run inside gcsns and subscribe to topics the bridges publish,
# so the host pipeline must already be up.
say "=== Preconditions ==="
ip netns list 2>/dev/null | grep -q gcsns \
    || { echo "ERROR: gcsns is missing. Run sitl_init.sh first." >&2; exit 1; }
say "  ok: gcsns exists"

[[ -f "$DDS_PROFILE" ]] \
    || { echo "ERROR: DDS profile missing: $DDS_PROFILE" >&2; exit 1; }
say "  ok: DDS profile present"

[[ -f "$PROJECT_DIR/install/setup.bash" ]] \
    || { echo "ERROR: uav_vision not built. Run: colcon build --packages-select uav_vision" >&2; exit 1; }
say "  ok: uav_vision workspace built"

for i in $BOARDS; do
    ssh -o ConnectTimeout=4 -o BatchMode=yes "${PI_HOST[$i]}" true 2>/dev/null \
        || { echo "ERROR: board $i unreachable. Run rpi_init.sh." >&2; exit 1; }
    say "  ok: board $i reachable"
done
say ""

sudo -v

# ── Detectors on the boards ──────────────────────────────────────────────────
say "=== Edge detectors ==="
say "  model=$(basename "$MODEL")  conf=$CONF  imgsz=$IMGSZ  debug=$DEBUG"
for i in $BOARDS; do
    log="/tmp/detector_uav$i.log"
    : > "$log"
    # -n, NOT -tt: a pty puts the local terminal in raw mode, so Ctrl+C becomes
    # a byte sent to the Pi and the trap below never fires. cleanup() kills the
    # remote detector explicitly instead, which is what -tt was there for.
    ssh -n "${PI_HOST[$i]}" "
        export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/uav2_ws/config/fastdds_hitl_eth.xml
        source /opt/ros/humble/setup.bash
        source \$HOME/uav2_ws/install/setup.bash
        exec $PI_VENV_PY $PI_DETECTOR --ros-args \
            -p uav_id:=$i -p processing_mode:=edge \
            -p model_path:=$MODEL -p conf_threshold:=$CONF \
            -p imgsz:=$IMGSZ -p publish_debug:=$DEBUG
    " > "$log" 2>&1 &
    PIDS+=("$!")
done

# Poll for the model-loaded banner: a fixed sleep reported a healthy detector
# as broken, because load time varies with the back-end.
deadline=$((SECONDS + DETECTOR_READY_SEC))
for i in $BOARDS; do
    log="/tmp/detector_uav$i.log"
    while ! grep -q 'Model loaded OK' "$log" 2>/dev/null; do
        (( SECONDS >= deadline )) && break
        sleep 2
    done
    if grep -q 'Model loaded OK' "$log" 2>/dev/null; then
        say "  ok: board $i detector loaded ($log)"
    else
        echo "  WARNING: board $i never reported 'Model loaded OK' — see $log" >&2
    fi
done
say ""

# ── Receivers inside gcsns ───────────────────────────────────────────────────
# gcsns is deliberate: from there the only route back to the board is
# gcsns -> ns-3 -> Pi, so every detection crosses the simulated radio. In the
# root namespace it would arrive over the unimpaired camera cable instead.
say "=== gcs_receivers (inside gcsns) ==="
for i in $BOARDS; do
    log="/tmp/gcs_receiver_uav$i.log"
    : > "$log"
    # uav_vision lives in <repo>/install, not <repo>/ros2/install.
    sudo ip netns exec gcsns sudo -H -u "$RUN_USER" \
        env FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
        bash -lc "source /opt/ros/humble/setup.bash && \
                  source '$PROJECT_DIR/install/setup.bash' && \
                  exec ros2 run uav_vision gcs_receiver --ros-args \
                       -p uav_id:=$i -p processing_mode:=edge" \
        > "$log" 2>&1 &
    PIDS+=("$!")
done

sleep "$RECEIVER_SETTLE_SEC"
for i in $BOARDS; do
    log="/tmp/gcs_receiver_uav$i.log"
    if grep -qi "not found\|Traceback" "$log" 2>/dev/null; then
        echo "  WARNING: board $i gcs_receiver failed — see $log" >&2
        sed 's/^/    /' "$log" >&2
    else
        say "  ok: board $i gcs_receiver running ($log)"
    fi
done
say ""

say "════════════════════════════════════════════════════════════"
say " DETECTORS READY"
say "════════════════════════════════════════════════════════════"
say ""
say "  Fly in another terminal:  ./scripts/netns/run_missions.sh"
say ""
say "  Logs   detectors: /tmp/detector_uav1.log  /tmp/detector_uav2.log"
say "         receivers: /tmp/gcs_receiver_uav1.log  /tmp/gcs_receiver_uav2.log"
say ""
say "  Ctrl+C here stops the detectors on both boards."
say ""

# `|| true` so a detector exiting non-zero does not skip past the EXIT trap.
wait || true
