#!/bin/bash
# run_hitl.sh — one-command HITL run: host pipeline + Pi edge detector + GCS.
#
# Wraps scripts/netns/launch_single_uav_netns.sh (which brings up netns, ns-3,
# Gazebo, SITL, micro_ros_agent and drone_bridge on THIS machine) and then
# starts the two things that script deliberately leaves manual:
#
#   * detector    on the Raspberry Pi 4B, over SSH  (the "edge" node)
#   * gcs_receiver on this host                     (receives the detections)
#
# Optionally flies the patrol mission too (--mission) and records the camera
# to a rosbag (--record) so the detector can later be tuned by replaying the
# bag instead of re-flying — see --help.
#
# Ctrl+C tears everything down, including the remote detector.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# This script lives in scripts/netns/, so the repo root is two levels up.
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# ── Configuration ────────────────────────────────────────────────────────────
PI_HOST="anton@10.0.0.2"
HOST_LINK_IP="10.0.0.1"           # must exist BEFORE Gazebo starts (see below)
PI_VENV_PY="\$HOME/yolo_env/bin/python"
PI_DETECTOR="\$HOME/uav2_ws/install/uav_vision/lib/uav_vision/detector"
# Medians measured on the Pi 4B by scripts/bench_backends.py — see
# docs/HITL_INTEGRATION_PLAN.md §10. The default is the fastest CORRECT back-end.
PI_MODEL_OPENVINO="/home/anton/models/yolo11n_openvino_model"       # 236 ms
PI_MODEL_NCNN="/home/anton/models/yolov8n_384x640_ncnn_model"       # 270 ms
PI_MODEL_PT="/home/anton/models/yolov8n.pt"                         # 1027 ms

DDS_PROFILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml"
PIPELINE="$PROJECT_DIR/scripts/netns/launch_single_uav_netns.sh"
MISSION="$PROJECT_DIR/ros2/uav_controller/uav_controller/uav1_patrol_mission.py"

PIPELINE_LOG="/tmp/hitl_pipeline.log"
GCS_LOG="/tmp/hitl_gcs.log"
DETECTOR_LOG="/tmp/hitl_detector.log"
MISSION_LOG="/tmp/hitl_mission.log"
BAG_DIR="$HOME/hitl_bags/cam_$(date +%Y%m%d_%H%M%S)"

# ── Options ──────────────────────────────────────────────────────────────────
MODEL="$PI_MODEL_OPENVINO"
CONF="0.4"
# Applies to the PyTorch model ONLY. The detector drops imgsz for directory
# models (NCNN, OpenVINO), whose input shape is frozen at export time.
IMGSZ="640"
DEBUG="False"
RUN_MISSION=0
RECORD=0
WITH_PI=1

usage() {
    cat <<'USAGE'
Usage: run_hitl.sh [options]

  --ncnn            use the NCNN model on the Pi       (270 ms)
  --pt              use the PyTorch model on the Pi    (1027 ms)
                    default is OpenVINO                (236 ms — fastest correct)
  --conf <v>        detector confidence threshold      (default 0.4)
  --imgsz <n>       detector inference size            (default 640)
                    PyTorch only — NCNN and OpenVINO exports have a fixed input
                    shape and the detector drops imgsz for them
  --debug           detector saves annotated JPEGs to /tmp/yolo_frames on the Pi
                    (costs CPU + an SD write per detection — not for timing runs)
  --mission         fly uav1_patrol_mission.py once the stack is up
  --record          record /uav1/camera/image_raw to a rosbag, so the detector
                    can later be tuned with `ros2 bag play` instead of re-flying
  --no-pi           host only; skip the remote detector (e.g. Pi unplugged)
  -h, --help        this text

Examples:
  ./scripts/run_hitl.sh --mission --record
  ./scripts/run_hitl.sh --pt --conf 0.25          # baseline comparison run
  ./scripts/run_hitl.sh --no-pi                   # host pipeline only
USAGE
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --pt)       MODEL="$PI_MODEL_PT"; shift ;;
        --ncnn)     MODEL="$PI_MODEL_NCNN"; shift ;;
        --conf)     CONF="$2"; shift 2 ;;
        --imgsz)    IMGSZ="$2"; shift 2 ;;
        --debug)    DEBUG="True"; shift ;;
        --mission)  RUN_MISSION=1; shift ;;
        --record)   RECORD=1; shift ;;
        --no-pi)    WITH_PI=0; shift ;;
        -h|--help)  usage; exit 0 ;;
        *)          echo "unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

PIDS=()
cleanup() {
    echo ""
    echo "=== Shutting down ==="
    # Stop the remote detector first — it is the only thing not on this machine
    # and would otherwise keep running after the local processes are gone.
    if (( WITH_PI )); then
        ssh -o ConnectTimeout=4 "$PI_HOST" \
            'pkill -f uav_vision/detector' 2>/dev/null \
            && echo "  Pi detector stopped" || true
    fi
    for pid in "${PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 1
    bash "$PROJECT_DIR/scripts/netns/kill_all_netns.sh" >/dev/null 2>&1 || true
    echo "=== Done ==="
    exit 0
}
trap cleanup INT TERM

# ═══════════════════════════════════════════════════════════════════════════
# Pre-flight — every one of these has caused a silent failure before
# ═══════════════════════════════════════════════════════════════════════════
echo "=== [1/6] Pre-flight checks ==="

[[ -f "$DDS_PROFILE" ]] || { echo "ERROR: DDS profile missing: $DDS_PROFILE" >&2; exit 1; }
[[ -x "$PIPELINE" || -f "$PIPELINE" ]] || { echo "ERROR: pipeline script missing: $PIPELINE" >&2; exit 1; }

# FastDDS reads the interface list ONCE, at participant creation, and Humble's
# FastDDS 2.6 has no dynamic interface detection. If 10.0.0.1 does not exist
# yet, Gazebo can never bind it and the Pi will never see the camera — and the
# failure is silent, so check up front.
if ! ip -4 addr show | grep -q "${HOST_LINK_IP}/"; then
    echo "ERROR: $HOST_LINK_IP is not assigned to any interface." >&2
    echo "       Plug in the USB ethernet adapter, then:" >&2
    echo "         nmcli connection show          # find the wired profile name" >&2
    echo "         sudo nmcli connection modify '<name>' ipv4.method manual \\" >&2
    echo "              ipv4.addresses $HOST_LINK_IP/24 ipv4.never-default yes" >&2
    echo "         sudo nmcli connection up '<name>'" >&2
    exit 1
fi
echo "  $HOST_LINK_IP present"

if (( WITH_PI )); then
    if ! ping -c 1 -W 2 "${PI_HOST#*@}" >/dev/null 2>&1; then
        echo "ERROR: Pi at ${PI_HOST#*@} is not reachable. Use --no-pi to skip it." >&2
        exit 1
    fi
    echo "  Pi reachable at ${PI_HOST#*@}"

    # A missing model only surfaces as a WARNING five seconds later, by which
    # point the whole stack is already up. Check it before paying for that.
    if ! ssh -o ConnectTimeout=4 "$PI_HOST" "test -e '$MODEL'" 2>/dev/null; then
        echo "ERROR: model not found on the Pi: $MODEL" >&2
        echo "       available:" >&2
        ssh -o ConnectTimeout=4 "$PI_HOST" 'ls -d ~/models/* 2>/dev/null' >&2 || true
        exit 1
    fi
    echo "  model present: $(basename "$MODEL")"

    # A 1280x720 RGB frame is 2.76 MB = ~2000 UDP fragments. The default 208 KB
    # socket buffer holds ~7% of one frame.
    #
    # BOTH directions must be checked. rmem alone is not enough: with wmem_max
    # left at the default, the DDS profile's 16 MB <sendBufferSize> is silently
    # clamped to 208 KB and the PUBLISHER discards fragments before they reach
    # the NIC. That failure is invisible to every interface counter — tx_dropped
    # stays 0 on the host and rx_errors stays 0 on the Pi, because nothing was
    # ever transmitted. Measured symptom: camera delivered at 0.19 Hz instead of
    # 5 Hz, with the host sending 2 Mbps in place of 110 Mbps.
    for host_desc in "local:" "remote:"; do
        for knob in rmem_max wmem_max; do
            if [[ $host_desc == local: ]]; then
                val=$(sysctl -n "net.core.$knob")
            else
                val=$(ssh -o ConnectTimeout=4 "$PI_HOST" "sysctl -n net.core.$knob" 2>/dev/null || echo 0)
            fi
            if (( val < 100000000 )); then
                echo "  WARNING ${host_desc%%:*} net.core.$knob=$val is too small for 2.76 MB frames." >&2
                echo "          sudo tee /etc/sysctl.d/60-ros2-dds.conf <<'EOF'" >&2
                echo "          net.core.rmem_max = 536870912" >&2
                echo "          net.core.rmem_default = 134217728" >&2
                echo "          net.core.wmem_max = 536870912" >&2
                echo "          net.core.wmem_default = 134217728" >&2
                echo "          net.ipv4.ipfrag_high_thresh = 134217728" >&2
                echo "          EOF" >&2
                echo "          sudo sysctl --system   # then RESTART the stack:" >&2
                echo "          FastDDS reads socket options once, at participant creation." >&2
            fi
        done
    done
fi

# Warm the sudo cache now so the pipeline does not stall on a password prompt
# after it has already been backgrounded.
sudo -v
echo ""

# ═══════════════════════════════════════════════════════════════════════════
echo "=== [2/6] Host pipeline (netns, ns-3, Gazebo, SITL, agent, bridge) ==="
: > "$PIPELINE_LOG"
bash "$PIPELINE" > "$PIPELINE_LOG" 2>&1 &
PIPELINE_PID=$!
PIDS+=("$PIPELINE_PID")
echo "  started (pid $PIPELINE_PID), log: $PIPELINE_LOG"
echo "  waiting for PIPELINE READY..."

deadline=$((SECONDS + 300))
while ! grep -q "PIPELINE READY" "$PIPELINE_LOG" 2>/dev/null; do
    if ! kill -0 "$PIPELINE_PID" 2>/dev/null; then
        echo "ERROR: pipeline exited early. Last 30 lines:" >&2
        tail -30 "$PIPELINE_LOG" >&2
        exit 1
    fi
    (( SECONDS >= deadline )) && {
        echo "ERROR: pipeline did not report ready within 300s. Last 30 lines:" >&2
        tail -30 "$PIPELINE_LOG" >&2
        exit 1
    }
    sleep 2
done
echo "  pipeline ready"

# Surface any stage warning. The pipeline's health gates print these, but this
# script only ever greps the log for PIPELINE READY, so a stage that degraded
# without failing was completely silent here. That is how a dead position
# publisher went unnoticed while every ns-3 node sat frozen for a whole run.
if grep -q "WARNING" "$PIPELINE_LOG" 2>/dev/null; then
    echo "  --- pipeline warnings ---" >&2
    grep -A2 "WARNING" "$PIPELINE_LOG" | sed 's/^/    /' >&2
    echo "  -------------------------" >&2
fi

# Confirm Gazebo actually bound the wired link. If it did not, nothing reaches
# the Pi and every downstream symptom looks like a broken detector instead.
GZ_PID=$(pgrep -f gzserver | head -1 || true)
if [[ -n "$GZ_PID" ]] && ! ss -ulnp 2>/dev/null | grep "pid=$GZ_PID" | grep -q "$HOST_LINK_IP:"; then
    echo "  WARNING: gzserver is NOT bound to $HOST_LINK_IP — the Pi will not see the camera." >&2
    echo "           It was probably started before the address existed." >&2
else
    echo "  gzserver bound to $HOST_LINK_IP"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════════════
echo "=== [3/6] Camera recording ==="
if (( RECORD )); then
    mkdir -p "$(dirname "$BAG_DIR")"
    FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
    bash -lc "source /opt/ros/humble/setup.bash && \
              ros2 bag record -o '$BAG_DIR' /uav1/camera/image_raw" \
        > /tmp/hitl_bag.log 2>&1 &
    PIDS+=("$!")
    echo "  recording to $BAG_DIR"
    echo "  replay later with:  ros2 bag play $BAG_DIR --loop"
else
    echo "  skipped (use --record to capture the camera for offline replay)"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════════════
echo "=== [4/6] Edge detector on the Pi ==="
if (( WITH_PI )); then
    echo "  model=$(basename "$MODEL")  conf=$CONF  imgsz=$IMGSZ  debug=$DEBUG"
    # -tt forces a pty so that killing the ssh client also kills the remote
    # process; without it the detector would survive Ctrl+C here.
    #
    # Output goes to its own LOG rather than this terminal. Interleaving the
    # detector's ~1 Hz lines with the mission's output through a pty produces
    # unreadable, half-overwritten text (the pty emits CR without LF, so lines
    # land on top of each other). Follow it in a second window instead.
    : > "$DETECTOR_LOG"
    ssh -tt "$PI_HOST" "
        export FASTRTPS_DEFAULT_PROFILES_FILE=\$HOME/uav2_ws/config/fastdds_hitl_eth.xml
        source /opt/ros/humble/setup.bash
        source \$HOME/uav2_ws/install/setup.bash
        exec $PI_VENV_PY $PI_DETECTOR --ros-args \
            -p uav_id:=1 -p processing_mode:=edge \
            -p model_path:=$MODEL -p conf_threshold:=$CONF \
            -p imgsz:=$IMGSZ -p publish_debug:=$DEBUG
    " > "$DETECTOR_LOG" 2>&1 &
    PIDS+=("$!")
    # Poll rather than sleeping a fixed 5 s: OpenVINO takes ~12 s to load on the
    # Pi 4B, so a fixed wait reported a healthy detector as broken on every run.
    # (The first INFERENCE is also slow -- ~13 s, because OpenVINO compiles the
    # model on the first call -- then it settles to ~250 ms. Both are normal.)
    det_deadline=$((SECONDS + 60))
    while ! grep -q 'Model loaded OK' "$DETECTOR_LOG" 2>/dev/null; do
        (( SECONDS >= det_deadline )) && break
        sleep 2
    done
    if grep -q 'Model loaded OK' "$DETECTOR_LOG" 2>/dev/null; then
        echo "  detector running, log: $DETECTOR_LOG"
        grep -m1 'inference:' "$DETECTOR_LOG" | sed 's/^/  /' || true
    else
        echo "  WARNING: detector did not report 'Model loaded OK' within 60 s." >&2
        echo "           check $DETECTOR_LOG" >&2
    fi
else
    echo "  skipped (--no-pi)"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════════════
echo "=== [5/6] gcs_receiver on the host ==="
: > "$GCS_LOG"
# Runs INSIDE gcsns (10.42.0.10), not the root namespace. That is the whole
# point of the experiment: from there the only route back to the Pi is
# gcsns -> br-gcs -> tap-gcs -> ns-3 -> tap-uav2 -> br-uav2 -> the Pi's VLAN 42
# leg, so every detection message crosses the simulated radio and picks up its
# loss and latency. In the root namespace it would instead reach the Pi over
# the direct 10.0.0.x cable with no impairment at all, and the numbers would be
# meaningless.
#
# uav_vision is built into <repo>/install, NOT <repo>/ros2/install — the latter
# only carries uav_controller. Sourcing the wrong overlay gives
# "Package 'uav_vision' not found" and gcs_receiver silently never starts.
sudo ip netns exec gcsns sudo -H -u "${SUDO_USER:-$USER}" \
    env FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
    bash -lc "source /opt/ros/humble/setup.bash && \
              source '$PROJECT_DIR/install/setup.bash' && \
              exec ros2 run uav_vision gcs_receiver --ros-args \
                   -p uav_id:=1 -p processing_mode:=edge" \
    > "$GCS_LOG" 2>&1 &
PIDS+=("$!")
sleep 3
if grep -qi "not found\|Traceback" "$GCS_LOG" 2>/dev/null; then
    echo "  WARNING: gcs_receiver failed to start — see $GCS_LOG" >&2
    sed 's/^/    /' "$GCS_LOG" >&2
else
    echo "  started, log: $GCS_LOG"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════════════
echo "=== [6/6] Mission ==="
if (( RUN_MISSION )); then
    echo "  flying uav1_patrol_mission.py..."
    # The mission talks to drone_bridge, which lives INSIDE gcsns — from the
    # root namespace those services are invisible and it would hang forever.
    sudo ip netns exec gcsns sudo -H -u "${SUDO_USER:-$USER}" bash -lc "
        source /opt/ros/humble/setup.bash
        source '$PROJECT_DIR/ros2/install/setup.bash'
        python3 '$MISSION'
    " > "$MISSION_LOG" 2>&1 || true
    tail -5 "$MISSION_LOG" | sed 's/^/  /'
    echo "  mission finished — full log: $MISSION_LOG"
else
    echo "  skipped (use --mission to fly automatically). To fly manually:"
    echo "    sudo ip netns exec gcsns sudo -H -u ${SUDO_USER:-$USER} bash -lc '"
    echo "        source /opt/ros/humble/setup.bash"
    echo "        source $PROJECT_DIR/ros2/install/setup.bash"
    echo "        python3 $MISSION'"
fi

echo ""
echo "════════════════════════════════════════════════════════════"
echo " HITL STACK RUNNING — Ctrl+C to stop everything"
echo "════════════════════════════════════════════════════════════"
echo "  pipeline : $PIPELINE_LOG"
echo "  gcs      : $GCS_LOG"
(( WITH_PI )) && echo "  detector : $DETECTOR_LOG"
(( RECORD )) && echo "  bag      : $BAG_DIR"
echo ""
echo "  watch detector   :  tail -f $DETECTOR_LOG"
echo "  watch detections :  tail -f $GCS_LOG"
echo "  camera rate      :  FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_PROFILE \\"
echo "                      ros2 topic hz /uav1/camera/image_raw"
echo ""

wait
