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
#
# One row per aircraft. Everything downstream loops over 1..NUAVS, so adding
# board 3 is one more entry in each array plus a 3-UAV launcher.
#
#   PI_HOST[i]      the board that carries aircraft i's vision
#   HOST_LINK[i]    the host's address on THAT board's sensor VLAN. It must
#                   exist before Gazebo starts: Fast DDS 2.6 reads its interface
#                   list once, at participant creation, and never re-checks.
declare -A PI_HOST=(  [1]="anton@10.0.0.2"  [2]="anton@10.0.1.2" )
declare -A HOST_LINK=( [1]="10.0.0.1"        [2]="10.0.1.1" )

#   MISSION_MODE[i]  what aircraft i does under --mission
#     survey : fly to the subjects, hold HOLD_SECONDS on station, return home
#     hover  : take off where it stands and hold (no transit)
#
#   UAV2 is parked beside the group in the 2-UAV world, so hovering keeps the
#   subjects in frame continuously. UAV1 patrols and sees them only in bursts.
#   Both end up at 25 m, so the two boards view the same scene from the same
#   height and their detection rates are comparable.
declare -A MISSION_MODE=( [1]="survey" [2]="hover" )
HOLD_SECONDS=30.0        # survey: seconds on station
# The camera is pitched 45 deg FORWARD-down, so at altitude A it sees the ground
# roughly 0.7*A to 1.5*A metres AHEAD -- never directly below. Hold this far
# short of the subjects so they land in the middle of that band.
#   ALTITUDE_M  25  ->  band 17-36 m ahead  ->  OFFSET_M 25
#   ALTITUDE_M  15  ->  band 10-22 m ahead  ->  OFFSET_M 15
# Set OFFSET_M 0 to hold directly overhead (they will be in the blind spot).
ALTITUDE_M=15.0
OFFSET_M=15.0


PI_HOST_DEFAULT="${PI_HOST[1]}"
HOST_LINK_IP="${HOST_LINK[1]}"    # must exist BEFORE Gazebo starts (see below)
PI_VENV_PY="\$HOME/yolo_env/bin/python"
PI_DETECTOR="\$HOME/uav2_ws/install/uav_vision/lib/uav_vision/detector"
# Medians measured on the Pi 4B by scripts/bench_backends.py — see
# docs/HITL_INTEGRATION_PLAN.md §10. The default is the fastest CORRECT back-end.
PI_MODEL_OPENVINO="/home/anton/models/yolo11n_openvino_model"       # 236 ms
PI_MODEL_NCNN="/home/anton/models/yolov8n_384x640_ncnn_model"       # 270 ms
PI_MODEL_PT="/home/anton/models/yolov8n.pt"                         # 1027 ms

DDS_PROFILE="$PROJECT_DIR/config/fastdds_hitl_eth.xml"
PIPELINE_1UAV="$PROJECT_DIR/scripts/netns/launch_single_uav_netns.sh"
PIPELINE_2UAV="$PROJECT_DIR/scripts/netns/launch_2uav_netns.sh"
MISSION="$PROJECT_DIR/ros2/uav_controller/uav_controller/uav1_patrol_mission.py"
VIEWER="$PROJECT_DIR/scripts/detection_viewer.py"

PIPELINE_LOG="/tmp/hitl_pipeline.log"
GCS_LOG="/tmp/hitl_gcs.log"
DETECTOR_LOG="/tmp/hitl_detector.log"
MISSION_LOG="/tmp/hitl_mission.log"
BAG_DIR="$HOME/hitl_bags/cam_$(date +%Y%m%d_%H%M%S)"

# ── Options ──────────────────────────────────────────────────────────────────
NUAVS=1                # --uavs N : how many aircraft to bring up
VIEW=0                 # --view   : open a camera+boxes window per aircraft
GUI=0                  # --gui    : open the Gazebo 3D viewer
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
  --uavs <N>        how many aircraft to bring up (1 or 2, default 1)
                    N=2 uses launch_2uav_netns.sh and drives BOTH Pi boards
  --view            open a camera+detections window per aircraft
                    (costs host CPU — close them before timing runs)
  --gui             open the Gazebo 3D viewer, to watch the drones fly
                    (heavy: the city heightmap can make it hang "not
                     responding" — Force Quit is safe, gzserver keeps running)
  --mission         run each aircraft's MISSION_MODE, in parallel
                    (see MISSION_MODE at the top: survey / hover)
  --record          record /uav1/camera/image_raw to a rosbag, so the detector
                    can later be tuned with `ros2 bag play` instead of re-flying
  --no-pi           host only; skip the remote detector (e.g. Pi unplugged)
  -h, --help        this text

Examples:
  ./scripts/run_hitl.sh --uavs 2 --view --mission   # the full two-board run
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
        --uavs)     NUAVS="$2"; shift 2 ;;
        --view)     VIEW=1; shift ;;
        --gui)      GUI=1; shift ;;
        --mission)  RUN_MISSION=1; shift ;;
        --record)   RECORD=1; shift ;;
        --no-pi)    WITH_PI=0; shift ;;
        -h|--help)  usage; exit 0 ;;
        *)          echo "unknown option: $1" >&2; usage; exit 1 ;;
    esac
done

case "$NUAVS" in
    1) PIPELINE="$PIPELINE_1UAV" ;;
    2) PIPELINE="$PIPELINE_2UAV" ;;
    *) echo "--uavs must be 1 or 2 (got '$NUAVS')" >&2; exit 1 ;;
esac

# Socket buffers, reported per machine.
#
# BOTH directions matter. rmem alone is not enough: with wmem_max left at the
# default, the DDS profile's 16 MB <sendBufferSize> is silently clamped to
# 208 KB and the PUBLISHER discards fragments before they reach the NIC. That
# failure is invisible to every interface counter -- tx_dropped stays 0 on the
# host and rx_errors stays 0 on the Pi, because nothing was ever transmitted.
# Measured symptom: camera delivered at 0.19 Hz against a 5 Hz source.
buf_warn() {
    echo "  WARNING $1: net.core.$2=$3 is too small for camera frames." >&2
    echo "          sudo tee /etc/sysctl.d/60-ros2-dds.conf <<'EOF'" >&2
    echo "          net.core.rmem_max = 536870912" >&2
    echo "          net.core.rmem_default = 134217728" >&2
    echo "          net.core.wmem_max = 536870912" >&2
    echo "          net.core.wmem_default = 134217728" >&2
    echo "          net.ipv4.ipfrag_high_thresh = 134217728" >&2
    echo "          EOF" >&2
    echo "          sudo sysctl --system   # then RESTART the stack:" >&2
    echo "          Fast DDS reads socket options once, at participant creation." >&2
}

PIDS=()
cleanup() {
    echo ""
    echo "=== Shutting down ==="
    # Stop the remote detector first — it is the only thing not on this machine
    # and would otherwise keep running after the local processes are gone.
    if (( WITH_PI )); then
        for i in $(seq 1 "$NUAVS"); do
            # [u]av_vision keeps pkill from matching its own command line, which
            # would kill the ssh session instead of the detector.
            ssh -o ConnectTimeout=4 "${PI_HOST[$i]}" \
                'pkill -f "[u]av_vision/detector"' 2>/dev/null \
                && echo "  detector stopped on ${PI_HOST[$i]#*@}" || true
        done
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
for i in $(seq 1 "$NUAVS"); do
    link="${HOST_LINK[$i]}"
    if ! ip -4 addr show | grep -q "${link}/"; then
        echo "ERROR: $link is not assigned to any interface (aircraft $i)." >&2
        echo "       Plug in the USB ethernet adapter and bring up its VLAN:" >&2
        echo "         nmcli connection show" >&2
        echo "         sudo nmcli connection up eth-cam${i/1/}" >&2
        exit 1
    fi
    echo "  aircraft $i: host link $link present"
done

if (( WITH_PI )); then
    for i in $(seq 1 "$NUAVS"); do
        host="${PI_HOST[$i]}"
        if ! ping -c 1 -W 2 "${host#*@}" >/dev/null 2>&1; then
            echo "ERROR: Pi $i at ${host#*@} is not reachable. Use --no-pi to skip." >&2
            exit 1
        fi
        # A missing model only surfaces as a WARNING a minute later, by which
        # point the whole stack is already up. Check it before paying for that.
        if ! ssh -o ConnectTimeout=4 "$host" "test -e '$MODEL'" 2>/dev/null; then
            echo "ERROR: model not found on Pi $i: $MODEL" >&2
            ssh -o ConnectTimeout=4 "$host" 'ls -d ~/models/* 2>/dev/null' >&2 || true
            exit 1
        fi
        echo "  aircraft $i: Pi ${host#*@} reachable, model present"
    done

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
    # Check the host once, then every board.
    for knob in rmem_max wmem_max; do
        val=$(sysctl -n "net.core.$knob")
        (( val < 100000000 )) && buf_warn "host" "$knob" "$val"
    done
    for i in $(seq 1 "$NUAVS"); do
        for knob in rmem_max wmem_max; do
            val=$(ssh -o ConnectTimeout=4 "${PI_HOST[$i]}" \
                      "sysctl -n net.core.$knob" 2>/dev/null || echo 0)
            (( val < 100000000 )) && buf_warn "Pi $i" "$knob" "$val"
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
echo "=== [4/6] Edge detectors on the Pi boards ==="
if (( WITH_PI )); then
    echo "  model=$(basename "$MODEL")  conf=$CONF  imgsz=$IMGSZ  debug=$DEBUG"
    for i in $(seq 1 "$NUAVS"); do
        host="${PI_HOST[$i]}"
        log="/tmp/hitl_detector_uav$i.log"
        : > "$log"
        # -tt forces a pty so killing the ssh client also kills the remote
        # process; without it the detector survives Ctrl+C here.
        ssh -tt "$host" "
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
    # OpenVINO takes ~12 s to load on a Pi 4B, so poll rather than sleep a
    # fixed time -- a fixed wait reported a healthy detector as broken.
    det_deadline=$((SECONDS + 90))
    for i in $(seq 1 "$NUAVS"); do
        log="/tmp/hitl_detector_uav$i.log"
        while ! grep -q 'Model loaded OK' "$log" 2>/dev/null; do
            (( SECONDS >= det_deadline )) && break
            sleep 2
        done
        if grep -q 'Model loaded OK' "$log" 2>/dev/null; then
            echo "  uav$i detector running, log: $log"
        else
            echo "  WARNING: uav$i detector never reported 'Model loaded OK'." >&2
            echo "           check $log" >&2
        fi
    done
else
    echo "  skipped (--no-pi)"
fi
echo ""

# ═══════════════════════════════════════════════════════════════════════════
echo "=== [5/6] gcs_receivers + viewers ==="
for i in $(seq 1 "$NUAVS"); do
    log="/tmp/hitl_gcs_uav$i.log"
    : > "$log"
    # Runs INSIDE gcsns. That is the whole point: from there the only route
    # back to the Pi is gcsns -> ns-3 -> the board, so every detection crosses
    # the simulated radio. In the root namespace it would arrive over the
    # unimpaired camera cable and the latency figures would be meaningless.
    #
    # uav_vision is built into <repo>/install, NOT <repo>/ros2/install -- the
    # latter only carries uav_controller.
    sudo ip netns exec gcsns sudo -H -u "${SUDO_USER:-$USER}" \
        env FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
        bash -lc "source /opt/ros/humble/setup.bash && \
                  source '$PROJECT_DIR/install/setup.bash' && \
                  exec ros2 run uav_vision gcs_receiver --ros-args \
                       -p uav_id:=$i -p processing_mode:=edge" \
        > "$log" 2>&1 &
    PIDS+=("$!")
done
sleep 3
for i in $(seq 1 "$NUAVS"); do
    log="/tmp/hitl_gcs_uav$i.log"
    if grep -qi "not found\|Traceback" "$log" 2>/dev/null; then
        echo "  WARNING: uav$i gcs_receiver failed — see $log" >&2
        sed 's/^/    /' "$log" >&2
    else
        echo "  uav$i gcs_receiver started, log: $log"
    fi
done

# The Gazebo 3D viewer. gzclient attaches to the already-running gzserver, so
# it can be opened and closed at any time without disturbing the simulation.
# It needs the same asset paths gzserver was given, or the city renders empty.
if (( GUI )); then
    CITY="$HOME/FYP/small_city_gazebo_world"
    GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$CITY/models:${GAZEBO_MODEL_PATH:-}" \
    GAZEBO_RESOURCE_PATH="$PROJECT_DIR:$PROJECT_DIR/worlds:$CITY:${GAZEBO_RESOURCE_PATH:-}" \
    GAZEBO_PLUGIN_PATH="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib:$HOME/ardupilot_gazebo/build:${GAZEBO_PLUGIN_PATH:-}" \
    GAZEBO_MODEL_DATABASE_URI= \
        gzclient > /tmp/hitl_gzclient.log 2>&1 &
    PIDS+=("$!")
    echo "  Gazebo viewer opened (gzclient)"
fi

# Viewers draw the Pi's boxes on the camera image, which is already local.
# Nothing extra crosses the cable, but Gazebo serialises each frame for one
# more subscriber -- host CPU only. Off by default; close before timing runs.
if (( VIEW )); then
    for i in $(seq 1 "$NUAVS"); do
        FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_PROFILE" \
        bash -lc "source /opt/ros/humble/setup.bash && \
                  exec python3 '$VIEWER' --ros-args \
                       -p image_topic:=/uav$i/camera/image_raw \
                       -p detection_topic:=/detections/uav$i" \
            > "/tmp/hitl_viewer_uav$i.log" 2>&1 &
        PIDS+=("$!")
        echo "  uav$i viewer window opened"
    done
fi
echo ""

echo "=== [6/6] Missions ==="
if (( RUN_MISSION )); then
    # In PARALLEL, one per aircraft. The mission talks to drone_bridge, which
    # lives inside gcsns -- from the root namespace those services are invisible
    # and it would hang forever.
    mission_pids=()
    for i in $(seq 1 "$NUAVS"); do
        log="/tmp/hitl_mission_uav$i.log"
        mode="${MISSION_MODE[$i]:-patrol}"
        case "$mode" in
            survey) cmd="python3 '$MISSION' --ros-args -p uav_id:=$i \
                          -p hold_seconds:=$HOLD_SECONDS \
                          -p altitude_m:=$ALTITUDE_M -p offset_m:=$OFFSET_M" ;;
            hover)  cmd="ros2 service call /uav$i/takeoff std_srvs/srv/Trigger {}" ;;
            *)      echo "  uav$i: unknown MISSION_MODE '$mode' — skipping" >&2; continue ;;
        esac
        echo "  uav$i: $mode -> $log"
        sudo ip netns exec gcsns sudo -H -u "${SUDO_USER:-$USER}" bash -lc "
            source /opt/ros/humble/setup.bash
            source '$PROJECT_DIR/ros2/install/setup.bash'
            $cmd
        " > "$log" 2>&1 &
        mission_pids+=("$!")
    done
    for pid in "${mission_pids[@]}"; do wait "$pid" || true; done
    for i in $(seq 1 "$NUAVS"); do
        echo "  --- uav$i ---"
        tail -3 "/tmp/hitl_mission_uav$i.log" | sed 's/^/    /'
    done
    echo "  missions finished"
else
    echo "  skipped (use --mission). To fly one manually:"
    echo "    sudo ip netns exec gcsns sudo -H -u ${SUDO_USER:-$USER} bash -lc '"
    echo "        source /opt/ros/humble/setup.bash"
    echo "        source $PROJECT_DIR/ros2/install/setup.bash"
    echo "        python3 $MISSION --ros-args -p uav_id:=1'"
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
