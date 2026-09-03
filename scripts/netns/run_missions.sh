#!/bin/bash
# Step 3 of 4: fly both aircraft. Run AFTER sitl_init.sh says PIPELINE READY.
#
#   ./scripts/netns/rpi_init.sh                 # 0 - verify both Raspberry Pi boards
#   ./scripts/netns/sitl_init.sh --gui --view   # 1 - host pipeline, leave running
#   ./scripts/netns/detector_start.sh           # 2 - Pi detectors + receivers
#   ./scripts/netns/run_missions.sh             # 3 - fly
#
# Split from the launcher on purpose: initialisation must be fully verified
# before a mission arms anything, and each half can then be debugged on its own.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"

MISSION="$PROJECT_DIR/ros2/uav_controller/uav_controller/two_drone_mission.py"
LOG=/tmp/mission_2uav.log
BOARDS="1 2"

# Thermal probes. Same hosts as detector_start.sh; keep the two in step.
declare -A PI_HOST=( [1]="anton@10.0.0.2"  [2]="anton@10.0.1.2" )
THERMAL_INTERVAL=2
THERMAL_MARKER=UAV_THERMAL_PROBE
THERMAL_PIDS=()

# Leading \r forces column 0: SITL and ssh children clear ONLCR, and without it
# every line prints one step further right than the last.
say() { printf '\r%s\n' "$*"; }

# ── Cleanup ─────────────────────────────────────────────────────────────────
# Kill by PATTERN, not $!. The mission runs under
# `sudo ip netns exec ... sudo ... bash -lc`, so $! is the OUTER sudo and
# killing it leaves the python child flying the aircraft.
CLEANED=0
cleanup() {
    (( CLEANED )) && return 0
    CLEANED=1

    # Remote loop first: a bare ssh kill can leave it sampling forever.
    for i in $BOARDS; do
        ssh -n -o ConnectTimeout=4 "${PI_HOST[$i]}" \
            "pkill -f -- '[U]AV_THERMAL_PROBE'" >/dev/null 2>&1 || true
    done
    for pid in "${THERMAL_PIDS[@]:-}"; do kill -TERM "$pid" 2>/dev/null || true; done

    if pgrep -f -- '[t]wo_drone_mission' >/dev/null 2>&1; then
        say ""
        say "=== Stopping the mission ==="
        sudo pkill -TERM -f -- '[t]wo_drone_mission' 2>/dev/null || true
        sleep 2
        sudo pkill -9 -f -- '[t]wo_drone_mission' 2>/dev/null || true
        say "  mission stopped — the aircraft keep their last command"
        say "  land them with:  ros2 service call /uavN/rtl std_srvs/srv/Trigger"
    fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# ── Preconditions ───────────────────────────────────────────────────────────
# Refuse to fly a half-built stack: each check names what to restart.
require() {
    local what="$1" test_cmd="$2"
    if ! eval "$test_cmd" >/dev/null 2>&1; then
        echo "ERROR: $what" >&2
        echo "       Run ./scripts/netns/sitl_init.sh and wait for PIPELINE READY." >&2
        exit 1
    fi
    say "  ok: $what"
}

say "=== Preconditions ==="
require "namespaces exist"     'ip netns list | grep -q gcsns'
require "ns-3 running"         'pgrep -f three_uav_tapbridge_integrated'
require "Gazebo running"       'pgrep -x gzserver'
require "SITL x2 running"      '[ "$(pgrep -cf /build/sitl/bin/arducopter)" -ge 2 ]'
require "micro-ROS agents x2"  '[ "$(pgrep -cf micro_ros_agent)" -ge 2 ]'
require "drone_bridge x2"      '[ "$(pgrep -cf uav_controller/drone_bridge)" -ge 2 ]'
[[ -f "$MISSION" ]] || { echo "ERROR: mission not found: $MISSION" >&2; exit 1; }
say "  ok: mission script exists"

# A bridge that never saw GPS cannot fly; catch it here, not 30 s into a flight.
for i in $BOARDS; do
    grep -q "GPS flowing" "/tmp/bridge_2uav_uav$i.log" 2>/dev/null \
        || { echo "ERROR: UAV$i bridge never reported GPS flowing." >&2
             echo "       tail /tmp/bridge_2uav_uav$i.log" >&2; exit 1; }
    say "  ok: UAV$i telemetry was flowing"
done

# Refuse to start a second mission on top of a running one.
pgrep -f -- '[t]wo_drone_mission' >/dev/null 2>&1 \
    && { echo "ERROR: a mission is already running." >&2
         echo "       stop it with: sudo pkill -f two_drone_mission" >&2; exit 1; }
say ""

# Warm the sudo cache: the launch below is backgrounded, and a password prompt
# on a backgrounded process stalls with no visible reason.
sudo -v

# ── Thermal probes ──────────────────────────────────────────────────────────
# Inference time is only comparable between runs if the boards were in the same
# thermal state; a throttled Pi runs identical maths slower, never differently.
say "=== Thermal probes ==="
for i in $BOARDS; do
    tlog="/tmp/thermal_uav$i.log"
    : > "$tlog"
    # The marker is only there to give cleanup() something to pkill on.
    ssh -n -o ConnectTimeout=5 "${PI_HOST[$i]}" "
        $THERMAL_MARKER=1
        while true; do
            printf '%s %s %s\n' \"\$(date +%s)\" \
                   \"\$(vcgencmd measure_temp)\" \"\$(vcgencmd get_throttled)\"
            sleep $THERMAL_INTERVAL
        done" >> "$tlog" 2>&1 &
    THERMAL_PIDS+=("$!")
    say "  board $i -> $tlog"
done
sleep 3
for i in $BOARDS; do
    [[ -s "/tmp/thermal_uav$i.log" ]] \
        || say "  WARNING: board $i thermal probe is silent — see /tmp/thermal_uav$i.log"
done
say ""

# ── Fly ─────────────────────────────────────────────────────────────────────
# Inside gcsns: drone_bridge lives there and its services are invisible from
# the root namespace, where the mission would wait forever.
say "=== Mission ==="
: > "$LOG"

sudo ip netns exec gcsns sudo -H -u "$RUN_USER" bash -lc "
    source /opt/ros/humble/setup.bash
    source '$PROJECT_DIR/ros2/install/setup.bash'
    exec python3 '$MISSION'
" > "$LOG" 2>&1 &
WRAPPER_PID=$!

say "  two_drone_mission -> $LOG"
say ""
say "  follow with:  tail -f $LOG"
say ""

STATUS=0
wait "$WRAPPER_PID" || STATUS=1

# ── Results ─────────────────────────────────────────────────────────────────
say ""
say "=== Results ==="
if grep -q "ABORTED\|FINISHED WITH ERRORS" "$LOG" 2>/dev/null; then
    say "  FAILED"
    grep -E "ABORTED|ERROR" "$LOG" | while read -r l; do say "        $l"; done || true
    STATUS=1
else
    say "  all aircraft completed the mission"
fi
say ""
grep -E "arrived|reached|mission complete" "$LOG" 2>/dev/null | tail -6 \
    | while read -r l; do say "        $l"; done || true
say ""

# ── Archive ─────────────────────────────────────────────────────────────────
# /tmp logs are truncated by the next run and cleared on reboot, so every run
# is copied out before anything can overwrite it. Timestamped: never collides.
ARCHIVE="$PROJECT_DIR/results/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$ARCHIVE"
for f in /tmp/mission_2uav.log /tmp/thermal_uav*.log \
         /tmp/detector_uav*.log /tmp/gcs_receiver_uav*.log; do
    [[ -e "$f" ]] && cp "$f" "$ARCHIVE"/ 2>/dev/null || true
done

# Record what produced these numbers; a run is not comparable without it.
{
    echo "date       : $(date -Is)"
    echo "mission    : $MISSION"
    echo "model      : ${MODEL:-<detector_start.sh default>}"
    git -C "$PROJECT_DIR" log -1 --format='commit     : %h %s' 2>/dev/null || true
    git -C "$PROJECT_DIR" status --short 2>/dev/null | sed 's/^/modified   : /' || true
} > "$ARCHIVE/run_info.txt"

"$SCRIPT_DIR/../summarise_run.sh" "$ARCHIVE" | tee "$ARCHIVE/summary.txt" \
    | while read -r l; do say "$l"; done
say "  archived -> $ARCHIVE"
say ""

exit "$STATUS"
