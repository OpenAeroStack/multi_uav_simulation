#!/bin/bash
# =============================================================================
# kill_all_city_sim.sh
#
# Forceful, standalone cleanup for the 3-UAV city simulation stack.
#
# WHY THIS EXISTS:
#   launch_city_dds.sh spawns deeply nested processes (sudo -> ip netns exec
#   -> sudo -u $RUN_USER -> bash -lc '...'), sometimes inside a different
#   network namespace. A single Ctrl+C in the terminal sends SIGINT only to
#   the foreground process group and often does not reliably propagate
#   through all those layers, especially across namespace boundaries. This
#   script does NOT rely on signal propagation at all — it finds and kills
#   every relevant process directly by pattern and by namespace membership,
#   then tears down every namespace/bridge/tap/veth unconditionally.
#
#   Safe to run any time — including when nothing is running (everything
#   is guarded with `|| true` / 2>/dev/null so it never errors out on
#   already-clean state).
#
# USAGE:
#   sudo bash scripts/kill_all_city_sim.sh
#
#   Run this BEFORE every fresh test run of launch_city_dds.sh, to guarantee
#   you're testing against a truly clean slate rather than leftover state
#   from a previous run that didn't fully stop.
# =============================================================================

set -uo pipefail   # deliberately NOT -e — we want to keep going even if one step fails

[[ "${EUID}" -ne 0 ]] && exec sudo -E bash "$0" "$@"

GREEN='\033[0;32m'; CYAN='\033[0;36m'; YELLOW='\033[1;33m'; RESET='\033[0m'
pass() { echo -e "${GREEN}[✓]${RESET} $*"; }
info() { echo -e "${CYAN}[i]${RESET} $*"; }
warn() { echo -e "${YELLOW}[!]${RESET} $*"; }

echo ""
echo "=== Killing all city simulation processes (by pattern, everywhere) ==="

PATTERNS=(
    'city_mission'
    'drone_bridge'
    'single_uav_mission'
    '/build/sitl/bin/arducopter'
    'micro_ros_agent.*udp4.*--port'
    'scratch_three-uav_three-uav'
    'three_uav_tapbridge_rt'
    'ns3.38-three-uav'
    'gzserver'
    'gzclient'
    'mavproxy'
)

for pattern in "${PATTERNS[@]}"; do
    pids="$(pgrep -f -- "$pattern" 2>/dev/null || true)"
    if [[ -n "$pids" ]]; then
        info "Killing '$pattern': $pids"
        # shellcheck disable=SC2086
        kill -TERM $pids 2>/dev/null || true
    fi
done

sleep 2

# Force-kill anything that survived SIGTERM
for pattern in "${PATTERNS[@]}"; do
    pids="$(pgrep -f -- "$pattern" 2>/dev/null || true)"
    if [[ -n "$pids" ]]; then
        warn "Force-killing '$pattern': $pids"
        # shellcheck disable=SC2086
        kill -KILL $pids 2>/dev/null || true
    fi
done

pass "Process patterns cleared."

echo ""
echo "=== Killing every process INSIDE each namespace, regardless of pattern ==="

for ns in gcsns uav1 uav2 uav3; do
    if ip netns list 2>/dev/null | awk '{print $1}' | grep -qx "$ns"; then
        pids="$(ip netns pids "$ns" 2>/dev/null || true)"
        if [[ -n "$pids" ]]; then
            info "Killing all PIDs inside $ns: $pids"
            # shellcheck disable=SC2086
            kill -KILL $pids 2>/dev/null || true
        fi
    fi
done

pass "Namespace-internal processes cleared."

echo ""
echo "=== Tearing down namespaces ==="
for ns in gcsns uav1 uav2 uav3; do
    ip netns del "$ns" 2>/dev/null && info "Deleted namespace $ns" || true
done

echo ""
echo "=== Tearing down bridges ==="
for br in br-gcs br-uav1 br-uav2 br-uav3; do
    ip link set "$br" down 2>/dev/null || true
    ip link del "$br" type bridge 2>/dev/null && info "Deleted bridge $br" || true
done

echo ""
echo "=== Tearing down taps and veths ==="
for link in \
    tap-gcs tap-uav1 tap-uav2 tap-uav3 \
    veth-gcs-host veth-uav1-host veth-uav2-host veth-uav3-host \
    sim-uav1-host sim-uav2-host sim-uav3-host \
    veth-fdm-root veth-gcs veth-uav1 veth-uav2 veth-uav3 \
    br-gcs br-uav1 br-uav2 br-uav3
do
    ip link del "$link" 2>/dev/null && info "Deleted $link" || true
done

echo ""
echo "=== Freeing Gazebo master port (11345) if still held ==="
if lsof -iTCP:11345 -sTCP:LISTEN >/dev/null 2>&1; then
    fuser -k 11345/tcp 2>/dev/null || true
    sleep 1
    pass "Port 11345 freed"
else
    info "Port 11345 already free"
fi

echo ""
echo "=== Removing stale PID/lock files ==="
rm -f /tmp/multi_uav_city_launcher.pid
rm -rf /tmp/multi_uav_sitl
rm -f /tmp/micro_ros_agent_uav*.log
rm -f /tmp/drone_bridge_uav*.log
rm -f /tmp/gazebo_city.log
rm -f /tmp/city_mission.log
rm -f /tmp/ns3_stdout.log
rm -f /tmp/snr_log.csv

echo ""
pass "Full cleanup complete. Safe to run launch_city_dds.sh fresh now."
echo ""
echo "Quick sanity check:"
echo "  ip netns list          (should be empty)"
echo "  ps aux | grep -E 'arducopter|gzserver|three-uav|drone_bridge'  (should show only this grep itself)"