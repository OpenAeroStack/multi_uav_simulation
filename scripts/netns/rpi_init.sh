#!/bin/bash
# Step 0 of 4: verify both Raspberry Pi edge nodes before anything else starts.
#
#   ./scripts/netns/rpi_init.sh                 # 0 - verify both Raspberry Pi boards
#   ./scripts/netns/sitl_init.sh --gui --view   # 1 - host pipeline, leave running
#   ./scripts/netns/detector_start.sh           # 2 - Pi detectors + receivers
#   ./scripts/netns/run_missions.sh             # 3 - fly
#
# Checks only the CAMERA VLAN end to end: the radio VLAN runs through ns-3,
# which is not up yet, so here we confirm the Pi holds its address and no more.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

# One row per board. Must match run_hitl.sh and sitl_init.sh.
declare -A PI_HOST=(   [1]="anton@10.0.0.2"  [2]="anton@10.0.1.2" )
declare -A HOST_LINK=( [1]="10.0.0.1"        [2]="10.0.1.1" )
declare -A CAM_VLAN_IF=( [1]="eth-cam"       [2]="eth-cam2" )
declare -A RF_PI_IP=(  [1]="10.42.0.12"      [2]="10.42.0.14" )

PI_MODEL="${PI_MODEL:-/home/anton/models/yolo11n_openvino_model}"
PI_DETECTOR='$HOME/uav2_ws/install/uav_vision/lib/uav_vision/detector'
PI_VENV_PY='$HOME/yolo_env/bin/python'
MIN_BUF=100000000          # 100 MB; a 1280x720 RGB frame is 2.76 MB
MAX_CLOCK_SKEW_MS=50       # detection latency is measured across this link
SSH="ssh -o ConnectTimeout=4 -o BatchMode=yes"

BOARDS="${BOARDS:-1 2}"
FAIL=0
WARN=0

ok()   { printf '  \033[32mok\033[0m    %s\n' "$1"; }
bad()  { printf '  \033[31mFAIL\033[0m  %s\n' "$1"; FAIL=$((FAIL + 1)); }
warn() { printf '  \033[33mwarn\033[0m  %s\n' "$1"; WARN=$((WARN + 1)); }
hint() { printf '        %s\n' "$1"; }

# ── host side ───────────────────────────────────────────────────────────────
echo "=== [1/5] Host network ==="

PI_LINK_IF="${PI_LINK_IF:-$(ls /sys/class/net 2>/dev/null | grep -m1 '^enx' || true)}"
if [[ -n "$PI_LINK_IF" && -d "/sys/class/net/$PI_LINK_IF" ]]; then
    ok "USB ethernet adapter present: $PI_LINK_IF"
else
    bad "no Pi-facing NIC found (expected an enx* interface)"
    hint "plug in the USB ethernet adapter, then: ip link show"
fi

# FastDDS 2.6 reads its interface list ONCE at participant creation, so these
# addresses must exist before Gazebo starts or the camera link never binds.
for i in $BOARDS; do
    link="${HOST_LINK[$i]}"
    if ip -4 addr show | grep -q "${link}/"; then
        ok "board $i: host address $link present on ${CAM_VLAN_IF[$i]}"
    else
        bad "board $i: $link is not assigned to any interface"
        hint "sudo nmcli connection up ${CAM_VLAN_IF[$i]}"
    fi
done
echo ""

# ── reachability ────────────────────────────────────────────────────────────
echo "=== [2/5] Pi reachability (camera VLAN) ==="
for i in $BOARDS; do
    ip_addr="${PI_HOST[$i]#*@}"
    if ping -c 2 -W 2 "$ip_addr" >/dev/null 2>&1; then
        rtt=$(ping -c 3 -W 2 "$ip_addr" 2>/dev/null |
              awk -F'/' '/rtt|round-trip/ {printf "%.2f", $5}')
        ok "board $i: $ip_addr reachable (${rtt:-?} ms avg)"
    else
        bad "board $i: $ip_addr does not answer ping"
        hint "check the cable, the switch, and VLAN ${CAM_VLAN_IF[$i]} on both ends"
        continue
    fi
    if $SSH "${PI_HOST[$i]}" true 2>/dev/null; then
        ok "board $i: ssh works without a password"
    else
        bad "board $i: ssh to ${PI_HOST[$i]} failed"
        hint "ssh-copy-id ${PI_HOST[$i]}"
    fi
done
echo ""

# ── Pi-side configuration ───────────────────────────────────────────────────
echo "=== [3/5] Pi configuration ==="
for i in $BOARDS; do
    $SSH "${PI_HOST[$i]}" true 2>/dev/null || { warn "board $i: skipped, no ssh"; continue; }

    # The radio VLAN cannot be pinged yet (ns-3 is down), so just confirm the
    # Pi holds the address the launcher will bridge into tap-uav$((i*2)).
    if $SSH "${PI_HOST[$i]}" "ip -4 addr show | grep -q '${RF_PI_IP[$i]}/'" 2>/dev/null; then
        ok "board $i: radio address ${RF_PI_IP[$i]} configured"
    else
        bad "board $i: ${RF_PI_IP[$i]} missing on the Pi"
        hint "on the Pi:  sudo bash pi_hitl_link.sh"
    fi

    if $SSH "${PI_HOST[$i]}" "test -e '$PI_MODEL'" 2>/dev/null; then
        ok "board $i: detector model present"
    else
        bad "board $i: model not found: $PI_MODEL"
        hint "$SSH ${PI_HOST[$i]} 'ls -d ~/models/*'"
    fi

    if $SSH "${PI_HOST[$i]}" "test -x $PI_DETECTOR" 2>/dev/null; then
        ok "board $i: uav_vision detector installed"
    else
        bad "board $i: detector missing or not executable"
        hint "on the Pi:  cd ~/uav2_ws && colcon build --packages-select uav_vision"
    fi

    if $SSH "${PI_HOST[$i]}" "test -x $PI_VENV_PY" 2>/dev/null; then
        ok "board $i: yolo_env python present"
    else
        bad "board $i: ~/yolo_env/bin/python missing"
    fi
done
echo ""

# ── socket buffers ──────────────────────────────────────────────────────────
# A 2.76 MB frame is ~2000 UDP fragments. Undersized buffers drop them silently:
# no counter moves on either side, the camera just arrives at 0.19 Hz.
echo "=== [4/5] Socket buffers ==="
for knob in rmem_max wmem_max; do
    val=$(sysctl -n "net.core.$knob" 2>/dev/null || echo 0)
    if (( val >= MIN_BUF )); then
        ok "host: net.core.$knob = $val"
    else
        warn "host: net.core.$knob = $val (want >= $MIN_BUF)"
        hint "sudo sysctl -w net.core.$knob=536870912"
    fi
done
for i in $BOARDS; do
    $SSH "${PI_HOST[$i]}" true 2>/dev/null || continue
    for knob in rmem_max wmem_max; do
        val=$($SSH "${PI_HOST[$i]}" "sysctl -n net.core.$knob" 2>/dev/null || echo 0)
        if (( val >= MIN_BUF )); then
            ok "board $i: net.core.$knob = $val"
        else
            warn "board $i: net.core.$knob = $val (want >= $MIN_BUF)"
            hint "on the Pi:  sudo sysctl -w net.core.$knob=536870912"
        fi
    done
done
echo ""

# ── clock ───────────────────────────────────────────────────────────────────
# Detection latency is measured as a difference between host and Pi timestamps,
# so any skew lands directly in the published numbers.
echo "=== [5/5] Clock sync ==="
for i in $BOARDS; do
    $SSH "${PI_HOST[$i]}" true 2>/dev/null || { warn "board $i: skipped, no ssh"; continue; }
    pi_epoch=$($SSH "${PI_HOST[$i]}" 'date +%s%3N' 2>/dev/null || echo "")
    host_epoch=$(date +%s%3N)
    if [[ -z "$pi_epoch" ]]; then
        warn "board $i: could not read the Pi clock"
        continue
    fi
    skew=$(( pi_epoch > host_epoch ? pi_epoch - host_epoch : host_epoch - pi_epoch ))
    if (( skew <= MAX_CLOCK_SKEW_MS )); then
        ok "board $i: clock within ${skew} ms of the host"
    else
        warn "board $i: clock is ${skew} ms off (want <= ${MAX_CLOCK_SKEW_MS} ms)"
        hint "on the Pi:  sudo systemctl restart chrony && chronyc sources"
    fi
done
echo ""

# ── verdict ─────────────────────────────────────────────────────────────────
echo "════════════════════════════════════════════════════════════"
if (( FAIL > 0 )); then
    echo " NOT READY — $FAIL check(s) failed, $WARN warning(s)"
    echo "════════════════════════════════════════════════════════════"
    echo " Fix the FAIL lines above, then run this script again."
    exit 1
fi
if (( WARN > 0 )); then
    echo " READY WITH WARNINGS — $WARN warning(s)"
    echo "════════════════════════════════════════════════════════════"
    echo " Safe to fly, but the warnings will show up in your measurements."
else
    echo " ALL CHECKS PASSED — both boards ready"
    echo "════════════════════════════════════════════════════════════"
fi
echo ""
echo " Next:  ./scripts/netns/sitl_init.sh --gui --view"
echo "        ./scripts/netns/detector_start.sh"
echo "        ./scripts/netns/run_missions.sh"
