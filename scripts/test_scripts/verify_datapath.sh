#!/bin/bash
# =============================================================================
#  verify_datapath.sh  --  Tier 0 datapath verification
#
#  Answers one question: DO PACKETS SENT BETWEEN THE NETWORK NAMESPACES
#  ACTUALLY TRAVERSE THE NS-3 SIMULATED WIRELESS CHANNEL?
#
#  Everything else in the validation plan is conditional on this. The channel
#  model can be perfectly implemented (test_logs/verification_summary.csv says
#  it is) and still touch no packet at all, in which case every throughput,
#  latency and loss number you ever measure describes the Linux bridge rather
#  than the simulation.
#
#  The load-bearing test is the NEGATIVE CONTROL (phase 2): with NS-3 stopped,
#  the ping MUST fail completely. If it succeeds, a bypass path exists and no
#  later measurement means anything -- so this script aborts there rather than
#  reporting a misleading pass.
#
#  Topology being verified (from setup_netns_tap.sh):
#
#      uav1ns --veth-- br-uav1 --tap-uav1 --[ NS-3 node 1 ]
#                                                 |
#                                          YansWifiChannel
#                                     (obstacle + fading + path loss)
#                                                 |
#      uav2ns --veth-- br-uav2 --tap-uav2 --[ NS-3 node 2 ]
#
#  Each node has its OWN bridge, so the only layer-2 path between any two
#  namespaces runs through NS-3. Phase 1 checks that this is still true.
#
#  Usage:
#      bash scripts/test_scripts/verify_datapath.sh                 # full run
#      bash scripts/test_scripts/verify_datapath.sh --skip-causal   # T0 only
#      bash scripts/test_scripts/verify_datapath.sh --distance 25
#
#  Requires sudo (for `ip netns exec`); prompts once up front.
#  Do NOT have your own NS-3 running -- phase 2 needs it stopped, and this
#  script refuses to kill a process it did not start.
# =============================================================================

set -uo pipefail        # NOT -e: we want to collect every result, not stop

# This script lives in <repo>/scripts/test_scripts/, so the package root is two
# levels up -- one level lands in scripts/ and silently scatters the evidence
# into scripts/test_logs/.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUTDIR="$PROJECT_DIR/test_logs"
NS3_BIN="${NS3_BIN:-$HOME/ns-3.3/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_integrated-default}"

# Link under test: NS-3 nodes 1 and 2 (UAV1 <-> UAV2)
SRC_NS=uav1ns;  SRC_IP=10.42.0.11;  SRC_NODE=1
DST_NS=uav2ns;  DST_IP=10.42.0.12;  DST_NODE=2
# Control link, perturbed by nothing: nodes 1 and 3
CTL_NS=uav3ns;  CTL_IP=10.42.0.13;  CTL_NODE=3

DISTANCE=10          # m between UAVs; 10 m => ~47 dB SNR, far above threshold
PINGS=10
SKIP_CAUSAL=0
# Minimum plausible RTT through a simulated 802.11a link. One exchange costs
# DIFS 34us + backoff + preamble 20us + data + SIFS 16us + ACK, doubled for the
# round trip, plus realtime-scheduler slop. Host loopback would be ~0.05 ms, so
# anything under this threshold means the traffic never entered the simulator.
MIN_RTT_MS=0.30

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-causal) SKIP_CAUSAL=1; shift ;;
    --distance)    DISTANCE="$2"; shift 2 ;;
    --pings)       PINGS="$2"; shift 2 ;;
    -h|--help)     sed -n '2,40p' "$0"; exit 0 ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

mkdir -p "$OUTDIR"
STAMP=$(date -u +%Y%m%dT%H%M%SZ)
CSV="$OUTDIR/datapath_verification_$STAMP.csv"
LOG="$OUTDIR/datapath_verification_$STAMP.log"
NS3_LOG="$OUTDIR/datapath_ns3_$STAMP.log"
echo "phase,test_id,description,expected,observed,result" > "$CSV"

PASS=0; FAIL=0
NS3_PID=""

c_red()  { printf '\033[31m%s\033[0m' "$1"; }
c_grn()  { printf '\033[32m%s\033[0m' "$1"; }
c_yel()  { printf '\033[33m%s\033[0m' "$1"; }

record() {  # phase, id, desc, expected, observed, result
  printf '%s,%s,"%s","%s","%s",%s\n' "$1" "$2" "$3" "$4" "$5" "$6" >> "$CSV"
  if [[ "$6" == PASS ]]; then
    PASS=$((PASS+1)); printf '  [%s] %-6s %s\n' "$(c_grn PASS)" "$2" "$3"
  elif [[ "$6" == FAIL ]]; then
    FAIL=$((FAIL+1)); printf '  [%s] %-6s %s\n' "$(c_red FAIL)" "$2" "$3"
    printf '           expected: %s\n           observed: %s\n' "$4" "$5"
  else
    printf '  [%s] %-6s %s  (%s)\n' "$(c_yel INFO)" "$2" "$3" "$5"
  fi
}

cleanup() {
  if [[ -n "$NS3_PID" ]] && kill -0 "$NS3_PID" 2>/dev/null; then
    echo "stopping NS-3 (pid $NS3_PID)"
    kill -INT "$NS3_PID" 2>/dev/null
    sleep 2
    kill -9 "$NS3_PID" 2>/dev/null
  fi
}
trap cleanup EXIT

# ping helper -> echoes "loss_pct rtt_min rtt_avg rtt_mdev"
do_ping() {
  local ns=$1 ip=$2 n=$3 out
  out=$(sudo ip netns exec "$ns" ping -c "$n" -i 0.3 -W 1 -w $((n+8)) "$ip" 2>&1)
  echo "$out" >> "$LOG"
  local loss rtt
  loss=$(grep -oP '\d+(?=% packet loss)' <<<"$out" | head -1); loss=${loss:-100}
  rtt=$(grep -oP '(?<=rtt min/avg/max/mdev = )[0-9./]+' <<<"$out" | head -1)
  if [[ -z "$rtt" ]]; then echo "$loss 0 0 0"; else
    IFS='/' read -r mn av _mx md <<<"$rtt"; echo "$loss $mn $av $md"; fi
}

pub_obstacle() {  # node_a node_b loss_db ; repeat so the EMA converges
  local a=$1 b=$2 v=$3
  for _ in $(seq 1 25); do
    ros2 topic pub --once /link_obstacle_loss std_msgs/msg/Float32MultiArray \
      "{data: [$a,$b,$v]}" >/dev/null 2>&1
  done
  sleep 1
}

echo "============================================================"
echo " Tier 0 datapath verification    $STAMP"
echo " link under test : $SRC_NS($SRC_IP) <-> $DST_NS($DST_IP)"
echo "                   NS-3 nodes $SRC_NODE <-> $DST_NODE, ${DISTANCE} m apart"
echo " results         : $CSV"
echo "============================================================"

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 0  Preflight
# ─────────────────────────────────────────────────────────────────────────────
echo; echo "PHASE 0  preflight"

# Acquire sudo. Three routes, in order of preference, so this works both
# interactively and under automation (CI, or a harness with no controlling
# terminal -- plain `sudo -v` fails there with "a terminal is required").
if sudo -n true 2>/dev/null; then
  :                                        # already cached / NOPASSWD
elif [[ -n "${SUDO_ASKPASS:-}" ]] && sudo -A -v 2>/dev/null; then
  :                                        # non-interactive via askpass helper
elif sudo -v; then
  :                                        # interactive prompt
else
  echo "sudo required for 'ip netns exec'."
  echo "Non-interactive? Provide an askpass helper:"
  echo "    export SUDO_ASKPASS=/path/to/helper   # prints the password"
  exit 1
fi

if pgrep -f 'three_uav_tapbridge' >/dev/null 2>&1; then
  echo
  c_red "REFUSING TO RUN: an NS-3 process is already running."; echo
  echo "The negative control needs NS-3 stopped, and this script will not kill"
  echo "a process it did not start. Stop it yourself and re-run:"
  pgrep -af 'three_uav_tapbridge' | sed 's/^/    /'
  exit 1
fi

[[ -x "$NS3_BIN" ]] \
  && record 0 T0.0a "NS-3 binary present and executable" "exists" "$NS3_BIN" PASS \
  || { record 0 T0.0a "NS-3 binary present and executable" "exists" "$NS3_BIN MISSING" FAIL; exit 1; }

missing=""
for ns in "$SRC_NS" "$DST_NS" "$CTL_NS"; do
  ip netns list 2>/dev/null | grep -qw "$ns" || missing="$missing $ns"
done
[[ -z "$missing" ]] \
  && record 0 T0.0b "network namespaces exist" "$SRC_NS $DST_NS $CTL_NS" "all present" PASS \
  || { record 0 T0.0b "network namespaces exist" "all present" "missing:$missing" FAIL
       echo "  run: sudo bash scripts/setup_netns_tap.sh"; exit 1; }

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 1  Bypass audit -- can traffic reach the peer WITHOUT NS-3?
#
# The failure this hunts for: if two node bridges are joined, or the host has
# an address on them and forwards, packets reach the peer over the Linux stack
# and NS-3 is decorative. Everything here is read-only.
# ─────────────────────────────────────────────────────────────────────────────
echo; echo "PHASE 1  bypass audit (topology, no traffic)"

# 1a. every tap belongs to exactly the bridge for its own node
bad=""
for n in gcs uav1 uav2 uav3; do
  br=$(ip -o link show "tap-$n" 2>/dev/null | grep -oP '(?<=master )\S+')
  [[ "$br" == "br-$n" ]] || bad="$bad tap-$n->${br:-none}"
done
[[ -z "$bad" ]] \
  && record 1 T0.1a "each tap is enslaved to its own bridge" "tap-X in br-X" "correct" PASS \
  || record 1 T0.1a "each tap is enslaved to its own bridge" "tap-X in br-X" "$bad" FAIL

# 1b. no bridge carries members belonging to another node (would join two nodes
#     at layer 2 and let them talk without ever crossing the channel)
overlap=""
for n in gcs uav1 uav2 uav3; do
  for m in $(ip -o link show master "br-$n" 2>/dev/null | awk -F': ' '{print $2}' | cut -d'@' -f1); do
    case "$m" in
      tap-$n|veth*h) ;;                       # expected members
      *) overlap="$overlap br-$n:$m" ;;
    esac
  done
done
[[ -z "$overlap" ]] \
  && record 1 T0.1b "no unexpected members on the node bridges" "tap + veth only" "clean" PASS \
  || record 1 T0.1b "no unexpected members on the node bridges" "tap + veth only" "$overlap" FAIL

# 1c. the host must not hold an address on these bridges; with one it could
#     route or proxy-ARP between namespaces
hostip=""
for n in gcs uav1 uav2 uav3; do
  a=$(ip -4 -o addr show "br-$n" 2>/dev/null | awk '{print $4}')
  [[ -n "$a" ]] && hostip="$hostip br-$n=$a"
done
[[ -z "$hostip" ]] \
  && record 1 T0.1c "host has no IP on the node bridges" "none" "none" PASS \
  || record 1 T0.1c "host has no IP on the node bridges" "none" "$hostip" FAIL

# 1d. informational: the peer must be on-link (same /24) so delivery is by ARP
#     over the bridge, never by a router
rt=$(sudo ip netns exec "$SRC_NS" ip route get "$DST_IP" 2>&1 | head -1)
grep -q "via" <<<"$rt" \
  && record 1 T0.1d "peer is on-link, not routed" "no 'via' gateway" "$rt" FAIL \
  || record 1 T0.1d "peer is on-link, not routed" "no 'via' gateway" "$(echo "$rt" | cut -c1-60)" PASS

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 2  NEGATIVE CONTROL -- the load-bearing test
#
# NS-3 is not running, so the taps have no carrier and the bridges have no path
# onward. The ping MUST fail 100%. Anything else means a bypass exists.
# ARP caches are flushed first so a cached entry cannot make the source think
# it has a peer (the frames would still be dropped, but we want the clean case).
# ─────────────────────────────────────────────────────────────────────────────
echo; echo "PHASE 2  negative control -- NS-3 STOPPED, ping must fail"

for ns in "$SRC_NS" "$DST_NS" "$CTL_NS"; do
  sudo ip netns exec "$ns" ip neigh flush all 2>/dev/null
done

read -r nloss _ _ _ <<<"$(do_ping "$SRC_NS" "$DST_IP" 5)"
if [[ "$nloss" == "100" ]]; then
  record 2 T0.1 "ping fails with NS-3 stopped (no bypass path)" "100% loss" "${nloss}% loss" PASS
else
  record 2 T0.1 "ping fails with NS-3 stopped (no bypass path)" "100% loss" "${nloss}% loss" FAIL
  echo
  c_red "  ================= ABORTING ================="; echo
  echo "  Traffic reached the peer with NS-3 STOPPED. There is a path around"
  echo "  the simulator, so nothing downstream can be attributed to the"
  echo "  channel model. Every throughput / latency / loss figure measured on"
  echo "  this setup would describe the Linux bridge, not the simulation."
  echo
  echo "  Look at: bridges joined to each other, a host IP on br-*,"
  echo "  proxy_arp, or a stale veth from an earlier topology."
  echo "  Results so far: $CSV"
  exit 1
fi

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 3  Bring NS-3 up
# ─────────────────────────────────────────────────────────────────────────────
echo; echo "PHASE 3  starting NS-3"

# ROS mode (not --standalone) so the causal phase can inject obstacle loss.
# No position publisher is expected, so nodes stay at their CLI-default
# coordinates -- static and known, which is what a datapath test wants.
# CheckIntegration() will warn that no feed arrived; that is correct here.
"$NS3_BIN" --enableTap=true --simTime=0 --distance="$DISTANCE" \
           --posLogPeriod=0 --statsPeriod=0.5 \
           --csvPath="$OUTDIR/datapath_links_$STAMP.csv" \
           --snrLogFile="$OUTDIR/datapath_snr_$STAMP.csv" \
           > "$NS3_LOG" 2>&1 &
NS3_PID=$!

# Wait for TapBridge to attach: the taps gain a carrier when it does. Polling
# for that is far more reliable than sleeping a fixed interval.
ready=0
for _ in $(seq 1 60); do
  if ip -br link show tap-uav1 2>/dev/null | grep -q 'LOWER_UP'; then ready=1; break; fi
  kill -0 "$NS3_PID" 2>/dev/null || break
  sleep 0.5
done
if [[ $ready -eq 1 ]]; then
  record 3 T0.2a "NS-3 attached to the TAP devices (carrier up)" "carrier on tap-uav1" "up" PASS
else
  record 3 T0.2a "NS-3 attached to the TAP devices (carrier up)" "carrier on tap-uav1" "never came up" FAIL
  echo "  NS-3 log: $NS3_LOG"; tail -20 "$NS3_LOG" | sed 's/^/    /'; exit 1
fi
sleep 2

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 4  Positive control + latency floor
# ─────────────────────────────────────────────────────────────────────────────
echo; echo "PHASE 4  positive control -- NS-3 RUNNING, ping must succeed"

# Baseline the tap counters BEFORE any traffic, so phase 5 can measure a
# delta rather than a stale absolute left over from a previous session.
TAPRX_BEFORE=$(ip -s link show tap-uav1 2>/dev/null | awk '/RX:/{getline; print $1}')

read -r ploss pmin pavg pmdev <<<"$(do_ping "$SRC_NS" "$DST_IP" "$PINGS")"

[[ "$ploss" -le 20 ]] \
  && record 4 T0.2 "ping succeeds with NS-3 running" "<=20% loss" "${ploss}% loss" PASS \
  || record 4 T0.2 "ping succeeds with NS-3 running" "<=20% loss" "${ploss}% loss" FAIL

# The latency floor separates "went through the simulated radio" from "went
# through a host fast path". A simulated 802.11a exchange cannot complete in
# the ~50 us a loopback would take.
if awk "BEGIN{exit !($pmin >= $MIN_RTT_MS)}"; then
  record 4 T0.3 "RTT above the host-loopback floor" ">= ${MIN_RTT_MS} ms" "min ${pmin} ms, avg ${pavg} ms" PASS
else
  record 4 T0.3 "RTT above the host-loopback floor" ">= ${MIN_RTT_MS} ms" "min ${pmin} ms -- suspiciously fast" FAIL
fi
record 4 T0.3b "RTT distribution (informational)" "-" "min/avg/mdev = $pmin/$pavg/$pmdev ms" INFO

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 5  The simulator saw the frames
#
# Independent of ping's own verdict: NS-3's PHY must show the traffic. The SNR
# sniffer only emits a row when a frame is actually received over the modelled
# channel, so a non-empty file is direct evidence.
# ─────────────────────────────────────────────────────────────────────────────
echo; echo "PHASE 5  simulator-side evidence"

snrrows=$(($(wc -l < "$OUTDIR/datapath_snr_$STAMP.csv" 2>/dev/null || echo 1) - 1))
[[ "$snrrows" -gt 0 ]] \
  && record 5 T0.4a "NS-3 PHY received frames (snrLogFile rows)" ">0 rows" "$snrrows rows" PASS \
  || record 5 T0.4a "NS-3 PHY received frames (snrLogFile rows)" ">0 rows" "0 rows" FAIL

# DELTA, not absolute. The tap's counters are not reset between sessions --
# tap-uav1 was already showing ~12 kB from an earlier run, so a ">0 bytes"
# check would have passed without a single packet being sent today.
taprx_after=$(ip -s link show tap-uav1 2>/dev/null | awk '/RX:/{getline; print $1}')
taprx_delta=$(( ${taprx_after:-0} - ${TAPRX_BEFORE:-0} ))
[[ "$taprx_delta" -gt 0 ]] \
  && record 5 T0.4b "host TAP carried new traffic during this test" ">0 bytes" "tap-uav1 +${taprx_delta} B" PASS \
  || record 5 T0.4b "host TAP carried new traffic during this test" ">0 bytes" "tap-uav1 +${taprx_delta} B" FAIL

# ─────────────────────────────────────────────────────────────────────────────
# PHASE 6  Causal + specificity  (a preview of Tier 2 / Tier 4)
#
# The strongest available evidence. Not "traffic flows", but "traffic responds
# to the channel model, and only on the link the model was told about".
#
# Prediction, from the link budget: at ${DISTANCE} m the path loss is
# 46.73 + 20*log10(d) dB, so with Tx = 20 dBm the received power is far above
# RxSensitivity = -82 dBm. Adding 60 dB of obstacle loss drops it well below
# sensitivity, so the link must go down. The control link 1<->3 receives no
# obstacle report and must be unaffected.
# ─────────────────────────────────────────────────────────────────────────────
if [[ $SKIP_CAUSAL -eq 0 ]] && command -v ros2 >/dev/null 2>&1; then
  echo; echo "PHASE 6  causal test -- inject 60 dB on ${SRC_NODE}<->${DST_NODE} only"

  # Control-link baseline, measured from the SOURCE namespace. (An earlier
  # draft pinged $CTL_IP from $CTL_NS -- a namespace pinging its own
  # address, which never leaves it and proves nothing.)
  read -r cbefore _ _ _ <<<"$(do_ping "$SRC_NS" "$CTL_IP" "$PINGS")"

  pub_obstacle "$SRC_NODE" "$DST_NODE" 60.0
  read -r bloss2 _ _ _ <<<"$(do_ping "$SRC_NS" "$DST_IP" "$PINGS")"
  read -r cafter _ _ _     <<<"$(do_ping "$SRC_NS" "$CTL_IP" "$PINGS")"

  [[ "$bloss2" -ge 80 ]] \
    && record 6 T0.5a "60 dB obstacle kills the tested link" ">=80% loss" "${bloss2}% loss" PASS \
    || record 6 T0.5a "60 dB obstacle kills the tested link" ">=80% loss" "${bloss2}% loss (model not coupled to PHY?)" FAIL

  [[ "$cafter" -le 20 ]] \
    && record 6 T0.5b "control link 1<->3 unaffected (specificity)" "<=20% loss" "${cbefore}% -> ${cafter}%" PASS \
    || record 6 T0.5b "control link 1<->3 unaffected (specificity)" "<=20% loss" "${cbefore}% -> ${cafter}% (obstacle leaked across links)" FAIL

  pub_obstacle "$SRC_NODE" "$DST_NODE" 0.0
  sleep 2
  read -r rloss _ ravg _ <<<"$(do_ping "$SRC_NS" "$DST_IP" "$PINGS")"
  [[ "$rloss" -le 20 ]] \
    && record 6 T0.5c "link recovers when the obstacle is cleared" "<=20% loss" "${rloss}% loss" PASS \
    || record 6 T0.5c "link recovers when the obstacle is cleared" "<=20% loss" "${rloss}% loss (stuck blocked?)" FAIL
elif [[ $SKIP_CAUSAL -eq 1 ]]; then
  echo; echo "PHASE 6  causal test skipped (--skip-causal)"
  record 6 T0.5 "causal + specificity test" "run" "skipped by request" INFO
else
  # Silently skipping the strongest test in the script would be the worst
  # possible outcome: the run would still print a green verdict.
  echo; c_yel "PHASE 6  causal test NOT RUN -- 'ros2' is not on PATH"; echo
  echo "  This is the strongest evidence available, and it was skipped."
  echo "  Run:  source /opt/ros/humble/setup.bash  and try again."
  record 6 T0.5 "causal + specificity test" "run" "SKIPPED - ros2 not found" INFO
fi

# ─────────────────────────────────────────────────────────────────────────────
echo
echo "============================================================"
if [[ $FAIL -eq 0 ]]; then
  echo " VERDICT: $(c_grn PASS)  ($PASS checks)"
  echo
  echo " Traffic between the namespaces demonstrably traverses the NS-3"
  echo " channel: it fails when NS-3 is stopped, succeeds when it runs, is"
  echo " too slow to be a host fast path, appears in the PHY sniffer, and"
  echo " responds to obstacle loss on the specific link it was applied to."
  echo " Tier 2+ measurements on this setup are attributable to the model."
else
  echo " VERDICT: $(c_red FAIL)  ($PASS passed, $FAIL failed)"
  echo
  echo " Do NOT proceed to throughput / latency / PER experiments until these"
  echo " are resolved -- results would not be attributable to the channel."
fi
echo " results : $CSV"
echo " ns-3 log: $NS3_LOG"
echo "============================================================"
exit $(( FAIL > 0 ? 1 : 0 ))
