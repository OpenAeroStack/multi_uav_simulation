#!/bin/bash
# =============================================================================
# setup_netns_tap_bridged.sh
#
# small_city_ns3 branch — corrected network plumbing for the NS-3 TapBridge
# integration.
#
# WHAT THIS FIXES vs. the inherited setup_netns_tap_patched.sh:
#
#   The inherited script created, per UAV: a namespace, a veth pair
#   (veth-uavN <-> eth0 inside the namespace), and a standalone tap-uavN
#   (for NS-3's TapBridge to attach to). But nothing ever joined veth-uavN
#   and tap-uavN together. A packet leaving the namespace via eth0 arrives
#   at veth-uavN in the root namespace and has nowhere to go from there —
#   no bridge, no IP route to tap-uavN. It never reaches NS-3.
#
#   This version adds a Linux bridge (br-uavN) joining veth-uavN and
#   tap-uavN at L2, with NO ip address on either bridge member — matching
#   NS-3's documented TapBridge UseLocal requirement (the tap device must
#   not carry an IP; only the namespace-side interface does) and the
#   standard netns-to-tap-via-bridge pattern used elsewhere (e.g. the
#   CMU SEI ns-3/Docker tutorial).
#
#   Because there's now a real L2 path, several of the inherited script's
#   workarounds are no longer needed and have been REMOVED:
#     - unique IP per veth-uavN root side (10.42.0.20X)          -> removed
#     - static ARP pinning the "gateway" to the veth MAC          -> removed
#     - /32 host routes for reply routing                        -> removed
#     - iptables MASQUERADE                                      -> removed
#   Each namespace just gets a normal IP on eth0 in 10.42.0.0/24 and ARPs
#   directly for other nodes; NS-3's ad-hoc wifi carries broadcast/ARP like
#   any other L2 segment.
#
# VALIDATION BEFORE TRUSTING THIS:
#   After running this script (before starting NS-3), prove the bridge
#   relays frames with zero simulation involvement:
#
#     sudo tcpdump -i veth-uav1 -n &
#     sudo tcpdump -i tap-uav1 -n &
#     sudo ip netns exec uav1 ping -c3 10.42.0.99   # unreachable IP is fine,
#                                                     # we just want to see
#                                                     # the ARP broadcast
#
#   You should see the identical ARP request appear on BOTH veth-uav1 and
#   tap-uav1. If it does, the L2 relay is proven correct before NS-3 or
#   SITL ever enter the picture.
#
# =============================================================================

set -euo pipefail

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
pass() { echo -e "${GREEN}[✓]${RESET} $*"; }
fail() { echo -e "${RED}[✗]${RESET} $*"; }
info() { echo -e "${CYAN}[i]${RESET} $*"; }
warn() { echo -e "${YELLOW}[!]${RESET} $*"; }
step() { echo -e "\n${BOLD}${YELLOW}══ $* ══${RESET}"; }

[[ "${EUID}" -ne 0 ]] && exec sudo -E "$0" "$@"

OWNER_USER="${SUDO_USER:-${USER}}"
UAV_COUNT=3

GCS_IP="10.42.0.10"

# ── Step 0: Teardown ──────────────────────────────────────────────────────────
step "Step 0 — Tearing down old state"

ip netns del   "gcsns"    2>/dev/null && info "Deleted namespace gcsns"  || true
ip link del    "tap-gcs"  2>/dev/null && info "Deleted tap-gcs"          || true
ip link del    "veth-gcs" 2>/dev/null && info "Deleted veth-gcs"         || true
ip link del    "br-gcs"   2>/dev/null && info "Deleted br-gcs"           || true

for i in $(seq 1 "${UAV_COUNT}"); do
  ip netns del   "uav${i}"       2>/dev/null && info "Deleted namespace uav${i}"   || true
  ip link del    "tap-uav${i}"   2>/dev/null && info "Deleted tap-uav${i}"         || true
  ip link del    "veth-uav${i}"  2>/dev/null && info "Deleted veth-uav${i}"        || true
  ip link del    "br-uav${i}"    2>/dev/null && info "Deleted br-uav${i}"          || true
  # clean up any stale artifacts from the pre-bridge version of this script
  ip link del    "veth-host${i}" 2>/dev/null || true
  ip route del "10.42.0.$((10 + i))/32" 2>/dev/null || true
done
ip route del "${GCS_IP}/32" 2>/dev/null || true
# remove leftover MASQUERADE rule from the pre-bridge script, if present
iptables -t nat -D POSTROUTING -s 10.42.0.0/24 -j MASQUERADE 2>/dev/null || true
pass "Old state cleaned up"

# Note: no ip_forward / MASQUERADE needed anymore — there is no IP routing
# happening between namespaces and root. It's pure L2 bridging now.

# ── Step 1: GCS namespace + tap-gcs + bridge ──────────────────────────────────
step "Step 1 — Creating GCS namespace (gcsns), tap-gcs, and br-gcs"

FAIL_ITEMS=()
PASS_COUNT=0

{
  GCS_MAC="02:00:00:00:00:00"

  info "────────────── GCS ──────────────"
  info "  Namespace : gcsns"
  info "  TAP       : tap-gcs   (no IP — NS-3 TapBridge UseLocal requirement)"
  info "  Bridge    : br-gcs    joins veth-gcs <-> tap-gcs, no IP on either member"
  info "  eth0(gcsns): ${GCS_IP}/24 — the only IP in this segment on the root side"

  ip netns add gcsns
  ip netns exec gcsns ip link set lo up

  # tap-gcs — standalone, NS-3 TapBridge will attach in UseLocal mode.
  # Deliberately no ip address on this device.
  ip tuntap add dev tap-gcs mode tap user "${OWNER_USER}"
  ip link set tap-gcs address "${GCS_MAC}"
  ip link set tap-gcs up
  ethtool -K tap-gcs tx off rx off 2>/dev/null || true

  # veth pair: veth-gcs (root) <-> eth0 (gcsns)
  ip link add veth-gcs type veth peer name eth0-gcs-tmp
  ip link set eth0-gcs-tmp netns gcsns
  ip netns exec gcsns ip link set eth0-gcs-tmp name eth0
  ip netns exec gcsns ip link set eth0 address "${GCS_MAC}"

  # veth-gcs (root side) gets NO ip address — it's a bridge member now.
  ip link set veth-gcs up
  ethtool -K veth-gcs tx off 2>/dev/null || true

  # Bridge joining veth-gcs and tap-gcs at L2. This is the fix.
  ip link add br-gcs type bridge
  ip link set veth-gcs master br-gcs
  ip link set tap-gcs master br-gcs
  ip link set br-gcs up

  # Only the namespace side carries an IP.
  ip netns exec gcsns ip addr add "${GCS_IP}/24" dev eth0
  ip netns exec gcsns ip link set eth0 up
  ip netns exec gcsns ethtool -K eth0 tx off 2>/dev/null || true

  # No default route / gateway needed inside gcsns for talking to UAVs —
  # they're all on the same 10.42.0.0/24 L2 segment via the bridge + NS-3
  # wifi channel. Normal ARP resolves everything directly.

  BR_STATE=$(ip -br link show br-gcs | awk '{print $2}')
  pass "GCS: br-gcs created (state=${BR_STATE}), members=veth-gcs+tap-gcs"
  pass "  Note: tap-gcs and br-gcs may show DOWN/NO-CARRIER for tap-gcs until NS-3 attaches"
  (( PASS_COUNT++ )) || true
}

# ── Step 2: Per-UAV setup ─────────────────────────────────────────────────────
step "Step 2 — Creating namespace + bridge + tap for ${UAV_COUNT} UAVs"

for i in $(seq 1 "${UAV_COUNT}"); do
  NS="uav${i}"
  TAP="tap-uav${i}"
  VETH_ROOT="veth-uav${i}"
  BR="br-uav${i}"
  MAC_SUFFIX=$(printf "%02x" "${i}")
  SHARED_MAC="02:00:00:00:00:${MAC_SUFFIX}"
  ETH0_NS_IP="10.42.0.$((10 + i))"

  info "────────────── UAV${i} ──────────────"
  info "  Namespace : ${NS}"
  info "  TAP       : ${TAP}  (no IP)"
  info "  Bridge    : ${BR}   joins ${VETH_ROOT} <-> ${TAP}, no IP on either member"
  info "  eth0(${NS}): ${ETH0_NS_IP}/24"

  ip netns add "${NS}"
  ip netns exec "${NS}" ip link set lo up

  # TAP — standalone, no IP, NS-3 TapBridge attaches here.
  ip tuntap add dev "${TAP}" mode tap user "${OWNER_USER}"
  ip link set "${TAP}" address "${SHARED_MAC}"
  ip link set "${TAP}" up
  ethtool -K "${TAP}" tx off rx off 2>/dev/null || true

  # Veth pair: veth-uavN (root) <-> eth0 (namespace)
  ip link add "${VETH_ROOT}" type veth peer name "eth0-tmp"
  ip link set "eth0-tmp" netns "${NS}"
  ip netns exec "${NS}" ip link set "eth0-tmp" name eth0
  ip netns exec "${NS}" ip link set eth0 address "${SHARED_MAC}"

  # veth-uavN (root side) gets NO ip address — bridge member.
  ip link set "${VETH_ROOT}" up
  ethtool -K "${VETH_ROOT}" tx off 2>/dev/null || true

  # Bridge joining veth-uavN and tap-uavN. This is the fix — the missing
  # piece from the inherited script. No IP on the bridge itself either;
  # it's a pure L2 relay.
  ip link add "${BR}" type bridge
  ip link set "${VETH_ROOT}" master "${BR}"
  ip link set "${TAP}" master "${BR}"
  ip link set "${BR}" up

  # Only the namespace side carries an IP.
  ip netns exec "${NS}" ip addr add "${ETH0_NS_IP}/24" dev eth0
  ip netns exec "${NS}" ip link set eth0 up
  ip netns exec "${NS}" ethtool -K eth0 tx off 2>/dev/null || true

  # No default route needed — GCS and other UAVs are on the same
  # 10.42.0.0/24 L2 segment (via bridge -> tap -> NS-3 wifi channel).
  # Direct ARP resolves them; no gateway fiction required.

  BR_STATE=$(ip -br link show "${BR}" | awk '{print $2}')
  pass "UAV${i}: ${BR} created (state=${BR_STATE}), members=${VETH_ROOT}+${TAP}"
  pass "  Note: ${TAP} may show DOWN/NO-CARRIER until NS-3 attaches — expected"
  (( PASS_COUNT++ )) || true
done

# ── Step 3: Bridge membership verification (no NS-3, no IP routing involved) ──
step "Step 3 — Verifying bridge membership (structural check only)"

check_bridge_member() {
  local br="$1" dev="$2" label="$3"
  local master
  master=$(ip link show "${dev}" 2>/dev/null | grep -oP 'master \K\S+' || true)
  if [[ "${master}" == "${br}" ]]; then
    pass "${label}: ${dev} is a member of ${br}"
    (( PASS_COUNT++ )) || true
  else
    fail "${label}: ${dev} is NOT a member of ${br} (master='${master:-none}')"
    FAIL_ITEMS+=("${label}: ${dev} not in ${br}")
  fi
}

check_bridge_member "br-gcs" "veth-gcs" "GCS"
check_bridge_member "br-gcs" "tap-gcs"  "GCS"
for i in $(seq 1 "${UAV_COUNT}"); do
  check_bridge_member "br-uav${i}" "veth-uav${i}" "UAV${i}"
  check_bridge_member "br-uav${i}" "tap-uav${i}"  "UAV${i}"
done

# Note: we deliberately do NOT ping-test here. tap-uavN/tap-gcs are not
# opened by anything yet (NS-3 hasn't attached), so there is no far end to
# reply. Bridge membership is the correct thing to verify at this stage.
# The tcpdump-based ARP-broadcast check described in the file header is the
# real end-to-end proof, and it should be run manually once, once, before
# building anything further on top of this script.

# ── Summary ───────────────────────────────────────────────────────────────────
step "Summary"
TOTAL=$(( (UAV_COUNT + 1) * 2 ))
echo ""
if [[ ${#FAIL_ITEMS[@]} -eq 0 ]]; then
  pass "All structural checks passed (${PASS_COUNT}/${TOTAL})"
  echo ""
  echo -e "${YELLOW}NOTE: tap-gcs and tap-uav1/2/3 will show DOWN until NS-3 is started."
  echo -e "      That is correct — the TAPs are ready for NS-3 to open.${RESET}"
  echo ""
  echo -e "${CYAN}Before proceeding to Phase 4 (attaching NS-3), run the manual tcpdump"
  echo -e "ARP-broadcast check described at the top of this script to prove the"
  echo -e "L2 relay works end-to-end.${RESET}"
else
  echo -e "${RED}${#FAIL_ITEMS[@]} FAILED:${RESET}"
  for item in "${FAIL_ITEMS[@]}"; do
    fail "  ${item}"
  done
  echo ""
  echo "Passed: ${PASS_COUNT} / ${TOTAL}"
  exit 1
fi

echo ""
echo "Next: run the manual bridge-relay validation (see file header), then"
echo "proceed to attaching NS-3 (three-uav scratch program) to the taps."