#!/bin/bash
# =============================================================================
# setup_netns_tap_patched.sh  — Two targeted fixes:
#
#  FIX 1: Give veth-uav2 and veth-uav3 their own IPs so UAV2/3 can
#          ping 10.42.0.1.  The "only assign IP to veth-uav1" logic
#          was wrong — Linux allows the same subnet on multiple interfaces,
#          it just can't have the SAME IP on two interfaces simultaneously.
#          Solution: each veth-uavN gets a unique IP in 10.42.0.0/24:
#            veth-uav1 = 10.42.0.201/24
#            veth-uav2 = 10.42.0.202/24
#            veth-uav3 = 10.42.0.203/24
#          The namespace default gw stays 10.42.0.1 — Linux picks the
#          correct outbound interface via the connected route on each veth.
#
#  FIX 2: TAP state check — TAP is DOWN until NS-3 opens it.
#          At setup time this is EXPECTED. Change the check from
#          [fail if DOWN] to [warn if DOWN, pass if no bridge master].
#          The real TAP-UP check happens in launch_multi_uav.sh step 4.
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

# ── GCS configuration ─────────────────────────────────────────────────────────
GCS_IP="10.42.0.10"           # IP of GCS node on NS-3 WiFi channel
GCS_VETH_ROOT_IP="10.42.0.209" # unique root-side IP for veth-gcs (avoids /24 ambiguity)

# ── Step 0: Teardown ──────────────────────────────────────────────────────────
step "Step 0 — Tearing down old state"

# Teardown GCS resources first
ip netns del   "gcsns"    2>/dev/null && info "Deleted namespace gcsns"  || true
ip link del    "tap-gcs"  2>/dev/null && info "Deleted tap-gcs"          || true
ip link del    "veth-gcs" 2>/dev/null && info "Deleted veth-gcs"         || true
ip route del "${GCS_IP}/32" 2>/dev/null || true

for i in $(seq 1 "${UAV_COUNT}"); do
  ip netns del   "uav${i}"       2>/dev/null && info "Deleted namespace uav${i}"   || true
  ip link del    "tap-uav${i}"   2>/dev/null && info "Deleted tap-uav${i}"         || true
  ip link del    "veth-uav${i}"  2>/dev/null && info "Deleted veth-uav${i}"        || true
  ip link del    "veth-host${i}" 2>/dev/null && info "Deleted veth-host${i}"       || true
  ip link del    "br-uav${i}"    2>/dev/null && info "Deleted legacy br-uav${i}"   || true
  # Remove stale /32 host routes from previous runs
  ip route del "10.42.0.$((10 + i))/32" 2>/dev/null || true
done
ip addr del 10.42.0.1/32 dev lo 2>/dev/null || true
pass "Old state cleaned up"

# ── Step 1: sysctl + iptables ─────────────────────────────────────────────────
step "Step 1 — Enabling IP forwarding + iptables MASQUERADE"
sysctl -w net.ipv4.ip_forward=1 >/dev/null
pass "ip_forward = 1"
if ! iptables -t nat -C POSTROUTING -s 10.42.0.0/24 -j MASQUERADE 2>/dev/null; then
  iptables -t nat -A POSTROUTING -s 10.42.0.0/24 -j MASQUERADE
fi
pass "iptables MASQUERADE ready for 10.42.0.0/24"

# ── Step 2a: GCS namespace + tap-gcs setup ──────────────────────────────────
step "Step 2a — Creating GCS namespace (gcsns) and tap-gcs"

FAIL_ITEMS=()
PASS_COUNT=0

{
  GCS_MAC="02:00:00:00:00:00"   # unique MAC for GCS node (UAVs use 01/02/03)

  info "────────────── GCS ──────────────"
  info "  Namespace : gcsns"
  info "  TAP       : tap-gcs  (standalone, MAC=${GCS_MAC})"
  info "  veth      : veth-gcs (${GCS_VETH_ROOT_IP}/24) ↔ eth0 (${GCS_IP}/24)"
  info "  default gw: 10.42.0.1 via eth0 (GCS sees NS-3 WiFi channel)"

  # 1. Create GCS namespace
  ip netns add gcsns
  ip netns exec gcsns ip link set lo up

  # 2. tap-gcs — standalone (NS-3 TapBridge will attach)
  ip tuntap add dev tap-gcs mode tap user "${OWNER_USER}"
  ip link set tap-gcs address "${GCS_MAC}"
  ip link set tap-gcs up
  ethtool -K tap-gcs tx off rx off 2>/dev/null || true

  # 3. veth pair: veth-gcs (root) ↔ eth0 (gcsns)
  ip link add veth-gcs type veth peer name eth0-gcs-tmp
  ip link set eth0-gcs-tmp netns gcsns
  ip netns exec gcsns ip link set eth0-gcs-tmp name eth0
  ip netns exec gcsns ip link set eth0 address "${GCS_MAC}"

  # Unique root-side IP so routing table doesn't ambiguously match other /24 entries
  ip addr add "${GCS_VETH_ROOT_IP}/24" dev veth-gcs
  ip link set veth-gcs up
  ethtool -K veth-gcs tx off 2>/dev/null || true

  # /32 host route: replies to GCS IP go back through veth-gcs (same fix as UAVs)
  ip route replace "${GCS_IP}/32" dev veth-gcs

  ip netns exec gcsns ip addr add "${GCS_IP}/24" dev eth0
  ip netns exec gcsns ip link set eth0 up
  ip netns exec gcsns ethtool -K eth0 tx off 2>/dev/null || true

  # Default route in gcsns via NS-3 WiFi subnet gateway
  ip netns exec gcsns ip route replace default via 10.42.0.1 dev eth0

  # Static ARP for gateway inside gcsns (10.42.0.1 is virtual — pin it to veth-gcs MAC)
  VETH_GCS_MAC=$(ip link show veth-gcs | awk '/ether/{print $2}')
  ip netns exec gcsns arp -s 10.42.0.1 "${VETH_GCS_MAC}" 2>/dev/null || true

  # TAP state check — DOWN is normal until NS-3 attaches
  TAP_MASTER=$(ip link show tap-gcs 2>/dev/null | grep -oP 'master \K\S+' || true)
  if [[ -n "${TAP_MASTER}" ]]; then
    fail "GCS: tap-gcs has bridge master '${TAP_MASTER}' — must be standalone"
    FAIL_ITEMS+=("GCS: tap-gcs has bridge master")
  else
    TAP_STATE=$(ip -br link show tap-gcs | awk '{print $2}')
    pass "GCS: tap-gcs standalone (no bridge master), state=${TAP_STATE}"
    pass "  Note: DOWN/NO-CARRIER is expected until NS-3 attaches"
    (( PASS_COUNT++ )) || true
  fi
}

# ── Step 2b: Per-UAV setup ────────────────────────────────────────────────────
step "Step 2b — Creating namespace + interface slices for ${UAV_COUNT} UAVs"

for i in $(seq 1 "${UAV_COUNT}"); do
  NS="uav${i}"
  TAP="tap-uav${i}"
  VETH_A_ROOT="veth-uav${i}"
  VETH_B_ROOT="veth-host${i}"
  MAC_SUFFIX=$(printf "%02x" "${i}")
  SHARED_MAC="02:00:00:00:00:${MAC_SUFFIX}"

  # ── FIX 1: unique IP per veth-uavN root side ──────────────────────────────
  # Each veth-uavN gets 10.42.0.20N/24  (unique, same subnet as namespaces).
  # UAVs default-gw to 10.42.0.1 — since each ns eth0 is on 10.42.0.0/24,
  # the kernel resolves 10.42.0.1 via the directly connected veth-uavN route.
  ETH0_ROOT_IP="10.42.0.$((200 + i))"    # 10.42.0.201 / .202 / .203
  ETH0_NS_IP="10.42.0.$((10 + i))"       # 10.42.0.11  / .12  / .13
  ETH0_GW="10.42.0.1"                    # default gw inside ns (stays same)
  ETH1_ROOT_IP="10.42.${i}.1"            # 10.42.1.1   / 2.1  / 3.1
  ETH1_NS_IP="10.42.${i}.2"             # 10.42.1.2   / 2.2  / 3.2

  info "────────────── UAV${i} ──────────────"
  info "  Namespace : ${NS}"
  info "  TAP       : ${TAP}  (standalone, MAC=${SHARED_MAC})"
  info "  veth-A    : ${VETH_A_ROOT} (${ETH0_ROOT_IP}/24) ↔ eth0 (${ETH0_NS_IP}/24)"
  info "  veth-B    : ${VETH_B_ROOT} (${ETH1_ROOT_IP}/24) ↔ eth1 (${ETH1_NS_IP}/24)"
  info "  default gw: ${ETH0_GW} via eth0"

  # 1. Create namespace
  ip netns add "${NS}"
  ip netns exec "${NS}" ip link set lo up

  # 2. TAP — standalone, no bridge
  ip tuntap add dev "${TAP}" mode tap user "${OWNER_USER}"
  ip link set "${TAP}" address "${SHARED_MAC}"
  ip link set "${TAP}" up
  ethtool -K "${TAP}" tx off rx off 2>/dev/null || true

  # 3. Veth pair A (NS-3 WiFi path)
  ip link add "${VETH_A_ROOT}" type veth peer name "eth0-tmp"
  ip link set "eth0-tmp" netns "${NS}"
  ip netns exec "${NS}" ip link set "eth0-tmp" name eth0
  ip netns exec "${NS}" ip link set eth0 address "${SHARED_MAC}"

  # ── FIX 1 applied here: always assign IP (unique per interface) ───────────
  ip addr add "${ETH0_ROOT_IP}/24" dev "${VETH_A_ROOT}"
  ip link set "${VETH_A_ROOT}" up
  ethtool -K "${VETH_A_ROOT}" tx off 2>/dev/null || true

  # ── FIX 3: /32 host route so root-ns replies go back the right veth ────────
  # Problem: veth-uav1/2/3 all share 10.42.0.0/24.  Linux picks veth-uav1
  # (first /24 match) when sending a reply to 10.42.0.12 or 10.42.0.13,
  # so uav2/uav3 never receive ICMP replies → ping fails.
  # Fix: add a /32 host route for each namespace IP pointing at its own veth.
  # /32 beats any /24, so replies always go through the correct peer.
  ip route replace "${ETH0_NS_IP}/32" dev "${VETH_A_ROOT}"

  ip netns exec "${NS}" ip addr add "${ETH0_NS_IP}/24" dev eth0
  ip netns exec "${NS}" ip link set eth0 up
  ip netns exec "${NS}" ethtool -K eth0 tx off 2>/dev/null || true

  # 4. Veth pair B (direct DDS path)
  ip link add "${VETH_B_ROOT}" type veth peer name "eth1-tmp"
  ip link set "eth1-tmp" netns "${NS}"
  ip netns exec "${NS}" ip link set "eth1-tmp" name eth1

  ip addr add "${ETH1_ROOT_IP}/24" dev "${VETH_B_ROOT}"
  ip link set "${VETH_B_ROOT}" up
  ethtool -K "${VETH_B_ROOT}" tx off 2>/dev/null || true

  ip netns exec "${NS}" ip addr add "${ETH1_NS_IP}/24" dev eth1
  ip netns exec "${NS}" ip link set eth1 up
  ip netns exec "${NS}" ethtool -K eth1 tx off 2>/dev/null || true

  # 5. Default route: via NS-3 path (eth0)
  ip netns exec "${NS}" ip route replace default via "${ETH0_GW}" dev eth0

  # 6. Static ARP for gateway inside namespace
  #    Without this, ARP for 10.42.0.1 may fail because no device
  #    is actually assigned that IP — it's a "virtual" gateway.
  #    Pinning it to the veth-A root MAC makes ARP resolve immediately.
  VETH_A_MAC=$(ip link show "${VETH_A_ROOT}" | awk '/ether/{print $2}')
  ip netns exec "${NS}" arp -s "${ETH0_GW}" "${VETH_A_MAC}" 2>/dev/null || true

  # ── FIX 2: TAP state check — DOWN is expected before NS-3 starts ─────────
  TAP_MASTER=$(ip link show "${TAP}" 2>/dev/null | grep -oP 'master \K\S+' || true)
  if [[ -n "${TAP_MASTER}" ]]; then
    fail "UAV${i}: ${TAP} has bridge master '${TAP_MASTER}' — must be standalone"
    FAIL_ITEMS+=("UAV${i}: TAP has bridge master")
  else
    # DOWN here is NORMAL — NS-3 hasn't opened the TAP fd yet
    TAP_STATE=$(ip -br link show "${TAP}" | awk '{print $2}')
    pass "UAV${i}: ${TAP} standalone (no bridge master), state=${TAP_STATE}"
    pass "  Note: DOWN/NO-CARRIER is expected until NS-3 attaches"
    (( PASS_COUNT++ )) || true
  fi
done

# ── Step 3: Connectivity tests ────────────────────────────────────────────────
step "Step 3 — Connectivity Tests"

for i in $(seq 1 "${UAV_COUNT}"); do
  NS="uav${i}"

  # Test 1: ping veth-A root IP (not .1, but the actual assigned IP .20N)
  VETH_A_IP="10.42.0.$((200 + i))"
  if ip netns exec "${NS}" ping -c2 -W2 "${VETH_A_IP}" >/dev/null 2>&1; then
    pass "UAV${i}: ping ${VETH_A_IP} (veth-A root IP) — OK"
    (( PASS_COUNT++ )) || true
  else
    fail "UAV${i}: ping ${VETH_A_IP} (veth-A root IP) — FAILED"
    FAIL_ITEMS+=("UAV${i}: ping veth-A ${VETH_A_IP}")
  fi

  # Test 2: ping veth-B gateway (direct DDS path)
  VETH_B_GW="10.42.${i}.1"
  if ip netns exec "${NS}" ping -c2 -W2 "${VETH_B_GW}" >/dev/null 2>&1; then
    pass "UAV${i}: ping ${VETH_B_GW} (veth-B gateway, DDS path) — OK"
    (( PASS_COUNT++ )) || true
  else
    fail "UAV${i}: ping ${VETH_B_GW} (veth-B gateway) — FAILED"
    FAIL_ITEMS+=("UAV${i}: ping veth-B ${VETH_B_GW}")
  fi

  # Test 3: default route is via eth0
  DEF_ROUTE=$(ip netns exec "${NS}" ip route show default | head -1)
  if echo "${DEF_ROUTE}" | grep -q "eth0"; then
    pass "UAV${i}: default route via eth0 — ${DEF_ROUTE}"
    (( PASS_COUNT++ )) || true
  else
    fail "UAV${i}: default route NOT via eth0 — got: '${DEF_ROUTE}'"
    FAIL_ITEMS+=("UAV${i}: wrong default route")
  fi
done

# ── Step 3 (cont): GCS connectivity test ─────────────────────────────────────
# Ping the veth-gcs root-side IP from inside gcsns
if ip netns exec gcsns ping -c2 -W2 "${GCS_VETH_ROOT_IP}" >/dev/null 2>&1; then
  pass "GCS: ping ${GCS_VETH_ROOT_IP} (veth-gcs root IP) — OK"
  (( PASS_COUNT++ )) || true
else
  fail "GCS: ping ${GCS_VETH_ROOT_IP} (veth-gcs root IP) — FAILED"
  FAIL_ITEMS+=("GCS: ping veth-gcs ${GCS_VETH_ROOT_IP}")
fi

# ── Summary ───────────────────────────────────────────────────────────────────
step "Summary"
# 1 TAP check + 3 connectivity per UAV (=4 each) + 1 GCS TAP + 1 GCS ping = UAV_COUNT*4 + 2
TOTAL=$((UAV_COUNT * 4 + 2))
echo ""
if [[ ${#FAIL_ITEMS[@]} -eq 0 ]]; then
  pass "All checks passed (${PASS_COUNT}/${TOTAL})"
  echo ""
  echo -e "${YELLOW}NOTE: tap-gcs and tap-uav1/2/3 will show DOWN until NS-3 is started."
  echo -e "      That is correct — the TAPs are ready for NS-3 to open.${RESET}"
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
echo "Next: run launch_multi_uav_netns.sh — NS-3 will open tap-gcs + tap-uav1/2/3"