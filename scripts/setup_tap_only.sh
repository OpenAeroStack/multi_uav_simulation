#!/bin/bash
# =============================================================================
# setup_tap_only.sh
#
# Replaces setup_netns_tap_patched.sh for the "realistic ns-3 channel" flow.
#
# Why: the netns/veth/MASQUERADE setup builds a path from each namespace's
# eth0 out to the real internet — it never touches tap-gcs/tap-uav1-3 at all
# (the old br-uavN bridging that would have connected them was removed).
# So today ns-3's Nakagami/log-distance model sees zero application traffic.
#
# Fix: skip namespaces entirely for this link. Give each TAP device the real
# application IP directly, in the SAME namespace ns-3 runs in (ns-3's
# TapBridge can only attach to TAPs that live in its own netns). Everything
# that needs the wireless-degraded link (GCS <-> drone_bridge ROS 2 traffic)
# talks over these IPs; everything else (Gazebo/SITL/DDS-agent/MAVLink)
# stays on loopback, untouched.
#
# Run this BEFORE starting ns-3. ns-3 will show the TAPs as DOWN/NO-CARRIER
# until it attaches — that's expected, same as before.
# =============================================================================
 
set -euo pipefail
 
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; RESET='\033[0m'
pass() { echo -e "${GREEN}[✓]${RESET} $*"; }
info() { echo -e "${CYAN}[i]${RESET} $*"; }
 
[[ "${EUID}" -ne 0 ]] && exec sudo -E "$0" "$@"
 
# Owner must match whichever user actually runs `./ns3 run ...`
# (tuntap "user" grants that user permission to open the tap fd without sudo).
OWNER="${SUDO_USER:-${USER}}"
 
# tap-name -> app IP (matches Ipv4AddressHelper base 10.42.0.0/24 starting .10
# in three_uav_tapbridge_rt.cc, and the GCS_LAT/etc code in city_mission.py)
declare -A IP=(
  [tap-gcs]=10.42.0.10
  [tap-uav1]=10.42.0.11
  [tap-uav2]=10.42.0.12
  [tap-uav3]=10.42.0.13
)
declare -A MAC=(
  [tap-gcs]=02:00:00:00:00:00
  [tap-uav1]=02:00:00:00:00:01
  [tap-uav2]=02:00:00:00:00:02
  [tap-uav3]=02:00:00:00:00:03
)
 
info "Tearing down old TAPs / stale netns state (if any)"
for t in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
  ip link del "$t" 2>/dev/null || true
done
# Clean up leftovers from the old netns approach so nothing shadows 10.42.0.0/24
for ns in gcsns uav1 uav2 uav3; do
  ip netns del "$ns" 2>/dev/null || true
done
for v in veth-gcs veth-uav1 veth-uav2 veth-uav3; do
  ip link del "$v" 2>/dev/null || true
done
iptables -t nat -D POSTROUTING -s 10.42.0.0/24 -j MASQUERADE 2>/dev/null || true
 
info "Creating TAP devices (owner=${OWNER})"
for t in tap-gcs tap-uav1 tap-uav2 tap-uav3; do
  ip tuntap add dev "$t" mode tap user "${OWNER}"
  ip link set "$t" address "${MAC[$t]}"
  ip addr add "${IP[$t]}/24" dev "$t"
  ip link set "$t" up
  ethtool -K "$t" tx off rx off 2>/dev/null || true
  pass "$t = ${IP[$t]} (mac=${MAC[$t]})"
done
 
echo ""
echo "NOTE: all 4 TAPs will show DOWN/NO-CARRIER until ns-3 opens them via"
echo "      TapBridge — that's correct. Start ns-3 next."
 
