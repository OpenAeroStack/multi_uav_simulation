#!/bin/bash
# pi_hitl_link.sh — run ON THE RASPBERRY PI. Splits its single ethernet port
# into the two links the HITL experiment needs.
#
#   VLAN 10 -> 10.0.0.2/24    camera IN.  Unimpaired. This is the drone's
#                             internal camera cable, not a radio, so it must
#                             never enter ns-3.
#   VLAN 42 -> 10.42.0.12/24  detections OUT. This IS the radio: the host
#                             bridges the matching VLAN into br-uav2 ->
#                             tap-uav2 -> ns-3 -> tap-gcs -> gcsns, so every
#                             detection message crosses the simulated wireless
#                             channel and picks up its loss and latency.
#
# The Pi occupies the UAV2 slot of the 4-node ns-3 binary (node 0 = GCS,
# node 1 = SITL in uav1ns, node 2 = this Pi).
#
# Routing enforces the separation without any firewall rules: Gazebo lives on
# 10.0.0.1 and can only reach 10.0.0.2; gcsns lives on 10.42.0.10 and can only
# reach 10.42.0.12. Neither can take the other's path even by mistake.
#
# Run the host's launch script FIRST — it creates br-uav2 and the host-side
# VLANs. Then:   sudo bash pi_hitl_link.sh
#
# Non-persistent by design (ip commands, cleared on reboot). Make it permanent
# with netplan once the addresses are settled.

set -euo pipefail

PARENT="${PARENT:-eth0}"
CAM_VLAN=10
RF_VLAN=42
CAM_IP="10.0.0.2/24"
RF_IP="10.42.0.12/24"

[[ $EUID -eq 0 ]] || { echo "ERROR: run with sudo" >&2; exit 1; }
[[ -d "/sys/class/net/$PARENT" ]] || { echo "ERROR: no interface $PARENT" >&2; exit 1; }

echo "=== Splitting $PARENT into camera (VLAN $CAM_VLAN) + radio (VLAN $RF_VLAN) ==="
modprobe 8021q

# Any address left on the untagged parent would keep answering for the camera
# subnet and mask a broken VLAN, so move it rather than leaving it behind.
ip addr del "$CAM_IP" dev "$PARENT" 2>/dev/null || true

for spec in "$CAM_VLAN:$CAM_IP:camera (unimpaired)" "$RF_VLAN:$RF_IP:radio (via ns-3)"; do
    vid="${spec%%:*}"; rest="${spec#*:}"; addr="${rest%%:*}"; label="${rest#*:}"
    dev="$PARENT.$vid"
    ip link del "$dev" 2>/dev/null || true
    ip link add link "$PARENT" name "$dev" type vlan id "$vid"
    ip addr add "$addr" dev "$dev"
    ip link set "$dev" up
    printf '  %-10s %-16s %s\n' "$dev" "$addr" "$label"
done

ip link set "$PARENT" up
echo ""
echo "=== Verify ==="
ip -br addr show | grep -E "^$PARENT" | sed 's/^/  /'
echo ""
echo "  camera link :  ping -c2 10.0.0.1     (host, direct)"
echo "  radio link  :  ping -c2 10.42.0.10   (gcsns, THROUGH ns-3 — expect"
echo "                 higher latency and occasional loss; that is the point)"
