#!/usr/bin/env bash
set -Eeuo pipefail

RUN_USER="${SUDO_USER:-$USER}"
RESET=0
[[ "${1:-}" == "--reset" ]] && RESET=1
[[ $# -le 1 ]] || { echo "Usage: $0 [--reset]" >&2; exit 2; }

sudo -v

nodes=(gcs uav1 uav2 uav3)
namespaces=(gcsns uav1ns uav2ns uav3ns)
addresses=(10.42.0.10/24 10.42.0.11/24 10.42.0.12/24 10.42.0.13/24)

if (( RESET )); then
    for index in "${!nodes[@]}"; do
        node="${nodes[$index]}"
        namespace="${namespaces[$index]}"
        sudo ip netns del "$namespace" 2>/dev/null || true
        sudo ip link del "br-$node" 2>/dev/null || true
        sudo ip link del "tap-$node" 2>/dev/null || true
        sudo ip link del "veth${index}h" 2>/dev/null || true
    done
fi

# Refuse before creating anything if any part of a non-reset topology remains.
for index in "${!nodes[@]}"; do
    node="${nodes[$index]}"
    namespace="${namespaces[$index]}"
    if sudo ip netns list | awk '{print $1}' | grep -Fxq "$namespace" ||
       ip link show "tap-$node" >/dev/null 2>&1 ||
       ip link show "br-$node" >/dev/null 2>&1 ||
       ip link show "veth${index}h" >/dev/null 2>&1; then
        echo "ERROR: topology component for $node already exists; use --reset for an explicit scoped rebuild." >&2
        exit 1
    fi
done

for index in "${!nodes[@]}"; do
    node="${nodes[$index]}"
    namespace="${namespaces[$index]}"
    address="${addresses[$index]}"
    tap="tap-$node"
    bridge="br-$node"
    host_veth="veth${index}h"
    namespace_veth="veth${index}n"

    sudo ip netns add "$namespace"
    sudo ip tuntap add dev "$tap" mode tap user "$RUN_USER"
    sudo ip link add "$bridge" type bridge
    sudo ip link set "$tap" master "$bridge"
    sudo ip link set "$tap" up
    sudo ip link set "$bridge" up
    sudo ip link add "$host_veth" type veth peer name "$namespace_veth"
    sudo ip link set "$host_veth" master "$bridge"
    sudo ip link set "$host_veth" up
    sudo ip link set "$namespace_veth" netns "$namespace"
    sudo ip netns exec "$namespace" ip link set lo up
    sudo ip netns exec "$namespace" ip link set "$namespace_veth" up
    sudo ip netns exec "$namespace" ip addr add "$address" dev "$namespace_veth"
    sudo ethtool -K "$host_veth" rx off tx off sg off tso off gso off gro off 2>/dev/null || true
    sudo ip netns exec "$namespace" ethtool -K "$namespace_veth" \
        rx off tx off sg off tso off gso off gro off 2>/dev/null || true
done

echo "PASS: provisioned four isolated namespace/bridge/TAP paths."
echo "  gcsns=10.42.0.10  uav1ns=.11  uav2ns=.12  uav3ns=.13"
echo "  Inter-node communication is impossible until NS-3 attaches all four TAPs."
