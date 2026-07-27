#!/usr/bin/env bash

NUM_UAVS="${NUM_UAVS:-3}"

NUM_UAVS="${NUM_UAVS:-3}"

MIN_UAVS=3
MAX_UAVS=6


if ! [[ "$NUM_UAVS" =~ ^[0-9]+$ ]]; then
    echo "ERROR: NUM_UAVS must be an integer." >&2
    return 1 2>/dev/null || exit 1
fi

if (( NUM_UAVS < MIN_UAVS || NUM_UAVS > MAX_UAVS )); then
    echo "ERROR: NUM_UAVS must be between $MIN_UAVS and $MAX_UAVS." >&2
    return 1 2>/dev/null || exit 1
fi

GCS_WIRELESS_IP="10.42.0.10"

uav_wireless_ip() {
    local id="$1"
    echo "10.42.0.$((10 + id))"
}

uav_dds_port() {
    local id="$1"
    echo "$((2018 + id))"
}

uav_mavlink_port() {
    local id="$1"
    echo "$((5750 + 10 * id))"
}

uav_gazebo_fdm_port() {
    local id="$1"
    echo "$((8992 + 10 * id))"
}

uav_sitl_udp_port() {
    local id="$1"
    echo "$((8993 + 10 * id))"
}

uav_management_host_ip() {
    local id="$1"
    echo "172.31.${id}.1"
}

uav_management_namespace_ip() {
    local id="$1"
    echo "172.31.${id}.2"
}

uav_namespace() {
    local id="$1"
    echo "uav${id}"
}

uav_tap() {
    local id="$1"
    echo "tap-uav${id}"
}

uav_bridge() {
    local id="$1"
    echo "br-uav${id}"
}

uav_wireless_veth() {
    local id="$1"
    echo "veth-uav${id}-host"
}

uav_management_veth() {
    local id="$1"
    echo "sim-uav${id}-host"
}
