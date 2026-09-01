#!/usr/bin/env bash

# ============================================================
# Dynamic ns-3 Linux Wireless Topology Setup
#
# Creates:
#
#   gcsns
#   uav1 ... uavN
#
# Wireless side:
#
#   gcsns:wifi0       -> 10.42.0.10
#   uav1:wifi0        -> 10.42.0.11
#   uav2:wifi0        -> 10.42.0.12
#   ...
#
# TAP/bridge side:
#
#   br-gcs  <-> tap-gcs
#   br-uav1 <-> tap-uav1
#   br-uav2 <-> tap-uav2
#   ...
#
# Gazebo/SITL simulation side:
#
#   root sim-uav1-host 172.31.1.1
#          |
#          +---- uav1:sim0 172.31.1.2
#
#   root sim-uav2-host 172.31.2.1
#          |
#          +---- uav2:sim0 172.31.2.2
#
# IMPORTANT:
#
# The root namespace receives NO 10.42.0.x IP address.
#
# This is intentional.
#
# Communication between gcsns and uavN must eventually pass
# through ns-3 via the TAP devices.
#
# Usage:
#
#   sudo NS3_USER="$USER" \
#       ./scripts/setup_ns3_wireless_topology.sh
#
# ============================================================

set -euo pipefail


# ============================================================
# Terminal helpers
# ============================================================

info()
{
    echo "[network] $*"
}


warn()
{
    echo "[network] WARNING: $*" >&2
}


die()
{
    echo "[network] ERROR: $*" >&2
    exit 1
}


# ============================================================
# Root check
# ============================================================

if [[ "$EUID" -ne 0 ]]; then

    die "This script must run as root.

Run:

    sudo NS3_USER=\"\$USER\" $0"
fi


# ============================================================
# Resolve project directory
# ============================================================

SCRIPT_DIR="$(
    cd "$(dirname "${BASH_SOURCE[0]}")"
    pwd
)"


# Support both:
#
# project/scripts/setup...
#
# and:
#
# project/setup...
#
if [[ -f "$SCRIPT_DIR/../generated/fleet.json" ]]; then

    PROJECT_DIR="$(
        cd "$SCRIPT_DIR/.."
        pwd
    )"

elif [[ -f "$SCRIPT_DIR/generated/fleet.json" ]]; then

    PROJECT_DIR="$SCRIPT_DIR"

else

    PROJECT_DIR="$(
        cd "$SCRIPT_DIR/.."
        pwd
    )"
fi


FLEET_JSON="${FLEET_JSON:-$PROJECT_DIR/generated/fleet.json}"


# ============================================================
# User that will later run ns-3
# ============================================================

NS3_USER="${NS3_USER:-${SUDO_USER:-}}"


if [[ -z "$NS3_USER" ]]; then

    die "NS3_USER could not be determined.

Run:

    sudo NS3_USER=\"\$USER\" $0"
fi


if ! id "$NS3_USER" >/dev/null 2>&1; then

    die "User does not exist: $NS3_USER"
fi


# ============================================================
# Dependency checks
# ============================================================

for command in \
    ip \
    bridge \
    python3
do

    command -v "$command" >/dev/null 2>&1 ||
        die "Required command not found: $command"

done


if [[ ! -f "$FLEET_JSON" ]]; then

    die "Generated fleet configuration not found:

    $FLEET_JSON

Generate it first:

    cd $PROJECT_DIR
    python3 scripts/generate_fleet.py --clean"
fi


info "Project directory : $PROJECT_DIR"
info "Fleet manifest    : $FLEET_JSON"
info "ns-3 user         : $NS3_USER"


# ============================================================
# Read global configuration from fleet.json
# ============================================================

eval "$(
python3 - "$FLEET_JSON" <<'PY'

import ipaddress
import json
import shlex
import sys


path = sys.argv[1]


with open(path, "r", encoding="utf-8") as f:

    data = json.load(f)


network = ipaddress.ip_network(
    data["network"]["wireless_subnet"],
    strict=False
)


gcs = data["gcs"]


values = {

    "NUM_UAVS":
        int(data["num_uavs"]),

    "WIRELESS_SUBNET":
        str(network),

    "WIRELESS_PREFIX":
        int(network.prefixlen),

    "GCS_NS":
        gcs["namespace"],

    "GCS_TAP":
        gcs["tap"],

    "GCS_IP":
        gcs["wireless_ip"],
}


for key, value in values.items():

    print(
        f"{key}={shlex.quote(str(value))}"
    )
PY
)"


if (( NUM_UAVS < 1 )); then

    die "NUM_UAVS must be at least 1."
fi


info "UAV count         : $NUM_UAVS"
info "Wireless subnet   : $WIRELESS_SUBNET"
info "GCS namespace     : $GCS_NS"
info "GCS wireless IP   : $GCS_IP"


# ============================================================
# Load per-UAV configuration
#
# Columns:
#
# id
# namespace
# tap
# wireless_ip
# sim_host_ip
# sim_namespace_ip
# ============================================================

mapfile -t UAV_ROWS < <(

python3 - "$FLEET_JSON" <<'PY'

import ipaddress
import json
import sys


path = sys.argv[1]


with open(path, "r", encoding="utf-8") as f:

    data = json.load(f)


for uav in data["uavs"]:

    uav_id = int(
        uav["id"]
    )

    namespace = str(
        uav["namespace"]
    )

    tap = str(
        uav["tap"]
    )

    wireless_ip = str(
        uav["wireless_ip"]
    )

    # In our current architecture fleet.json stores:
    #
    # UAV1 sim_ip = 172.31.1.1
    # UAV2 sim_ip = 172.31.2.1
    #
    # This is the ROOT/Gazebo side address.
    sim_host_ip = ipaddress.ip_address(
        uav["sim_ip"]
    )

    # Namespace/SITL side receives the next address.
    sim_namespace_ip = (
        sim_host_ip + 1
    )


    print(
        "\t".join(
            [
                str(uav_id),
                namespace,
                tap,
                wireless_ip,
                str(sim_host_ip),
                str(sim_namespace_ip),
            ]
        )
    )
PY
)


if [[ "${#UAV_ROWS[@]}" -ne "$NUM_UAVS" ]]; then

    die "fleet.json reports $NUM_UAVS UAVs but contains ${#UAV_ROWS[@]} UAV entries."
fi


# ============================================================
# Linux interface-name length validation
#
# Linux IFNAMSIZ gives us effectively 15 visible characters.
#
# Current naming is safe for our present target scale
# (approximately <= 10 UAVs).
# ============================================================

validate_ifname()
{
    local name="$1"

    if (( ${#name} > 15 )); then

        die "Linux interface name is too long:

    $name

Maximum supported visible length is 15 characters.

For much larger UAV IDs we will later use shorter dynamic
interface names."
    fi
}


# ============================================================
# Namespace cleanup helper
# ============================================================

delete_namespace()
{
    local namespace="$1"

    if ! ip netns list |
        awk '{print $1}' |
        grep -qx "$namespace"
    then
        return 0
    fi


    info "Removing old namespace: $namespace"


    mapfile -t namespace_pids < <(
        ip netns pids "$namespace" 2>/dev/null || true
    )


    if [[ "${#namespace_pids[@]}" -gt 0 ]]; then

        info "Stopping processes in $namespace: ${namespace_pids[*]}"

        kill -TERM "${namespace_pids[@]}" \
            2>/dev/null || true

        sleep 0.5


        for pid in "${namespace_pids[@]}"; do

            if kill -0 "$pid" 2>/dev/null; then

                kill -KILL "$pid" \
                    2>/dev/null || true
            fi

        done
    fi


    ip netns del "$namespace" \
        2>/dev/null || true
}


# ============================================================
# Root-interface cleanup helper
# ============================================================

delete_root_interface()
{
    local interface="$1"


    if ! ip link show dev "$interface" \
        >/dev/null 2>&1
    then
        return 0
    fi


    info "Removing old interface: $interface"


    ip link set dev "$interface" down \
        2>/dev/null || true


    ip link delete dev "$interface" \
        2>/dev/null || true
}


# ============================================================
# Clean previous dynamic topology
#
# This removes ALL interfaces/namespaces created by this
# project's naming convention, including UAVs from an earlier
# run with a larger fleet.
#
# Example:
#
# previous run: N=5
# current run : N=3
#
# uav4 and uav5 must not remain.
# ============================================================

cleanup_previous_topology()
{
    info "Cleaning previous dynamic ns-3 topology..."


    # --------------------------------------------------------
    # Namespaces
    # --------------------------------------------------------

    while read -r namespace; do

        [[ -n "$namespace" ]] || continue


        if [[ "$namespace" == "gcsns" ]] ||
           [[ "$namespace" =~ ^uav[0-9]+$ ]]
        then

            delete_namespace "$namespace"
        fi

    done < <(
        ip netns list |
        awk '{print $1}'
    )


    # --------------------------------------------------------
    # Root interfaces
    # --------------------------------------------------------

    mapfile -t root_interfaces < <(

        ip -o link show |
        awk -F': ' '{print $2}' |
        cut -d'@' -f1
    )


    # Delete non-bridge interfaces first.
    for interface in "${root_interfaces[@]}"; do

        if [[ "$interface" == "tap-gcs" ]] ||
           [[ "$interface" =~ ^tap-uav[0-9]+$ ]] ||
           [[ "$interface" == "veth-gcs-host" ]] ||
           [[ "$interface" =~ ^veth-uav[0-9]+-host$ ]] ||
           [[ "$interface" =~ ^sim-uav[0-9]+-host$ ]]
        then

            delete_root_interface "$interface"
        fi

    done


    # Re-read because deleting veths changes the link list.
    mapfile -t root_interfaces < <(

        ip -o link show |
        awk -F': ' '{print $2}' |
        cut -d'@' -f1
    )


    # Delete bridges afterwards.
    for interface in "${root_interfaces[@]}"; do

        if [[ "$interface" == "br-gcs" ]] ||
           [[ "$interface" =~ ^br-uav[0-9]+$ ]]
        then

            delete_root_interface "$interface"
        fi

    done


    info "Previous topology cleanup complete."
}


# ============================================================
# Create one isolated bridge
# ============================================================

create_bridge()
{
    local bridge_name="$1"


    validate_ifname "$bridge_name"


    info "Creating bridge: $bridge_name"


    ip link add \
        name "$bridge_name" \
        type bridge


    # Disable STP.
    ip link set \
        dev "$bridge_name" \
        type bridge \
        stp_state 0 \
        forward_delay 0


    ip link set \
        dev "$bridge_name" \
        up
}


# ============================================================
# Create TAP owned by the user that will run ns-3
# ============================================================

create_tap()
{
    local tap_name="$1"
    local bridge_name="$2"


    validate_ifname "$tap_name"


    info "Creating TAP: $tap_name"


    ip tuntap add \
        dev "$tap_name" \
        mode tap \
        user "$NS3_USER"


    ip link set \
        dev "$tap_name" \
        master "$bridge_name"


    ip link set \
        dev "$tap_name" \
        up
}


# ============================================================
# Create namespace
# ============================================================

create_namespace()
{
    local namespace="$1"


    info "Creating namespace: $namespace"


    ip netns add "$namespace"


    # Loopback is DOWN by default inside a new namespace.
    ip netns exec "$namespace" \
        ip link set lo up
}


# ============================================================
# Configure wireless namespace interface
#
# Root:
#
#     veth-X-host
#          |
#          +---- br-X
#
# Namespace:
#
#     wifi0 = 10.42.0.x
#
# IMPORTANT:
#
# The ROOT side receives no IP address.
# ============================================================

create_wireless_veth()
{
    local namespace="$1"
    local bridge_name="$2"
    local host_if="$3"
    local temporary_ns_if="$4"
    local wireless_ip="$5"


    validate_ifname "$host_if"
    validate_ifname "$temporary_ns_if"


    info "Creating wireless veth for $namespace"


    ip link add \
        "$host_if" \
        type veth \
        peer name "$temporary_ns_if"


    # Root side joins the isolated bridge.
    ip link set \
        dev "$host_if" \
        master "$bridge_name"


    ip link set \
        dev "$host_if" \
        up


    # Move peer into namespace.
    ip link set \
        "$temporary_ns_if" \
        netns "$namespace"


    # Every namespace can safely use the same internal
    # interface name because namespaces are isolated.
    ip netns exec "$namespace" \
        ip link set \
        "$temporary_ns_if" \
        name wifi0


    ip netns exec "$namespace" \
        ip addr add \
        "${wireless_ip}/${WIRELESS_PREFIX}" \
        dev wifi0


    ip netns exec "$namespace" \
        ip link set \
        wifi0 \
        up


    # Connected route normally appears automatically.
    #
    # We explicitly ensure it exists because this makes
    # diagnostics more predictable.
    ip netns exec "$namespace" \
        ip route replace \
        "$WIRELESS_SUBNET" \
        dev wifi0 \
        src "$wireless_ip"
}


# ============================================================
# Create dedicated Gazebo <-> SITL simulation veth
#
# This link does NOT pass through ns-3.
#
# It exists only for Gazebo's FDM communication with the SITL
# instance.
#
# Example UAV1:
#
#   root:
#       sim-uav1-host = 172.31.1.1
#
#   uav1:
#       sim0          = 172.31.1.2
#
# SITL later uses:
#
#       --sim-address 172.31.1.1
#
# ============================================================

create_simulation_veth()
{
    local uav_id="$1"
    local namespace="$2"
    local root_ip="$3"
    local namespace_ip="$4"


    local host_if="sim-uav${uav_id}-host"
    local temporary_ns_if="su${uav_id}ns"


    validate_ifname "$host_if"
    validate_ifname "$temporary_ns_if"


    info "Creating Gazebo/SITL link for UAV$uav_id"


    ip link add \
        "$host_if" \
        type veth \
        peer name "$temporary_ns_if"


    # Root / Gazebo side.
    ip addr add \
        "${root_ip}/24" \
        dev "$host_if"


    ip link set \
        dev "$host_if" \
        up


    # SITL namespace side.
    ip link set \
        "$temporary_ns_if" \
        netns "$namespace"


    ip netns exec "$namespace" \
        ip link set \
        "$temporary_ns_if" \
        name sim0


    ip netns exec "$namespace" \
        ip addr add \
        "${namespace_ip}/24" \
        dev sim0


    ip netns exec "$namespace" \
        ip link set \
        sim0 \
        up
}


# ============================================================
# Build GCS network
# ============================================================

create_gcs()
{
    local bridge_name="br-gcs"
    local host_if="veth-gcs-host"
    local temporary_ns_if="veth-gcs-ns"


    info "============================================================"
    info "Creating GCS network"
    info "============================================================"


    create_namespace \
        "$GCS_NS"


    create_bridge \
        "$bridge_name"


    create_tap \
        "$GCS_TAP" \
        "$bridge_name"


    create_wireless_veth \
        "$GCS_NS" \
        "$bridge_name" \
        "$host_if" \
        "$temporary_ns_if" \
        "$GCS_IP"
}


# ============================================================
# Build one UAV network
# ============================================================

create_uav()
{
    local uav_id="$1"
    local namespace="$2"
    local tap="$3"
    local wireless_ip="$4"
    local sim_host_ip="$5"
    local sim_namespace_ip="$6"


    local bridge_name="br-uav${uav_id}"

    local wireless_host_if="veth-uav${uav_id}-host"

    # Temporary name exists only until moved to the namespace.
    local wireless_ns_if="vu${uav_id}ns"


    info "============================================================"
    info "Creating UAV$uav_id network"
    info "============================================================"


    create_namespace \
        "$namespace"


    create_bridge \
        "$bridge_name"


    create_tap \
        "$tap" \
        "$bridge_name"


    create_wireless_veth \
        "$namespace" \
        "$bridge_name" \
        "$wireless_host_if" \
        "$wireless_ns_if" \
        "$wireless_ip"


    create_simulation_veth \
        "$uav_id" \
        "$namespace" \
        "$sim_host_ip" \
        "$sim_namespace_ip"
}


# ============================================================
# Verification helpers
# ============================================================

require_namespace()
{
    local namespace="$1"


    ip netns list |
        awk '{print $1}' |
        grep -qx "$namespace" ||
        die "Namespace missing: $namespace"
}


require_root_interface()
{
    local interface="$1"


    ip link show dev "$interface" \
        >/dev/null 2>&1 ||
        die "Root interface missing: $interface"
}


require_namespace_interface()
{
    local namespace="$1"
    local interface="$2"


    ip netns exec "$namespace" \
        ip link show dev "$interface" \
        >/dev/null 2>&1 ||
        die "$namespace interface missing: $interface"
}


bridge_contains()
{
    local bridge_name="$1"
    local interface="$2"


    bridge link show |
        grep -E \
        "[[:space:]]${interface}(@[^: ]+)?:.*master ${bridge_name}([[:space:]]|$)" \
        >/dev/null 2>&1
}


# ============================================================
# Verify topology without ns-3
# ============================================================

verify_topology()
{
    info "============================================================"
    info "Verifying generated topology"
    info "============================================================"


    # --------------------------------------------------------
    # GCS
    # --------------------------------------------------------

    require_namespace "$GCS_NS"

    require_root_interface "br-gcs"
    require_root_interface "$GCS_TAP"
    require_root_interface "veth-gcs-host"

    require_namespace_interface \
        "$GCS_NS" \
        wifi0


    if ! ip netns exec "$GCS_NS" \
        ip -o -4 addr show dev wifi0 |
        grep -q \
        "inet ${GCS_IP}/${WIRELESS_PREFIX}"
    then

        die "GCS wifi0 does not have expected IP:

    ${GCS_IP}/${WIRELESS_PREFIX}"
    fi


    if ! bridge_contains \
        "br-gcs" \
        "$GCS_TAP"
    then

        die "$GCS_TAP is not attached to br-gcs"
    fi


    if ! bridge_contains \
        "br-gcs" \
        "veth-gcs-host"
    then

        die "veth-gcs-host is not attached to br-gcs"
    fi


    # --------------------------------------------------------
    # UAVs
    # --------------------------------------------------------

    for row in "${UAV_ROWS[@]}"; do

        IFS=$'\t' read -r \
            uav_id \
            namespace \
            tap \
            wireless_ip \
            sim_host_ip \
            sim_namespace_ip \
            <<< "$row"


        local bridge_name="br-uav${uav_id}"

        local wireless_host_if="veth-uav${uav_id}-host"

        local sim_host_if="sim-uav${uav_id}-host"


        require_namespace \
            "$namespace"


        require_root_interface \
            "$bridge_name"


        require_root_interface \
            "$tap"


        require_root_interface \
            "$wireless_host_if"


        require_root_interface \
            "$sim_host_if"


        require_namespace_interface \
            "$namespace" \
            wifi0


        require_namespace_interface \
            "$namespace" \
            sim0


        # Wireless IP.
        if ! ip netns exec "$namespace" \
            ip -o -4 addr show dev wifi0 |
            grep -q \
            "inet ${wireless_ip}/${WIRELESS_PREFIX}"
        then

            die "$namespace wifi0 does not have:

    ${wireless_ip}/${WIRELESS_PREFIX}"
        fi


        # Simulation namespace IP.
        if ! ip netns exec "$namespace" \
            ip -o -4 addr show dev sim0 |
            grep -q \
            "inet ${sim_namespace_ip}/24"
        then

            die "$namespace sim0 does not have:

    ${sim_namespace_ip}/24"
        fi


        # Simulation root IP.
        if ! ip -o -4 addr show dev "$sim_host_if" |
            grep -q \
            "inet ${sim_host_ip}/24"
        then

            die "$sim_host_if does not have:

    ${sim_host_ip}/24"
        fi


        if ! bridge_contains \
            "$bridge_name" \
            "$tap"
        then

            die "$tap is not attached to $bridge_name"
        fi


        if ! bridge_contains \
            "$bridge_name" \
            "$wireless_host_if"
        then

            die "$wireless_host_if is not attached to $bridge_name"
        fi

    done


    # --------------------------------------------------------
    # Critical anti-bypass check
    # --------------------------------------------------------

    info "Checking that root owns no $WIRELESS_SUBNET address..."


    if ip -o -4 addr show |
        grep -qE 'inet 10[.]42[.]0[.]'
    then

        echo
        ip -o -4 addr show |
            grep -E 'inet 10[.]42[.]0[.]' >&2
        echo

        die "Root namespace has a 10.42.0.x IP.

That could create a path that bypasses ns-3."
    fi


    info "PASS: root has no 10.42.0.x wireless endpoint."


    echo
    info "Topology verification passed."
}


# ============================================================
# Display final state
# ============================================================

show_topology()
{
    echo
    echo "============================================================"
    echo "NETWORK NAMESPACES"
    echo "============================================================"

    ip netns list


    echo
    echo "============================================================"
    echo "ROOT BRIDGES"
    echo "============================================================"

    for bridge_name in \
        br-gcs \
        $(seq 1 "$NUM_UAVS" |
          sed 's/^/br-uav/')
    do

        echo
        echo "--- $bridge_name"

        bridge link show \
            master "$bridge_name" \
            2>/dev/null || true
    done


    echo
    echo "============================================================"
    echo "NAMESPACE ADDRESSES"
    echo "============================================================"


    echo
    echo "--- $GCS_NS"

    ip netns exec "$GCS_NS" \
        ip -brief addr


    for row in "${UAV_ROWS[@]}"; do

        IFS=$'\t' read -r \
            uav_id \
            namespace \
            tap \
            wireless_ip \
            sim_host_ip \
            sim_namespace_ip \
            <<< "$row"


        echo
        echo "--- $namespace"

        ip netns exec "$namespace" \
            ip -brief addr
    done


    echo
    echo "============================================================"
    echo "ROOT TAP DEVICES"
    echo "============================================================"

    ip -brief link show "$GCS_TAP"


    for row in "${UAV_ROWS[@]}"; do

        IFS=$'\t' read -r \
            uav_id \
            namespace \
            tap \
            wireless_ip \
            sim_host_ip \
            sim_namespace_ip \
            <<< "$row"


        ip -brief link show "$tap"

    done


    echo
    echo "============================================================"
    echo "WIRELESS SUMMARY"
    echo "============================================================"

    printf \
        "%-8s %-16s %-16s %-16s\n" \
        "NODE" \
        "NAMESPACE" \
        "IP" \
        "TAP"


    printf \
        "%-8s %-16s %-16s %-16s\n" \
        "GCS" \
        "$GCS_NS" \
        "$GCS_IP" \
        "$GCS_TAP"


    for row in "${UAV_ROWS[@]}"; do

        IFS=$'\t' read -r \
            uav_id \
            namespace \
            tap \
            wireless_ip \
            sim_host_ip \
            sim_namespace_ip \
            <<< "$row"


        printf \
            "%-8s %-16s %-16s %-16s\n" \
            "UAV$uav_id" \
            "$namespace" \
            "$wireless_ip" \
            "$tap"

    done


    echo
    echo "============================================================"
    echo "IMPORTANT"
    echo "============================================================"

    echo
    echo "At this stage ns-3 is NOT running."
    echo
    echo "Therefore:"
    echo
    echo "  gcsns -> uavN wireless pings SHOULD NOT work yet."
    echo
    echo "This is correct."
    echo
    echo "The bridges are intentionally isolated until ns-3 opens"
    echo "tap-gcs and tap-uavN."
    echo
}


# ============================================================
# Main
# ============================================================

info "Starting dynamic topology setup."


cleanup_previous_topology


# ------------------------------------------------------------
# GCS
# ------------------------------------------------------------

create_gcs


# ------------------------------------------------------------
# UAV1 ... UAVN
# ------------------------------------------------------------

for row in "${UAV_ROWS[@]}"; do

    IFS=$'\t' read -r \
        uav_id \
        namespace \
        tap \
        wireless_ip \
        sim_host_ip \
        sim_namespace_ip \
        <<< "$row"


    create_uav \
        "$uav_id" \
        "$namespace" \
        "$tap" \
        "$wireless_ip" \
        "$sim_host_ip" \
        "$sim_namespace_ip"

done


# ------------------------------------------------------------
# Validate
# ------------------------------------------------------------

verify_topology


# ------------------------------------------------------------
# Show result
# ------------------------------------------------------------

show_topology


echo
info "Dynamic ns-3 topology setup completed successfully."