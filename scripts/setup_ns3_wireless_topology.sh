#!/usr/bin/env bash
# Create the isolated Layer-2 endpoint topology used by the ns-3 wireless link.
# Fleet size is read from NUM_UAVS and config/fleet_config.sh.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FLEET_CONFIG="${FLEET_CONFIG:-$PROJECT_DIR/config/fleet_config.sh}"
REQUESTED_NUM_UAVS="${NUM_UAVS:-}"


if [[ ! -f "$FLEET_CONFIG" ]]; then
    echo "ERROR: Fleet configuration not found: $FLEET_CONFIG" >&2
    exit 1
fi

# shellcheck source=/dev/null
source "$FLEET_CONFIG"

echo "Configured topology fleet size: $NUM_UAVS"


if [[ ${EUID} -ne 0 ]]; then
    exec sudo -E bash "$0" "$@"
fi

if [[ ! -f "$FLEET_CONFIG" ]]; then
    echo "ERROR: Fleet configuration not found: $FLEET_CONFIG" >&2
    exit 1
fi

# fleet_config.sh may intentionally use variables that are not yet defined.
set +u
# shellcheck source=/dev/null
source "$FLEET_CONFIG"
set -u

# A value supplied by the launcher overrides the default in fleet_config.sh.
if [[ -n "$REQUESTED_NUM_UAVS" ]]; then
    NUM_UAVS="$REQUESTED_NUM_UAVS"
else
    NUM_UAVS="${NUM_UAVS:-3}"
fi

MIN_UAVS="${MIN_UAVS:-3}"
MAX_UAVS="${MAX_UAVS:-6}"

if ! [[ "$NUM_UAVS" =~ ^[0-9]+$ ]]; then
    echo "ERROR: NUM_UAVS must be an integer, got: $NUM_UAVS" >&2
    exit 1
fi
if (( NUM_UAVS < MIN_UAVS || NUM_UAVS > MAX_UAVS )); then
    echo "ERROR: NUM_UAVS must be between $MIN_UAVS and $MAX_UAVS, got: $NUM_UAVS" >&2
    exit 1
fi

NS3_USER="${NS3_USER:-${SUDO_USER:-}}"
if [[ -z "$NS3_USER" || "$NS3_USER" == root ]]; then
    echo "ERROR: Cannot determine the normal user that will run ns-3." >&2
    echo "Run through sudo from that user, or set NS3_USER explicitly." >&2
    exit 1
fi
if ! id "$NS3_USER" >/dev/null 2>&1; then
    echo "ERROR: ns-3 user does not exist: $NS3_USER" >&2
    exit 1
fi

ENDPOINTS=(gcs)
UAV_ENDPOINTS=()

declare -A NAMESPACE
declare -A ADDRESS
declare -A MAC
declare -A TAP_MAC
declare -A MANAGEMENT_HOST_ADDRESS
declare -A MANAGEMENT_UAV_ADDRESS
declare -A MANAGEMENT_HOST_IP

# GCS configuration
NAMESPACE[gcs]="gcsns"
ADDRESS[gcs]="${GCS_WIRELESS_IP}/24"
MAC[gcs]="02:00:00:00:00:00"
TAP_MAC[gcs]="02:aa:00:00:00:10"

# Generate UAV1 ... UAVN from fleet_config.sh
for ((uid=1; uid<=NUM_UAVS; uid++)); do
    endpoint="uav${uid}"

    ENDPOINTS+=("$endpoint")
    UAV_ENDPOINTS+=("$endpoint")

    NAMESPACE["$endpoint"]="$(uav_namespace "$uid")"
    ADDRESS["$endpoint"]="$(uav_wireless_ip "$uid")/24"

    MAC["$endpoint"]="02:00:00:00:00:0${uid}"
    TAP_MAC["$endpoint"]="02:aa:00:00:00:1${uid}"

    MANAGEMENT_HOST_ADDRESS["$endpoint"]="$(uav_management_host_ip "$uid")/30"
    MANAGEMENT_UAV_ADDRESS["$endpoint"]="$(uav_management_namespace_ip "$uid")/30"
    MANAGEMENT_HOST_IP["$endpoint"]="$(uav_management_host_ip "$uid")"
done

disable_veth_offloading() {
    local interface="$1"

    if command -v ethtool >/dev/null 2>&1; then
        ethtool -K "$interface" rx off tx off sg off tso off gso off gro off \
            2>/dev/null || true
    fi
}

remove_stale_topology() {
    local endpoint ns bridge tap host_veth management_host_veth
    local uid

    echo "Removing stale ns-3 wireless topology..."

    # Always remove GCS.
    ip netns del gcsns 2>/dev/null || true
    ip link set br-gcs down 2>/dev/null || true
    ip link del br-gcs type bridge 2>/dev/null || true
    ip link del tap-gcs 2>/dev/null || true
    ip link del veth-gcs-host 2>/dev/null || true

    # Remove all supported UAV endpoints, including inactive ones.
    for ((uid=1; uid<=MAX_UAVS; uid++)); do
        endpoint="uav${uid}"
        ns="$(uav_namespace "$uid")"
        bridge="$(uav_bridge "$uid")"
        tap="$(uav_tap "$uid")"
        host_veth="$(uav_wireless_veth "$uid")"
        management_host_veth="$(uav_management_veth "$uid")"

        ip netns del "$ns" 2>/dev/null || true

        ip link set "$bridge" down 2>/dev/null || true
        ip link del "$bridge" type bridge 2>/dev/null || true

        ip link del "$tap" 2>/dev/null || true
        ip link del "$host_veth" 2>/dev/null || true
        ip link del "$management_host_veth" 2>/dev/null || true
    done
}

create_endpoint() {
    local endpoint="$1"
    local ns="${NAMESPACE[$endpoint]}"
    local bridge="br-$endpoint"
    local tap="tap-$endpoint"
    local host_veth="veth-$endpoint-host"
    local address="${ADDRESS[$endpoint]}"
    local wifi_mac="${MAC[$endpoint]}"
    local tap_mac="${TAP_MAC[$endpoint]}"

    ip netns add "$ns"
    ip link add "$host_veth" type veth peer name wifi0 netns "$ns"

    ip netns exec "$ns" sysctl -qw net.ipv6.conf.wifi0.disable_ipv6=1
    ip netns exec "$ns" ip link set wifi0 address "$wifi_mac"
    ip netns exec "$ns" ip addr add "$address" dev wifi0
    ip netns exec "$ns" ip link set lo up
    ip netns exec "$ns" ip link set wifi0 up

    ip tuntap add dev "$tap" mode tap user "$NS3_USER"
    ip link set "$tap" address "$tap_mac"

    ip link add name "$bridge" type bridge stp_state 0
    sysctl -qw "net.ipv6.conf.$host_veth.disable_ipv6=1"
    sysctl -qw "net.ipv6.conf.$tap.disable_ipv6=1"
    sysctl -qw "net.ipv6.conf.$bridge.disable_ipv6=1"
    ip link set "$host_veth" master "$bridge"
    ip link set "$tap" master "$bridge"
    ip link set "$host_veth" up
    ip link set "$tap" up
    ip link set "$bridge" up

    disable_veth_offloading "$host_veth"
    ip netns exec "$ns" bash -c \
        'command -v ethtool >/dev/null 2>&1 && ethtool -K wifi0 rx off tx off sg off tso off gso off gro off 2>/dev/null || true'
}

create_management_link() {
    local endpoint="$1"
    local ns="${NAMESPACE[$endpoint]}"
    local host_veth="sim-$endpoint-host"
    local host_address="${MANAGEMENT_HOST_ADDRESS[$endpoint]}"
    local uav_address="${MANAGEMENT_UAV_ADDRESS[$endpoint]}"

    ip link add "$host_veth" type veth peer name sim0 netns "$ns"
    sysctl -qw "net.ipv6.conf.$host_veth.disable_ipv6=1"
    ip netns exec "$ns" sysctl -qw net.ipv6.conf.sim0.disable_ipv6=1

    ip addr add "$host_address" dev "$host_veth"
    ip netns exec "$ns" ip addr add "$uav_address" dev sim0
    ip link set "$host_veth" up
    ip netns exec "$ns" ip link set sim0 up

    disable_veth_offloading "$host_veth"
    ip netns exec "$ns" bash -c \
        'command -v ethtool >/dev/null 2>&1 && ethtool -K sim0 rx off tx off sg off tso off gso off gro off 2>/dev/null || true'
}

verify_endpoint() {
    local endpoint="$1"
    local ns="${NAMESPACE[$endpoint]}"
    local bridge="br-$endpoint"
    local tap="tap-$endpoint"
    local host_veth="veth-$endpoint-host"
    local expected_address="${ADDRESS[$endpoint]}"
    local port_count

    port_count="$(find "/sys/class/net/$bridge/brif" -mindepth 1 -maxdepth 1 -type l | wc -l)"
    if [[ "$port_count" -ne 2 || \
          ! -e "/sys/class/net/$bridge/brif/$host_veth" || \
          ! -e "/sys/class/net/$bridge/brif/$tap" ]]; then
        echo "ERROR: $bridge does not contain exactly $host_veth and $tap" >&2
        return 1
    fi

    if [[ "$(cat "/sys/class/net/$bridge/bridge/stp_state")" -ne 0 ]]; then
        echo "ERROR: STP is enabled on $bridge" >&2
        return 1
    fi

    if ip -o addr show dev "$bridge" | grep -q . || \
       ip -o addr show dev "$tap" | grep -q . || \
       ip -o addr show dev "$host_veth" | grep -q .; then
        echo "ERROR: root-side interfaces for $endpoint must not have IP addresses" >&2
        return 1
    fi

    if ! ip netns exec "$ns" ip -o -4 addr show dev wifi0 | grep -Fq " $expected_address "; then
        echo "ERROR: $ns/wifi0 does not have $expected_address" >&2
        return 1
    fi

    if ip netns exec "$ns" ip route show default | grep -q .; then
        echo "ERROR: unexpected default route in $ns" >&2
        return 1
    fi
}

verify_management_link() {
    local endpoint="$1"
    local ns="${NAMESPACE[$endpoint]}"
    local host_veth="sim-$endpoint-host"
    local expected_host_address="${MANAGEMENT_HOST_ADDRESS[$endpoint]}"
    local expected_uav_address="${MANAGEMENT_UAV_ADDRESS[$endpoint]}"
    local peer expected_host_ip

    if ! ip -o -4 addr show dev "$host_veth" | grep -Fq " $expected_host_address "; then
        echo "ERROR: $host_veth does not have $expected_host_address" >&2
        return 1
    fi
    if ! ip netns exec "$ns" ip -o -4 addr show dev sim0 | grep -Fq " $expected_uav_address "; then
        echo "ERROR: $ns/sim0 does not have $expected_uav_address" >&2
        return 1
    fi

    expected_host_ip="${MANAGEMENT_HOST_IP[$endpoint]}"

    echo "VERIFY: ip netns exec $ns ping -c 1 -W 1 $expected_host_ip"

    ip netns exec "$ns" \
        ping -c 1 -W 1 "$expected_host_ip" >/dev/null

    for peer in "${UAV_ENDPOINTS[@]}"; do
        [[ "$peer" == "$endpoint" ]] && continue

        echo "VERIFY (must fail): ip netns exec $ns ping -c 1 -W 1 ${MANAGEMENT_HOST_IP[$peer]}"

        if ip netns exec "$ns" \
            ping -c 1 -W 1 "${MANAGEMENT_HOST_IP[$peer]}" \
            >/dev/null 2>&1; then

            echo "ERROR: $ns can reach another UAV's management host IP: ${MANAGEMENT_HOST_IP[$peer]}" >&2
            return 1
        fi
    done
}

show_summary() {
    local endpoint ns bridge tap host_veth management_host_veth

    echo
    echo "ns-3 wireless topology interface summary"
    echo "Configured fleet size: $NUM_UAVS"
    echo "TAP owner: $NS3_USER"

    for endpoint in "${ENDPOINTS[@]}"; do
        ns="${NAMESPACE[$endpoint]}"
        bridge="br-$endpoint"
        tap="tap-$endpoint"
        host_veth="veth-$endpoint-host"
        management_host_veth="sim-$endpoint-host"

        echo
        echo "[$endpoint] $ns/wifi0 <-> $host_veth <-> $bridge <-> $tap"
        ip netns exec "$ns" ip -brief link show wifi0
        ip netns exec "$ns" ip -brief -4 addr show wifi0
        ip -brief link show "$host_veth"
        ip -brief link show "$bridge"
        ip -brief link show "$tap"
        bridge link show master "$bridge"
        echo "STP state: $(cat "/sys/class/net/$bridge/bridge/stp_state") (0=disabled)"
        echo "Namespace routes:"
        ip netns exec "$ns" ip route show

        if [[ "$endpoint" != gcs ]]; then
            echo "Management: $management_host_veth <-> $ns/sim0"
            ip -brief -4 addr show "$management_host_veth"
            ip netns exec "$ns" ip -brief -4 addr show sim0
        fi
    done
}

echo "Configured fleet size from fleet config: $NUM_UAVS"
remove_stale_topology

for endpoint in "${ENDPOINTS[@]}"; do
    create_endpoint "$endpoint"
done

for endpoint in "${UAV_ENDPOINTS[@]}"; do
    create_management_link "$endpoint"
done

for endpoint in "${ENDPOINTS[@]}"; do
    verify_endpoint "$endpoint"
done

for endpoint in "${UAV_ENDPOINTS[@]}"; do
    verify_management_link "$endpoint"
done

show_summary

echo
echo "Topology creation and verification completed successfully."
echo "Configured UAV count: $NUM_UAVS"