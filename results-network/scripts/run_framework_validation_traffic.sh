#!/bin/bash
# Actual application UDP traffic: gcsns (10.42.0.10) -> uav1ns (10.42.0.11).
# This path crosses both TAP devices and the ns-3 802.11a channel.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUN_NAME="${1:-framework-run}"
DURATION="${2:-120}"
RATE_MBPS="${3:-1.0}"
PACKET_SIZE="${PACKET_SIZE:-1200}"
PORT="${PORT:-5202}"
RUN_USER="${SUDO_USER:-$USER}"
OUT_DIR="$PROJECT_DIR/results-network/data/framework-validation/$RUN_NAME"
ENDPOINT="$SCRIPT_DIR/framework_udp_endpoint.py"

[[ "$RUN_NAME" =~ ^[A-Za-z0-9._-]+$ ]] || {
    echo "ERROR: run name may contain only letters, numbers, dot, underscore, and dash." >&2
    exit 1
}
[[ "$DURATION" =~ ^[1-9][0-9]*$ ]] || {
    echo "ERROR: duration must be a positive whole number of seconds." >&2
    exit 1
}

mkdir -p "$OUT_DIR"

for namespace in gcsns uav1ns; do
    sudo ip netns list | awk '{print $1}' | grep -qx "$namespace" || {
        echo "ERROR: namespace $namespace is not running; launch the simulation first." >&2
        exit 1
    }
done

receiver_pid=""
ping_pid=""
cleanup() {
    [[ -n "$receiver_pid" ]] && sudo kill "$receiver_pid" 2>/dev/null || true
    [[ -n "$ping_pid" ]] && sudo kill "$ping_pid" 2>/dev/null || true
}
trap cleanup INT TERM EXIT

echo "Starting UDP receiver in uav1ns at 10.42.0.11:$PORT"
sudo ip netns exec uav1ns sudo -H -u "$RUN_USER" \
    python3 "$ENDPOINT" receive \
    --bind 10.42.0.11 --port "$PORT" \
    --max-runtime "$((DURATION + 15))" --idle-timeout 3 \
    --output "$OUT_DIR/udp_received.csv" \
    > "$OUT_DIR/receiver.log" 2>&1 &
receiver_pid=$!

deadline=$((SECONDS + 10))
until sudo ip netns exec uav1ns ss -H -lun "sport = :$PORT" | grep -q .; do
    sudo kill -0 "$receiver_pid" 2>/dev/null || {
        echo "ERROR: UDP receiver exited early." >&2
        cat "$OUT_DIR/receiver.log" >&2
        exit 1
    }
    ((SECONDS >= deadline)) && {
        echo "ERROR: UDP receiver port did not open." >&2
        exit 1
    }
    sleep 0.2
done

sudo ip netns exec gcsns ping -D -n -i 1 -w "$DURATION" 10.42.0.11 \
    > "$OUT_DIR/ping.log" 2>&1 &
ping_pid=$!

echo "Sending ${RATE_MBPS} Mbps UDP from gcsns to uav1ns for ${DURATION}s"
sudo ip netns exec gcsns sudo -H -u "$RUN_USER" \
    python3 "$ENDPOINT" send \
    --host 10.42.0.11 --port "$PORT" --duration "$DURATION" \
    --rate-mbps "$RATE_MBPS" --packet-size "$PACKET_SIZE" \
    --output "$OUT_DIR/udp_sent.csv" \
    > "$OUT_DIR/sender.log" 2>&1

wait "$receiver_pid" || true
receiver_pid=""
wait "$ping_pid" || true
ping_pid=""

{
    echo "direction=gcsns_to_uav1ns"
    echo "source=10.42.0.10"
    echo "destination=10.42.0.11"
    echo "udp_port=$PORT"
    echo "duration_s=$DURATION"
    echo "offered_rate_mbps=$RATE_MBPS"
    echo "packet_size_bytes=$PACKET_SIZE"
} > "$OUT_DIR/traffic_metadata.txt"

trap - INT TERM EXIT
echo "Traffic logs written to $OUT_DIR"
echo "  sent:     $OUT_DIR/udp_sent.csv"
echo "  received: $OUT_DIR/udp_received.csv"
echo "  ping:     $OUT_DIR/ping.log"
