#!/usr/bin/env bash
#
# dashboard_bridge.sh
# -------------------
# Makes the dashboard reachable from your browser.
#
# viz_dashboard.py has to run inside the gcsns network namespace, because that
# is the only place the ROS 2 topics exist. But a network namespace has its own
# entire network stack: the dashboard's 127.0.0.1:8050 is NOT the 127.0.0.1
# your browser talks to, and the root namespace deliberately holds no address
# on 10.42.0.0/24, so there is no IP route into gcsns either. The browser gets
# ERR_CONNECTION_REFUSED.
#
# The way through is the *filesystem*: `ip netns exec` changes only the network
# namespace, not the mount namespace, so a UNIX socket under /tmp is visible
# from both sides. Two socat hops relay TCP across it:
#
#     browser -> root ns TCP:8050 -> /tmp/uav_dashboard.sock -> gcsns TCP:8050
#
# No IP addresses, routes or veths are added, so none of the launcher's
# topology checks (which assert the root namespace owns no 10.42.0.x address)
# are affected.
#
# Usage:
#     scripts/dashboard_bridge.sh              # bridge port 8050 from gcsns
#     scripts/dashboard_bridge.sh 8051 gcsns   # explicit port / namespace
#
# Leave it running; Ctrl-C tears both hops down.

set -euo pipefail

PORT="${1:-8050}"
NAMESPACE="${2:-gcsns}"
SOCKET="/tmp/uav_dashboard_${NAMESPACE}_${PORT}.sock"

command -v socat >/dev/null || {
    echo "ERROR: socat is not installed (sudo apt install socat)" >&2
    exit 1
}

ip netns list 2>/dev/null | grep -qw "$NAMESPACE" || {
    echo "ERROR: network namespace '$NAMESPACE' does not exist." >&2
    echo "       Start the simulation first." >&2
    exit 1
}

INNER_PID=""
OUTER_PID=""

cleanup() {
    [[ -n "$OUTER_PID" ]] && kill "$OUTER_PID" 2>/dev/null || true
    [[ -n "$INNER_PID" ]] && sudo kill "$INNER_PID" 2>/dev/null || true
    sudo rm -f "$SOCKET" 2>/dev/null || true
    echo
    echo "Dashboard bridge stopped."
}
trap cleanup EXIT INT TERM

sudo rm -f "$SOCKET"

# Hop 1: inside the namespace, expose the dashboard's TCP port as a UNIX socket
# on the shared filesystem. mode=666 so the unprivileged outer hop can open it.
sudo ip netns exec "$NAMESPACE" \
    socat "UNIX-LISTEN:${SOCKET},fork,mode=666" \
          "TCP:127.0.0.1:${PORT}" &
INNER_PID=$!

for _ in {1..25}; do
    [[ -S "$SOCKET" ]] && break
    sleep 0.2
done

[[ -S "$SOCKET" ]] || {
    echo "ERROR: the namespace side never created $SOCKET" >&2
    exit 1
}

# Hop 2: in the root namespace, accept browser connections and pass them down
# the socket. Bound to loopback only -- this is a local debugging view.
socat "TCP-LISTEN:${PORT},fork,reuseaddr,bind=127.0.0.1" \
      "UNIX-CONNECT:${SOCKET}" &
OUTER_PID=$!

sleep 1

if ! kill -0 "$OUTER_PID" 2>/dev/null; then
    echo "ERROR: could not listen on 127.0.0.1:${PORT} in the root namespace." >&2
    echo "       Is something else already using that port?" >&2
    exit 1
fi

echo "Dashboard bridge up:"
echo "    http://localhost:${PORT}    ->  ${NAMESPACE} 127.0.0.1:${PORT}"
echo "    socket: ${SOCKET}"
echo
echo "Leave this running. Ctrl-C to stop."

wait "$OUTER_PID"
