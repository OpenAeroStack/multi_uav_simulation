#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

AUTO_SETUP_NETNS="${AUTO_SETUP_NETNS:-1}"
NS3_HOME="${NS3_HOME:-}"
NS3_PID_FILE="/tmp/ns3_pid.txt"
SYNC_NS3_SCRATCH="${SYNC_NS3_SCRATCH:-1}"

# NS-3 scenario runtime knobs (used when starting the three-uav TapBridge program)
NS3_SIM_DURATION_SEC="${NS3_SIM_DURATION_SEC:-0}"
NS3_ANIM_FILE="${NS3_ANIM_FILE:-/tmp/three_uav_anim.xml}"
NS3_FLOWMON_XML="${NS3_FLOWMON_XML:-/tmp/three_uav_flowmon.xml}"
NS3_DELAY_MS="${NS3_DELAY_MS:-20}"
NS3_LOSS_RATE="${NS3_LOSS_RATE:-0}"
NS3_ENABLE_FLOWMON="${NS3_ENABLE_FLOWMON:-1}"

resolve_ns3_home() {
  if [[ -n "$NS3_HOME" ]]; then
    return 0
  fi

  # Default to the newest matching ns-3 allinone install under $HOME.
  # Example: $HOME/ns-allinone-3.38/ns-3.38
  local candidate
  candidate=$(ls -d "$HOME"/ns-allinone-3.*/ns-3.* 2>/dev/null | sort -V | tail -n 1 || true)
  if [[ -z "$candidate" ]]; then
    echo "ERROR: NS3_HOME is not set and no default ns-3 install found under $HOME/ns-allinone-3.xx/ns-3.xx"
    exit 1
  fi
  NS3_HOME="$candidate"
}

sync_ns3_scratch_program() {
  if [[ "$SYNC_NS3_SCRATCH" != "1" ]]; then
    return 0
  fi

  echo "=== Syncing three-uav scratch program into NS-3 tree ==="
  mkdir -p "$NS3_HOME/scratch/three-uav"
  cp "$PROJECT_DIR/ns3/CMakeLists.txt" "$NS3_HOME/scratch/three-uav/CMakeLists.txt"
  cp "$PROJECT_DIR/ns3/three_uav_tapbridge_rt.cc" "$NS3_HOME/scratch/three-uav/three_uav_tapbridge_rt.cc"

  echo "=== Building NS-3 target: three-uav ==="
  (cd "$NS3_HOME" && ./ns3 build three-uav)
}

wait_for_tcp_port() {
  local host="$1"
  local port="$2"
  local timeout_sec="$3"
  local start_ts
  start_ts=$(date +%s)
  while true; do
    # /dev/tcp is supported by bash.
    if (echo >"/dev/tcp/$host/$port") >/dev/null 2>&1; then
      return 0
    fi
    if (( $(date +%s) - start_ts >= timeout_sec )); then
      echo "ERROR: Timed out waiting for $host:$port"
      return 1
    fi
    sleep 1
  done
}

cleanup_ns3() {
  if [[ -f "$NS3_PID_FILE" ]]; then
    local pid
    pid=$(cat "$NS3_PID_FILE" 2>/dev/null || true)
    if [[ "$pid" =~ ^[0-9]+$ ]]; then
      echo "=== Stopping NS-3 (PID $pid) ==="
      sudo kill "$pid" 2>/dev/null || true
    fi
    rm -f "$NS3_PID_FILE" || true
  fi
}
trap cleanup_ns3 EXIT INT TERM

if [[ "$AUTO_SETUP_NETNS" == "1" ]]; then
  echo "=== Preparing namespace and TAP devices ==="
  "$PROJECT_DIR/scripts/setup_netns_tap.sh"
fi

echo "=== Launching Gazebo + ArduCopter SITL (3 UAV baseline) ==="

# Launch Gazebo+SITL in background so we can start NS-3 after SITL is up.
"$PROJECT_DIR/launch/launch_multi_uav.sh" &
MULTI_UAV_PID=$!

echo "=== Waiting for SITL MAVLink TCP ports (5760/5770/5780) to be ready ==="
wait_for_tcp_port 127.0.0.1 5760 600
wait_for_tcp_port 127.0.0.1 5770 600
wait_for_tcp_port 127.0.0.1 5780 600

echo "=== Waiting 3 seconds for TAP devices to be ready ==="
sleep 3

resolve_ns3_home
if [[ ! -d "$NS3_HOME" ]]; then
  echo "ERROR: NS3_HOME does not exist: $NS3_HOME"
  exit 1
fi

sync_ns3_scratch_program

echo "=== Starting NS-3 three-uav scenario in background ==="
echo "NS-3 outputs: anim=${NS3_ANIM_FILE} flowmon=${NS3_FLOWMON_XML} duration=${NS3_SIM_DURATION_SEC}s"
(
  set -euo pipefail
  cd "$NS3_HOME"
  ./ns3 run --enable-sudo "three-uav \
    --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
    --simDurationSec=${NS3_SIM_DURATION_SEC} \
    --delayMs=${NS3_DELAY_MS} --lossRate=${NS3_LOSS_RATE} \
    --enableFlowMonitor=${NS3_ENABLE_FLOWMON} --flowmonXml=${NS3_FLOWMON_XML} \
    --animFile=${NS3_ANIM_FILE}"
) &
NS3_PID=$!
echo "$NS3_PID" > "$NS3_PID_FILE"
echo "NS-3 PID: $NS3_PID (saved to $NS3_PID_FILE)"

wait "$MULTI_UAV_PID"
