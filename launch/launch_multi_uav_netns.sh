#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_DIR/setup.sh"

AUTO_SETUP_NETNS="${AUTO_SETUP_NETNS:-1}"
START_NS3="${START_NS3:-0}"
NS3_HOME="${NS3_HOME:-}"
NS3_SCENARIO_NAME="${NS3_SCENARIO_NAME:-three_uav_tapbridge_rt}"

if [[ "$AUTO_SETUP_NETNS" == "1" ]]; then
  echo "=== Preparing namespace and TAP devices ==="
  "$PROJECT_DIR/scripts/setup_netns_tap.sh"
fi

if [[ "$START_NS3" == "1" ]]; then
  if [[ -z "$NS3_HOME" ]]; then
    echo "ERROR: START_NS3=1 but NS3_HOME is not set"
    exit 1
  fi
  if [[ ! -d "$NS3_HOME" ]]; then
    echo "ERROR: NS3_HOME does not exist: $NS3_HOME"
    exit 1
  fi

  echo "=== Starting NS-3 real-time TapBridge scenario in background ==="
  (
    set -euo pipefail
    cd "$NS3_HOME"
    ./ns3 run "$NS3_SCENARIO_NAME"
  ) &
  NS3_PID=$!
  echo "NS-3 PID: $NS3_PID"
fi

echo "=== Launching Gazebo + ArduCopter SITL (3 UAV baseline) ==="
"$PROJECT_DIR/launch/launch_multi_uav.sh"
