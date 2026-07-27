#!/usr/bin/env bash

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

pass() {
    printf '[PASS] %s\n' "$1"
}

fail() {
    printf '[FAIL] %s\n' "$1"
}

contains() {
    local pattern="$1"
    local path="$2"
    local description="$3"

    if grep -RqsE "$pattern" "$path" 2>/dev/null; then
        pass "$description"
    else
        fail "$description"
    fi
}

echo "=== Four-UAV readiness audit ==="
echo "Project: $ROOT"
echo

if [[ -f "$ROOT/params/uav4_dds.parm" ]]; then
    pass "params/uav4_dds.parm exists"
else
    fail "params/uav4_dds.parm exists"
fi

contains '2022' \
    "$ROOT/params/uav4_dds.parm" \
    "UAV4 DDS port 2022 configured"

contains '10\.42\.0\.14' \
    "$ROOT/config" \
    "UAV4 wireless IP 10.42.0.14 configured"

contains '5790' \
    "$ROOT/config" \
    "UAV4 MAVLink port 5790 configured"

contains '9032' \
    "$ROOT/config" \
    "UAV4 Gazebo servo port 9032 configured"

contains '9033' \
    "$ROOT/config" \
    "UAV4 SITL FDM port 9033 configured"

contains 'uav4|tap-uav4|br-uav4' \
    "$ROOT/scripts" \
    "UAV4 namespace/TAP/bridge referenced in scripts"

contains '172\.31\.4\.(1|2)' \
    "$ROOT/scripts" \
    "UAV4 management network configured"

contains '10\.42\.0\.14|tap-uav4|uav4' \
    "$ROOT/ns3" \
    "UAV4 referenced in NS-3 source"

contains 'uav4|iris_4|9032|9033' \
    "$ROOT/worlds $ROOT/models" \
    "UAV4 referenced in Gazebo world or model"

contains '2022' \
    "$ROOT/scripts/launch_city_dynamic_clustering.sh" \
    "Launcher starts UAV4 micro-ROS Agent"

contains '5790' \
    "$ROOT/scripts/launch_city_dynamic_clustering.sh" \
    "Launcher starts UAV4 MAVLink connection"

contains '_mission_uav4' \
    "$ROOT/ros2/uav_controller/uav_controller/fleet_mission.py" \
    "fleet_mission contains UAV4 mission handler"

python3 - "$ROOT/config/mission_plans.yaml" <<'PY'
import sys
from pathlib import Path

import yaml

path = Path(sys.argv[1])
config = yaml.safe_load(path.read_text())

uavs = config.get("uavs", {})
plan = uavs.get(4, uavs.get("4", {}))
waypoints = plan.get("waypoints", []) if isinstance(plan, dict) else []

if waypoints:
    print(f"[PASS] UAV4 has {len(waypoints)} mission waypoint(s)")
else:
    print("[FAIL] UAV4 has no mission waypoints")
PY

echo
echo "Audit complete."
