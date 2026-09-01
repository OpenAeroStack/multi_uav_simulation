#!/usr/bin/env bash
#
# run_dashboard.sh
# ----------------
# Starts viz_dashboard.py in the ROOT network namespace, which is where every
# networking topic it reads is published.
#
# Which namespace publishes what (see launch_city_dynamic_clustering.sh):
#
#   ROOT   world_pos_publisher    -> /uav_world_positions
#          gzserver raycast plugin-> /link_obstacle_loss
#          ns3_ros2_bridge        -> /ns3_link_snr, /ns3_link_rssi
#          dynamic_cluster_manager-> /cluster/*        (run_ros_in_root)
#
#   gcsns  micro_ros_agent, drone_bridge, city_mission
#                                 -> /uav{N}/gps, rel_alt, mode, armed
#
# So the dashboard belongs in ROOT: that gets the whole network picture, and
# `http://localhost:8050` then works directly with no port forwarding.
#
# The trade-off is that the per-UAV Alt/Mode/State columns come from the
# drone_bridge nodes inside gcsns and will read "—". Every networking panel --
# topology, link matrix, SNR, obstacle loss, clustering, relay -- is complete.
#
# IMPORTANT: start this AFTER the simulation is up, and restart it for every
# new simulation run. It is a ROS participant like any other; the launcher's
# cleanup does not stop it, and one left over from a previous run will keep
# serving a page that never updates.
#
# Usage:
#     scripts/run_dashboard.sh                 # auto-detect fleet size
#     scripts/run_dashboard.sh --num-uavs 4    # any viz_dashboard.py flag

set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG="${DASHBOARD_LOG:-/tmp/viz_dashboard.log}"

python3 -c 'import dash, plotly' 2>/dev/null || {
    echo "ERROR: dash/plotly are missing. Install them with:" >&2
    echo "           pip3 install dash plotly" >&2
    exit 1
}

# A dashboard left running from a previous simulation sits in that run's
# now-deleted network namespace: still serving HTTP, permanently empty. It is
# the single most confusing failure mode here, so refuse to add a second one.
if pgrep -f "viz_dashboard.py" >/dev/null 2>&1; then
    echo "ERROR: a viz_dashboard.py is already running:" >&2
    pgrep -af "viz_dashboard.py" | sed 's/^/       /' >&2
    echo >&2
    echo "       If it is from an earlier simulation run it will never show" >&2
    echo "       data again. Stop it first:" >&2
    echo "           pkill -f viz_dashboard.py" >&2
    exit 1
fi

echo "Starting dashboard in the ROOT namespace"
echo "  log: $LOG"
echo "  open http://localhost:8050  (no port bridge needed)"
echo

# Mirrors the launcher's run_ros_in_root: same setup files, same domain id.
# `set +u` around the sourcing because ROS setup.bash reads unset variables
# such as AMENT_TRACE_SETUP_FILES and aborts under nounset.
exec bash -lc '
    set -e
    set +u
    source /opt/ros/humble/setup.bash
    for setup_file in "$1" "$2" "$3"; do
        [[ -f "$setup_file" ]] && source "$setup_file"
    done
    set -u
    export ROS_DOMAIN_ID=0
    shift 3
    python3 "$@" 2>&1 | tee "'"$LOG"'"
' ros-root-shell \
    "$PROJECT_DIR/install/setup.bash" \
    "$PROJECT_DIR/ros2/install/setup.bash" \
    "$HOME/ardu_ws/install/setup.bash" \
    "$PROJECT_DIR/scripts/viz_dashboard.py" "$@"
