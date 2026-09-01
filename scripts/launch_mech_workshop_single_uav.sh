#!/bin/bash
# Phase-1 bring-up for the mech_workshop_validation world: gzserver + UAV1
# SITL + DDS GPS telemetry only. No mission logic. A separate mission script
# is expected to attach afterward (two-phase pattern used elsewhere in this
# project).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"

GAZEBO_LOG=/tmp/gazebo_mech_workshop.log
AGENT_LOG=/tmp/micro_ros_agent_uav1_mech_workshop.log
SITL_WORKDIR=/tmp/mech_workshop_sitl/uav1
SITL_LOG="$SITL_WORKDIR/arducopter.log"
PID_FILE="/tmp/multi_uav_mech_workshop_single_launcher.pid"

GAZEBO_FDM_TIMEOUT_SEC="${GAZEBO_FDM_TIMEOUT_SEC:-60}"
SITL_READY_TIMEOUT_SEC="${SITL_READY_TIMEOUT_SEC:-60}"
AGENT_READY_TIMEOUT_SEC="${AGENT_READY_TIMEOUT_SEC:-60}"
DDS_TOPIC_TIMEOUT_SEC="${DDS_TOPIC_TIMEOUT_SEC:-120}"
ENABLE_GAZEBO_GUI="${ENABLE_GAZEBO_GUI:-1}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

CLEANUP_RUNNING=0
CLEANUP_COMPLETE=0

SITL_PID=""
AGENT_PID=""
GAZEBO_PID=""
GZCLIENT_PID=""


## Stop any previous launcher instance and its child processes.

stop_previous_launcher() {
    local old_pid=""

    if [[ -f "$PID_FILE" ]]; then
        old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
    fi

    if [[ "$old_pid" =~ ^[0-9]+$ ]] &&
       [[ "$old_pid" != "$$" ]] &&
       kill -0 "$old_pid" 2>/dev/null; then

        echo "Stopping previous launcher: PID=$old_pid"

        sudo kill -TERM "$old_pid" 2>/dev/null || true

        for _ in {1..30}; do
            if ! kill -0 "$old_pid" 2>/dev/null; then
                break
            fi
            sleep 0.1
        done

        if kill -0 "$old_pid" 2>/dev/null; then
            echo "Previous launcher did not stop; forcing termination."
            sudo kill -KILL "$old_pid" 2>/dev/null || true
        fi
    fi

    rm -f "$PID_FILE"
}


############# cleanup previous instances #############

cleanup_previous_namespaces() {
    if sudo ip netns list | awk '{print $1}' | grep -qx uav1; then
        echo "Cleaning namespace: uav1"

        local -a namespace_pids=()
        mapfile -t namespace_pids < <(sudo ip netns pids uav1 2>/dev/null || true)

        if [[ ${#namespace_pids[@]} -gt 0 ]]; then
            sudo kill -TERM "${namespace_pids[@]}" 2>/dev/null || true
            sleep 0.5
            for pid in "${namespace_pids[@]}"; do
                if kill -0 "$pid" 2>/dev/null; then
                    sudo kill -KILL "$pid" 2>/dev/null || true
                fi
            done
        fi

        sudo ip netns delete uav1 2>/dev/null || true
    fi
}

cleanup_previous_links() {
    local link
    # tap-uav1/br-uav1 are torn down defensively in case a full ns-3-topology
    # launcher (e.g. launch_city_dynamic_clustering.sh) left them behind; this
    # script never creates them itself (see the ns-3 note printed at startup).
    for link in sim-uav1-host tap-uav1 br-uav1; do
        if ip link show "$link" >/dev/null 2>&1; then
            echo "Deleting stale interface: $link"
            sudo ip link delete "$link" 2>/dev/null || true
        fi
    done
}

cleanup_previous_instances() {
    echo "=== Cleaning previous mech_workshop single-UAV instances ==="

    local patterns=(
        'micro_ros_agent.*udp4.*--port 2019'
        '/build/sitl/bin/arducopter.*--instance 0'
        'gzserver.*mech_workshop_validation\.world'
        'gzclient'
    )

    local pattern pid
    local -a stale_pids=()
    local -a remaining_pids=()

    for pattern in "${patterns[@]}"; do
        while read -r pid; do
            [[ -n "$pid" ]] || continue
            [[ "$pid" == "$$" ]] && continue
            [[ "$pid" == "$PPID" ]] && continue
            stale_pids+=("$pid")
        done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
    done

    if [[ ${#stale_pids[@]} -gt 0 ]]; then
        mapfile -t stale_pids < <(printf '%s\n' "${stale_pids[@]}" | sort -nu)

        echo "Stopping stale processes: ${stale_pids[*]}"
        sudo kill -TERM "${stale_pids[@]}" 2>/dev/null || true
        sleep 2

        remaining_pids=()
        for pid in "${stale_pids[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                remaining_pids+=("$pid")
            fi
        done

        if [[ ${#remaining_pids[@]} -gt 0 ]]; then
            echo "Force-stopping remaining processes: ${remaining_pids[*]}"
            sudo kill -KILL "${remaining_pids[@]}" 2>/dev/null || true
        fi
    else
        echo "No stale simulation processes found."
    fi

    cleanup_previous_namespaces
    cleanup_previous_links

    echo "Previous simulation cleanup complete."
}


############# signal handling / cleanup #############

cleanup() {
    local root pid child index deadline
    local -a roots=()
    local -a tree=()
    local -a children=()

    if (( CLEANUP_RUNNING || CLEANUP_COMPLETE )); then
        return 0
    fi
    CLEANUP_RUNNING=1

    [[ -n "$AGENT_PID" ]] && roots+=("$AGENT_PID")
    [[ -n "$SITL_PID" ]] && roots+=("$SITL_PID")
    [[ -n "$GZCLIENT_PID" ]] && roots+=("$GZCLIENT_PID")
    [[ -n "$GAZEBO_PID" ]] && roots+=("$GAZEBO_PID")

    echo
    echo "=== mech_workshop single-UAV launcher cleanup ==="

    for root in "${roots[@]}"; do
        [[ "$root" =~ ^[0-9]+$ ]] || continue
        kill -0 "$root" 2>/dev/null || continue
        tree=("$root")
        index=0
        while (( index < ${#tree[@]} )); do
            pid="${tree[$index]}"
            mapfile -t children < <(pgrep -P "$pid" 2>/dev/null || true)
            for child in "${children[@]}"; do
                tree+=("$child")
            done
            ((index += 1))
        done
        for ((index=${#tree[@]} - 1; index >= 0; index--)); do
            kill -TERM "${tree[$index]}" 2>/dev/null || true
        done
        deadline=$((SECONDS + 3))
        while kill -0 "$root" 2>/dev/null && (( SECONDS < deadline )); do
            sleep 0.1
        done
        for ((index=${#tree[@]} - 1; index >= 0; index--)); do
            if kill -0 "${tree[$index]}" 2>/dev/null; then
                kill -KILL "${tree[$index]}" 2>/dev/null || true
            fi
        done
        wait "$root" 2>/dev/null || true
    done

    sudo ip netns del uav1 2>/dev/null || true
    sudo ip link del sim-uav1-host 2>/dev/null || true

    echo "Cleanup complete. Log files were preserved."
    CLEANUP_RUNNING=0
    CLEANUP_COMPLETE=1
}

final_cleanup() {
    cleanup

    if [[ -f "$PID_FILE" ]] &&
       [[ "$(cat "$PID_FILE" 2>/dev/null)" == "$$" ]]; then
        rm -f "$PID_FILE"
    fi
}

handle_signal() {
    local status="$1"
    trap - EXIT INT TERM
    final_cleanup
    exit "$status"
}

trap final_cleanup EXIT
trap 'handle_signal 130' INT
trap 'handle_signal 143' TERM


############# environment / parameter setup #############

source "$PROJECT_DIR/setup.sh"
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
BASE_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
UAV1_DEFAULTS="$BASE_DEFAULTS,$PROJECT_DIR/params/uav1_dds.parm"

# This world's actual spherical_coordinates origin (worlds/mech_workshop_validation.world).
HOME_GPS="6.0769447,80.1909952,0,0"

WORLD_PATH="$PROJECT_DIR/worlds/mech_workshop_validation.world"

validate_dds_param_file() {
    local file="$1"
    local expected_port="$2"

    echo "Checking DDS parameters: $file"

    if [[ ! -f "$file" ]]; then
        echo "ERROR: DDS parameter file does not exist: $file" >&2
        return 1
    fi

    grep -Eq '^[[:space:]]*DDS_ENABLE[[:space:]]+1[[:space:]]*$' "$file" || {
        echo "ERROR: DDS_ENABLE must be 1 in $file" >&2
        return 1
    }
    grep -Eq '^[[:space:]]*DDS_IP0[[:space:]]+10[[:space:]]*$' "$file" || {
        echo "ERROR: DDS_IP0 must be 10 in $file" >&2
        return 1
    }
    grep -Eq '^[[:space:]]*DDS_IP1[[:space:]]+42[[:space:]]*$' "$file" || {
        echo "ERROR: DDS_IP1 must be 42 in $file" >&2
        return 1
    }
    grep -Eq '^[[:space:]]*DDS_IP2[[:space:]]+0[[:space:]]*$' "$file" || {
        echo "ERROR: DDS_IP2 must be 0 in $file" >&2
        return 1
    }
    grep -Eq '^[[:space:]]*DDS_IP3[[:space:]]+10[[:space:]]*$' "$file" || {
        echo "ERROR: DDS_IP3 must be 10 in $file" >&2
        return 1
    }
    grep -Eq "^[[:space:]]*DDS_UDP_PORT[[:space:]]+${expected_port}[[:space:]]*$" "$file" || {
        echo "ERROR: DDS_UDP_PORT must be $expected_port in $file" >&2
        return 1
    }
    grep -Eq '^[[:space:]]*DDS_DOMAIN_ID[[:space:]]+0[[:space:]]*$' "$file" || {
        echo "ERROR: DDS_DOMAIN_ID must be 0 in $file" >&2
        return 1
    }

    echo "DDS configuration valid: 10.42.0.10:$expected_port"
}

validate_dds_param_file "$PROJECT_DIR/params/uav1_dds.parm" 2019

if [[ ! -f "$WORLD_PATH" ]]; then
    echo "ERROR: Gazebo world file not found: $WORLD_PATH" >&2
    exit 1
fi

sudo -v

stop_previous_launcher
cleanup_previous_instances

echo "$$" > "$PID_FILE"

cat <<'EOF'
=== ns-3 wireless bridging: SKIPPED ===
No 2-node ns-3 scenario exists yet. setup_ns3_wireless_topology.sh hardcodes
ENDPOINTS=(gcs uav1 uav2 uav3), and the compiled scratch programs
(three_uav_tapbridge_integrated.cc etc.) take fixed --tap0..--tap3 args for
four nodes -- neither can be reused for a single tap pair without new ns-3
C++ code. A dedicated 2-node ns-3 scenario is needed before this script can
carry real RF-simulated GCS<->UAV1 traffic.

For this validation run, UAV1's SITL and its micro_ros_agent both run inside
the same "uav1" network namespace, with 10.42.0.10 (the DDS peer address
baked into params/uav1_dds.parm) aliased onto that namespace's loopback.
DDS traffic never leaves the namespace, so no bridge/TAP is required.
EOF


############# ROS 2 environment (needed for micro_ros_agent + topic checks) #############

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/FYP/ardu_ws/install/setup.bash" ]]; then
    source "$HOME/FYP/ardu_ws/install/setup.bash"
fi
if [[ -f "$PROJECT_DIR/ros2/install/setup.bash" ]]; then
    source "$PROJECT_DIR/ros2/install/setup.bash"
fi
set -u

export ROS_DOMAIN_ID
export ROS2CLI_NO_DAEMON=1


############# Gazebo environment #############

export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$HOME/ardupilot_gazebo/models:$HOME/.gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MASTER_URI="http://127.0.0.1:11345"

OBSTACLE_PLUGIN_NAME="libobstacle_raycast_plugin.so"
OBSTACLE_PLUGIN_FILE="$PROJECT_DIR/install/multi_uav_gazebo_plugins/lib/$OBSTACLE_PLUGIN_NAME"
if [[ ! -f "$OBSTACLE_PLUGIN_FILE" ]]; then
    mapfile -t obstacle_plugin_candidates < <(
        find "$PROJECT_DIR/install" "$PROJECT_DIR/ros2/install" \
            -type f -name "$OBSTACLE_PLUGIN_NAME" -print 2>/dev/null |
        sort
    )
    if [[ ${#obstacle_plugin_candidates[@]} -eq 0 ]]; then
        echo "ERROR: Installed obstacle plugin was not found." >&2
        echo "Expected: $PROJECT_DIR/install/multi_uav_gazebo_plugins/lib/$OBSTACLE_PLUGIN_NAME" >&2
        echo "Searched only: $PROJECT_DIR/install and $PROJECT_DIR/ros2/install" >&2
        exit 1
    fi
    OBSTACLE_PLUGIN_FILE="${obstacle_plugin_candidates[0]}"
fi
OBSTACLE_PLUGIN_DIR="$(dirname "$OBSTACLE_PLUGIN_FILE")"

plugin_dependencies="$(ldd "$OBSTACLE_PLUGIN_FILE" 2>&1)" || {
    echo "ERROR: ldd failed for obstacle plugin: $OBSTACLE_PLUGIN_FILE" >&2
    printf '%s\n' "$plugin_dependencies" >&2
    exit 1
}
if grep -Fq 'not found' <<<"$plugin_dependencies"; then
    echo "ERROR: Obstacle plugin has unresolved shared-library dependencies:" >&2
    printf '%s\n' "$plugin_dependencies" >&2
    exit 1
fi

ARDUPILOT_GAZEBO_PLUGIN="$HOME/ardupilot_gazebo/build/libArduPilotPlugin.so"
if [[ ! -r "$ARDUPILOT_GAZEBO_PLUGIN" ]]; then
    echo "ERROR: ArduPilot Gazebo plugin not found:" >&2
    echo "  $ARDUPILOT_GAZEBO_PLUGIN" >&2
    exit 1
fi
if ldd "$ARDUPILOT_GAZEBO_PLUGIN" | grep -q "not found"; then
    echo "ERROR: ArduPilot Gazebo plugin has missing dependencies:" >&2
    ldd "$ARDUPILOT_GAZEBO_PLUGIN" >&2
    exit 1
fi
ARDUPILOT_PLUGIN_DIR="$(dirname "$ARDUPILOT_GAZEBO_PLUGIN")"

export GAZEBO_PLUGIN_PATH="$ARDUPILOT_PLUGIN_DIR:$OBSTACLE_PLUGIN_DIR:${GAZEBO_PLUGIN_PATH:-}"

echo "Final GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "Final GAZEBO_PLUGIN_PATH: $GAZEBO_PLUGIN_PATH"

if sudo lsof -iTCP:11345 -sTCP:LISTEN >/dev/null 2>&1; then
    echo "Stopping stale Gazebo process on TCP port 11345..."
    sudo fuser -k 11345/tcp 2>/dev/null || true
    sleep 2
fi
if sudo lsof -iTCP:11345 -sTCP:LISTEN >/dev/null 2>&1; then
    echo "ERROR: Gazebo master port 11345 is still occupied:" >&2
    sudo lsof -iTCP:11345 -sTCP:LISTEN >&2
    exit 1
fi


############# uav1 namespace (management link only, no ns-3 tap/bridge) #############

create_uav1_namespace() {
    echo "=== Setting up uav1 network namespace ==="
    sudo ip netns add uav1

    sudo ip link add sim-uav1-host type veth peer name sim0 netns uav1
    sudo ip addr add 172.31.1.1/30 dev sim-uav1-host
    sudo ip link set sim-uav1-host up
    sudo ip netns exec uav1 ip addr add 172.31.1.2/30 dev sim0
    sudo ip netns exec uav1 ip link set sim0 up
    sudo ip netns exec uav1 ip link set lo up

    # uav1_dds.parm targets 10.42.0.10 (the GCS address in the full ns-3
    # topology). Alias it onto uav1's own loopback so DDS packets resolve
    # locally instead of requiring the (nonexistent) 2-node ns-3 bridge.
    sudo ip netns exec uav1 ip addr add 10.42.0.10/32 dev lo
}

create_uav1_namespace


############# Gazebo #############

wait_for_gazebo_fdm_port() {
    local timeout_sec="$1"
    local deadline=$((SECONDS + timeout_sec))

    echo "Waiting for Gazebo ArduPilot FDM port 9002..."
    while (( SECONDS < deadline )); do
        if ss -H -lun "sport = :9002" | grep -q .; then
            echo "Gazebo FDM port ready: 9002"
            return 0
        fi
        if [[ -n "$GAZEBO_PID" ]] && ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
            echo "ERROR: Gazebo exited while waiting for the FDM port." >&2
            tail -n 120 "$GAZEBO_LOG" >&2 || true
            return 1
        fi
        sleep 0.25
    done

    echo "ERROR: Gazebo FDM port 9002 did not become ready." >&2
    echo "----- Gazebo log -----" >&2
    tail -n 120 "$GAZEBO_LOG" >&2 || true
    echo "----- UDP sockets -----" >&2
    ss -lunp >&2 || true
    return 1
}

echo "=== Starting Gazebo in the root namespace ==="
: >"$GAZEBO_LOG"
gzserver --verbose "$WORLD_PATH" >"$GAZEBO_LOG" 2>&1 &
GAZEBO_PID=$!
echo "Gazebo running: PID=$GAZEBO_PID log=$GAZEBO_LOG"

wait_for_gazebo_fdm_port "$GAZEBO_FDM_TIMEOUT_SEC"

start_gazebo_gui_once() {
    if [[ "$ENABLE_GAZEBO_GUI" != "1" ]]; then
        echo "Gazebo GUI disabled; gzserver continues headlessly."
        return 0
    fi

    pkill -TERM -x gzclient 2>/dev/null || true
    sleep 1

    : >/tmp/gzclient_mech_workshop.log
    gzclient >/tmp/gzclient_mech_workshop.log 2>&1 &
    GZCLIENT_PID=$!
    sleep 2

    if ! kill -0 "$GZCLIENT_PID" 2>/dev/null; then
        echo "ERROR: gzclient exited during startup." >&2
        cat /tmp/gzclient_mech_workshop.log >&2 || true
        return 1
    fi
    echo "Gazebo GUI started: PID=$GZCLIENT_PID log=/tmp/gzclient_mech_workshop.log"
}

start_gazebo_gui_once


############# UAV1 SITL #############

launch_sitl() {
    local namespace="$1"
    local instance="$2"
    local sysid="$3"
    local mavlink_port="$4"
    local gazebo_ip="$5"
    local defaults="$6"

    local work_dir="$SITL_WORKDIR"
    local log_file="$SITL_LOG"
    local pid_file="$work_dir/arducopter.pid"
    local exit_file="$work_dir/arducopter.exit"
    local wrapper_pid
    local actual_pid
    local group_name

    group_name="$(id -gn "$RUN_USER")"

    mkdir -p "$work_dir"
    chown -R "$RUN_USER:$group_name" "$work_dir"

    rm -f "$pid_file" "$exit_file"
    : >"$log_file"
    chown "$RUN_USER:$group_name" "$log_file"

    sudo ip netns exec "$namespace" \
        sudo -H -u "$RUN_USER" \
        bash -c '
            work_dir="$1"
            log_file="$2"
            pid_file="$3"
            exit_file="$4"
            shift 4

            cd "$work_dir" || exit 90

            rm -f "$exit_file"
            exec </dev/null

            stdbuf -oL -eL "$@" >>"$log_file" 2>&1 &
            child_pid=$!

            printf "%s\n" "$child_pid" >"$pid_file"

            set +e
            wait "$child_pid"
            status=$?
            set -e

            printf "%s\n" "$status" >"$exit_file"
            exit "$status"
        ' sitl-shell \
        "$work_dir" \
        "$log_file" \
        "$pid_file" \
        "$exit_file" \
        "$BINARY" \
        --model gazebo-iris \
        --speedup 1 \
        --sysid "$sysid" \
        --instance "$instance" \
        --defaults "$defaults" \
        --sim-address "$gazebo_ip" \
        --home "$HOME_GPS" \
        --serial0="tcp:0.0.0.0:$mavlink_port" \
        >/dev/null 2>&1 &

    wrapper_pid=$!

    local deadline=$((SECONDS + 10))
    while [[ ! -s "$pid_file" ]]; do
        if ! kill -0 "$wrapper_pid" 2>/dev/null; then
            set +e
            wait "$wrapper_pid"
            local wrapper_status=$?
            set -e
            echo "ERROR: $namespace SITL wrapper exited before publishing the ArduCopter PID." >&2
            echo "Wrapper status: $wrapper_status" >&2
            echo "----- $log_file -----" >&2
            cat "$log_file" >&2 || true
            return 1
        fi
        if (( SECONDS >= deadline )); then
            echo "ERROR: Timed out waiting for $namespace ArduCopter PID file: $pid_file" >&2
            return 1
        fi
        sleep 0.1
    done

    actual_pid="$(tr -dc '0-9' <"$pid_file")"
    if [[ -z "$actual_pid" ]]; then
        echo "ERROR: Invalid ArduCopter PID file for $namespace:" >&2
        cat "$pid_file" >&2 || true
        return 1
    fi

    SITL_PID="$actual_pid"
    echo "Started $namespace ArduCopter: actual PID=$actual_pid wrapper PID=$wrapper_pid log=$log_file"
}

wait_for_sitl_ready() {
    local namespace="$1"
    local udp_port="$2"
    local process_pid="$3"
    local log_file="$4"

    local exit_file="$SITL_WORKDIR/arducopter.exit"
    local deadline=$((SECONDS + SITL_READY_TIMEOUT_SEC))
    local exit_code

    echo "Waiting for $namespace SITL UDP port $udp_port..."
    while (( SECONDS < deadline )); do
        if ! kill -0 "$process_pid" 2>/dev/null; then
            exit_code="unknown"
            if [[ -r "$exit_file" ]]; then
                exit_code="$(tr -dc '0-9' <"$exit_file")"
                [[ -n "$exit_code" ]] || exit_code="unknown"
            fi
            echo "ERROR: $namespace ArduCopter exited during startup." >&2
            echo "ArduCopter exit code: $exit_code" >&2
            echo "----- $log_file -----" >&2
            cat "$log_file" >&2 || true
            return 1
        fi

        if sudo ip netns exec "$namespace" ss -H -lun "sport = :$udp_port" | grep -q .; then
            echo "$namespace ArduCopter is alive; UDP port $udp_port is open"
            return 0
        fi
        sleep 0.25
    done

    echo "ERROR: $namespace ArduCopter is alive, but UDP port $udp_port did not open." >&2
    echo "----- $log_file -----" >&2
    cat "$log_file" >&2 || true
    return 1
}

echo "--- Starting UAV1 SITL ---"
launch_sitl uav1 0 1 5760 172.31.1.1 "$UAV1_DEFAULTS"
wait_for_sitl_ready uav1 9003 "$SITL_PID" "$SITL_LOG"
echo "UAV1 SITL startup complete."


############# DDS agent (inside uav1 netns) + GPS verification #############

start_micro_ros_agent() {
    local port="$1"
    local log_file="$2"

    : >"$log_file"
    sudo ip netns exec uav1 \
        sudo -H -u "$RUN_USER" \
        bash -lc '
            set -e
            set +u
            source /opt/ros/humble/setup.bash
            source "$1"
            source "$2"
            set -u
            exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$3"
        ' agent-shell \
        "$HOME/FYP/ardu_ws/install/setup.bash" \
        "$PROJECT_DIR/ros2/install/setup.bash" \
        "$port" \
        >"$log_file" 2>&1 &

    AGENT_PID=$!
    echo "Started UAV1 micro_ros_agent: PID=$AGENT_PID UDP=$port log=$log_file"
}

agent_port_is_ready() {
    sudo ip netns exec uav1 ss -H -lun "sport = :2019" | grep -q .
}

echo "=== Starting micro_ros_agent inside uav1 netns ==="
start_micro_ros_agent 2019 "$AGENT_LOG"

deadline=$((SECONDS + AGENT_READY_TIMEOUT_SEC))
while true; do
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        echo "ERROR: UAV1 micro_ros_agent exited early." >&2
        cat "$AGENT_LOG" >&2 || true
        exit 1
    fi
    if agent_port_is_ready; then
        break
    fi
    if (( SECONDS >= deadline )); then
        echo "ERROR: Timed out waiting for the uav1 DDS UDP port 2019." >&2
        cat "$AGENT_LOG" >&2 || true
        exit 1
    fi
    sleep 0.2
done
echo "micro_ros_agent port ready inside uav1 netns."

run_ros_in_uav1ns() {
    sudo ip netns exec uav1 \
        sudo -H -u "$RUN_USER" \
        bash -lc '
            set -e
            set +u
            source /opt/ros/humble/setup.bash
            source "$1"
            source "$2"
            set -u
            export ROS_DOMAIN_ID="$3"
            export ROS2CLI_NO_DAEMON=1
            shift 3
            exec "$@"
        ' ros-shell \
        "$HOME/FYP/ardu_ws/install/setup.bash" \
        "$PROJECT_DIR/ros2/install/setup.bash" \
        "$ROS_DOMAIN_ID" "$@"
}

gps_message_is_valid() {
    awk -F ': *' '
        /^[[:space:]]*latitude:/  { lat=$2; gsub(/[[:space:]]/, "", lat) }
        /^[[:space:]]*longitude:/ { lon=$2; gsub(/[[:space:]]/, "", lon) }
        END {
            number="^-?[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$"
            if (lat !~ number || lon !~ number) exit 1
            if (lat < -90 || lat > 90 || lon < -180 || lon > 180) exit 1
            if ((lat + 0) == 0 && (lon + 0) == 0) exit 1
            exit 0
        }
    '
}

wait_for_dds_gps() {
    local topic="/ap/v1/navsat"
    local deadline=$((SECONDS + DDS_TOPIC_TIMEOUT_SEC))
    local info message
    local publisher_seen=false

    echo "Waiting for an active publisher and valid GPS on $topic..."
    while (( SECONDS < deadline )); do
        if info="$(run_ros_in_uav1ns ros2 topic info "$topic" 2>/dev/null)" &&
           awk '/Publisher count:/ { found=1; ok=(($3 + 0) > 0) } END { exit !(found && ok) }' \
               <<<"$info"; then
            publisher_seen=true
            if message="$(run_ros_in_uav1ns timeout 3 ros2 topic echo \
                "$topic" sensor_msgs/msg/NavSatFix --once 2>/dev/null)" &&
               gps_message_is_valid <<<"$message"; then
                echo "DDS GPS ready: $topic"
                return 0
            fi
        fi
        sleep 0.5
    done

    if [[ "$publisher_seen" == false ]]; then
        echo "No active publisher appeared for $topic within ${DDS_TOPIC_TIMEOUT_SEC}s." >&2
    else
        echo "Publisher exists for $topic, but no valid GPS message arrived within ${DDS_TOPIC_TIMEOUT_SEC}s." >&2
    fi
    echo "----- SITL log -----" >&2
    tail -n 80 "$SITL_LOG" >&2 || true
    echo "----- micro_ros_agent log -----" >&2
    tail -n 80 "$AGENT_LOG" >&2 || true
    return 1
}

echo "=== Verifying UAV1 AP_DDS GPS telemetry ==="
if ! wait_for_dds_gps; then
    echo "Aborting: UAV1 DDS GPS is not ready." >&2
    exit 1
fi


############# ready: hand off to a separate mission script #############

cat <<EOF

=== mech_workshop_validation world is up; UAV1 is alive with valid GPS ===
Gazebo:          PID=$GAZEBO_PID  log=$GAZEBO_LOG
UAV1 SITL:       PID=$SITL_PID    log=$SITL_LOG   MAVLink=tcp:0.0.0.0:5760
micro_ros_agent: PID=$AGENT_PID   log=$AGENT_LOG  DDS udp4:2019 (inside netns uav1)
GPS topic:       /ap/v1/navsat (verified)

Ready for a separate mission script to attach. Press Ctrl+C to tear down.
EOF

while true; do
    if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
        echo "ERROR: Gazebo exited unexpectedly." >&2
        tail -n 120 "$GAZEBO_LOG" >&2 || true
        exit 1
    fi
    if ! kill -0 "$SITL_PID" 2>/dev/null; then
        echo "ERROR: UAV1 SITL exited unexpectedly." >&2
        tail -n 120 "$SITL_LOG" >&2 || true
        exit 1
    fi
    if ! kill -0 "$AGENT_PID" 2>/dev/null; then
        echo "ERROR: micro_ros_agent exited unexpectedly." >&2
        tail -n 120 "$AGENT_LOG" >&2 || true
        exit 1
    fi
    sleep 5
done
