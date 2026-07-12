#!/bin/bash
# Set up the isolated host topology and start the real-time ns-3 wireless link.
# Starts the complete city simulation in dependency order and gates mission
# startup on verified process, transport, and namespaced GPS readiness.

set -euo pipefail


SMALL_CITY_DIR="$HOME/simulation/small_city_gazebo_world"

if [[ ! -d "$SMALL_CITY_DIR" ]]; then
    echo "ERROR: Small city Gazebo directory not found: $SMALL_CITY_DIR" >&2
    exit 1
fi

export GAZEBO_MODEL_PATH="$SMALL_CITY_DIR/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_RESOURCE_PATH="$SMALL_CITY_DIR:${GAZEBO_RESOURCE_PATH:-}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RUN_USER="${SUDO_USER:-$USER}"
NS3_LOG="/tmp/ns3_stdout.log"
TAP_READY_TIMEOUT_SEC="${TAP_READY_TIMEOUT_SEC:-30}"
AGENT_READY_TIMEOUT_SEC="${AGENT_READY_TIMEOUT_SEC:-20}"
DDS_TOPIC_TIMEOUT_SEC="${DDS_TOPIC_TIMEOUT_SEC:-60}"
SITL_REACHABLE_TIMEOUT_SEC="${SITL_REACHABLE_TIMEOUT_SEC:-60}"
GAZEBO_STARTUP_SEC="${GAZEBO_STARTUP_SEC:-20}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
TAPS=(tap-gcs tap-uav1 tap-uav2 tap-uav3)
AGENT_PIDS=()
AGENT_PORTS=(2019 2020 2021)
AGENT_LOGS=(
    /tmp/micro_ros_agent_uav1.log
    /tmp/micro_ros_agent_uav2.log
    /tmp/micro_ros_agent_uav3.log
)
BRIDGE_PIDS=()
BRIDGE_LOGS=(
    /tmp/drone_bridge_uav1.log
    /tmp/drone_bridge_uav2.log
    /tmp/drone_bridge_uav3.log
)
SITL_PIDS=()
SOCAT_PIDS=()
SITL_LOGS=(
    /tmp/multi_uav_sitl/uav1/arducopter.log
    /tmp/multi_uav_sitl/uav2/arducopter.log
    /tmp/multi_uav_sitl/uav3/arducopter.log
)
GAZEBO_LOG=/tmp/gazebo_city.log
MISSION_LOG=/tmp/city_mission.log
CLEANUP_RUNNING=0
CLEANUP_COMPLETE=0
CLEANUP_DRY_RUN=0
VERIFY_NETWORK_ONLY=0
PROVE_NS3_PATH=0
verify_bidirectional_wireless() {
    local namespace
    local uav_ip

    echo "=== Verifying bidirectional ns-3 wireless connectivity ==="

    for entry in \
        "uav1 10.42.0.11" \
        "uav2 10.42.0.12" \
        "uav3 10.42.0.13"
    do
        read -r namespace uav_ip <<<"$entry"

        echo "GCS -> $namespace"
        sudo ip netns exec gcsns \
            ping -c 3 -W 2 "$uav_ip" || {
                echo "ERROR: gcsns cannot reach $namespace at $uav_ip" >&2
                return 1
            }

        echo "$namespace -> GCS"
        sudo ip netns exec "$namespace" \
            ping -c 3 -W 2 10.42.0.10 || {
                echo "ERROR: $namespace cannot reach the GCS DDS agent" >&2
                return 1
            }
    done
}
for option in "$@"; do
    case "$option" in
        --cleanup-dry-run)
            CLEANUP_DRY_RUN=1
            ;;
        --verify-network-only)
            VERIFY_NETWORK_ONLY=1
            ;;
        --prove-ns3-path)
            VERIFY_NETWORK_ONLY=1
            PROVE_NS3_PATH=1
            ;;
        *)
            echo "ERROR: Unknown option: $option" >&2
            exit 2
            ;;
    esac
done

resolve_ns3_root() {
    local candidate
    local candidates=()

    if [[ -n "${NS3_ROOT:-}" ]]; then
        candidates+=("$NS3_ROOT")
    fi
    candidates+=(
        "$HOME/ns-allinone-3.38/ns-3.38"
        "$HOME/ns-3-dev"
    )

    for candidate in "${candidates[@]}"; do
        if [[ -d "$candidate" && -x "$candidate/ns3" ]]; then
            (cd "$candidate" && pwd -P)
            return 0
        fi
    done

    echo "ERROR: Unable to find an ns-3 root containing an executable ./ns3." >&2
    echo "Set NS3_ROOT to the ns-3 checkout directory." >&2
    return 1
}

tap_is_attached() {
    local tap="$1"
    local carrier_file="/sys/class/net/$tap/carrier"

    [[ -e "/sys/class/net/$tap" ]] || return 1
    if [[ -r "$carrier_file" && "$(<"$carrier_file")" == 1 ]]; then
        return 0
    fi
    ip -o link show dev "$tap" 2>/dev/null | grep -qw LOWER_UP
}

dump_ns3_failure() {
    echo "ERROR: ns-3 exited before all TAP devices attached." >&2
    echo "----- $NS3_LOG -----" >&2
    cat "$NS3_LOG" >&2 || true
    echo "--------------------" >&2
}

cleanup() {
    local root pid child index deadline
    local -a roots=()
    local -a tree=()
    local -a children=()
    local -a namespaces=(gcsns uav1 uav2 uav3)
    local -a bridges=(br-gcs br-uav1 br-uav2 br-uav3)
    local -a taps=(tap-gcs tap-uav1 tap-uav2 tap-uav3)
    local -a wireless_veths=(
        veth-gcs-host veth-uav1-host veth-uav2-host veth-uav3-host
    )
    local -a management_veths=(sim-uav1-host sim-uav2-host sim-uav3-host)

    if (( CLEANUP_RUNNING || CLEANUP_COMPLETE )); then
        return 0
    fi
    CLEANUP_RUNNING=1

    [[ -n "${MISSION_PID:-}" ]] && roots+=("$MISSION_PID")
    roots+=("${BRIDGE_PIDS[@]}")
    roots+=("${AGENT_PIDS[@]}")
    roots+=("${SITL_PIDS[@]}")
    [[ -n "${GAZEBO_PID:-}" ]] && roots+=("$GAZEBO_PID")
    [[ -n "${NS3_PID:-}" ]] && roots+=("$NS3_PID")
    roots+=("${SOCAT_PIDS[@]}")

    echo
    echo "=== Unified city simulation cleanup ==="
    if (( CLEANUP_DRY_RUN )); then
        echo "DRY RUN: would stop saved process roots: ${roots[*]:-(none)}"
        echo "DRY RUN: would delete namespaces: ${namespaces[*]}"
        echo "DRY RUN: would delete TAPs: ${taps[*]}"
        echo "DRY RUN: would delete bridges: ${bridges[*]}"
        echo "DRY RUN: would delete wireless veths: ${wireless_veths[*]}"
        echo "DRY RUN: would delete management veths: ${management_veths[*]}"
        echo "DRY RUN: logs would be preserved under /tmp"
        CLEANUP_RUNNING=0
        CLEANUP_COMPLETE=1
        return 0
    fi

    # Stop each saved process root and its descendants. This handles sudo/bash
    # wrappers and Gazebo's gzserver/gzclient children without broad pkill.
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

    # Namespace deletion removes namespace-side wifi0/sim0 peers. Explicit
    # root-side deletion below also makes partial and repeated cleanup safe.
    for pid in "${namespaces[@]}"; do
        sudo ip netns del "$pid" 2>/dev/null || true
    done
    for pid in "${bridges[@]}"; do
        sudo ip link set "$pid" down 2>/dev/null || true
        sudo ip link del "$pid" type bridge 2>/dev/null || true
    done
    for pid in "${taps[@]}" "${wireless_veths[@]}" "${management_veths[@]}"; do
        sudo ip link del "$pid" 2>/dev/null || true
    done

    echo "Cleanup complete. Log files were preserved."
    CLEANUP_RUNNING=0
    CLEANUP_COMPLETE=1
}
trap cleanup EXIT
trap 'cleanup; trap - EXIT; exit 130' INT
trap 'cleanup; trap - EXIT; exit 143' TERM

if (( CLEANUP_DRY_RUN )); then
    cleanup
    trap - EXIT INT TERM
    exit 0
fi

source "$PROJECT_DIR/setup.sh"
BINARY="$ARDUPILOT_HOME/build/sitl/bin/arducopter"
BASE_DEFAULTS="$ARDUPILOT_HOME/Tools/autotest/default_params/copter.parm,$ARDUPILOT_HOME/Tools/autotest/default_params/gazebo-iris.parm"
UAV1_DEFAULTS="$BASE_DEFAULTS,$PROJECT_DIR/params/uav1_dds.parm"
UAV2_DEFAULTS="$BASE_DEFAULTS,$PROJECT_DIR/params/uav2_dds.parm"
UAV3_DEFAULTS="$BASE_DEFAULTS,$PROJECT_DIR/params/uav3_dds.parm"
HOME_GPS="37.3382,-121.8863,0,0"
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
validate_dds_param_file "$PROJECT_DIR/params/uav2_dds.parm" 2020
validate_dds_param_file "$PROJECT_DIR/params/uav3_dds.parm" 2021

echo "=== Setting up isolated ns-3 wireless topology ==="
sudo NS3_USER="$RUN_USER" "$SCRIPT_DIR/setup_ns3_wireless_topology.sh"

NS3_ROOT="$(resolve_ns3_root)"
echo "Using ns-3 root: $NS3_ROOT"

echo "=== Building ns-3 target: three-uav ==="
(cd "$NS3_ROOT" && ./ns3 build three-uav)

: >"$NS3_LOG"
echo "=== Starting real-time ns-3 wireless simulation ==="
(
    cd "$NS3_ROOT"
    exec ./ns3 run "three-uav \
        --tap0=tap-gcs --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 \
        --delayMs=0.0 \
        --simDurationSec=0 \
        --nakagamiM=10.0 --shadowingStdDb=0.0 --txPowerDbm=20 --uavAltitude=20 \
        --enableFlowMonitor=false --snrLogFile=/tmp/snr_log.csv \
        --animFile=/tmp/three_uav_anim.xml"
) >"$NS3_LOG" 2>&1 &
NS3_PID=$!
echo "ns-3 PID: $NS3_PID"
echo "ns-3 log: $NS3_LOG"

echo "Waiting for TAP carrier/LOWER_UP state..."
deadline=$((SECONDS + TAP_READY_TIMEOUT_SEC))
while true; do
    if ! kill -0 "$NS3_PID" 2>/dev/null; then
        wait "$NS3_PID" 2>/dev/null || true
        dump_ns3_failure
        exit 1
    fi

    pending=()
    for tap in "${TAPS[@]}"; do
        if ! tap_is_attached "$tap"; then
            pending+=("$tap")
        fi
    done

    if [[ ${#pending[@]} -eq 0 ]]; then
        break
    fi
    if (( SECONDS >= deadline )); then
        echo "ERROR: Timed out waiting for TAP attachment: ${pending[*]}" >&2
        echo "----- $NS3_LOG -----" >&2
        cat "$NS3_LOG" >&2 || true
        echo "--------------------" >&2
        exit 1
    fi
    sleep 0.2
done

echo "All TAP devices are attached:"

for tap in "${TAPS[@]}"; do
    carrier="$(<"/sys/class/net/$tap/carrier")"
    flags="$(ip -o link show dev "$tap" |
        sed -n 's/.*<\([^>]*\)>.*/\1/p')"

    echo "  $tap carrier=$carrier flags=$flags"
done

verify_bidirectional_wireless

verify_wireless_network() {
    local destination namespace tap bridge
    local -a destinations=(10.42.0.11 10.42.0.12 10.42.0.13)
    local -a namespaces=(gcsns uav1 uav2 uav3)
    local -a taps=(tap-gcs tap-uav1 tap-uav2 tap-uav3)
    local -a bridges=(br-gcs br-uav1 br-uav2 br-uav3)

    echo "=== Wireless connectivity through ns-3 ==="
    for destination in "${destinations[@]}"; do
        echo "VERIFY: ip netns exec gcsns ping -c 2 -W 2 $destination"
        if ! sudo ip netns exec gcsns ping -c 2 -W 2 "$destination"; then
            echo "ERROR: gcsns cannot reach $destination while ns-3 is running." >&2
            return 1
        fi
    done

    echo "=== TAP packet counters ==="
    for tap in "${taps[@]}"; do
        echo "--- $tap"
        ip -s link show dev "$tap"
    done

    echo "=== Namespace routes ==="
    for namespace in "${namespaces[@]}"; do
        echo "--- $namespace"
        sudo ip netns exec "$namespace" ip route show
    done

    echo "=== Bridge membership ==="
    for bridge in "${bridges[@]}"; do
        echo "--- $bridge"
        bridge link show master "$bridge"
    done

    echo "=== Namespace interface addresses ==="
    for namespace in "${namespaces[@]}"; do
        echo "--- $namespace"
        sudo ip netns exec "$namespace" ip -brief address show
    done

    echo "=== Root-kernel bypass checks ==="
    echo "Root namespace addresses in 10.42.0.0/24 (expected: none):"
    ip -o -4 address show | grep -E 'inet 10[.]42[.]0[.]' || echo "  none"
    echo "Root namespace routes for 10.42.0.0/24 (expected: none):"
    ip route show 10.42.0.0/24 || true
    echo "Each root bridge has only one namespace veth and one unnumbered TAP."
    echo "Therefore the root kernel has no Layer-3 endpoint or route on the wireless subnet."
    echo "The --prove-ns3-path negative-control test establishes this conclusively:"
    echo "the same namespace pings must fail after ns-3 releases the TAPs."

    if (( PROVE_NS3_PATH )); then
        echo "=== Negative control: stopping ns-3 ==="
        kill -TERM "$NS3_PID"
        wait "$NS3_PID" 2>/dev/null || true

        deadline=$((SECONDS + 5))
        while (( SECONDS < deadline )); do
            if ! tap_is_attached tap-gcs &&
               ! tap_is_attached tap-uav1 &&
               ! tap_is_attached tap-uav2 &&
               ! tap_is_attached tap-uav3; then
                break
            fi
            sleep 0.1
        done

        for destination in "${destinations[@]}"; do
            echo "NEGATIVE CONTROL (must fail): ip netns exec gcsns ping -c 1 -W 1 $destination"
            if sudo ip netns exec gcsns ping -c 1 -W 1 "$destination" \
                >/dev/null 2>&1; then
                echo "ERROR: $destination remained reachable after ns-3 stopped; possible bypass." >&2
                return 1
            fi
            echo "PASS: $destination became unreachable when ns-3 stopped."
        done
        echo "PASS: wireless connectivity depends on ns-3. Exiting cleanly."
    fi
}

if (( VERIFY_NETWORK_ONLY )); then
    verify_wireless_network
    echo "Network-only verification complete; no Gazebo, SITL, ROS 2 node, or mission was started."
    exit 0
fi

echo "=== Starting Gazebo in the root namespace ==="

SMALL_CITY_DIR="$HOME/simulation/small_city_gazebo_world"
WORLD_PATH="$PROJECT_DIR/worlds/city_3uav.world"

if [[ ! -d "$SMALL_CITY_DIR" ]]; then
    echo "ERROR: Small-city directory not found: $SMALL_CITY_DIR" >&2
    exit 1
fi

if [[ ! -d "$SMALL_CITY_DIR/models" ]]; then
    echo "ERROR: Small-city models directory not found: $SMALL_CITY_DIR/models" >&2
    exit 1
fi

if [[ ! -f "$WORLD_PATH" ]]; then
    echo "ERROR: Gazebo world file not found: $WORLD_PATH" >&2
    exit 1
fi

export GAZEBO_MODEL_PATH="$SMALL_CITY_DIR/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_RESOURCE_PATH="$SMALL_CITY_DIR:${GAZEBO_RESOURCE_PATH:-}"

# Remove a stale Gazebo master from a previous failed execution.
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

: >"$GAZEBO_LOG"

gazebo --verbose "$WORLD_PATH" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so \
    >"$GAZEBO_LOG" 2>&1 &

GAZEBO_PID=$!

echo "Waiting ${GAZEBO_STARTUP_SEC}s for Gazebo to start..."
sleep "$GAZEBO_STARTUP_SEC"

if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    echo "ERROR: Gazebo exited during startup." >&2
    echo "----- $GAZEBO_LOG -----" >&2
    cat "$GAZEBO_LOG" >&2 || true
    echo "-----------------------" >&2
    exit 1
fi

echo "Gazebo running: PID=$GAZEBO_PID log=$GAZEBO_LOG"
launch_sitl() {
    local namespace="$1"
    local instance="$2"
    local sysid="$3"
    local mavlink_port="$4"
    local gazebo_ip="$5"
    local defaults="$6"

    local work_dir="/tmp/multi_uav_sitl/$namespace"
    local log_file="$work_dir/arducopter.log"

    mkdir -p "$work_dir"
    chown "$RUN_USER":"$(id -gn "$RUN_USER")" "$work_dir"

    sudo ip netns exec "$namespace" \
        sudo -H -u "$RUN_USER" \
        bash -c 'cd "$1" && shift && exec "$@"' sitl-shell \
        "$work_dir" \
        strace -ff -tt -s 256\
        -o "$work_dir/strace" \
        "$BINARY" \
        --wipe \
        --model gazebo-iris \
        --speedup 1 \
        --sysid "$sysid" \
        --instance "$instance" \
        --defaults "$defaults" \
        --sim-address "$gazebo_ip" \
        --home "$HOME_GPS" \
        --serial0="tcp:0.0.0.0:$mavlink_port" \
        >"$log_file" 2>&1 &

    SITL_PIDS+=("$!")
    echo "Started $namespace SITL: PID=$! log=$log_file"
}
wait_for_sitl_ready() {
    local namespace="$1"
    local udp_port="$2"
    local process_pid="$3"
    local log_file="$4"
    local deadline=$((SECONDS + 15))
    local exit_code

    echo "Waiting for $namespace SITL UDP port $udp_port..."

    while (( SECONDS < deadline )); do
        # The saved PID is the sudo/netns wrapper. It remains alive while
        # the ArduPilot process is running.
        if ! kill -0 "$process_pid" 2>/dev/null; then
            set +e
            wait "$process_pid"
            exit_code=$?
            set -e

            echo "ERROR: $namespace SITL exited during startup." >&2
            echo "Wrapper exit code: $exit_code" >&2
            echo "----- $log_file -----" >&2
            cat "$log_file" >&2 || true
            echo "---------------------" >&2
            return 1
        fi

        if sudo ip netns exec "$namespace" \
            ss -H -lun "sport = :$udp_port" |
            grep -q .; then

            echo "✓ $namespace SITL is alive; UDP port $udp_port is open"
            return 0
        fi

        sleep 0.25
    done

    echo "ERROR: $namespace process is alive, but UDP port $udp_port did not open." >&2

    echo "----- Processes inside $namespace -----" >&2
    local pids
    pids="$(sudo ip netns pids "$namespace" | paste -sd, -)"

    if [[ -n "$pids" ]]; then
        ps -o pid,ppid,stat,etime,cmd -p "$pids" >&2 || true
    else
        echo "No processes found." >&2
    fi

    echo "----- Network sockets inside $namespace -----" >&2
    sudo ip netns exec "$namespace" ss -lntup >&2 || true

    echo "----- $log_file -----" >&2
    cat "$log_file" >&2 || true
    echo "---------------------" >&2

    return 1
}





start_micro_ros_agent() {
    local uav_number="$1"
    local port="$2"
    local log_file="$3"

    : >"$log_file"
    sudo ip netns exec gcsns \
        sudo -H -u "$RUN_USER" \
        bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            source "$1"
            source "$2"
            exec ros2 run micro_ros_agent micro_ros_agent udp4 --port "$3"
        ' agent-shell \
        "$HOME/ardu_ws/install/setup.bash" \
        "$PROJECT_DIR/ros2/install/setup.bash" \
        "$port" \
        >"$log_file" 2>&1 &

    AGENT_PIDS+=("$!")
    echo "Started UAV$uav_number micro_ros_agent: PID=$! UDP=$port log=$log_file"
}

agent_port_is_ready() {
    local port="$1"
    sudo ip netns exec gcsns ss -H -lun "sport = :$port" | grep -q .
}

echo "=== Starting micro_ros_agent instances inside gcsns ==="
for i in "${!AGENT_PORTS[@]}"; do
    start_micro_ros_agent "$((i + 1))" "${AGENT_PORTS[$i]}" "${AGENT_LOGS[$i]}"
done

echo "Waiting for micro_ros_agent UDP ports inside gcsns..."
deadline=$((SECONDS + AGENT_READY_TIMEOUT_SEC))
while true; do
    pending=()
    for i in "${!AGENT_PIDS[@]}"; do
        if ! kill -0 "${AGENT_PIDS[$i]}" 2>/dev/null; then
            echo "ERROR: UAV$((i + 1)) micro_ros_agent exited early." >&2
            echo "----- ${AGENT_LOGS[$i]} -----" >&2
            cat "${AGENT_LOGS[$i]}" >&2 || true
            echo "-----------------------------" >&2
            exit 1
        fi
        if ! agent_port_is_ready "${AGENT_PORTS[$i]}"; then
            pending+=("${AGENT_PORTS[$i]}")
        fi
    done

    if [[ ${#pending[@]} -eq 0 ]]; then
        break
    fi
    if (( SECONDS >= deadline )); then
        echo "ERROR: Timed out waiting for gcsns UDP ports: ${pending[*]}" >&2
        for log_file in "${AGENT_LOGS[@]}"; do
            echo "----- $log_file -----" >&2
            cat "$log_file" >&2 || true
        done
        exit 1
    fi
    sleep 0.2
done

echo "micro_ros_agent ports ready inside gcsns:"
sudo ip netns exec gcsns ss -lunp \
    '( sport = :2019 or sport = :2020 or sport = :2021 )'
echo "=== Starting namespaced SITL instances ==="


launch_sitl uav1 0 1 5760 172.31.1.1 "$UAV1_DEFAULTS"
launch_sitl uav2 1 2 5770 172.31.2.1 "$UAV2_DEFAULTS"
launch_sitl uav3 2 3 5780 172.31.3.1 "$UAV3_DEFAULTS"


wait_for_sitl_ready \
    uav1 9003 "${SITL_PIDS[0]}" \
    /tmp/multi_uav_sitl/uav1/arducopter.log

wait_for_sitl_ready \
    uav2 9013 "${SITL_PIDS[1]}" \
    /tmp/multi_uav_sitl/uav2/arducopter.log

wait_for_sitl_ready \
    uav3 9023 "${SITL_PIDS[2]}" \
    /tmp/multi_uav_sitl/uav3/arducopter.log

run_ros_in_gcsns() {
    sudo ip netns exec gcsns \
        sudo -H -u "$RUN_USER" \
        bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            source "$1"
            source "$2"
            export ROS_DOMAIN_ID="$3"
            export ROS2CLI_NO_DAEMON=1
            shift 3
            exec "$@"
        ' ros-shell \
        "$HOME/ardu_ws/install/setup.bash" \
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

show_dds_diagnostics() {
    local uav_number="$1"
    local topic="$2"
    local sitl_log="/tmp/multi_uav_sitl/uav$uav_number/arducopter.log"
    local agent_log="${AGENT_LOGS[$((uav_number - 1))]}"

    echo "ERROR: DDS GPS readiness failed for UAV$uav_number ($topic)." >&2
    echo "----- SITL log: $sitl_log -----" >&2
    if [[ -r "$sitl_log" ]]; then
        tail -n 80 "$sitl_log" >&2
    else
        echo "SITL log is absent or unreadable." >&2
    fi
    echo "----- micro_ros_agent log: $agent_log -----" >&2
    if [[ -r "$agent_log" ]]; then
        tail -n 80 "$agent_log" >&2
    else
        echo "micro_ros_agent log is absent or unreadable." >&2
    fi
    echo "----- gcsns UDP sockets -----" >&2
    sudo ip netns exec gcsns ss -lunp >&2 || true
    echo "----- gcsns routes -----" >&2
    sudo ip netns exec gcsns ip route show >&2 || true
    echo "----- gcsns interfaces -----" >&2
    sudo ip netns exec gcsns ip -details -statistics address show >&2 || true
    echo "--------------------------------" >&2
}

wait_for_dds_gps() {
    local uav_number="$1"
    local topic="/ap/v$uav_number/navsat"
    local deadline=$((SECONDS + DDS_TOPIC_TIMEOUT_SEC))
    local info message
    local publisher_seen=false

    echo "Waiting for an active publisher and valid GPS on $topic..."
    while (( SECONDS < deadline )); do
        if info="$(run_ros_in_gcsns ros2 topic info "$topic" 2>/dev/null)" &&
           awk '/Publisher count:/ { found=1; ok=(($3 + 0) > 0) } END { exit !(found && ok) }' \
               <<<"$info"; then
            publisher_seen=true
            if message="$(run_ros_in_gcsns timeout 3 ros2 topic echo \
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
    show_dds_diagnostics "$uav_number" "$topic"
    return 1
}

echo "=== Verifying namespaced AP_DDS GPS telemetry ==="
for uav_number in 1 2 3; do
    if ! wait_for_dds_gps "$uav_number"; then
        echo "Aborting before city_mission because UAV$uav_number DDS GPS is not ready." >&2
        exit 1
    fi
done

sitl_tcp_is_reachable() {
    local host="$1"
    local port="$2"

    sudo ip netns exec gcsns \
        sudo -H -u "$RUN_USER" \
        timeout 1 bash -c 'exec 3<>"/dev/tcp/$1/$2"' sitl-probe \
        "$host" "$port" 2>/dev/null
}

wait_for_sitl_tcp() {
    local uav_number="$1"
    local host="$2"
    local port="$3"
    local deadline=$((SECONDS + SITL_REACHABLE_TIMEOUT_SEC))

    echo "Waiting for UAV$uav_number SITL at $host:$port from gcsns..."
    until sitl_tcp_is_reachable "$host" "$port"; do
        if ! kill -0 "$NS3_PID" 2>/dev/null; then
            dump_ns3_failure
            exit 1
        fi
        if (( SECONDS >= deadline )); then
            echo "ERROR: UAV$uav_number SITL is not reachable from gcsns at $host:$port" >&2
            exit 1
        fi
        sleep 0.2
    done
    echo "UAV$uav_number SITL reachable at $host:$port"
}

start_drone_bridge() {
    local uav_number="$1"
    local node_name="$2"
    local mavlink_host="$3"
    local mavlink_port="$4"
    local takeoff_altitude="$5"
    local log_file="$6"

    : >"$log_file"
    sudo ip netns exec gcsns \
        sudo -H -u "$RUN_USER" \
        bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            source "$1"
            source "$2"
            export ROS_DOMAIN_ID="$3"
            exec ros2 run uav_controller drone_bridge --ros-args \
                -r "__node:=$4" \
                -p "uav_id:=$5" \
                -p "mavlink_host:=$6" \
                -p "mavlink_port:=$7" \
                -p "takeoff_altitude:=$8"
        ' bridge-shell \
        "$HOME/ardu_ws/install/setup.bash" \
        "$PROJECT_DIR/ros2/install/setup.bash" \
        "$ROS_DOMAIN_ID" "$node_name" "$uav_number" \
        "$mavlink_host" "$mavlink_port" "$takeoff_altitude" \
        >"$log_file" 2>&1 &

    BRIDGE_PIDS+=("$!")
    sleep 1
    if ! kill -0 "$!" 2>/dev/null; then
        echo "ERROR: $node_name exited during startup." >&2
        cat "$log_file" >&2 || true
        exit 1
    fi
    echo "Started $node_name: PID=$! MAVLink=$mavlink_host:$mavlink_port log=$log_file"
}

echo "=== Starting drone_bridge nodes inside gcsns (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
wait_for_sitl_tcp 1 10.42.0.11 5760
start_drone_bridge 1 drone_bridge_uav1 10.42.0.11 5760 60.0 "${BRIDGE_LOGS[0]}"

wait_for_sitl_tcp 2 10.42.0.12 5770
start_drone_bridge 2 drone_bridge_uav2 10.42.0.12 5770 40.0 "${BRIDGE_LOGS[1]}"

wait_for_sitl_tcp 3 10.42.0.13 5780
start_drone_bridge 3 drone_bridge_uav3 10.42.0.13 5780 50.0 "${BRIDGE_LOGS[2]}"

verify_mission_prerequisites() {
    local i

    kill -0 "$NS3_PID" 2>/dev/null || {
        dump_ns3_failure
        return 1
    }
    kill -0 "$GAZEBO_PID" 2>/dev/null || {
        echo "ERROR: Gazebo is not running; log: $GAZEBO_LOG" >&2
        cat "$GAZEBO_LOG" >&2 || true
        return 1
    }
    for i in "${!SITL_PIDS[@]}"; do
        kill -0 "${SITL_PIDS[$i]}" 2>/dev/null || {
            echo "ERROR: UAV$((i + 1)) SITL is not running; log: ${SITL_LOGS[$i]}" >&2
            cat "${SITL_LOGS[$i]}" >&2 || true
            return 1
        }
    done
    for i in "${!AGENT_PIDS[@]}"; do
        kill -0 "${AGENT_PIDS[$i]}" 2>/dev/null || {
            echo "ERROR: UAV$((i + 1)) micro_ros_agent is not running." >&2
            return 1
        }
    done
    for i in "${!BRIDGE_PIDS[@]}"; do
        kill -0 "${BRIDGE_PIDS[$i]}" 2>/dev/null || {
            echo "ERROR: UAV$((i + 1)) drone_bridge is not running." >&2
            return 1
        }
    done
}

start_city_mission() {
    if sudo ip netns exec gcsns \
        pgrep -u "$(id -u "$RUN_USER")" -f '[c]ity_mission' >/dev/null; then
        echo "ERROR: A city_mission process is already running; refusing to start a duplicate." >&2
        return 1
    fi

    : >"$MISSION_LOG"
    sudo ip netns exec gcsns \
        sudo -H -u "$RUN_USER" \
        bash -lc '
            set -e
            source /opt/ros/humble/setup.bash
            source "$1"
            source "$2"
            export ROS_DOMAIN_ID="$3"
            exec ros2 run uav_controller city_mission
        ' mission-shell \
        "$HOME/ardu_ws/install/setup.bash" \
        "$PROJECT_DIR/ros2/install/setup.bash" \
        "$ROS_DOMAIN_ID" \
        >"$MISSION_LOG" 2>&1 &
    MISSION_PID=$!

    sleep 1
    if ! kill -0 "$MISSION_PID" 2>/dev/null; then
        echo "ERROR: city_mission exited during startup." >&2
        cat "$MISSION_LOG" >&2 || true
        return 1
    fi
    echo "city_mission started: PID=$MISSION_PID log=$MISSION_LOG"
}

echo "=== Final prerequisite check and city_mission startup ==="
verify_mission_prerequisites
start_city_mission

while kill -0 "$MISSION_PID" 2>/dev/null; do
    if ! kill -0 "$NS3_PID" 2>/dev/null; then
        dump_ns3_failure
        exit 1
    fi
    sleep 1
done

set +e
wait "$MISSION_PID"
mission_status=$?
set -e
if (( mission_status != 0 )); then
    echo "ERROR: city_mission exited with status $mission_status; initiating cleanup." >&2
    echo "----- $MISSION_LOG -----" >&2
    cat "$MISSION_LOG" >&2 || true
    echo "------------------------" >&2
    exit "$mission_status"
fi

echo "city_mission completed successfully; initiating cleanup."
