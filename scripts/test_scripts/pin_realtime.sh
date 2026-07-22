#!/bin/bash
# pin_realtime.sh
# ---------------
# Improves real-time fidelity of the Gazebo <-> NS-3 federation by (1) forcing
# the CPU to max clock and (2) giving the latency-sensitive processes dedicated
# cores so they don't fight the OS scheduler.
#
# CPU layout on this host (Intel Core 7 150U, hybrid):
#   P-cores (5.4 GHz): 0,1  and  2,3      <- real-time / latency sensitive
#   E-cores (4.0 GHz): 4..11              <- background
#
# Allocation:
#   gzserver (physics + raycast) -> P-core 0,1
#   NS-3 (RealtimeSimulatorImpl) -> P-core 2,3   (pin at launch, see below)
#   world_pos_publisher          -> E-core 4
#   arducopter x3                -> E-cores 5,6,7
#   (leaves 8..11 for ROS daemon, mission, OS)
#
# WHEN TO RUN:
#   - governor: once, BEFORE the run (sticky until reboot).
#   - pinning : AFTER launch_multi_uav_new.sh has started everything, because
#     those processes are spawned with fresh PIDs each run. Re-run this after
#     any restart. Changing affinity of your own processes needs no sudo; the
#     governor change needs sudo.
#
# Pin NS-3 itself at launch instead of here (it's started manually):
#   taskset -c 2,3 \
#     ~/ns-3.3/build/scratch/multi_uav_simulation/ns3.38-three_uav_tapbridge_obstacle_loss-default \
#     --tapBase=tap-uav

set -uo pipefail

echo "=== CPU governor -> performance (needs sudo) ==="
sudo cpupower frequency-set -g performance >/dev/null 2>&1 \
  && echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)" \
  || echo "WARN: could not set governor (need sudo / cpupower)"

pin() {  # pin <cpus> <pgrep-pattern> <label>
  local cpus=$1 pat=$2 label=$3 n=0
  for p in $(pgrep -f "$pat"); do
    [ "$p" = "$$" ] && continue
    if taskset -cp "$cpus" "$p" >/dev/null 2>&1; then
      echo "  $label PID $p -> CPUs $cpus"; n=$((n+1))
    fi
  done
  [ "$n" -eq 0 ] && echo "  ($label: no process matched '$pat' — is the stack running?)"
}

echo "=== pinning processes ==="
pin 0,1   'gzserver'             "gzserver"
pin 2,3   'ns3.38-three'         "NS-3"
pin 4     'world_pos_publisher'  "relay"
# each arducopter to its own E-core (5,6,7); pin them together is also fine
i=5
for p in $(pgrep -f 'arducopter'); do
  taskset -cp "$i" "$p" >/dev/null 2>&1 && echo "  SITL PID $p -> CPU $i"
  i=$((i+1)); [ "$i" -gt 7 ] && i=7
done

echo "=== done. Verify with: for p in \$(pgrep -f 'ns3.38-three|gzserver|arducopter'); do taskset -cp \$p; done ==="
