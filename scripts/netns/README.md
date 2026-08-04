# netns pipeline — setup notes & troubleshooting

Notes from getting `launch_single_uav_netns.sh` running across multiple
machines. Everything here is stuff that is **not** in the code and **not**
in `ardu_ws`'s own README — it's host/environment setup that has to exist
before this script will work, plus the failure modes we actually hit while
setting it up on new machines.

## External dependencies (not in this repo)

This script assumes two things already exist on the host, built and
installed separately:

1. **`ardu_ws`** — ArduPilot built with DDS support (`--enable-DDS`), plus
   ROS2/micro-ROS. Build exactly per that workspace's own README. The one
   pitfall worth repeating here: `microxrceddsgen` on `PATH` **must**
   resolve to ArduPilot's fork
   (`https://github.com/ArduPilot/Micro-XRCE-DDS-Gen`), not eProsima's
   upstream — the wrong one silently breaks AP_DDS code generation during
   `colcon build`. Verify with:
   ```bash
   readlink -f $(which microxrceddsgen)
   ```
   To confirm DDS actually got compiled into the binary you're about to run:
   ```bash
   strings "$ARDUPILOT_HOME/build/sitl/bin/arducopter" | grep -i "ap/v1/navsat"
   ```
   If that's empty, DDS isn't in the build and `DDS_ENABLE 1` in the params
   file will just be silently ignored.

2. **`ardupilot_gazebo`** — the Gazebo Classic plugin that actually
   implements the FDM protocol Gazebo and SITL talk over
   (`libArduPilotPlugin.so`). This is a **separate clone**, not part of
   `ardu_ws` or this repo:
   ```bash
   git clone https://github.com/khancyr/ardupilot_gazebo
   cd ardupilot_gazebo && mkdir build && cd build
   cmake .. && make -j4 && sudo make install
   ```
   This installs into the Gazebo plugin dir (e.g.
   `/usr/lib/x86_64-linux-gnu/gazebo-11/plugins/`), which `setup.sh` already
   points `GAZEBO_PLUGIN_PATH` at. No script changes needed — just the build.
   The model SDF (`models/iris_1_netns/model.sdf`) hard-requires
   `libArduPilotPlugin.so` by filename.

## Host configuration this script needs

**Firewall must allow traffic on the `sim1h`/`sim1n` link.** SITL and
Gazebo exchange physics/FDM data over a dedicated veth pair
(`172.31.1.1` root ns ↔ `172.31.1.2` inside `uav1ns`), *not* over loopback.
If the host firewall's default INPUT policy is deny (`ufw` active with its
default rules, or a bare `iptables -P INPUT DROP`), it silently drops that
traffic — Gazebo never receives SITL's packets even though both processes
are alive and the interface shows the packets on the wire (`tcpdump` sees
them; the app-level socket never does, because netfilter drops them first).

Fix:
```bash
sudo ufw allow in on sim1h
```
(specific one "sudo ufw allow in on sim1h to any port 9002 proto udp")
(Or make sure `ufw` is inactive if you're not relying on it.) This is a
persistent `ufw` rule — it survives reboot as long as the `ufw` service
itself is enabled (`systemctl is-enabled ufw`). It does **not** need to be
re-applied per terminal/session.

This is the single easiest failure mode to misdiagnose, because everything
*looks* fine: Gazebo starts, SITL starts, the port is bound
(`ss -uln | grep 9002` shows it), and there's no error anywhere — SITL just
sits forever inside `SITL::Gazebo::recv_fdm()` waiting for a reply that
never comes, and nothing downstream (MAVLink heartbeat, DDS, GPS) ever
starts because ArduPilot never gets past that point in its own boot
sequence.

## Known quirks in the script itself

- **The `strace -f -e trace=none -o /dev/null` wrapper around the SITL
  launch (Step 6) is intentional, not leftover debug cruft.** On some
  machines, SITL would crash silently right after printing
  `Loaded defaults from ...` — a startup race condition (most likely the
  physics veth link or the Gazebo plugin's socket not being fully ready
  yet). Wrapping the process in `strace` slows it down just enough (ptrace
  overhead) to avoid the race. It's fragile in principle (depends on
  `strace` being installed and on that overhead staying "enough"), but it's
  confirmed to fix the issue on every machine that hit it. Don't remove it
  without re-testing on a machine that needs it.

- **`cleanup()` only runs on Ctrl-C (`trap cleanup INT TERM`), not on a
  normal script exit.** If any of the readiness checks fail and the script
  exits via `exit 1` (e.g. the DDS/navsat timeout), ns-3, Gazebo, the
  micro-ROS agent, and SITL are all left running in the background. Before
  re-running after a failure, clean up manually:
  ```bash
  sudo pkill -9 -f arducopter
  sudo pkill -9 -f gzserver
  sudo pkill -9 -f gzclient
  sudo pkill -9 -f micro_ros_agent
  sudo pkill -9 -f three_uav_tapbridge_integrated
  sudo ss -ulnp | grep -E ":9002|:9003"   # confirm nothing stale is still bound
  ```
  A stale/orphaned Gazebo instance squatting on `172.31.1.1:9002` from a
  previous failed run looks identical to the firewall problem above (bound
  port, no reply) — check for leftovers before assuming it's the firewall.

- **The pre-flight cleanup's `pkill -f` self-matches.** Step 0 runs
  `sudo pkill -9 -f -- "$pattern"` for each process name — but `pkill -f`
  matches against the *full command line*, and its own invocation contains
  the pattern text, so it kills itself. This is why you see
  `line 47: NNNN Killed sudo pkill -9 -f -- "$pattern"` printed on every
  run. It's cosmetic and harmless on its own, but it means the intended
  target may not always actually get signaled — if you suspect a stale
  process survived cleanup, verify with `pgrep -af <name>` rather than
  trusting the cleanup step silently.

## Setting this up on a new machine — checklist

1. Build `ardu_ws` per its own README (ArduPilot + DDS + ROS2 workspace).
   Confirm `microxrceddsgen` resolves to ArduPilot's fork before building.
2. Build and `sudo make install` `ardupilot_gazebo` (separate clone, see
   above). Confirm `libArduPilotPlugin.so` lands in the Gazebo plugin dir.
3. Edit `setup.sh` at the repo root — set `ARDUPILOT_HOME` to wherever
   `ardu_ws/src/ardupilot` actually lives on this machine.
4. Make sure the firewall allows the `sim1h` link (see above) — either
   `sudo ufw allow in on sim1h`, or confirm `ufw`/`iptables` INPUT policy is
   permissive.
5. Run `bash scripts/netns/launch_single_uav_netns.sh`.

## Troubleshooting quick-reference

| Symptom | Likely cause | Check |
|---|---|---|
| SITL dies silently right after `Loaded defaults from ...`, `kill -0` fails | Startup race condition | Confirm the `strace` wrapper is still in Step 6 |
| `strings` on the arducopter binary shows no `ap/v1/navsat` | DDS not compiled in | Check `microxrceddsgen` fork, rebuild `ardupilot_sitl` |
| SITL alive, FDM port "open", but no MAVLink heartbeat / no DDS publisher ever | Main thread stuck in `Gazebo::recv_fdm()` — physics link not delivering | `gdb -p <arducopter PID> -batch -ex "thread apply all bt"` (kill the `strace` wrapper first, only one tracer allowed) |
| Above backtrace confirmed, and `sudo ss -uln \| grep 9002` shows nothing bound | `ArduPilotPlugin` never loaded/bound | `grep -in "plugin\|ArduPilotPlugin" /tmp/gazebo_netns.log`, confirm the plugin build/install |
| Above backtrace confirmed, port **is** bound, packets visible with `tcpdump` on `sim1h`/`sim1n` but never received by the app | Firewall dropping the traffic, or a stale orphaned process holding the port | Check `ufw status` / `iptables -S INPUT`; check `pgrep -af gazebo` for leftovers from a previous failed run |
| `/tmp/gazebo_netns.log` never shows `"ArduPilot controller online detected"` | Plugin never received a packet from SITL (same firewall/orphan check as above) | Compare against a known-good log where this line does appear |
