# NS-3 TapBridge Real-Time Skeleton (3 UAV)

This directory contains an initial NS-3 real-time skeleton for bridging three Linux TAP interfaces into an NS-3 network.

This scenario produces two different XML outputs:

- **NetAnim XML** (for the NetAnim GUI): controlled by `--animFile` (default `three_uav_anim.xml`).
- **FlowMonitor XML** (for per-flow performance statistics): controlled by `--flowmonXml` (default `three_uav_flowmon.xml`).

These files are **not interchangeable**: NetAnim cannot open FlowMonitor XML.

## Files

- `three_uav_tapbridge_rt.cc`: NS-3 scenario using `RealtimeSimulatorImpl` + `TapBridge`.
- `CMakeLists.txt`: helper to build a `three-uav` executable in an ns-3 (CMake) tree.

**Note on TapBridge mode:** this scenario uses WiFi devices. WiFi devices do **not** support `SendFrom()`, so `TapBridge` must run in **UseLocal** mode (ns-3 has `tap-wifi-*` examples that do the same).

## Expected Linux Interfaces

The default TAP names expected by the scenario are:

- `tap-uav1`
- `tap-uav2`
- `tap-uav3`

Create these with:

```bash
# From the project root (multi_uav_simulation/):
sudo bash scripts/setup_netns_tap.sh
```

If you want an easy cross-UAV connectivity test (ping between namespaces), you can create the namespaces in a shared subnet:

```bash
sudo SHARED_SUBNET=1 bash scripts/setup_netns_tap.sh
```

## Build and Run (inside your ns-3 tree)

### ns-3.38+ (recommended, CMake build)

This repo includes a ready-to-use `CMakeLists.txt` that builds a `three-uav` program inside `scratch/three-uav/`.

Example with a local ns-3 checkout at `$NS3_HOME`:

```bash
export NS3_HOME=$HOME/ns-3-dev

# Copy the scenario into a scratch program directory
mkdir -p "$NS3_HOME"/scratch/three-uav
cp ns3/CMakeLists.txt "$NS3_HOME"/scratch/three-uav/
cp ns3/three_uav_tapbridge_rt.cc "$NS3_HOME"/scratch/three-uav/

cd "$NS3_HOME"
./ns3 build

# Run (Ctrl+C to stop if simDurationSec=0)
./ns3 run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=20 --lossRate=0.01 --simDurationSec=60 --animFile=three_uav_anim.xml --flowmonXml=three_uav_flowmon.xml"

# If you hit permission errors opening TAP devices, use ns-3's built-in sudo support
# (do NOT prefix the command with sudo):
./ns3 run --enable-sudo "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=20 --lossRate=0.01 --simDurationSec=60 --animFile=three_uav_anim.xml --flowmonXml=three_uav_flowmon.xml"
```

For repeatable performance runs, set a finite duration and export FlowMonitor XML:

```bash
sudo ./ns3 run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=50 --lossRate=0.02 --simDurationSec=60 --enableFlowMonitor=1 --flowmonXml=results.xml --animFile=anim.xml"

# Correct form (no sudo prefix):
./ns3 run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=50 --lossRate=0.02 --simDurationSec=60 --enableFlowMonitor=1 --flowmonXml=results.xml --animFile=anim.xml"
./ns3 run --enable-sudo "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=50 --lossRate=0.02 --simDurationSec=60 --enableFlowMonitor=1 --flowmonXml=results.xml --animFile=anim.xml"
```

### Older ns-3 (`./waf` build)

If your ns-3 tree uses Waf instead of `./ns3` (CMake), you can still use this scenario, but you must integrate it into your scratch programs the Waf way (depends on your ns-3 version/layout).

If you already have a Waf-based scratch program build path working, run it like:

```bash
./waf configure
./waf build
sudo ./waf --run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=20 --lossRate=0.01 --simDurationSec=60 --animFile=three_uav_anim.xml"
```

## Runtime Parameters

- `--tap1`, `--tap2`, `--tap3`: TAP names.
- `--delayMs`: link delay in milliseconds.
- `--lossRate`: per-device receive loss probability (0.0 to 1.0).
- `--dataRate`: channel data rate, default `50Mbps`.
- `--simDurationSec`: stop after N seconds (`0` means run continuously).
- `--enableFlowMonitor`: `1`/`0`, enable or disable FlowMonitor (default enabled).
- `--flowmonXml`: XML output path for FlowMonitor stats, default `three_uav_flowmon.xml`.
- `--animFile`: XML output path for NetAnim, default `three_uav_anim.xml`.
- `--tapMode`: TapBridge mode (`UseLocal` default; `UseBridge` requires devices that support `SendFrom()` and will fail with WiFi).

## How To Verify NS-3 + TapBridge Is Working

You should verify **two things**:

1. The scenario starts and detects/opens all TAP devices.
2. Packets can flow between the TAPs through the NS-3 WiFi channel.

### A) Confirm TAP devices exist

From the host:

```bash
ip link show tap-uav1
ip link show tap-uav2
ip link show tap-uav3
```

### B) (Recommended) Ping between namespaces (shared subnet)

1) Create the namespaces/TAPs in shared-subnet mode:

```bash
sudo SHARED_SUBNET=1 bash scripts/setup_netns_tap.sh
```

2) Start NS-3 for a short, finite duration (so XML files finalize):

```bash
sudo ./ns3 run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --simDurationSec=30 --animFile=/tmp/three_uav_anim.xml --flowmonXml=/tmp/three_uav_flowmon.xml"

# Correct form (no sudo prefix):
./ns3 run "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --simDurationSec=30 --animFile=/tmp/three_uav_anim.xml --flowmonXml=/tmp/three_uav_flowmon.xml"
./ns3 run --enable-sudo "three-uav --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --simDurationSec=30 --animFile=/tmp/three_uav_anim.xml --flowmonXml=/tmp/three_uav_flowmon.xml"
```

3) While NS-3 is running, generate traffic:

```bash
ip netns exec uav1 ping -c 5 10.42.0.12
ip netns exec uav2 ping -c 5 10.42.0.13
```

If the pings succeed, TapBridge forwarding through NS-3 is working.

## Open the NetAnim XML

NetAnim opens the file produced by `--animFile` (example: `/tmp/three_uav_anim.xml`).

If you installed NetAnim via `ns-allinone`, it is usually located under your ns-allinone directory and can be started like:

```bash
cd ~/ns-allinone-*/netanim-*/
./NetAnim
```

Then in the GUI:

- **File → Open** → select `three_uav_anim.xml` (or `/tmp/three_uav_anim.xml`).

## What You Get From FlowMonitor

## What You Get From FlowMonitor

At the end of the run, the scenario prints:

- Throughput (`throughputMbps`)
- Average delay (`avgDelayMs`)
- Average jitter (`avgJitterMs`)
- Packet loss (`lossPct`)

It also writes detailed per-flow metrics to `results.xml` (or your configured file).

## Feeding UAV Positions Into NS-3

This baseline file does not yet consume external position streams, but the recommended integration path is:

1. Gazebo publishes UAV pose updates.
2. A bridge process forwards pose updates to NS-3 in real time.
3. NS-3 updates each node's `MobilityModel` from those updates.

Transport options for step 2:

- UDP socket bridge: simple and low-overhead.
- Shared memory: lowest latency on one machine.
- ROS 2 bridge: best when the rest of your stack is ROS 2.

For your current project, ROS 2 bridge is usually the most maintainable choice.

## Notes

- The scenario is intentionally minimal and intended as a baseline for wireless and mobility-specific extensions.
- It runs in real time and is suitable for connecting external ROS 2 DDS participants through TAP interfaces.
