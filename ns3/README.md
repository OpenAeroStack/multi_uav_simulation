# NS-3 TapBridge Real-Time Skeleton (3 UAV)

This directory contains an initial NS-3 real-time skeleton for bridging three Linux TAP interfaces into an NS-3 network.

## Files

- `three_uav_tapbridge_rt.cc`: NS-3 scenario using `RealtimeSimulatorImpl` + `TapBridge`.

## Expected Linux Interfaces

The default TAP names expected by the scenario are:

- `tap-uav1`
- `tap-uav2`
- `tap-uav3`

Create these with:

```bash
sudo bash scripts/setup_netns_tap.sh
```

## Build and Run (inside your ns-3 tree)

Example with a local ns-3 checkout at `$NS3_HOME`:

```bash
export NS3_HOME=$HOME/ns-3-dev
cp ns3/three_uav_tapbridge_rt.cc "$NS3_HOME"/scratch/
cd "$NS3_HOME"
./ns3 build
sudo ./ns3 run "scratch/three_uav_tapbridge_rt --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=20 --lossRate=0.01"
```

For repeatable performance runs, set a finite duration and export FlowMonitor XML:

```bash
./ns3 run "scratch/three_uav_tapbridge_rt --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=50 --lossRate=0.02 --simDurationSec=60 --enableFlowMonitor=1 --flowmonXml=results.xml"
```

If your ns-3 version uses `./waf`:

```bash
./waf configure
./waf build
sudo ./waf --run "scratch/three_uav_tapbridge_rt --tap1=tap-uav1 --tap2=tap-uav2 --tap3=tap-uav3 --delayMs=20 --lossRate=0.01"
```

## Runtime Parameters

- `--tap1`, `--tap2`, `--tap3`: TAP names.
- `--delayMs`: link delay in milliseconds.
- `--lossRate`: per-device receive loss probability (0.0 to 1.0).
- `--dataRate`: channel data rate, default `50Mbps`.
- `--simDurationSec`: stop after N seconds (`0` means run continuously).
- `--enableFlowMonitor`: `1`/`0`, enable or disable FlowMonitor (default enabled).
- `--flowmonXml`: XML output path for FlowMonitor stats, default `three_uav_flowmon.xml`.

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
