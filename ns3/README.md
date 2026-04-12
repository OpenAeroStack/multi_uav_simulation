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

## Notes

- The scenario is intentionally minimal and intended as a baseline for wireless and mobility-specific extensions.
- It runs in real time and is suitable for connecting external ROS 2 DDS participants through TAP interfaces.
