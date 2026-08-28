# Network-only multi-UAV scaling experiment

This experiment evaluates the existing four-node NS-3/TapBridge framework independently of the Edge-versus-Ground human-detection case study. It requires no Gazebo, SITL, flight DDS, camera, relay, detector, or YOLO process. ROS 2 is used only to provide the fixed position and zero-obstacle-loss control messages consumed by the existing integrated NS-3 executable.

## Topology

The unchanged integrated topology is node 0 = GCS, node 1 = UAV1, node 2 = UAV2, and node 3 = UAV3. Each Linux namespace has its own veth, isolated bridge, and TAP. All four NS-3 Wi-Fi devices use the same existing `YansWifiChannel` and ad-hoc 802.11a configuration.

The fixed validated coordinates are:

- GCS: `(0, 6, 2.9)`
- UAV1: `(0, 2, 20)`
- UAV2: `(0, -2, 20)`
- UAV3: `(7, 1, 18)`

All six obstacle-loss inputs are fixed at 0 dB. This geometry is deliberately asymmetric. Jain fairness therefore describes observed throughput allocation under this fixed validated geometry; it is not an equal-distance MAC-fairness benchmark.

## Traffic matrix

Each active UAV sends iperf3 UDP traffic to `gcsns` at 500 Kbit/s using the same default UDP datagram behavior as `scripts/test_scripts/iperf3_channel_test.sh`. UAV1, UAV2, and UAV3 use ports 5201, 5202, and 5203, respectively. Every official window is 30 seconds.

| Condition | Active UAVs | Aggregate offered load |
|---|---|---:|
| N1 | UAV1 | 0.5 Mbit/s |
| N2 | UAV1, UAV2 | 1.0 Mbit/s |
| N3 | UAV1, UAV2, UAV3 | 1.5 Mbit/s |

Official repetitions use RNG runs 1, 2, and 3, for nine runs total. The runner passes the requested value to NS-3 as `--rngRun=N`, which calls `RngSeedManager::SetRun` in the existing scenario.

## Setup and preflight

Provision a clean, isolated topology explicitly:

```bash
bash results/conference_paper/network_scaling/setup_network_scaling.sh --reset
```

Then run the bounded preflight:

```bash
bash results/conference_paper/network_scaling/preflight.sh
```

The preflight first verifies that communication fails while NS-3 is stopped. This negative control demonstrates that the isolated Linux bridges do not provide a bypass. It then launches its own NS-3 process, checks all three UAV-to-GCS paths, exercises one 500-Kbit/s UDP flow, checks position/link integration, and compares simulated-time progress with wall time. It never starts an official run.

## Official runs

```bash
bash results/conference_paper/network_scaling/run_network_scaling_once.sh 1 network_n1_rng1 1
```

Use unique run IDs. The runner refuses to overwrite existing raw or processed results and refuses to run while another integrated NS-3 process exists.

All iperf3 servers are started first. Client wrapper processes are then placed in every active UAV namespace with the same future epoch release time. Each wrapper waits independently until that boundary and records its actual start immediately before executing iperf3. The processor rejects a run if client starts differ by more than 50 ms, begin outside the shared boundary tolerance, or any client runs for less than 29.5 seconds.

TAP counters are captured immediately around the common traffic interval. TAP measurements contain all interface traffic, including ARP and iperf3 control traffic; per-flow application goodput and loss come from the independent iperf3 JSON outputs.

## Outputs

For each run, `raw/<run_id>/` contains iperf3 JSON, server logs, client start/end timestamps, TAP snapshots, fixed-feed logs, NS-3 logs, and metadata. `processed/` contains one per-UAV CSV, one run-summary CSV, and one TAP-delta CSV.

After every successful run the processor rebuilds:

- `final/all_runs.csv`
- `final/aggregate_by_uav_count.csv`
- aggregate offered load and goodput plot
- aggregate packet-loss plot
- per-UAV received-goodput plot
- Jain-fairness plot

Plots retain individual RNG observations and show mean ± sample SD where a grouped mean is plotted.

## Interpretation constraint

Older Phase B measurements reported an approximately 3 Mbit/s goodput ceiling, whereas the current integrated source documents an approximately 1.4 Mbit/s sustainable real-time application limit on this host. Available simulated-time versus wall-time progress is archived and summarized. If N3 falls materially behind real time, its limitation must be described as an end-to-end framework throughput limitation under the evaluated host configuration, not as pure wireless capacity.
