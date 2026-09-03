# Wireless-channel validation

All new channel-validation data, analysis code, and plots live under this
directory. The ns-3 propagation and PHY configuration is unchanged.

## Record a run

The single-UAV launcher accepts validation settings through environment
variables. From the project root, start a 20 ms GCS-to-UAV1 recording with:

```bash
VALIDATION_ENABLED=true \
VALIDATION_LINK=0-1 \
VALIDATION_INTERVAL_MS=20 \
VALIDATION_OUTPUT="$PWD/results-network/data/los/los_50m.csv" \
bash scripts/netns/launch_single_uav_netns.sh
```

Use a different output filename for every run. Stop the launcher with Ctrl+C
after the required hover period. The CSV is flushed at least once per second.

The focused CSV contains **periodic model evaluations**, not successful packet
reception measurements. Each faded RSSI is returned by the actual configured
propagation chain and therefore contains one custom Nakagami realization. The
logger separately evaluates the deterministic log-distance stage to expose the
path-loss-only and after-obstacle baselines.

## Analyze

Install the required Python packages if necessary: `pandas`, `numpy`, and
`matplotlib`. SciPy is optional and enables the Gamma maximum-likelihood fit.

```bash
python3 results-network/scripts/analyze_channel_validation.py \
  results-network/data/los/los_50m.csv \
  --output-dir results-network/plots/los_50m
```

Compare separate LoS and NLoS runs by supplying both files:

```bash
python3 results-network/scripts/analyze_channel_validation.py \
  results-network/data/los/los_50m.csv \
  results-network/data/nlos/nlos_50m.csv \
  --output-dir results-network/plots/los-vs-nlos-50m
```

For log-distance validation, provide all stationary-distance CSV files in one
command. The regression uses only `path_loss_only_rssi_dbm`:

```bash
python3 results-network/scripts/analyze_channel_validation.py \
  results-network/data/log-distance/*.csv \
  --output-dir results-network/plots/log-distance
```

Optional time filtering is available through `--start-time` and `--end-time`.
The script automatically removes rows recorded before both position and
obstacle reports were received.

If the analysis dependencies are not installed, create a local environment:

```bash
python3 -m venv results-network/.venv
results-network/.venv/bin/pip install pandas numpy matplotlib scipy
```

Then substitute `results-network/.venv/bin/python` for `python3` in the
analysis commands.

## Minimal experiment procedure

### A. LoS Nakagami

1. Ensure the Gazebo ray between the GCS and UAV1 is clear.
2. Launch validation to `data/los/los_50m.csv`.
3. Use `scripts/mission/goto_comparison_pose.py` or manual control to hold UAV1
   at approximately 50 m link distance.
4. Wait for the position to settle, record about 60 seconds, and press Ctrl+C.
5. Analyze that CSV. Its usable rows should report `LoS` and configured `m=3`.

### B. NLoS Nakagami

1. Choose a hover point at approximately the same distance with a building
   intersecting the GCS-to-UAV1 ray.
2. Launch validation to `data/nlos/nlos_50m.csv`.
3. Hold position for about 60 seconds and press Ctrl+C.
4. Analyze the LoS and NLoS files together. NLoS rows should report configured
   `m=1`; do not combine the states into one fitted distribution.

### C. Log-distance

1. Record separate stationary files near 20, 40, 60, 80, and 100 m under
   `data/log-distance/` (or record one continuous run with clear hover periods).
2. Keep the link clear when possible; the regression uses the obstacle-free
   `path_loss_only_rssi_dbm` column regardless.
3. Analyze all files in one command using the wildcard example above. The
   expected fitted exponent is approximately 2.0.

## Integrated framework validation

This experiment validates that the integrated Gazebo–ROS 2–ns-3–TapBridge
framework produces the expected network-level response when a simulated
physical obstruction changes the wireless channel state. It does not claim
validation of a real-world UAV radio channel.

### 1. Launch the framework logger

```bash
cd ~/FYP/multi_uav_sim
FRAMEWORK_VALIDATION=true \
FRAMEWORK_VALIDATION_LINK=0-1 \
FRAMEWORK_VALIDATION_INTERVAL_MS=100 \
FRAMEWORK_VALIDATION_OUTPUT="$PWD/results-network/data/framework-validation/run1/framework.csv" \
SNR_LOG="$PWD/results-network/data/framework-validation/run1/packet_snr.csv" \
bash scripts/netns/launch_single_uav_netns.sh
```

Wait for `PIPELINE READY`. The framework CSV combines periodic channel-model
evaluations with cumulative and interval `WifiPhy` trace counters. PHY counters
are node-level radio events, not application packet-delivery measurements.

### 2. Start actual wireless-side UDP traffic

In another terminal:

```bash
cd ~/FYP/multi_uav_sim
bash results-network/scripts/run_framework_validation_traffic.sh run1 120 1.0
```

This sends sequenced UDP datagrams at 1 Mbps from `gcsns` (`10.42.0.10`) to
`uav1ns` (`10.42.0.11`). It therefore crosses the GCS TAP, ns-3 Wi-Fi channel,
and UAV TAP. It does not use the `172.31.1.x` management link. Sent and received
sequence logs provide actual application PDR, loss, and received throughput.

While traffic runs:

1. Keep GCS–UAV1 clear for 20–30 seconds.
2. Fly UAV1 behind a building, preferably without greatly changing distance.
3. Confirm the ns-3 log changes from LoS (`m=3`) to NLoS (`m=1`).
4. Hold in NLoS for 30–60 seconds.
5. Optionally return to LoS, then let the traffic helper finish.
6. Stop the main launcher with Ctrl+C.

### 3. Analyze the synchronized run

```bash
source results-network/.venv/bin/activate
python3 results-network/scripts/analyze_framework_validation.py \
  results-network/data/framework-validation/run1/framework.csv \
  --sent-csv results-network/data/framework-validation/run1/udp_sent.csv \
  --received-csv results-network/data/framework-validation/run1/udp_received.csv \
  --ping-log results-network/data/framework-validation/run1/ping.log \
  --output-dir results-network/plots/framework-validation/run1
```

The main timeline aligns channel state and real packet outcomes using wall-clock
timestamps. A two-second settling region around each state transition is
excluded from LoS/NLoS summary comparisons by default.
