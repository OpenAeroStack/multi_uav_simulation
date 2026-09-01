# Real Mission Planner TLOG extractor

`extract_real_tlog.py` converts a real Mission Planner MAVLink `.tlog` into
analysis-ready CSV datasets without modifying the source log. It is standalone
and does not need to be added to or built in the ROS 2 workspace.

## Run with the default input

```bash
python3 scripts/ground_truth_test/extract_real_tlog.py
```

The default input is:

```text
/home/randilsk/FYP/logs_using/0-30m (copy)/2026-08-31 18-03-14.tlog
```

## Run with an explicit input

Quote paths containing spaces or parentheses:

```bash
python3 scripts/ground_truth_test/extract_real_tlog.py \
  "/home/randilsk/FYP/logs_using/0-30m (copy)/2026-08-31 18-03-14.tlog"
```

By default, output is created under
`/home/randilsk/FYP/logs_using/processed_real_logs/`. Existing output is never
overwritten; a numeric suffix is added when the expected run directory exists.

## Use an agreed comparison origin

Without origin arguments, the first valid `GLOBAL_POSITION_INT` sample defines
the local ENU origin. To compare real and simulated positions, give both datasets
the same WGS84 origin:

```bash
python3 scripts/ground_truth_test/extract_real_tlog.py \
  "/home/randilsk/FYP/logs_using/0-30m (copy)/2026-08-31 18-03-14.tlog" \
  --origin-lat 6.0773722 \
  --origin-lon 80.1907552 \
  --origin-alt 29.48
```

`--origin-alt` is optional when latitude and longitude are supplied. If omitted,
the altitude from the first valid global-position sample is used.

## Scientific interpretation

- The primary common clock is the Mission Planner `.tlog` packet-arrival
  timestamp. Autopilot `time_boot_ms` and `time_usec` values remain separate.
- `vehicle_state.csv` is asynchronous: each row identifies its source message.
  Values from ATTITUDE, VFR_HUD, and HEARTBEAT are not artificially joined.
- `GLOBAL_POSITION_INT` velocity is NED: `vz_down_mps` is positive downward.
- `LOCAL_POSITION_NED` is kept in its original North-East-Down frame.
- Radio values are preserved as raw device-reported values; they are not labeled
  dBm or SNR.
- MAVLink sequence gaps are diagnostic estimates, not proof of RF packet loss.
  Time gaps are never converted into invented lost-packet counts.

Detailed units, coordinate origin, timing statistics, warnings, message counts,
and source provenance are recorded in each run's `metadata.json`.
