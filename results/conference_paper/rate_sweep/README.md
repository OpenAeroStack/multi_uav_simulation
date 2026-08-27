# Conference-paper Edge vs Ground rate sweep

This experiment compares Edge and Ground processing at offered application
rates of 1, 2, and 5 Hz. Ground uses independently encoded JPEG quality 5
frames. Detector configuration is fixed at YOLOv8n, confidence 0.25,
`imgsz=960`, and person class 0.

The runner preserves the Phase F observation pose and the existing QoS, DDS,
MTU, socket-buffer, and NS-3 configuration. It performs no transport tuning.

## Run one condition

Start the existing single-UAV infrastructure with the required RNG run, move
UAV1 to the fixed Phase F pose, and then run:

```bash
bash results/conference_paper/rate_sweep/run_rate_once.sh \
  <edge|ground> <run_id> <rng_run> <1|2|5>
```

Run IDs are immutable: the runner refuses to overwrite an existing raw run
directory or processed CSV.

## Measurement definition

After DDS endpoint discovery, the runner waits a fixed five seconds. The
official window is then exactly 60 seconds and never waits for a detector
result. The relay is stopped at the window boundary, followed by a fixed
five-second detector drain.

The denominator in both modes is the relay sequence IDs published during the
official wall-clock interval. Edge sequence IDs travel in the raw ROS Image
header; Ground sequence IDs travel in the CompressedImage header. Detector
events are matched to those IDs, so drain-period completions of official
frames are retained while pre-window and post-window publications are
excluded. Missing downstream frames remain explicit rows in the frame trace.

TAP byte and packet deltas describe total interface traffic, including DDS,
telemetry, discovery, and other traffic; they are not pure image-traffic
counters. P95 latency uses linear interpolation and no outlier removal.

## Outputs

- `raw/<run_id>/`: relay/detector events and logs, raw `pidstat`, TAP snapshots,
  interface/kernel configuration, NS-3 log, official-start pose, and metadata.
- `processed/frame_trace_<run_id>.csv`: one row per official relay publication.
- `processed/rate_summary_<run_id>.csv`: one summary row for the run.
- `processed/tap_deltas_<run_id>.csv`: validated interface-counter deltas.

No new telemetry logger is used. The saved pose/NavSat messages document the
vehicle state available at the official start.
