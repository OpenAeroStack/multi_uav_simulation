# Results Manifest

One row per artifact. Fill this in **as you collect**, not afterwards — this is
what makes "where did this number come from?" answerable months later.

## Environment record

Fill once per machine, update if anything changes.

| Item | Value |
|---|---|
| Machine | 	Lenovo ThinkStation P3 Tower |
| CPU / cores | Intel i7-13700, 16 cores / 24 threads |
| RAM | 	30 GiB total |
| OS | 	Ubuntu 22.04.5 LTS, kernel 5.15.0-186-generic |
| Gazebo version | 11.10.2 |
| ROS 2 distro | Humble |
| ns-3 version | 	3.38 |
| ArduPilot commit | ec09e44a09 (2026-04-28), ArduPilot-4.6 0-beta1-6579-gec09e44a09 |
| YOLO model / weights | YOLOv8n, yolov8n.pt (6.5 MB), ultralytics 8.4.115, torch 2.13.0+cu130 (CUDA present but unused — driver too old, runs CPU-only) |
| World file | 	small_city_single_uav_netns.world |
| Camera config (FOV / res / pitch) | FOV 0.9 rad, 1280×720, pitch 60° (1.0472 rad) from vertical, 20 Hz update|
|SITL home | 6.0790684, 80.1915283 |
|jpeg_quality (default) | 50 |
## Run log

| # | Artifact (path) | Phase | Run ID | RNG seed | Mode | Generating command | Target figure/table | Notes |
|---|---|---|---|---|---|---|---|---|
| 1 | | | | | | | | |

## Figure/table checklist

Tick as produced. ★ = load-bearing, V = validation figure.

### Methodology / apparatus
- [ ] 1. Table — emulation floor breakdown (A2)
- [ ] 2. ★ V6 — warm-up determination, Welch's method (A4)
- [ ] 3. V7 — autocorrelation across frame lags (A4) *(appendix candidate)*

### Network layer
- [ ] 4. Plot — SNR vs distance, Friis overlay (B1)
- [ ] 5. ★ V1 — predicted vs observed SNR, 1:1 line, R²/RMSE (B1)
- [ ] 6. ★ V2 — SNR residuals vs distance (B1)
- [ ] 7. Plot — RTT vs distance, emulation floor marked (B2)
- [ ] 8. Plot — throughput vs distance vs theoretical maximum (B3)

### Middleware
- [ ] 9. ★ Table — telemetry health, C0–C3 (C)

### Application
- [ ] 10. Table — bandwidth per mode (D1)
- [ ] 11. Plot — frame size across mission (D1)
- [ ] 12. ★ Table — detection recall/precision per mode (D4)
- [ ] 13. ★ Plot — JPEG quality sweep, dual axis (D5/D6)
- [ ] 14. ★ Plot — frame-rate saturation envelope, prediction marked (D7)
- [ ] 15. ★ V1 — predicted vs measured frame delivery ratio (D8)

### Comparison
- [ ] 16. ★ V3 — latency ECDFs, edge vs ground on shared axes (F)
- [ ] 17. V4 — box/violin plot, latency distribution per mode (F)
- [ ] 18. V5 — Q–Q plot justifying test choice (F) *(appendix)*
- [ ] 19. V8 — replication convergence with confidence intervals (F)
- [ ] 20. Table — CPU/memory per mode (E)
- [ ] 21. ★ Table — headline comparison (F)
- [ ] 22. Screenshot — annotated detection during flight

## Rules

- `raw/` is **never** edited or overwritten. If a run is bad, keep it and note
  why in the run log — a documented failed run is evidence, a deleted one is a gap.
- Every run gets a unique Run ID. Suggested format: `<phase>_<mode>_<n>`,
  e.g. `d1_ground_02`.
- Record the RNG seed for every ns-3 run. Repeats with the same seed are not
  independent replications.
- Note the warm-up exclusion window applied to any computed statistic.


Emulation floor: management veth (no NS-3) = 0.032ms. NS-3 in path = ~158-161ms regardless of separation (confirmed via live position tracking: 5.3m vs 146.8m, both from /tmp/ns3_single.log), statistically indistinguishable between the two. Verified not attributable to --delayMs (source-confirmed default off) or propagation delay (negligible at six orders of magnitude smaller than the measured floor at these ranges). Attributed to ns-3 real-time discrete-event scheduling and TapBridge overhead. Initial test session was contaminated by a missing world_pos_publisher.py (nodes frozen at static placeholder coordinates); fixed by adding the publisher as a permanent pipeline step, confirmed via live coordinate tracking before this result was accepted.