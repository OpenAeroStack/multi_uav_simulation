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


Good call — you've got a strong, well-evidenced limitations write-up already (5 candidate causes tested with real methodology, 2 confirmed real bugs found and fixed along the way), and further chasing this has diminishing returns against your timeline. Let's close it out and move to A3.

Manifest note for the ns-3 floor investigation, final version:

Emulation floor: management veth = 0.032ms; NS-3 in path = ~150-160ms across all conditions tested. Ruled out with direct evidence: artificial delay (--delayMs, source-confirmed off), propagation physics (negligible at these ranges), OS CPU scheduling (chrt -f 50 real-time priority, no improvement), frozen mobility positions (separate bug found and fixed via world_pos_publisher.py), synchronous ROS blocking in the main loop (source-confirmed rclcpp::spin() runs on a dedicated thread, ApplyFeed() uses a short, non-blocking mutex critical section). SynchronizationMode confirmed BestEffort (ns-3 3.38 default) — falls behind wall-clock time with no cap, matching the observed pattern of highly variable per-direction hop times (6-120ms) summing to a roughly consistent total. Tap-level packet capture confirms the delay occurs entirely within NS-3's simulated channel traversal (destination-side reply generation measured at 14-34µs). Leading unconfirmed hypothesis: real-time computational cost of the WiFi PHY channel model (Nakagami fading, SNR/interference calculations) exceeding available real-time budget on this hardware. Not fully isolated within project scope.


| A3 | phase_a_apparatus/raw/metrics_uav1_edge_a3_loopback_20260804_190641.csv | A | a3_loopback | n/a | edge (loopback) | detector.py + metrics_logger.py, both in uav1ns, drone stationary | A3 methodology sentence | overhead=3.7ms mean, sd=0.5ms, n=86 — instrument validated |

Friis validation: R²=0.860, RMSE=0.77dB, mean bias=-0.77dB across 5 confirmed distances (58.5-113.8m). Residuals near-constant (0.03dB spread), indicating the model correctly reproduces distance-dependent decay; the consistent offset attributed to Nakagami-m fading (mLos=3.0, source-confirmed default) applied on top of deterministic Friis. 150,393 packets, nodes 0/1 only.


Here's the B2 write-up, ready to use:

Table:

Distance (m)	Mean RTT (ms)	Min–Max (ms)	Packet loss
58.5	159.8	129–197	0%
69.5	153.3	119–185	0%
83.9	159.4	130–194	0%
97.7	159.9	135–204	0%
113.8	158.1	127–212	0%

Interpretation (paste-ready):

RTT vs distance shows no meaningful trend across the tested range (58.5–113.8m) — mean RTT stays within a narrow 153.3–159.9ms band, and all five points show 0% packet loss. This is consistent with the A2 emulation floor (~150–160ms) dominating over any real propagation-delay effect at this scale, matching what B1's Friis validation independently confirmed: signal strength stays well above the sensitivity threshold (16+ dB margin) throughout this range, so the link never approaches a distance-limited regime where RTT would actually be expected to change.

Manifest entry:

B2: RTT vs distance, same 5 confirmed positions as B1 (58.5–113.8m). Mean RTT 153–160ms, 0% loss at all points — flat, no distance trend, consistent with A2's characterized emulation floor. Source: manual ping -c 30 bursts run immediately following each B1 distance confirmation (same session, same log).

Offered	Received	Loss
2 Mbps	1.69 Mbps	11%
5 Mbps	2.85 Mbps	40%
8 Mbps	3.02 Mbps	59.6%

Throughput testing shows a soft goodput ceiling of approximately 3 Mbps (against a configured 6 Mbps PHY rate), consistent with typical 802.11 MAC-layer overhead. Notably, even at an offered rate well below this ceiling (2 Mbps), 11% packet loss was observed — indicating a baseline, congestion-independent loss rate attributable to the Nakagami-m fading model characterized in B1, rather than queue-driven drops. Loss increases sharply above the ~3 Mbps ceiling (40% at 5 Mbps offered, 59.6% at 8 Mbps offered), consistent with congestion-driven drops once the channel's usable capacity is exceeded.

B4: Packet loss vs distance at 2 Mbps offered rate (sub-ceiling), 5 positions (58.5–113.8m). TAP-counter loss: 13.25–13.96% (flat, no distance trend). iperf3-reported loss: 11.0–12.0% (flat, no distance trend). Consistent ~2pp gap between TAP and application-level loss attributed to kernel/link-level drops occurring before the socket layer. Confirms the baseline loss found in B3 (11% at 113.8m) is not distance-specific — same rate holds throughout the tested range, consistent with B1's finding of near-constant SNR margin.

Condition	Messages	Rate	Max gap
C1 baseline (static, no vision)	253	2.11 Hz	1994ms
C0 mission (moving, no vision)	274	2.28 Hz	2243ms
C2 edge (valid run)	247	2.06 Hz	2355ms
C3 ground	184	1.53 Hz	3788ms

Phase C: DDS telemetry health under load. C1 baseline: 2.11Hz, max gap 1994ms. C0 (mission active, no vision): 2.28Hz, max gap 2243ms. C2 (edge): 2.06Hz, max gap 2355ms. C1/C0/C2 cluster closely — neither active navigation traffic nor edge-mode vision load meaningfully degrades GPS telemetry. C3 (ground): 1.53Hz (27% below baseline), max gap 3788ms (90% above baseline) — the only condition showing substantial degradation, consistent with ground mode's large compressed-frame payloads competing with telemetry for the same channel capacity. (One earlier C2 trial was excluded — contaminated by a concurrent metrics_logger process adding its own DDS discovery overhead.)

