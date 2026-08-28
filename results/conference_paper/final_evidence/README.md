# Conference Paper Evidence Package

## 1. Scope

This package freezes conference-paper evidence reconstructed from repository CSV files, JSON metadata, raw logs, frame traces, and source configuration. No numerical result was copied from the FYP report, presentation slides, or draft paper prose. Derived statistics record their source, calculation, and sample count in `data/`.

The audit deliberately reports missing evidence. In particular, the retained received-power log supports only one of the five configured distance windows, and the retained UDP logs are five fixed-load runs rather than an offered-load sweep. Historical multi-distance fit statistics and a multi-rate saturation curve are therefore **NOT VERIFIED FROM RAW REPOSITORY ARTIFACTS**.

## 2. Included experiments

### Network validation

- Received-power source: `results/phase_b_network/raw/ns3_snr_obstaclefree_20260804_220047.csv`.
- Friis parameters and confirmed windows: `results/scripts/b1_friis_validation.py`.
- Fixed-load UDP sources: the five `results/phase_b_network/raw/iperf_*.txt` logs.
- Independent TAP loss cross-check: `results/phase_b_network/processed/b4_loss_summary.csv`.

The received-power CSV ends near simulation time 740.2914 s. Only three valid node-0/node-1 packet samples in the configured 58.5 m, 728–770 s window are recoverable. The later 69.5–113.8 m windows contain no retained samples. Consequently, R² and a distance trend cannot be recalculated. The available three packets give expected Friis power −62.071485 dBm, observed mean −80.412067 dBm, packet-level RMSE 18.341013 dB, and mean bias −18.340582 dB. These values characterize only that retained fragment and must not be generalized.

The five complete iperf logs each offered 2 Mbit/s for 10 s. Their receiver goodputs are 1.67–1.69 Mbit/s and receiver-reported losses are 11–12%. Mean goodput is 1.682 Mbit/s and mean loss is 11.2% across five runs. Because offered load does not vary, these files cannot establish a saturation curve as offered load increases.

### JPEG quality sweep

The full run is `conference_qs_full_01`, sourced from:

- `results/conference_paper/quality_sweep/processed/quality_sweep_details_20260827_095558_UTC.csv`
- `results/conference_paper/quality_sweep/processed/quality_sweep_summary_20260827_095558_UTC.csv`
- `results/conference_paper/quality_sweep/processed/quality_sweep_metadata_20260827_095558_UTC.json`
- `results/conference_paper/quality_sweep/input/ground_truth_clean.csv`

The clean CSV has 60 rows and 60 unique filenames. The historical source has 61 rows because `frame_0033.png,5,` occurs twice; only the conference copy removes one identical row. The run used 60 original PNG frames, YOLOv8n, confidence 0.25, image size 960, and COCO class 0. Each JPEG quality was encoded independently from its original PNG. Evaluation is count-based, not bounding-box IoU or mAP.

### Edge/Ground rate experiment

Exactly these 12 complete official runs are included:

- `rate_edge_1hz_rng1`, `rate_edge_1hz_rng2`, `rate_edge_1hz_rng3`
- `rate_ground_1hz_rng1`, `rate_ground_1hz_rng2`, `rate_ground_1hz_rng3`
- `rate_edge_2hz_rng1`, `rate_edge_2hz_rng2`, `rate_edge_2hz_rng3`
- `rate_ground_2hz_rng1`, `rate_ground_2hz_rng2`, `rate_ground_2hz_rng3`

Each was verified to have a one-row processed summary and associated raw logs/traces. Aggregates use run-level means and sample standard deviations across RNG1–3.

### HITL

HITL results were not included in the quantitative conference evidence package because sufficient raw artifacts were not found. In particular, no repository CSV/log set independently supports Raspberry Pi camera receive rate, inference throughput, inference latency, or socket-buffer comparisons.

## 3. Excluded data

- Failed and debug rate-sweep runs.
- `*_v2` pilots and other pilot runs.
- All 5 Hz attempts.
- Transport-sweep runs.
- Failed/incomplete `network_scaling` runs; no completed conference multi-UAV scaling dataset was available.
- Historical Phase-F runs superseded by the official conference protocol.
- The short JPEG baseline check; the completed full sweep is used instead.
- Any value found only in report, slide, README, manifest, or draft prose without an underlying result artifact.
- Historical multi-distance Friis R²/RMSE and a multi-rate offered-load curve, because the necessary complete raw artifacts were not found.

## 4. Methodology

The rate experiment used three RNG repetitions per mode/rate, a fixed 60 s official window, and offered application rates of 1 and 2 Hz. Approximately 60 and 120 frames were offered respectively. Ground used JPEG Q5. The denominator is the number of frames intentionally admitted/published by the relay during the official window; processed-frame ratio is inference completions divided by those publications.

Primary latency is frame admission at the relay to the corresponding detection-result callback at the GCS, matched by experiment sequence ID. Ground admission is timestamped before JPEG encoding. Latency exists only for matched results that reached the GCS; missing frames do not have a finite measured latency. Reported group latency is the mean ± sample SD of the three run-level medians, not a pooled frame statistic.

TAP bitrate is `8 × interface byte delta / 60 s / 10^6`. It includes all measured interface traffic and is not pure vision payload. CPU is simulation-host process CPU, not electrical power or onboard embedded utilization.

For JPEG evaluation, `TP=min(gt_count,pred_count)`, `FP=max(pred_count−gt_count,0)`, and `FN=max(gt_count−pred_count,0)`. Precision, recall, and F1 are derived from those count totals. Exact-count accuracy is the fraction of frames with equal predicted and labelled person counts.

## 5. Final quantitative results

### A. Network validation recoverability

| Evidence | Result | Sample count |
|---|---:|---:|
| Recoverable Friis distance points | 1 (58.5 m) | 3 packets |
| Packet-level RMSE at that point | 18.341 dB | 3 packets |
| Packet-level mean bias | −18.341 dB | 3 packets |
| R² across distance | NOT VERIFIED | 1 point |
| Fixed-load receiver goodput | 1.682 Mbit/s mean | 5 runs |
| Fixed-load receiver loss | 11.2% mean | 5 runs |

### B. Selected JPEG rows

The table includes RAW, the severe Q5 condition, Q20 as an intermediate point, the selected Q40 candidate, and Q90 as a high-quality endpoint. Selection spans the tested range rather than selecting only favorable results.

| Input | Count F1 | Recall | Mean payload | Compression ratio |
|---|---:|---:|---:|---:|
| RAW | 0.849057 | 0.823171 | 2,764,800 B | 1.000× |
| Q5 | 0.300518 | 0.176829 | 18,458.9 B | 149.934× |
| Q20 | 0.818182 | 0.768293 | 52,674.8 B | 52.586× |
| Q40 | 0.857143 | 0.859756 | 89,069.9 B | 31.085× |
| Q90 | 0.854489 | 0.841463 | 284,747.2 B | 9.722× |

Small numerical differences above RAW are normal non-monotonic detector variation under image transformation on this limited count-labelled dataset; they are not evidence that compression improves detection.

### C. Edge/Ground rate sweep (mean ± sample SD, n=3 runs)

| Mode | Rate | Processed ratio | Result delivery | Median latency (ms) | UAV TAP TX (Mbit/s) |
|---|---:|---:|---:|---:|---:|
| Edge | 1 Hz | 1.000 ± 0.000 | 0.950 ± 0.087 | 253.0 ± 90.6 | 0.213 ± 0.073 |
| Edge | 2 Hz | 1.000 ± 0.000 | 1.000 ± 0.000 | 215.5 ± 4.1 | 0.183 ± 0.012 |
| Ground | 1 Hz | 0.617 ± 0.073 | 0.617 ± 0.073 | 523.6 ± 9.2 | 0.311 ± 0.027 |
| Ground | 2 Hz | 0.053 ± 0.010 | 0.053 ± 0.010 | 569.8 ± 7.7 | 0.430 ± 0.019 |

Ground 2 Hz has only 5, 7, and 7 matched results in its three runs. Its latency, inference, and CPU values describe this small surviving subset. Its low CPU is primarily a consequence of very few images reaching inference and must not be interpreted as computational efficiency.

## 6. Main observations supported by data

- Q5 sharply reduced count-based recall and F1 while producing the smallest JPEG payload.
- Moderate JPEG qualities recovered substantially more count-based performance; behavior across quality is non-monotonic.
- Edge completed every admitted frame at both tested rates.
- Ground processed about 61.7% of admitted frames at 1 Hz and about 5.3% at 2 Hz.
- Ground produced more measured UAV-side TAP TX traffic than Edge in these evaluated conditions.
- Several official Edge traces contain consecutive results with large post-inference delay that arrive at nearly the same GCS time. This supports temporary queuing or delayed delivery along the DDS/NS-3 path, but does not identify a particular retransmission mechanism.
- The retained network artifacts verify five fixed-load UDP outcomes. They do not verify throughput saturation as offered load rises or a multi-distance received-power trend.

## 7. Limitations

- Ground transport uses JPEG `CompressedImage` over DDS, not H.264/H.265/RTP.
- Human annotations are counts, not bounding boxes; no IoU/mAP claim is supported.
- The application case study uses one controlled observation pose.
- Simulation-host CPU percentages are not onboard hardware power measurements.
- Ground latency has survivorship bias because undelivered frames have no latency sample.
- Ground 2 Hz has very small successful-frame samples.
- TAP counters include non-vision interface traffic.
- No completed conference multi-UAV network-scaling dataset was available.
- Quantitative HITL evidence was insufficient.
- The retained received-power CSV is incomplete for the configured multi-distance validation windows.
- The fixed-load UDP runs do not constitute an offered-load sweep, and repository run labels contain historical distance-label inconsistencies.

## 8. Figure inventory

| Figure | Source | Intended section | Key message | Placement |
|---|---|---|---|---|
| `fig_network_validation_power` | retained SNR CSV fragment + Friis configuration | Network validation | only one distance/three packets are recoverable; agreement trend is not established | Supplemental / evidence audit |
| `fig_network_load` | five raw iperf logs | Network validation | measured fixed-2-Mbit/s goodput/loss outcomes | Supplemental; not an offered-load curve |
| `fig_jpeg_quality_tradeoff` | full quality summary | Perception/transport trade-off | severe Q5 loss versus payload savings and recovery at moderate quality | Main or supplemental |
| `fig_rate_processed_ratio` | 12 official run summaries | Application evaluation | Edge completion and Ground degradation with offered rate | Main |
| `fig_rate_latency` | 12 official run-level medians | Application evaluation | survivor-only latency; interpret with delivery ratio | Supplemental |
| `fig_rate_network_load` | 12 official TAP deltas | Application/network evaluation | measured UAV-side interface load | Supplemental |

All figures are provided as high-resolution PNG and vector PDF. Rate figures retain individual RNG observations and show mean ± sample SD.
