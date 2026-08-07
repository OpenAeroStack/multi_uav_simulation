# `results/` Source Index

## Root and Phases A–C

| Relative path/group | File type / columns | Purpose | Status | Experiment |
|---|---|---|---|---|
| `results/MANIFEST.md` | Markdown | Environment and phase interpretations | Supporting; B2/B3 only source | A–F |
| `phase_a_apparatus/processed/rtt_summary.csv` | condition, n/loss, RTT statistics | Current emulation-floor summary | Authoritative | A2 |
| `phase_a_apparatus/processed/rtt_summary_v1.csv` | same | Earlier summary | Superseded | A2 |
| Phase-A samples/metrics/figures | CSV/PNG | attempt-level RTT, logger control, warm-up/ACF | Mixed authoritative/supporting | A2–A4 |
| Phase-B B1 raw CSV/script/two PNGs | CSV/Python/PNG | Friis validation | Authoritative | B1 |
| `phase_b_network/processed/b4_loss_summary.csv` + iperf/TAP logs | rate, duration, counters/loss | Loss versus distance | Authoritative | B4 |
| Phase-C raw telemetry files | label, arrival time, gap | Condition recordings | Four selected; four excluded | C0–C3 |
| `phase_c_middleware/processed/c_telemetry_summary.csv` | duration/count/rate/gap statistics | Combined summary | Authoritative with mapped selection | C0–C3 |

No dedicated B2/B3 raw or processed artifacts were found.

## Phase D

| Relative path/group | File type / columns | Purpose | Status | Experiment |
|---|---|---|---|---|
| D1 bandwidth/delivery summaries + TAP logs | CSV/text | JPEG transport/load | Authoritative | D1/D5/D8 |
| `phase_d_application/raw/d4_reference_01/frames.csv` + 60 PNGs | capture index/images | D4 inputs | Authoritative | D4 |
| `.../ground_truth.csv` | frame ID, count, notes | manual labels | Authoritative with duplicated frame 33 | D4 |
| `processed/d4_detection_details_*.csv` | per-frame GT/prediction/TP/FP/FN/timing/payload | Detailed evaluation | Authoritative stored 61 rows | D4 |
| `processed/d4_detection_summary_*.csv` | representation, frames, TP/FP/FN, precision/recall/F1/exact count/timing | Published summary | Authoritative with duplicate caveat | D4 |

## Phases E–F and final registry

| Relative path/group | File type / columns | Purpose | Status | Experiment |
|---|---|---|---|---|
| `phase_e_resources/raw/resources_*.txt` | pidstat CPU/RSS | Detector+relay monitoring | Selected and superseded logs coexist | E/F |
| `phase_f_comparison/raw/metadata_*.txt` | protocol, official rows/counts, mode/config | Per-run metadata | Authoritative when registry-linked | F |
| `phase_f_comparison/raw/metrics_*.csv` | per-processed-frame timings/counts | Live system metrics | Authoritative when registry-linked | F |
| `final_edge_ground_comparison_primary/selected_runs.csv` | RNG, actual mode, run ID, official rows, linked files, verification | Six-run registry | Definitive | F |
| primary `provenance.md`, `exclusions.csv`, checksums | Markdown/CSV/text | Mode proof, exclusions, integrity | Definitive provenance | F |
| primary `processed/run_summary.csv` | per-run completion/latency/timing/resources | Run-level results | Authoritative | F |
| primary `processed/mode_summary.csv` | n, mean, sample SD, median/range | Three-run mode summaries | Authoritative | F |
| primary paired/latency/combined D4 files | CSV/Markdown | Within-RNG and validation summaries | Authoritative | D4/F |
| primary processed report drafts/value sources | Markdown/text | Existing Chapter-5 materials | Supporting; subsection drafted | F |
| primary `processed/figures/` | 11 PNG/PDF pairs + README | Final plots | Report-ready | D4/F |
| other `final_edge_ground_comparison*` directories | registries/summaries | Earlier analyses | Superseded | F provenance |

## Scripts

Collection and analysis scripts cover timing, B1/B4, telemetry, D1/D4, final-run execution, primary analysis, report materials and plotting. `run_comparison_once.sh` records official windows and monitors detector+relay; `analyze_final_edge_ground_primary.py` is the definitive processor.
