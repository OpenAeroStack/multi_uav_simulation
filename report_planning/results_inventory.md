# Results Inventory

| Result group | Source directory | Experiment purpose | Main files | Number of runs | Status | Proposed Chapter 5 section |
|---|---|---|---|---:|---|---|
| Measurement apparatus/timing | `results/phase_a_apparatus/` | Emulation floor, logger overhead, warm-up and autocorrelation | RTT CSVs, metrics log, two figures | 4 timing conditions + 1 logger run | Authoritative; ACF/warm-up provenance supporting | 1–2 |
| NS-3 analytical validation | `results/phase_b_network/` | Friis/SNR validation | 150,393-packet CSV, script, two figures | 1 five-position session | Authoritative | 3 |
| RTT/throughput/loss | `results/phase_b_network/` and manifest | RTT versus distance; offered-load ceiling; loss versus distance | B4 iperf/TAP logs and summary; B2/B3 narrative | 5 B4 runs; B2/B3 raw runs unavailable | Mixed confidence | 3 |
| Middleware/telemetry | `results/phase_c_middleware/` | Compare C0 mission, C1 baseline, C2 Edge, C3 Ground | 8 raw CSVs, combined summary | 4 selected + 4 excluded/empty | Selected rows authoritative | 5 |
| Image transport/compression | `results/phase_d_application/` | Bandwidth and byte/packet delivery versus JPEG quality | TAP logs and two summaries | 10 configurations | Authoritative for transport | 6 |
| D4 detection accuracy | `results/phase_d_application/` | Raw versus JPEG-Q5 detection accuracy | 60 images, 61-label ground truth, details/summary CSVs | 1 evaluation | Authoritative published analysis; duplicate documented | 6 |
| CPU/RSS resources | `results/phase_e_resources/` | Detector-plus-relay host-process demand | pidstat logs selected through registry | 6 selected + preliminary/superseded logs | Authoritative selected set | 6 |
| Final Edge/Ground comparison | `results/final_edge_ground_comparison_primary/` | RNG-paired completion, latency, timing, resources | registry, provenance, run/mode/paired summaries, figures | 6: 3 per mode | **Authoritative; analysis complete; subsection drafted** | 6 |
| Earlier final-comparison variants | `results/final_edge_ground_comparison*/` except `_primary` | Prior selection/protocol iterations | registries and summaries | Multiple | Superseded/excluded | Appendix provenance only |
| Dynamic link/clustering | `results_02/network_evaluation/dynamic_trial*/` | Mobile RSSI/SNR/loss and CH roles | bags, extracted CSVs, logs | 3 corrected + 1 original | Corrected three authoritative | 4 |
| LOS checks | `results_02/network_evaluation/los_*` | Stationary/manual LOS observations | bag and manual samples | 2 | Supporting; formats differ | 3/appendix |
| NetAnim | `results_02/netanim/` | Four-node mobility visualisation | 2 final + 2 incomplete XMLs | 2 | Demonstration only | 2/appendix |
| Integration configuration | `results_02/network_evaluation/logs/` | Namespace, bridge and TAP verification | clean/raw text snapshots | 1 snapshot | Supporting evidence | 2 |
