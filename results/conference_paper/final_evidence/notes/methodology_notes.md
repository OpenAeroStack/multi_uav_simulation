# Methodology Notes

## Provenance policy

Numerical evidence is admitted only when recoverable from raw/processed repository artifacts. README or report prose is never treated as a numerical source. `data/evidence_manifest.csv` records verification status. `NOT_VERIFIED` means the required underlying evidence was not found or was insufficient for the proposed calculation.

## Derived formulas

- Count TP: `min(gt_count, pred_count)`.
- Count FP: `max(pred_count - gt_count, 0)`.
- Count FN: `max(gt_count - pred_count, 0)`.
- Precision: `TP / (TP + FP)`.
- Recall: `TP / (TP + FN)`.
- F1: `2 * precision * recall / (precision + recall)`.
- Exact-count accuracy: exact-count frames divided by 60.
- Compression ratio: `2,764,800 / encoded_size_bytes` per frame, summarized by the original sweep.
- Processed-frame ratio: official inference completions divided by official relay publications.
- Result-delivery ratio: matched GCS result receipts divided by official relay publications.
- Pipeline latency: `(ground_result_receipt_time - frame_admission_time) * 1000` ms.
- TAP bitrate: `8 * byte_delta / 60 / 1,000,000` Mbit/s.
- Run-group mean: arithmetic mean of three run-level statistics.
- Run-group SD: sample standard deviation with denominator `n-1`; `n=3`.
- Friis expected power: `20 dBm - 20 log10(4*pi*d*f/c)`, with `d=58.5 m`, `f=5180 MHz`, and zero antenna gains.
- Received-power RMSE: square root of the mean squared packet-level error relative to the sole recoverable expected-power point.
- Received-power bias: mean packet-level observed-minus-expected error.

## Unit of replication

The RNG run is the experimental replicate for the Edge/Ground rate sweep. Frame observations are not pooled across runs to inflate the repetition count. JPEG quality results use 60 unique labelled frames and describe that fixed dataset rather than repeated independent experiments.

## Network-validation qualification

The configured Friis script defines five time/distance windows, but the retained SNR file ends during the first window. R² requires variation in expected power and is undefined with one recoverable distance. The generated power figure therefore visualizes the retained fragment only.

All five retained iperf logs use the same 2 Mbit/s offered rate and 10 s duration. Their receiver summary is authoritative for goodput, jitter, lost datagrams, total datagrams, and loss percentage. They cannot support an offered-load response curve.
