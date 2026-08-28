# Paper Claims Supported

## Supported quantitative claims

- The full 60-frame quality sweep shows Q5 count-based F1 0.300518 and recall 0.176829, versus RAW F1 0.849057 and recall 0.823171. Source: `results/conference_paper/quality_sweep/processed/quality_sweep_summary_20260827_095558_UTC.csv`.
- Q40 has count-based F1 0.857143, recall 0.859756, mean payload 89,069.916667 bytes, and mean compression ratio 31.084653. Source: the same full quality summary.
- Edge processed all official publications in all six official 1/2 Hz runs. Source: the six corresponding `rate_summary_*.csv` files.
- Ground processed-frame ratio decreased from 0.616667 ± 0.072648 at 1 Hz to 0.052778 ± 0.009623 at 2 Hz. Calculation: run-level mean ± sample SD across RNG1–3.
- Ground UAV-side TAP TX bitrate exceeded Edge in each aggregated 1/2 Hz condition. These are interface bitrates, not pure vision payload.

## Supported qualified long-tail claim

Supported wording:

> Some consecutive detection results experienced substantial post-inference delivery delay and subsequently arrived at the GCS in short bursts, indicating temporary queuing or delayed delivery along the DDS/NS-3 path.

Examples from official traces include:

- `frame_trace_rate_edge_1hz_rng1.csv`: sequences 52–54 have approximately 3231, 2227, and 1237 ms post-inference delay and receipt timestamps 1787837371.133908, 1787837371.134185, and 1787837371.134447 s.
- `frame_trace_rate_edge_1hz_rng1.csv`: sequences 60–63 include approximately 3895, 2899, 1923, and 941 ms post-inference delay and arrive within about 0.51 ms.
- `frame_trace_rate_edge_2hz_rng2.csv`: sequences 161–163 have approximately 2475, 2017, and 1515 ms post-inference delay and arrive within about 0.64 ms.
- `frame_trace_rate_edge_2hz_rng3.csv`: sequences 86–89 have approximately 3451, 2964, 2453, and 1933 ms post-inference delay and arrive within about 0.94 ms.

These traces support burst/queue/delayed-delivery interpretation. They do not prove a specific DDS retransmission mechanism.

## Claims not supported by retained raw evidence

- A numerical multi-distance Friis R² or propagation trend. Only one distance point is recoverable.
- Throughput saturation and increasing loss as offered load rises. All retained B4 iperf runs offered the same 2 Mbit/s.
- Quantitative Raspberry Pi/HITL performance.
- Completed conference multi-UAV network scaling.
- Any bounding-box mAP or IoU accuracy result.
- Any claim that JPEG compression improves detector accuracy relative to RAW.
