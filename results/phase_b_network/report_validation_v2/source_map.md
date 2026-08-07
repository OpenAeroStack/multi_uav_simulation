# Phase B Network Validation Source Map

| Result | Source | Evidence level | Main report use | Limitation |
|---|---|---|---|---|
| B1 analytical validation | `results/phase_b_network/raw/ns3_snr_obstaclefree_20260804_220047.csv`; `results/scripts/b1_friis_validation.py`; saved B1 figures; aggregate entry in `results/MANIFEST.md` | Report-authoritative; raw, script, figures and recorded aggregate | Validate the distance-dependent Friis trend using received signal power | The raw snapshot in this checkout ends at 740.2914 s and cannot reproduce four later windows; the completed aggregate and original figures are retained, but individual observed means are not recoverable from this snapshot. |
| Phase A emulation floor | `results/phase_a_apparatus/processed/rtt_summary.csv` and named raw RTT logs | Report-authoritative processed/raw chain | Establish the timing characteristic used to interpret B2 | The measurements identify an NS-3/TAP path floor but do not isolate its exact internal cause. |
| B2 RTT versus distance | `results/MANIFEST.md` | Manifest-supported descriptive result | Compare RTT across five tested distances | Aggregate experimental result recorded in the project manifest; detailed run-level logs were not retained separately. |
| B3 throughput versus offered load | `results/MANIFEST.md` | Manifest-supported descriptive result | Show the observed soft goodput ceiling and rising loss | Aggregate experimental result recorded in the project manifest; detailed run-level logs were not retained separately. |
| B4 packet loss versus distance | `results/phase_b_network/processed/b4_loss_summary.csv`; matching iperf/TAP logs; `results/scripts/b4_loss_measurement.sh` | Report-authoritative processed/raw/script chain | Compare TAP-counter and iperf loss over distance | Runs lasted 10 s. Two raw log labels contain stale distance text; matching uses their unique TAP counter totals and preserves both names in `b4_summary.csv`. |

