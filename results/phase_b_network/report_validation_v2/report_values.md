# Network Validation Report Values

This table covers the numerical values written in `network_validation.tex`. Ranges are listed as they appear in the prose, while each table row is recorded as a compact value tuple.

| Report value | Value | Source file | Evidence type |
|---|---|---|---|
| B1 tested positions/range | 5; 58.5--113.8 m | `results/scripts/b1_friis_validation.py` | Configured confirmed windows |
| B1 completed sample count | 150,393 | `results/MANIFEST.md` | Recorded aggregate |
| B1 fit | R² 0.860; RMSE 0.77 dB; bias -0.77 dB | `results/MANIFEST.md`; saved B1 figures | Recorded aggregate and figure |
| Phase A management path | 0.032 ms mean | `phase_a_timing_floor_summary.csv` | Authoritative processed result |
| Phase A colocated NS-3/TAP | 158.280 ms mean | `phase_a_timing_floor_summary.csv` | Authoritative processed result |
| Phase A operational NS-3/TAP | 160.700 ms mean | `phase_a_timing_floor_summary.csv` | Authoritative processed result |
| Phase A scheduling control | 152.310 ms mean | `phase_a_timing_floor_summary.csv` | Authoritative processed result |
| B2 at 58.5 m | 159.8 ms mean; 129--197 ms; 0% loss | `b2_manifest_summary.csv` | Manifest-supported descriptive result |
| B2 at 69.5 m | 153.3 ms mean; 119--185 ms; 0% loss | `b2_manifest_summary.csv` | Manifest-supported descriptive result |
| B2 at 83.9 m | 159.4 ms mean; 130--194 ms; 0% loss | `b2_manifest_summary.csv` | Manifest-supported descriptive result |
| B2 at 97.7 m | 159.9 ms mean; 135--204 ms; 0% loss | `b2_manifest_summary.csv` | Manifest-supported descriptive result |
| B2 at 113.8 m | 158.1 ms mean; 127--212 ms; 0% loss | `b2_manifest_summary.csv` | Manifest-supported descriptive result |
| B2 mean range | 153.3--159.9 ms | `b2_manifest_summary.csv` | Derived range of manifest aggregates |
| B3 at 2 Mbps offered | 1.69 Mbps received; 11.0% loss | `b3_manifest_summary.csv` | Manifest-supported descriptive result |
| B3 at 5 Mbps offered | 2.85 Mbps received; 40.0% loss | `b3_manifest_summary.csv` | Manifest-supported descriptive result |
| B3 at 8 Mbps offered | 3.02 Mbps received; 59.6% loss | `b3_manifest_summary.csv` | Manifest-supported descriptive result |
| Observed B3 soft ceiling | approximately 3 Mbps | `b3_manifest_summary.csv` | Cautious interpretation of manifest aggregate |
| B4 test setup | 2 Mbps; 10 s per position | `b4_summary.csv` | Authoritative processed/raw/script chain |
| B4 at 58.50 m | 13.96% TAP; 12.00% iperf loss | `b4_summary.csv` | Authoritative processed/raw result |
| B4 at 69.50 m | 13.58% TAP; 11.00% iperf loss | `b4_summary.csv` | Authoritative processed/raw result |
| B4 at 83.09 m | 13.44% TAP; 11.00% iperf loss | `b4_summary.csv` | Authoritative processed/raw result |
| B4 at 97.70 m | 13.25% TAP; 11.00% iperf loss | `b4_summary.csv` | Authoritative processed/raw result |
| B4 at 113.80 m | 13.49% TAP; 11.00% iperf loss | `b4_summary.csv` | Authoritative processed/raw result |
| B4 loss ranges | 13.25--13.96% TAP; 11--12% iperf | `b4_summary.csv` | Derived ranges of authoritative values |

