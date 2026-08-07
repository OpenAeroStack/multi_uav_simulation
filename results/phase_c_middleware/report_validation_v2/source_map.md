# Middleware and Telemetry Source Map

| Result | Authoritative source | Evidence type | Main-report use | Caveat |
|---|---|---|---|---|
| C0 mission/no vision | `results/phase_c_middleware/raw/telemetry_c0_mission_baseline_20260805_012216.csv` | Complete selected raw run matched to processed summary | Mission-active no-vision comparison | One selected run; messages are within-run observations. |
| C1 static/no vision | `results/phase_c_middleware/raw/telemetry_c1_baseline_20260805_001716.csv` | Complete selected raw run matched to processed summary | Static no-vision comparison | One selected run; messages are within-run observations. |
| C2 Edge | `results/phase_c_middleware/raw/telemetry_c2_edge_load_20260805_012804.csv` | Complete selected raw run matched to processed summary | Edge-processing comparison | Header-only attempts and contaminated run `004614` are excluded. |
| C3 Ground | `results/phase_c_middleware/raw/telemetry_c3_ground_load_20260805_011400.csv` | Complete selected raw run matched to processed summary | Ground-processing comparison | Incomplete run `011330` is excluded. |
| Message rates and gaps | Selected raw files; `results/phase_c_middleware/processed/c_telemetry_summary.csv`; `results/scripts/telemetry_health.py` | Direct timestamp calculation and stored summary verification | Four-condition table and optional descriptive figure | Rates use the configured 120 s capture duration; the comparison is descriptive and does not isolate a single cause. |

