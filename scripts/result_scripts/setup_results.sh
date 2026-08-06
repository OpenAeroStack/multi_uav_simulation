#!/bin/bash
# setup_results.sh — create the results folder tree and MANIFEST template.
# Safe to re-run: never overwrites an existing MANIFEST.md or any raw data.

set -uo pipefail

RESULTS_ROOT="${1:-$HOME/FYP/multi_uav_simulation/results}"

PHASES=(
    "phase_a_apparatus"
    "phase_b_network"
    "phase_c_middleware"
    "phase_d_application"
    "phase_e_resources"
    "phase_f_comparison"
    "phase_g_systems_findings"
)

echo "Creating results tree at: $RESULTS_ROOT"
for p in "${PHASES[@]}"; do
    mkdir -p "$RESULTS_ROOT/$p/raw"
    mkdir -p "$RESULTS_ROOT/$p/processed"
    mkdir -p "$RESULTS_ROOT/$p/figures"
    echo "  $p/{raw,processed,figures}"
done

mkdir -p "$RESULTS_ROOT/scripts"
echo "  scripts/"

# ---------------------------------------------------------------------------
# MANIFEST — never overwrite, it accumulates across the whole project
# ---------------------------------------------------------------------------
MANIFEST="$RESULTS_ROOT/MANIFEST.md"
if [[ -f "$MANIFEST" ]]; then
    echo ""
    echo "MANIFEST.md already exists — left untouched."
else
    cat > "$MANIFEST" << 'MANIFEST_EOF'
# Results Manifest

One row per artifact. Fill this in **as you collect**, not afterwards — this is
what makes "where did this number come from?" answerable months later.

## Environment record

Fill once per machine, update if anything changes.

| Item | Value |
|---|---|
| Machine | |
| CPU / cores | |
| RAM | |
| OS | |
| Gazebo version | |
| ROS 2 distro | |
| ns-3 version | |
| ArduPilot commit | |
| YOLO model / weights | |
| World file | |
| Camera config (FOV / res / pitch) | |
| Human model positions (ground truth) | |

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
MANIFEST_EOF
    echo ""
    echo "Created MANIFEST.md"
fi

# ---------------------------------------------------------------------------
# README pointing at the plan
# ---------------------------------------------------------------------------
README="$RESULTS_ROOT/README.md"
if [[ ! -f "$README" ]]; then
    cat > "$README" << 'README_EOF'
# Results

Collected according to `EXPERIMENT_PLAN.md`. Execution order:

- **Phase A** — validate the measurement apparatus (do first; everything
  downstream depends on it)
- **Phase B** — network layer characterisation
- **Phase C** — middleware / telemetry degradation
- **Phase D** — application layer (video, compression, detection)
- **Phase E** — resource utilisation
- **Phase F** — the edge vs ground comparison
- **Phase G** — systems findings

Each phase folder holds:
- `raw/` — untouched logs and CSVs straight from the run
- `processed/` — computed statistics, summary tables
- `figures/` — plots ready to drop into the report

`MANIFEST.md` maps every artifact to the report figure or table it becomes.
README_EOF
    echo "Created README.md"
fi

echo ""
echo "Done. Tree:"
find "$RESULTS_ROOT" -maxdepth 2 -type d | sort | sed "s|$RESULTS_ROOT|results|"