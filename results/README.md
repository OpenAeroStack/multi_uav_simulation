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
