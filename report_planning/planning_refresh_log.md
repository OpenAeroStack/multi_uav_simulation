# Planning Refresh Log

- Repository: `/home/randilsk/FYP/multi_uav_sim`
- Branch: `ns3-delay-varient`
- Refresh date: 2026-08-06
- Found: `results/`, `results_02/`, `report_planning/`
- Scope: Markdown planning files only; no result artifact changed.

| Previous planning statement | New evidence found | Updated status | Planning files changed |
|---|---|---|---|
| Final primary comparison absent | Registry, provenance, six linked runs, processed outputs and figures under `results/final_edge_ground_comparison_primary/` | Authoritative; analysis complete; subsection drafted | README, inventory, authoritative datasets, Phase F, issues, figures, maps, indexes, questions |
| D4 absent | Summary/details CSVs, ground truth, capture index and 60 images | Available; stored evaluation has 61 rows/60 unique IDs due duplicate `frame_0033.png` | Inventory, authoritative datasets, Phase D, issues, report-ready plan |
| Phase-E resources absent | Resource logs and six registry links; runner monitors relay + detector | Available and scope resolved | Inventory, authoritative datasets, Phase E/F, issues |
| Phase-C mapping unresolved | Raw row counts uniquely match summary rows; manifest identifies contamination | Selected/excluded mapping resolved | Phase C, authoritative datasets, issues, report-ready plan |
| Final figures absent | 11 PNG/PDF figure pairs and figure README | Available; four selected for main text | Figures, Phase F, report-ready plan |
| B2/B3 only in manifest | Rescan found B4 raw chain but no dedicated B2/B3 raw/processed artifacts | Still genuinely unresolved at raw-evidence level | Phase B, authoritative datasets, issues, maps |
| Dynamic/NetAnim status | Reverified corrected trial counts/means and XML endpoints | Prior classification retained with exact values | Dynamic/NetAnim phase notes, issues, report-ready plan |

Contradictions found: RNG3 Run IDs invert verified actual modes; D4 reports 61 evaluation frames although only 60 unique frame IDs/images exist; the expected directory name `phase_e_resource` is actually `results/phase_e_resources/`.
