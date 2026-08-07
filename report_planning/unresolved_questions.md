# Unresolved Questions Requiring Human Confirmation

The Phase B network subsection no longer requires a B2/B3 rerun decision. Their aggregate values are retained as manifest-supported descriptive results with the provenance limitation stated in the report.

The clustering implementation audit is also resolved for report drafting: controlled switching was implemented but untriggered, while the failover path was partial and not experimentally validated. No implementation-status question remains open for that subsection.

The Phase C raw-file mapping is resolved. The four selected files and exclusions are registered in `results/phase_c_middleware/report_validation_v2/selected_runs.csv`; no telemetry-selection question remains open.

| Question | Why it matters | Evidence already found | Required decision |
|---|---|---|---|
| Should manual LOS evidence be included? | It uses a different, non-normalised format | Five-sample text and separate stationary bag exist | Main report, appendix, or omit |
| Should NetAnim be appendix-only? | It is visualisation evidence with malformed endpoints | Two four-node traces; no packets/links | Confirm placement |
| What is the final completion status/wording of each project objective? | Section 7 needs exact defensible statuses | Evidence catalogue identifies coverage and gaps | Supply authoritative objective list/status |
| What exact Prism labels are used for Discussion and Limitations? | Cross-references must match the report structure | Labels are not in result artifacts | Supply exact labels |
