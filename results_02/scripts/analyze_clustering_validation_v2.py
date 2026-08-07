#!/usr/bin/env python3
"""Create the canonical clustering-transition validation package."""

from __future__ import annotations

import csv
import json
import subprocess
from collections import Counter
from pathlib import Path

import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "results_02/network_evaluation"
OUT = BASE / "clustering_validation_v2"
FIG = OUT / "figures"
TRIALS = ["dynamic_trial1", "dynamic_trial2", "dynamic_trial3"]
INIT_SECONDS = 0.01
SOURCE_COMMIT = "09cff0a"
MANAGER = "ros2/uav_controller/uav_controller/dynamic_cluster_manager.py"
EXTRACTOR = "scripts/extract_dynamic_bag.py"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def bool_text(value: bool) -> str:
    return "true" if value else "false"


def load_trial(name: str) -> dict[str, object]:
    directory = BASE / name
    assignments_raw = read_csv(directory / "extracted/cluster_assignments.csv")
    roles = read_csv(directory / "extracted/cluster_roles.csv")
    events_raw = read_csv(directory / "extracted/cluster_events.csv")
    scores = read_csv(directory / "extracted/cluster_scores_raw.csv")
    summary = read_csv(directory / "extracted/summary.csv")[0]
    links = read_csv(directory / "extracted/network_links.csv")

    assignments = []
    for source_row, row in enumerate(assignments_raw, start=2):
        payload = json.loads(row["assignment"])
        primary, backup = payload.get("primary_ch"), payload.get("backup_ch")
        complete = (
            isinstance(primary, int) and primary in (1, 2, 3)
            and isinstance(backup, int) and backup in (1, 2, 3)
            and primary != backup and len(payload.get("assignments", [])) == 3
        )
        assignments.append({"time": float(row["time_s"]), "row": source_row,
                            "primary": primary, "backup": backup,
                            "epoch": payload.get("epoch"), "complete": complete,
                            "payload": payload})
    if not assignments or not all(row["complete"] for row in assignments):
        raise ValueError(f"{name}: incomplete assignment state found")

    events = []
    for source_row, row in enumerate(events_raw, start=2):
        payload = json.loads(row["event"])
        events.append({"time": float(row["time_s"]), "row": source_row,
                       "payload": payload, "raw": row["event"]})

    clustering_times = ([row["time"] for row in assignments]
                        + [float(row["time_s"]) for row in roles]
                        + [row["time"] for row in events]
                        + [float(row["time_s"]) for row in scores])
    first = min(clustering_times)
    cutoff = first + INIT_SECONDS
    retained = [row for row in assignments if row["time"] >= cutoff]
    excluded = [row for row in assignments if row["time"] < cutoff]
    if not retained:
        raise ValueError(f"{name}: no assignment baseline after initialization")

    # Canonical consecutive role states; repeated publications collapse.
    canonical_states = []
    for row in retained:
        state = (row["primary"], row["backup"])
        if not canonical_states or state != canonical_states[-1]["state"]:
            canonical_states.append({**row, "state": state})

    all_state_transitions = []
    previous = assignments[0]
    for current in assignments[1:]:
        for role in ("primary", "backup"):
            if current[role] != previous[role]:
                all_state_transitions.append({"role": role, "previous": previous[role],
                                              "new": current[role], "previous_row": previous["row"],
                                              "new_row": current["row"], "time": current["time"],
                                              "during_init": current["time"] < cutoff})
        previous = current

    canonical_transitions = []
    previous = canonical_states[0]
    for current in canonical_states[1:]:
        for role in ("primary", "backup"):
            if current[role] != previous[role]:
                canonical_transitions.append({"role": role, "previous": previous[role],
                                              "new": current[role], "previous_row": previous["row"],
                                              "new_row": current["row"], "time": current["time"],
                                              "during_init": False})
        previous = current

    metrics = {}
    for metric in ("rssi_dbm", "snr_db", "obstacle_loss_db"):
        values = [float(row["value"]) for row in links if row["metric"] == metric]
        metrics[metric] = sum(values) / len(values)

    return {"name": name, "directory": directory, "assignments": assignments,
            "events": events, "summary": summary, "first": first, "cutoff": cutoff,
            "excluded": excluded, "retained": retained, "canonical_states": canonical_states,
            "all_state_transitions": all_state_transitions,
            "canonical_transitions": canonical_transitions, "metrics": metrics}


def match_event(transition: dict[str, object], events: list[dict[str, object]]) -> dict[str, object] | None:
    role = str(transition["role"])
    old_key, new_key = f"old_{role}", f"new_{role}"
    matches = [event for event in events
               if event["payload"].get(old_key) == transition["previous"]
               and event["payload"].get(new_key) == transition["new"]
               and abs(float(event["time"]) - float(transition["time"])) <= 0.01]
    return min(matches, key=lambda event: abs(float(event["time"]) - float(transition["time"]))) if matches else None


def build_audit(trial: dict[str, object]) -> list[dict[str, object]]:
    rows = []
    canonical_keys = {(item["role"], item["new_row"]) for item in trial["canonical_transitions"]}
    matched_event_rows = set()
    index = 0
    for transition in trial["all_state_transitions"]:
        index += 1
        event = match_event(transition, trial["events"])
        if event:
            matched_event_rows.add(event["row"])
        counted = (transition["role"], transition["new_row"]) in canonical_keys
        rows.append({
            "trial": trial["name"], "transition_index": index, "role": transition["role"],
            "previous_uav": transition["previous"], "new_uav": transition["new"],
            "timestamp_s": f"{transition['time']:.6f}",
            "elapsed_time_s": f"{float(transition['time']) - float(trial['first']):.6f}",
            "previous_assignment_row": transition["previous_row"],
            "new_assignment_row": transition["new_row"],
            "during_initialization": bool_text(bool(transition["during_init"])),
            "counted_in_canonical_metric": bool_text(counted),
            "matching_event_found": bool_text(event is not None),
            "matching_event_type": event["payload"].get("reason", "") if event else "",
            "notes": "sampled assignment-state change" if counted else "initialization assignment-state change",
        })

    # Preserve initialization event evidence that precedes or is not matched to extracted assignment changes.
    for event in trial["events"]:
        if event["time"] >= trial["cutoff"] or event["row"] in matched_event_rows:
            continue
        payload = event["payload"]
        for role in ("primary", "backup"):
            old, new = payload.get(f"old_{role}"), payload.get(f"new_{role}")
            if old == new:
                continue
            index += 1
            rows.append({
                "trial": trial["name"], "transition_index": index, "role": role,
                "previous_uav": old, "new_uav": new, "timestamp_s": f"{event['time']:.6f}",
                "elapsed_time_s": f"{float(event['time']) - float(trial['first']):.6f}",
                "previous_assignment_row": "", "new_assignment_row": "",
                "during_initialization": "true", "counted_in_canonical_metric": "false",
                "matching_event_found": "true", "matching_event_type": payload.get("reason", ""),
                "notes": f"initialization event-only evidence; cluster_events.csv row {event['row']}",
            })
    return rows


def implementation_rows() -> list[dict[str, str]]:
    source = f"{MANAGER} (commit {SOURCE_COMMIT}; repository history, absent current checkout)"
    specs = [
        ("initial election", "election_callback", "period=2.0 s", "implemented", "lines 701,715-718 choose first eligible candidate", "initial_election events T2/T3", "yes", "implemented_and_observed", "T1 recording begins after initial burst"),
        ("candidate scoring", "calculate_score", "weights=.40/.30/.20/.10; candidate SNR>=3 dB", "implemented", "lines 399-504 calculate score and eligibility", "scores and assignments recorded", "yes", "implemented_and_observed", ""),
        ("primary selection", "election_callback", "candidate sort by score", "implemented", "lines 689-702 select highest score", "valid primary in all assignments", "yes", "implemented_and_observed", ""),
        ("backup selection/reselection", "election_callback", "next eligible candidate", "implemented", "lines 770-795 select backup and label reselection", "canonical backup changes", "yes", "implemented_and_observed", ""),
        ("periodic election", "__init__/election_callback", "2.0 s", "implemented", "lines 41,71-73,220-223 schedule callback", "~2 s assignment spacing after startup", "yes", "implemented_and_observed", ""),
        ("controlled primary switching", "election_callback", "margin=.12; wins=3; hold=10 s", "implemented", "lines 725-768 compare scores, enforce hold/wins, assign primary", "no dedicated test", "no", "implemented_but_not_triggered", "zero canonical primary transitions"),
        ("switching margin", "election_callback", "0.12", "implemented", "lines 44,80-82,742-745 use margin in decision", "none", "no", "implemented_but_not_triggered", ""),
        ("consecutive-epoch requirement", "election_callback", "3 wins", "implemented", "lines 45,83-85,747-758 count challenger wins", "none", "no", "implemented_but_not_triggered", ""),
        ("minimum holding period", "election_callback", "10.0 s", "implemented", "lines 43,77-79,737-740 enforce hold", "none", "no", "implemented_but_not_triggered", ""),
        ("stale-measurement detection", "metrics_ready", "timeout=5.0 s", "implemented", "lines 331-361 reject missing/stale global metrics", "none", "not identified", "implemented_but_not_tested", "Does not specifically mark only the active primary stale"),
        ("GCS-SNR failure threshold", "election_callback", "-2.0 dB", "implemented", "lines 49,90-92,707-721 detect current primary link failure", "none", "no", "implemented_but_not_tested", ""),
        ("immediate backup promotion", "election_callback", "n/a", "not_found", "failure path lines 719-723 assigns proposed_primary, not stored backup", "none", "no", "not_implemented", "Best eligible candidate may equal prior backup but promotion is not explicit/guaranteed"),
        ("new backup after primary change", "election_callback", "next eligible candidate", "implemented", "lines 770-780 recompute backup after primary decision", "no primary change test", "no", "implemented_but_not_tested", "Handles no candidate by backup=0"),
        ("assignment/role/score publication", "publish_state", "transient-local reliable state QoS", "implemented", "lines 589-644 publish assignment, scores, primary, backup", "bag topics contain samples", "yes", "implemented_and_observed", ""),
        ("event publication", "publish_state", "only when changed", "implemented", "lines 646-668 publish reason and old/new IDs", "stored cluster events", "yes", "implemented_and_observed", ""),
    ]
    fields = ["feature", "source_file", "class_or_function", "relevant_constants", "implementation_status",
              "static_code_evidence", "existing_test_evidence", "observed_in_corrected_trials",
              "final_classification", "notes"]
    return [dict(zip(fields, (feature, source, function, constants, status, evidence, tests, observed, final, notes)))
            for feature, function, constants, status, evidence, tests, observed, final, notes in specs]


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    FIG.mkdir(parents=True, exist_ok=True)
    trials = [load_trial(name) for name in TRIALS]
    if len(trials) != 3 or any(trial["name"] == "dynamic_trial1_original" for trial in trials):
        raise ValueError("Exactly the three corrected trials must be included")

    audit_fields = ["trial", "transition_index", "role", "previous_uav", "new_uav", "timestamp_s",
                    "elapsed_time_s", "previous_assignment_row", "new_assignment_row", "during_initialization",
                    "counted_in_canonical_metric", "matching_event_found", "matching_event_type", "notes"]
    audit = [row for trial in trials for row in build_audit(trial)]
    write_csv(OUT / "transition_audit.csv", audit_fields, audit)

    summaries = []
    for trial in trials:
        transitions = trial["canonical_transitions"]
        events = trial["events"]
        primary = trial["retained"][0]["primary"]
        primary_retention = 100 * sum(row["primary"] == primary for row in trial["retained"]) / len(trial["retained"])
        unique_events = len({json.dumps(event["payload"], sort_keys=True) for event in events})
        init_events = sum(event["time"] < trial["cutoff"] for event in events)
        backup_ids = sorted({row["backup"] for row in trial["retained"]})
        notes = (f"first_cluster={trial['first']:.6f}; cutoff_timestamp={trial['cutoff']:.6f}; "
                 f"baseline={trial['retained'][0]['time']:.6f}; unique_events={unique_events}; "
                 f"summary_backup_changes={trial['summary']['backup_changes']}")
        summaries.append({
            "trial": trial["name"], "duration_s": trial["summary"]["duration_s"],
            "assignment_states_total": len(trial["assignments"]),
            "assignment_states_excluded_initialization": len(trial["excluded"]),
            "assignment_states_retained": len(trial["retained"]), "primary_ch": primary,
            "primary_retention_percent": f"{primary_retention:.3f}",
            "canonical_primary_transitions": sum(row["role"] == "primary" for row in transitions),
            "backup_ch_initial": trial["retained"][0]["backup"],
            "backup_identifiers_observed": ";".join(map(str, backup_ids)),
            "canonical_backup_transitions": sum(row["role"] == "backup" for row in transitions),
            "total_event_messages": len(events), "initialization_event_messages": init_events,
            "steady_state_event_messages": len(events) - init_events,
            "unique_event_messages": unique_events,
            "mean_rssi_dbm": f"{trial['metrics']['rssi_dbm']:.6f}",
            "mean_snr_db": f"{trial['metrics']['snr_db']:.6f}",
            "mean_obstacle_loss_db": f"{trial['metrics']['obstacle_loss_db']:.6f}",
            "verification_status": "verified", "notes": notes,
        })
    summary_fields = list(summaries[0])
    write_csv(OUT / "canonical_trial_summary.csv", summary_fields, summaries)

    summary_md = "# Canonical Clustering Trial Summary\n\n"
    summary_md += "| Trial | Duration (s) | Total states | Init excluded | Retained | Primary (retention) | Canonical primary | Initial backup | Backup IDs | Canonical backup | Events init/steady/total | Mean RSSI | Mean SNR | Mean obstacle loss |\n"
    summary_md += "|---|---:|---:|---:|---:|---|---:|---:|---|---:|---|---:|---:|---:|\n"
    for row in summaries:
        summary_md += (f"| {row['trial']} | {float(row['duration_s']):.3f} | {row['assignment_states_total']} | "
                       f"{row['assignment_states_excluded_initialization']} | {row['assignment_states_retained']} | "
                       f"UAV{row['primary_ch']} ({row['primary_retention_percent']}%) | {row['canonical_primary_transitions']} | "
                       f"UAV{row['backup_ch_initial']} | {row['backup_identifiers_observed']} | {row['canonical_backup_transitions']} | "
                       f"{row['initialization_event_messages']}/{row['steady_state_event_messages']}/{row['total_event_messages']} | "
                       f"{float(row['mean_rssi_dbm']):.3f} | {float(row['mean_snr_db']):.3f} | {float(row['mean_obstacle_loss_db']):.3f} |\n")
    summary_md += "\nUse the canonical assignment-state transition columns in the report; do not substitute event counts or legacy `backup_changes`.\n"
    (OUT / "canonical_trial_summary.md").write_text(summary_md, encoding="utf-8")

    metric_md = """# Canonical Cluster-Role Transition Metric

`canonical_primary_transitions` is the number of primary identifier changes between consecutive complete, chronologically ordered cluster-assignment states after initialization. `canonical_backup_transitions` is defined identically for the backup identifier. The first retained state establishes the baseline and is not a transition.

The deterministic initialization interval is the first 0.01 s relative to the earliest recorded clustering timestamp in each bag-derived dataset. The first complete assignment at or after that cutoff is the baseline. Exact consecutive `(primary_ch, backup_ch)` duplicates are collapsed; repeated publications, empty-to-valid initialization, and raw event messages are not counted. Trials are never concatenated.

The sampled `/cluster/assignment` state is authoritative. `/cluster/event` is supporting evidence because events can occur during initialization, before the retained baseline, or at a different sampling timestamp.
"""
    (OUT / "metric_definition.md").write_text(metric_md, encoding="utf-8")

    audit_lines = ["# Clustering Source Audit", "", "All CSV `time_s` values are seconds elapsed from the first ROS-bag message, as defined by the contemporaneous extractor at `scripts/extract_dynamic_bag.py` commit `09cff0a`, lines 163–179. They are not wall-clock timestamps.", "", "| Trial | First clustering timestamp | Cutoff timestamp | Baseline assignment | Excluded/retained | Duplicate timestamps | Complete assignments |", "|---|---:|---:|---:|---:|---|---|"]
    for trial in trials:
        times = [row["time"] for row in trial["assignments"]]
        audit_lines.append(f"| {trial['name']} | {trial['first']:.6f} | {trial['cutoff']:.6f} | {trial['retained'][0]['time']:.6f} (row {trial['retained'][0]['row']}) | {len(trial['excluded'])}/{len(trial['retained'])} | {'yes' if len(times) != len(set(times)) else 'no'} | {sum(row['complete'] for row in trial['assignments'])}/{len(trial['assignments'])} |")
    audit_lines += ["", "## Schemas", "", "- `cluster_assignments.csv`: `time_s` plus JSON `assignment`; JSON contains `primary_ch`, `backup_ch`, `epoch`, `status`, `num_uavs`, and a complete three-UAV assignment list with roles/parents/routes/scores/GCS SNR.", "- `cluster_roles.csv`: `time_s`, `role`, `uav_id`; separate primary and backup topic publications, used only as corroboration.", "- `cluster_events.csv`: `time_s` plus JSON `event`; payload contains epoch, reason, old/new primary and old/new backup. `reason` is the event type; there are no separate source/destination role fields.", "- `summary.csv`: bag duration, role/score sample counts, event messages, legacy sampled-topic change counts, initial/final roles and primary-time percentages.", "", "Assignment publication repeats complete states at the election rate. Consecutive repeated role states are expected publications, not duplicate CSV timestamps. Full messages can differ in scores even when the role state is unchanged. Event and assignment timestamps differ slightly because they are separate ROS messages.", "", "## Reconciliation of legacy, event, and canonical counts", "", "- Trial 1: legacy `backup_changes=8` consists of six pre-cutoff sampled state changes plus two canonical changes. Three steady events exist, but the 0.899 s event coincides with the first retained assignment and establishes the baseline, leaving two canonical transitions.", "- Trial 2: legacy `backup_changes=7` consists of three pre-cutoff sampled state changes plus four canonical changes. Eight initialization events were published faster than the extracted assignment/role-state sampling represented them; four steady events match the four canonical transitions.", "- Trial 3: legacy `backup_changes=3` equals the three canonical changes. Nine initialization events occurred before the retained baseline but were not represented as sampled role changes; three steady events match the canonical transitions.", "", "All event payloads are unique within their trial because their epochs and/or old/new states differ. Thus the discrepancy is not byte-identical event duplication; it is initialization exclusion, baseline treatment, and different topic publication/sampling timing."]
    (OUT / "source_audit.md").write_text("\n".join(audit_lines) + "\n", encoding="utf-8")

    impl = implementation_rows()
    impl_fields = list(impl[0])
    write_csv(OUT / "implementation_status.csv", impl_fields, impl)
    impl_md = "# Clustering Implementation Status\n\nThe contemporaneous implementation is available in repository commit `09cff0a` but its source file is absent from the current checkout. Static classifications therefore cite that committed snapshot; stored bags provide runtime evidence.\n\n| Feature | Function | Constants | Implementation | Trial observation | Final classification | Evidence |\n|---|---|---|---|---|---|---|\n"
    for row in impl:
        impl_md += f"| {row['feature']} | `{row['class_or_function']}` | {row['relevant_constants']} | {row['implementation_status']} | {row['observed_in_corrected_trials']} | {row['final_classification']} | {row['static_code_evidence']} |\n"
    impl_md += "\nControlled primary switching is fully present and reachable in the committed control flow but was not triggered. The failure-threshold path is only a partial match for the report's claimed emergency backup promotion: it selects the current best eligible candidate rather than explicitly promoting the stored backup, and global stale metrics stop election instead of triggering failover. No dedicated handover/failover test was found.\n"
    (OUT / "implementation_status.md").write_text(impl_md, encoding="utf-8")

    validation_md = """# Experimental Validation Status

| Feature | Implemented in code | Triggered in corrected trials | Dedicated test found | Experimentally validated | Report wording |
|---|---|---|---|---|---|
| Initial election | Yes | Yes (stored early events/states) | No separate test | Yes, within dynamic runs | A valid initial primary and backup were established. |
| Primary-head retention | Yes | Yes | Dynamic trials | Yes | The primary remained unchanged in each corrected trial. |
| Backup-head selection | Yes | Yes | Dynamic trials | Yes | A valid backup was selected in each trial. |
| Backup-head reselection | Yes | Yes | Dynamic trials | Yes | Canonical backup transitions were 2, 4 and 3. |
| Controlled primary handover | Yes | No | No | No | Implemented in the contemporaneous clustering node, but not triggered; trials validate stability, not handover. |
| Emergency primary failover | Partially: threshold-driven best-candidate replacement, not guaranteed backup promotion | No | No | No | A partial primary-link-failure path existed, but emergency backup promotion/failover was not experimentally validated. |
| Cluster assignment publication | Yes | Yes | Dynamic trials | Yes | Complete assignment states were published and recorded. |
| Cluster event publication | Yes | Yes | Dynamic trials | Yes for observed election/reselection events | Change events were published; event count is not the canonical transition count. |
"""
    (OUT / "experimental_validation_status.md").write_text(validation_md, encoding="utf-8")

    claims = """# Report Claims

## Claims supported for Chapter 5

- Complete valid primary/backup assignment states were recorded in all three corrected trials.
- Primary retention was 100% with zero canonical primary transitions in every corrected trial.
- Canonical backup transitions were 2, 4 and 3 for Trials 1–3 after the common initialization exclusion.
- Assignments, role identifiers, candidate scores and change events were published and stored.
- Controlled primary switching logic existed in the contemporaneous implementation but was not triggered by these trials.
- A threshold-driven primary-link-failure path existed, but it was not equivalent to guaranteed immediate backup promotion.

## Claims that must not be made

- Primary handover was successfully demonstrated or experimentally validated.
- Emergency backup promotion/failover was successfully demonstrated.
- Failover recovery time was measured.
- The implementation guarantees fault tolerance.
- Raw event-message totals equal role transitions.
- Three 3-UAV trials establish behavior for larger swarms.
"""
    (OUT / "report_claims.md").write_text(claims, encoding="utf-8")

    plan = """# Targeted Clustering Test Plan

Do not run these tests until the contemporaneous `dynamic_cluster_manager` source/launcher is restored into a controlled test branch. No safe deterministic metric-injection hook was found in the current checkout.

## 1. Controlled primary-handover test

- **Purpose:** trigger `better_candidate_stable` and verify one canonical primary transition.
- **Initial state:** launch the discovered `dynamic_cluster_manager` in the three-UAV city pipeline; wait beyond startup and the configured 10 s primary hold with valid `/ns3_link_snr`, `/ns3_link_rssi`, `/link_obstacle_loss`, and `/uav_world_positions` inputs.
- **Controlled trigger:** keep one eligible challenger more than the configured 0.12 score margin above the current primary for at least three consecutive 2 s elections. Existing mission movement is not deterministic enough; a minimal test-only publisher/hook for the four metric topics is needed if geometry cannot guarantee this.
- **Expected topics/event:** `/cluster/assignment`, `/cluster/primary_ch`, `/cluster/backup_ch`, `/cluster/scores`, `/cluster/event`; event reason `better_candidate_stable` with matching old/new IDs.
- **Pass criteria:** exactly one sampled canonical primary transition after initialization; three winning epochs and holding period evident; a valid new backup and complete assignment published; event matches state change.
- **Record:** challenger/current scores, epochs, hold time, role timeline and transition latency.
- **Duration:** about 30–45 s after readiness.
- **Save:** ROS bag, bag metadata, manager log, injected-metric log/config, extracted assignments/roles/scores/events/links, canonical v2 output.
- **Startup control:** begin trigger only after 0.01 s exclusion, metrics readiness, a stable baseline and the 10 s hold.

## 2. Emergency failover test

- **Purpose:** characterize the implemented `primary_link_failure` path and expose whether the stored backup is promoted.
- **Initial state:** stable primary and eligible backup, with all required metric streams fresh.
- **Controlled trigger:** drive only the active primary's GCS SNR below the implemented -2 dB threshold while keeping the backup eligible. A deterministic test metric publisher/hook is required; stopping all metrics would only exercise the global stale-data guard.
- **Expected topics/event:** same cluster topics; event reason `primary_link_failure`, complete old/new IDs, then recomputed backup.
- **Pass criteria for implemented path:** next election changes primary to the highest-ranked eligible candidate, publishes a complete state/event, and selects a distinct valid backup or explicitly reports backup 0 when none exists. Do not call this guaranteed backup promotion unless the old backup is demonstrably selected by policy.
- **Record:** threshold crossing, election-to-transition delay, old backup versus new primary, scores, data freshness and any no-candidate outcome.
- **Duration:** about 20–30 s after stable baseline.
- **Save:** same artifacts as handover test, plus failure-injection timestamps.
- **Startup control:** inject only after stable assignment and metric readiness; retain a pre-trigger baseline window.
"""
    (OUT / "targeted_test_plan.md").write_text(plan, encoding="utf-8")

    readme = """# Clustering Validation v2

Legacy `backup_changes` counted changes on separately published backup-role samples, while event totals included initialization and a baseline event; neither is the canonical report metric. This package defines transitions from complete `/cluster/assignment` states after a uniform 0.01 s clustering-relative initialization interval. The first retained state is the baseline, repeated role states collapse, and identifier changes are counted once.

The analysis uses only corrected `dynamic_trial1`, `dynamic_trial2`, and `dynamic_trial3`; `dynamic_trial1_original` is excluded from the main comparison. Rerun from the repository root with `python3 results_02/scripts/analyze_clustering_validation_v2.py`.

Use `canonical_trial_summary.csv/.md` and `figures/cluster_roles_timeline.*` in the final report. Use `transition_audit.csv` for traceability. `implementation_status.*` separates static implementation evidence (the contemporaneous committed source snapshot) from observed and experimentally validated behavior.
"""
    (OUT / "README.md").write_text(readme, encoding="utf-8")

    fig, axes = plt.subplots(3, 1, figsize=(10, 8.5), sharex=False)
    for ax, trial in zip(axes, trials):
        first = trial["first"]
        x = [row["time"] - first for row in trial["retained"]]
        primary = [row["primary"] for row in trial["retained"]]
        backup = [row["backup"] for row in trial["retained"]]
        ax.step(x, primary, where="post", label="Primary CH", color="#2878B5", linewidth=2)
        ax.step(x, backup, where="post", label="Backup CH", color="#F28E2B", linewidth=2)
        ax.axvspan(0, INIT_SECONDS, color="grey", alpha=0.2, label="Initialization")
        for transition in trial["canonical_transitions"]:
            color = "#D62728" if transition["role"] == "primary" else "#2CA02C"
            ax.axvline(transition["time"] - first, color=color, linestyle="--", alpha=0.8)
        ax.set_yticks([1, 2, 3], ["UAV1", "UAV2", "UAV3"])
        ax.set_ylim(0.7, 3.3)
        ax.set_title(trial["name"].replace("_", " ").title())
        ax.set_ylabel("Role holder")
        ax.grid(axis="x", alpha=0.2)
    axes[0].legend(ncol=3, loc="upper right")
    axes[-1].set_xlabel("Elapsed time from first clustering timestamp (s)")
    fig.suptitle("Canonical Primary and Backup Cluster-Head Roles")
    fig.tight_layout()
    fig.savefig(FIG / "cluster_roles_timeline.png", dpi=400)
    fig.savefig(FIG / "cluster_roles_timeline.pdf")
    plt.close(fig)

    print("Clustering validation v2")
    for row in summaries:
        print(f"  {row['trial']}: retained={row['assignment_states_retained']} primary={row['canonical_primary_transitions']} backup={row['canonical_backup_transitions']} events={row['total_event_messages']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
