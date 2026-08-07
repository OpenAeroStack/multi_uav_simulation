#!/usr/bin/env python3
"""Build the report-ready clustering package from corrected stored outputs."""

from __future__ import annotations

import csv
import json
import shutil
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "results_02/network_evaluation"
CANONICAL = BASE / "clustering_validation_v2"
OUT = BASE / "report_clustering_v2"
FIG = OUT / "figures"
EXPORT = ROOT / "report_export/images/chapter5/clustering"
TRIALS = ("dynamic_trial1", "dynamic_trial2", "dynamic_trial3")


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def uav(identifier: str) -> str:
    return f"UAV{identifier}"


def build_trial_summary() -> tuple[list[dict[str, object]], list[dict[str, str]]]:
    canonical = read_csv(CANONICAL / "canonical_trial_summary.csv")
    if [row["trial"] for row in canonical] != list(TRIALS):
        raise ValueError("Canonical summary must contain only corrected Trials 1-3")
    rows = []
    for index, source in enumerate(canonical, start=1):
        rows.append({
            "trial": f"Trial {index}",
            "duration_s": f"{float(source['duration_s']):.3f}",
            "assignment_states_retained": source["assignment_states_retained"],
            "primary_ch": uav(source["primary_ch"]),
            "primary_retention_percent": f"{float(source['primary_retention_percent']):.1f}",
            "canonical_primary_transitions": source["canonical_primary_transitions"],
            "initial_backup": uav(source["backup_ch_initial"]),
            "backup_identifiers_observed": ";".join(uav(item) for item in source["backup_identifiers_observed"].split(";")),
            "canonical_backup_transitions": source["canonical_backup_transitions"],
            "mean_rssi_dbm": f"{float(source['mean_rssi_dbm']):.3f}",
            "mean_snr_db": f"{float(source['mean_snr_db']):.3f}",
            "mean_obstacle_loss_db": f"{float(source['mean_obstacle_loss_db']):.3f}",
        })
    return rows, canonical


def build_link_summary() -> list[dict[str, object]]:
    rows = []
    for index, name in enumerate(TRIALS, start=1):
        links = read_csv(BASE / name / "extracted/network_links.csv")
        values = {
            metric: [float(row["value"]) for row in links if row["metric"] == metric]
            for metric in ("rssi_dbm", "snr_db", "obstacle_loss_db")
        }
        rows.append({
            "trial": f"Trial {index}",
            "mean_rssi_dbm": f"{statistics.fmean(values['rssi_dbm']):.6f}",
            "std_rssi_db": f"{statistics.stdev(values['rssi_dbm']):.6f}",
            "minimum_rssi_dbm": f"{min(values['rssi_dbm']):.6f}",
            "maximum_rssi_dbm": f"{max(values['rssi_dbm']):.6f}",
            "mean_snr_db": f"{statistics.fmean(values['snr_db']):.6f}",
            "std_snr_db": f"{statistics.stdev(values['snr_db']):.6f}",
            "minimum_snr_db": f"{min(values['snr_db']):.6f}",
            "maximum_snr_db": f"{max(values['snr_db']):.6f}",
            "mean_obstacle_loss_db": f"{statistics.fmean(values['obstacle_loss_db']):.6f}",
            "maximum_obstacle_loss_db": f"{max(values['obstacle_loss_db']):.6f}",
            "link_rows": len(links),
        })
    return rows


def copy_role_timeline() -> None:
    for suffix in ("png", "pdf"):
        source = CANONICAL / "figures" / f"cluster_roles_timeline.{suffix}"
        shutil.copy2(source, FIG / source.name)
        if suffix == "png":
            shutil.copy2(source, EXPORT / source.name)


def plot_trial_quality(rows: list[dict[str, object]]) -> None:
    labels = [str(row["trial"]) for row in rows]
    specifications = [
        ("mean_rssi_dbm", "Mean RSSI (dBm)", "#2878B5", (-66, -56)),
        ("mean_snr_db", "Mean SNR (dB)", "#59A14F", (29, 37)),
        ("mean_obstacle_loss_db", "Mean obstacle loss (dB)", "#F28E2B", (0, 3.2)),
    ]
    fig, axes = plt.subplots(3, 1, figsize=(8, 8.5))
    for ax, (field, ylabel, colour, limits) in zip(axes, specifications):
        values = [float(row[field]) for row in rows]
        bars = ax.bar(labels, values, color=colour, width=0.55)
        ax.set_ylabel(ylabel)
        ax.set_ylim(*limits)
        ax.grid(axis="y", alpha=0.25)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.3f}",
                    ha="center", va="bottom", fontsize=9)
    axes[-1].set_xlabel("Corrected dynamic trial")
    fig.suptitle("Trial-Level Dynamic Link Quality")
    fig.tight_layout()
    fig.savefig(FIG / "link_quality_by_trial.png", dpi=300)
    fig.savefig(FIG / "link_quality_by_trial.pdf")
    shutil.copy2(FIG / "link_quality_by_trial.png", EXPORT / "link_quality_by_trial.png")
    plt.close(fig)


def canonical_trial1_transitions() -> list[float]:
    audit = read_csv(CANONICAL / "transition_audit.csv")
    return [float(row["timestamp_s"]) for row in audit
            if row["trial"] == "dynamic_trial1"
            and row["role"] == "backup"
            and row["counted_in_canonical_metric"] == "true"]


def plot_rssi_timeline() -> None:
    rows = read_csv(BASE / "dynamic_trial1/extracted/network_links.csv")
    fig, ax = plt.subplots(figsize=(9, 5.2))
    colours = {1: "#2878B5", 2: "#F28E2B", 3: "#59A14F"}
    for destination in (1, 2, 3):
        selected = [row for row in rows if row["metric"] == "rssi_dbm"
                    and int(row["source"]) == 0 and int(row["destination"]) == destination]
        ax.plot([float(row["time_s"]) for row in selected],
                [float(row["value"]) for row in selected],
                label=f"GCS $\\rightarrow$ UAV{destination}", color=colours[destination],
                linewidth=1.4, alpha=0.9)
    for index, timestamp in enumerate(canonical_trial1_transitions()):
        ax.axvline(timestamp, color="#8C564B", linestyle="--", linewidth=1.2,
                   label="Canonical backup transition" if index == 0 else None)
    ax.set_xlabel("Elapsed time from bag start (s)")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_title("Corrected Trial 1 GCS-to-UAV RSSI")
    ax.legend(ncol=2)
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(FIG / "representative_rssi_timeline.png", dpi=300)
    fig.savefig(FIG / "representative_rssi_timeline.pdf")
    shutil.copy2(FIG / "representative_rssi_timeline.png", EXPORT / "representative_rssi_timeline.png")
    plt.close(fig)


def parse_assignments() -> list[dict[str, object]]:
    rows = []
    for row in read_csv(BASE / "dynamic_trial1/extracted/cluster_assignments.csv"):
        payload = json.loads(row["assignment"])
        rows.append({"time": float(row["time_s"]), "payload": payload})
    # Apply the canonical 0.01 s exclusion relative to the earliest clustering time.
    time_sources = [
        ("cluster_assignments.csv", "time_s"),
        ("cluster_roles.csv", "time_s"),
        ("cluster_scores_raw.csv", "time_s"),
        ("cluster_events.csv", "time_s"),
    ]
    first_cluster = min(
        float(source[time_field])
        for filename, time_field in time_sources
        for source in read_csv(BASE / "dynamic_trial1/extracted" / filename)
    )
    return [row for row in rows if float(row["time"]) >= first_cluster + 0.01]


def nearest_positions(timestamp: float) -> tuple[float, dict[int, tuple[float, float]]]:
    rows = read_csv(BASE / "dynamic_trial1/extracted/positions.csv")
    times = sorted({float(row["time_s"]) for row in rows})
    chosen = min(times, key=lambda value: abs(value - timestamp))
    positions = {int(row["node_id"]): (float(row["x_m"]), float(row["y_m"]))
                 for row in rows if float(row["time_s"]) == chosen}
    return chosen, positions


def plot_topology() -> None:
    assignments = parse_assignments()
    selected = [("Initial state", assignments[0]), ("Final state", assignments[-1])]
    fig, axes = plt.subplots(1, 2, figsize=(11, 5.2))
    role_style = {
        "primary": ("#2878B5", "*", 230, "Primary CH"),
        "backup": ("#F28E2B", "D", 110, "Backup CH"),
        "member": ("#A0A0A0", "o", 100, "Ordinary member"),
    }
    for ax, (title, assignment) in zip(axes, selected):
        position_time, positions = nearest_positions(float(assignment["time"]))
        payload = assignment["payload"]
        primary, backup = int(payload["primary_ch"]), int(payload["backup_ch"])
        gx, gy = positions[0]
        ax.scatter(gx, gy, color="black", marker="X", s=130, label="GCS", zorder=3)
        ax.annotate("GCS", (gx, gy), xytext=(5, 6), textcoords="offset points")
        for identifier in (1, 2, 3):
            role = "primary" if identifier == primary else "backup" if identifier == backup else "member"
            colour, marker, size, legend = role_style[role]
            x, y = positions[identifier]
            ax.scatter(x, y, color=colour, marker=marker, s=size, label=legend, zorder=3)
            ax.annotate(f"UAV{identifier}", (x, y), xytext=(5, 6), textcoords="offset points")
        ax.set_title(f"{title}\nassignment {float(assignment['time']):.3f} s; position {position_time:.3f} s")
        ax.set_xlabel("x position (m)")
        ax.set_ylabel("y position (m)")
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(alpha=0.25)
    handles, labels = axes[0].get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    fig.legend(unique.values(), unique.keys(), loc="lower center", ncol=4)
    fig.suptitle("Corrected Trial 1 Initial and Final Role Topology")
    fig.tight_layout(rect=(0, 0.09, 1, 0.94))
    fig.savefig(FIG / "representative_topology_initial_final.png", dpi=300)
    fig.savefig(FIG / "representative_topology_initial_final.pdf")
    shutil.copy2(FIG / "representative_topology_initial_final.png",
                 EXPORT / "representative_topology_initial_final.png")
    plt.close(fig)


def main() -> int:
    FIG.mkdir(parents=True, exist_ok=True)
    EXPORT.mkdir(parents=True, exist_ok=True)
    trial_rows, _ = build_trial_summary()
    link_rows = build_link_summary()
    write_csv(OUT / "trial_summary.csv", trial_rows)
    write_csv(OUT / "link_quality_summary.csv", link_rows)
    copy_role_timeline()
    plot_trial_quality(trial_rows)
    plot_rssi_timeline()
    plot_topology()
    print("Report clustering v2")
    for row in trial_rows:
        print(f"  {row['trial']}: primary={row['canonical_primary_transitions']} "
              f"backup={row['canonical_backup_transitions']} links="
              f"{next(item['link_rows'] for item in link_rows if item['trial'] == row['trial'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
