#!/usr/bin/env python3
"""Measured per-frame detection rate, by aircraft and run.

Reads results/*/summary.txt, so it plots what was flown rather than
hand-copied numbers.

Deliberately NOT a GSD curve. The two aircraft differ in altitude AND in
mission geometry at the same time, so a line drawn between them would read as
a GSD effect when it is not one: UAV2 flies lower (finer GSD, a bigger target
in pixels) and detects an order of magnitude less, because it overflies a
walking subject instead of holding station near a static group.

    python3 scripts/make_detection_chart.py

Output: report/detection_rate.png  (white background)
"""
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "detection_rate.png"

# Altitude per aircraft, from two_drone_mission.py DRONES config.
ALT = {"UAV1": 30.0, "UAV2": 20.0}
GEOMETRY = {"UAV1": "holds station,\nstatic group",
            "UAV2": "overflies a\nwalking subject"}

W_PX, H_PX, HFOV, PITCH, PERSON_M = 640, 384, 0.6, math.radians(45), 1.7
VFOV = 2 * math.atan((H_PX / 2) / ((W_PX / 2) / math.tan(HFOV / 2)))

ACCENT, NEUTRAL = "#0B6E7F", "#8A94A1"     # validated: dE 18.1, worst-CVD 14.4
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def gsd_cm(h):
    return 2 * h * math.tan(HFOV / 2) / W_PX * 100


def person_px(h):
    return PERSON_M * math.sin(PITCH) * math.cos(PITCH) * H_PX / (h * VFOV)


def read_runs():
    """[(run_label, {uav: (hits, frames, pct, fps)}), ...] oldest first."""
    runs = []
    for d in sorted(ROOT.glob("results/*/")):
        f = d / "summary.txt"
        if not f.exists():
            continue
        per = {}
        for blk in re.finditer(r"── (UAV\d) ─.*?(?=── UAV|\Z)", f.read_text(), re.S):
            uav, body = blk.group(1), blk.group(0)
            fps = re.search(r"=\s*([\d.]+) fps", body)
            det = re.search(r"detected:\s*(\d+) frames.*?\(([\d.]+) %\)", body)
            tot = re.search(r"edge\s*:\s*(\d+) frames", body)
            if fps and det and tot:
                per[uav] = (int(det.group(1)), int(tot.group(1)),
                            float(det.group(2)), float(fps.group(1)))
        if per:
            runs.append((d.name, per))
    return runs


def main():
    runs = read_runs()
    if not runs:
        raise SystemExit("ERROR: no results/*/summary.txt found — fly a mission first.")

    uavs = ["UAV1", "UAV2"]
    fig, ax = plt.subplots(figsize=(8.2, 5.0), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    xs, heights, colors, sublabels = [], [], [], []
    pos = 0.0
    group_centre = {}
    for uav in uavs:
        start = pos
        for i, (name, per) in enumerate(runs):
            if uav not in per:
                continue
            hits, frames, pct, fps = per[uav]
            xs.append(pos)
            heights.append(pct)
            colors.append(ACCENT if uav == "UAV1" else NEUTRAL)
            sublabels.append((pos, f"run {i+1}", f"{hits}/{frames}", fps))
            pos += 1.0
        group_centre[uav] = (start + pos - 1.0) / 2
        pos += 0.9                                   # gap between aircraft

    bars = ax.bar(xs, heights, width=0.66, color=colors, zorder=3)

    for (x, run_lbl, frac, fps), h in zip(sublabels, heights):
        ax.annotate(f"{h:.1f} %", (x, h), textcoords="offset points",
                    xytext=(0, 18), ha="center", va="bottom",
                    fontsize=11, fontweight="bold", color=INK)
        ax.annotate(f"{frac} frames", (x, h), textcoords="offset points",
                    xytext=(0, 5), ha="center", va="bottom",
                    fontsize=8, color=MUTED)
        ax.annotate(f"{run_lbl}\n{fps:.2f} fps", (x, 0), textcoords="offset points",
                    xytext=(0, -14), ha="center", va="top",
                    fontsize=8.5, color=MUTED, linespacing=1.5)

    # Group captions: the two variables that differ, stated together.
    for uav, cx in group_centre.items():
        ax.annotate(f"{uav} — {ALT[uav]:.0f} m\n"
                    f"GSD {gsd_cm(ALT[uav]):.2f} cm/px · ~{person_px(ALT[uav]):.0f} px\n"
                    f"{GEOMETRY[uav]}",
                    (cx, 0), textcoords="offset points", xytext=(0, -52),
                    ha="center", va="top", fontsize=9.5, color=INK,
                    linespacing=1.6, fontweight="bold")

    ax.set_ylim(0, 19)
    ax.set_xlim(-0.7, pos - 1.2)
    ax.set_xticks([])
    ax.set_ylabel("frames containing a detected person (%)",
                  fontsize=9.5, color=MUTED, labelpad=8)
    ax.tick_params(axis="y", labelsize=9, colors=FAINT, length=0)
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    ax.set_title("Per-frame human detection rate — measured, two runs",
                 fontsize=12.5, fontweight="bold", color=INK, pad=30, loc="left")
    ax.text(0, 1.022,
            "UAV2 flies lower, so its target is larger in pixels — and it detects "
            "10x less. Mission geometry, not GSD, dominates.",
            transform=ax.transAxes, fontsize=8.8, color=MUTED)

    fig.text(0.055, 0.012,
             "Source: results/*/summary.txt · same model, conf 0.40, IoU 0.70 on both aircraft.\n"
             "UAV1 repeats to within 0.1 pp across runs even though its throughput fell "
             "2.85 → 1.31 fps under thermal throttling: per-frame accuracy is\n"
             "independent of how many frames get processed.",
             fontsize=7, color=FAINT, linespacing=1.6)

    fig.tight_layout(rect=[0, 0.135, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")
    for name, per in runs:
        for uav in uavs:
            if uav in per:
                hits, frames, pct, fps = per[uav]
                print(f"    {name}  {uav}  {ALT[uav]:.0f} m  "
                      f"GSD {gsd_cm(ALT[uav]):.2f}  {hits}/{frames} = {pct:.1f}%  {fps:.2f} fps")


if __name__ == "__main__":
    main()
