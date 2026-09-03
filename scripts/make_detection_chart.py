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

# Altitude is read per run from that run's own mission log, because the config
# only ever describes the most recent flight.
GEOMETRY = {"UAV1": "holds station,\nstatic group",
            "UAV2": "overflies\nwalkers on the road"}
ALT_FALLBACK = {"UAV1": 30.0, "UAV2": 20.0}

W_PX, H_PX, HFOV, PITCH, PERSON_M = 640, 384, 0.6, math.radians(45), 1.7
VFOV = 2 * math.atan((H_PX / 2) / ((W_PX / 2) / math.tan(HFOV / 2)))

ACCENT, NEUTRAL = "#0B6E7F", "#8A94A1"     # validated: dE 18.1, worst-CVD 14.4
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def gsd_cm(h):
    return 2 * h * math.tan(HFOV / 2) / W_PX * 100


def person_px(h):
    return PERSON_M * math.sin(PITCH) * math.cos(PITCH) * H_PX / (h * VFOV)


def altitudes_of(run_dir):
    """{uav: altitude} from the run's own mission log."""
    log = run_dir / "mission_2uav.log"
    out = {}
    if log.exists():
        for m in re.finditer(r"\[(UAV\d)\] climbing to ([\d.]+) m", log.read_text()):
            out[m.group(1)] = float(m.group(2))
    return out


def read_runs():
    """[(label, {uav: (hits, frames, pct, fps, alt)}), ...] oldest first."""
    runs = []
    for d in sorted(ROOT.glob("results/*/")):
        f = d / "summary.txt"
        if not f.exists():
            continue
        alts = altitudes_of(d)
        per = {}
        for blk in re.finditer(r"── (UAV\d) ─.*?(?=── UAV|\Z)", f.read_text(), re.S):
            uav, body = blk.group(1), blk.group(0)
            fps = re.search(r"=\s*([\d.]+) fps", body)
            det = re.search(r"detected:\s*(\d+) frames.*?\(([\d.]+) %\)", body)
            tot = re.search(r"edge\s*:\s*(\d+) frames", body)
            if fps and det and tot:
                per[uav] = (int(det.group(1)), int(tot.group(1)),
                            float(det.group(2)), float(fps.group(1)),
                            alts.get(uav, ALT_FALLBACK.get(uav, float("nan"))))
        if per:
            runs.append((d.name, per))
    return runs


def plot_sweep(runs, uav):
    """Detection rate against altitude — only valid when altitude is the sole
    variable, i.e. one aircraft flown repeatedly by altitude_sweep.sh."""
    pts = sorted((per[uav][4], per[uav][2], per[uav][0], per[uav][1])
                 for _, per in runs if uav in per)
    fig, ax = plt.subplots(figsize=(8.0, 5.0), dpi=150)
    fig.patch.set_facecolor("white"); ax.set_facecolor("white")

    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    ax.plot(xs, ys, "-o", color=ACCENT, linewidth=2.2, markersize=7, zorder=3)
    for alt, pct, hits, frames in pts:
        ax.annotate(f"{pct:.1f} %", (alt, pct), textcoords="offset points",
                    xytext=(0, 13), ha="center", fontsize=10,
                    fontweight="bold", color=INK)
        ax.annotate(f"{hits}/{frames}", (alt, pct), textcoords="offset points",
                    xytext=(0, -16), ha="center", fontsize=8, color=MUTED)

    ax.set_xticks(xs)
    ax.set_xticklabels([f"{a:.0f} m\n{gsd_cm(a):.2f} cm/px\n~{person_px(a):.0f} px"
                        for a in xs], fontsize=8.5, color=INK, linespacing=1.7)
    ax.set_ylim(0, max(ys) * 1.35 + 1)
    ax.set_ylabel("frames containing a detected person (%)",
                  fontsize=9.5, color=MUTED, labelpad=8)
    ax.tick_params(axis="y", labelsize=9, colors=FAINT, length=0)
    ax.tick_params(axis="x", length=0, pad=6)
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    ax.set_title(f"Detection rate against altitude — {uav}",
                 fontsize=12.5, fontweight="bold", color=INK, pad=30, loc="left")
    ax.text(0, 1.022, "Same mission and geometry at every point, so altitude "
                      "is the only variable that changes.",
            transform=ax.transAxes, fontsize=8.8, color=MUTED)
    fig.text(0.055, 0.015,
             "Source: results/*/summary.txt · altitude read from each run's own "
             "mission log · conf 0.40, IoU 0.70, YOLO11n OpenVINO.",
             fontsize=7, color=FAINT)
    fig.tight_layout(rect=[0, 0.05, 1, 1])
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    return pts


def main():
    runs = read_runs()
    if not runs:
        raise SystemExit("ERROR: no results/*/summary.txt found — fly a mission first.")

    # A sweep (one aircraft, 3+ distinct altitudes) supports a curve. Anything
    # else does not: the aircraft differ in geometry as well as altitude.
    for uav in ("UAV1", "UAV2"):
        alts = {per[uav][4] for _, per in runs if uav in per}
        if len(alts) >= 3:
            pts = plot_sweep(runs, uav)
            print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB  "
                  f"(sweep: {uav}, {len(pts)} altitudes)")
            for alt, pct, hits, frames in pts:
                print(f"    {alt:5.0f} m  GSD {gsd_cm(alt):.2f}  "
                      f"{hits}/{frames} = {pct:.1f}%")
            return

    uavs = ["UAV1", "UAV2"]
    fig, ax = plt.subplots(figsize=(10.0, 5.2), dpi=150)
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
            hits, frames, pct, fps, _alt = per[uav]
            xs.append(pos)
            heights.append(pct)
            colors.append(ACCENT if uav == "UAV1" else NEUTRAL)
            sublabels.append((pos, f"run {i+1}", f"{hits}/{frames}", fps))
            pos += 1.0
        group_centre[uav] = (start + pos - 1.0) / 2
        pos += 0.9                                   # gap between aircraft

    bars = ax.bar(xs, heights, width=0.66, color=colors, zorder=3)

    # Five runs per aircraft: the labels have to shrink or they collide.
    for (x, run_lbl, frac, fps), h in zip(sublabels, heights):
        ax.annotate(f"{h:.1f} %", (x, h), textcoords="offset points",
                    xytext=(0, 15), ha="center", va="bottom",
                    fontsize=9, fontweight="bold", color=INK)
        ax.annotate(frac, (x, h), textcoords="offset points",
                    xytext=(0, 4), ha="center", va="bottom",
                    fontsize=7, color=MUTED)
        ax.annotate(f"{run_lbl}\n{fps:.2f} fps", (x, 0), textcoords="offset points",
                    xytext=(0, -14), ha="center", va="top",
                    fontsize=8.5, color=MUTED, linespacing=1.5)

    # Group captions: the two variables that differ, stated together.
    run_alt = {u: next(per[u][4] for _, per in runs if u in per) for u in group_centre}
    for uav, cx in group_centre.items():
        ax.annotate(f"{uav} — {run_alt[uav]:.0f} m\n"
                    f"GSD {gsd_cm(run_alt[uav]):.2f} cm/px · ~{person_px(run_alt[uav]):.0f} px\n"
                    f"{GEOMETRY[uav]}",
                    (cx, 0), textcoords="offset points", xytext=(0, -52),
                    ha="center", va="top", fontsize=9.5, color=INK,
                    linespacing=1.6, fontweight="bold")

    ax.set_ylim(0, max(heights) * 1.28 + 1)
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

    ax.set_title(f"Per-frame human detection rate — {len(runs)} runs",
                 fontsize=12.5, fontweight="bold", color=INK, pad=16, loc="left")

    fig.text(0.055, 0.018, "Source: results/*/summary.txt · conf 0.40, IoU 0.70",
             fontsize=7, color=FAINT)

    fig.tight_layout(rect=[0, 0.115, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")
    for name, per in runs:
        for uav in uavs:
            if uav in per:
                hits, frames, pct, fps, alt = per[uav]
                print(f"    {name}  {uav}  {alt:.0f} m  "
                      f"GSD {gsd_cm(alt):.2f}  {hits}/{frames} = {pct:.1f}%  {fps:.2f} fps")


if __name__ == "__main__":
    main()
