#!/usr/bin/env python3
"""Target size by altitude, with the detection rates actually measured.

Every value plotted is either computed exactly from the camera geometry or
measured in a real run. Nothing is interpolated or assumed:

  * the curve      - exact, px = H*sin(p)*cos(p)*384 / (h*VFOV)
  * the markers    - the same formula at the requested altitudes
  * the two labels - measured, results/*/summary.txt

The detection rate at the un-flown altitudes is deliberately NOT drawn. Only
two altitudes have been flown, and they differ in mission geometry as well as
height, so any curve through them would attribute to altitude an effect caused
by dwell time. Fly scripts/netns/altitude_sweep.sh to fill this in for real.

    python3 scripts/make_altitude_chart.py

Output: report/altitude_prediction.png  (white background)
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "altitude_prediction.png"

W_PX, H_PX, HFOV = 640, 384, 0.6
PITCH, PERSON_M = math.radians(45), 1.7
VFOV = 2 * math.atan((H_PX / 2) / ((W_PX / 2) / math.tan(HFOV / 2)))

# 0 m is omitted: the aircraft is on the ground there and the formula diverges.
MARKS = [10, 15, 20, 25, 30, 35]

# Measured, from results/*/summary.txt. Geometry differs between the aircraft,
# which is exactly why they are annotations and not a fitted line.
MEASURED = {
    30: ("UAV1 measured\n14.0 % and 14.1 %", "holds station on a static group"),
    20: ("UAV2 measured\n1.5 % and 0.8 %", "overflies a walking subject"),
}

ACCENT, NEUTRAL, CAUTION = "#0B6E7F", "#8A94A1", "#B0430B"
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def gsd_cm(h):
    return 2 * h * math.tan(HFOV / 2) / W_PX * 100


def person_px(h):
    return PERSON_M * math.sin(PITCH) * math.cos(PITCH) * H_PX / (h * VFOV)


def main():
    h = np.linspace(8, 40, 400)
    fig, ax = plt.subplots(figsize=(8.6, 5.2), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    ax.axhspan(0, 30, color=CAUTION, alpha=0.08, zorder=1)
    ax.axhline(30, color=CAUTION, linewidth=1.2, linestyle="--", zorder=2)
    ax.plot(h, [person_px(x) for x in h], color=ACCENT, linewidth=2.2, zorder=3)

    for m in MARKS:
        px = person_px(m)
        ax.plot([m], [px], "o", ms=7, color=ACCENT, zorder=5)
        ax.annotate(f"{px:.0f} px", (m, px), textcoords="offset points",
                    xytext=(0, 11), ha="center", fontsize=9.5,
                    fontweight="bold", color=INK)

    # Left end of the band: the right end collides with the 35 m marker label.
    ax.text(9.2, 14, "below ~30 px, detection becomes unreliable",
            fontsize=8.5, color=CAUTION, ha="left", va="center")

    # Measured rates: called out against the altitude they were flown at, with
    # the mission geometry named so the two are never read as one trend.
    for alt, (label, geom) in MEASURED.items():
        px = person_px(alt)
        ax.annotate(f"{label}\n({geom})", (alt, px),
                    textcoords="offset points", xytext=(14, 46),
                    fontsize=8.5, color=INK, linespacing=1.5,
                    bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                              edgecolor=NEUTRAL, linewidth=1),
                    arrowprops=dict(arrowstyle="-", color=NEUTRAL, linewidth=1))

    ax.set_xticks(MARKS)
    ax.set_xticklabels([f"{m} m\n{gsd_cm(m):.2f} cm/px" for m in MARKS],
                       fontsize=9, color=INK, linespacing=1.7)
    ax.set_xlim(8, 40)
    ax.set_ylim(0, 130)
    ax.set_ylabel("standing person (pixels tall)", fontsize=9.5,
                  color=MUTED, labelpad=8)
    ax.set_xlabel("altitude  ·  ground sampling distance", fontsize=9.5,
                  color=MUTED, labelpad=8)
    ax.tick_params(axis="y", labelsize=9, colors=FAINT, length=0)
    ax.tick_params(axis="x", length=0, pad=6)
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    ax.set_title("Target size by altitude — computed, with measured rates marked",
                 fontsize=12.5, fontweight="bold", color=INK, pad=32, loc="left")
    ax.text(0, 1.028,
            "The curve is exact geometry, not a fit. Detection rate is shown only "
            "where it was actually flown.",
            transform=ax.transAxes, fontsize=8.8, color=MUTED)

    fig.text(0.055, 0.014,
             "Camera as flown: 640×384, HFOV 0.6 rad, 45° pitch, 1.7 m target · "
             "px = H·sin(p)·cos(p)·384 / (h·VFOV) ≈ 889/h\n"
             "Detection rate at the un-flown altitudes is left blank on purpose — "
             "run scripts/netns/altitude_sweep.sh to measure it.",
             fontsize=7, color=FAINT, linespacing=1.6)

    fig.tight_layout(rect=[0, 0.075, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")
    for m in MARKS:
        print(f"    {m:2d} m   GSD {gsd_cm(m):.2f} cm/px   person {person_px(m):5.1f} px")


if __name__ == "__main__":
    main()
