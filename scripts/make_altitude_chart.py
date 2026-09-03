#!/usr/bin/env python3
"""Target size against altitude.

The curve is exact geometry for the delivered camera, not a fit:
px = H*sin(p)*cos(p)*384 / (h*VFOV), which reduces to about 889/h at 45 deg.

No detection rate is plotted. Only two altitudes have been flown and they
differ in mission geometry as well as height, so a curve through them would
attribute to altitude an effect caused by dwell time on target. Fly
scripts/netns/altitude_sweep.sh to measure it properly.

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

ACCENT, CAUTION = "#0B6E7F", "#B0430B"
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def person_px(h):
    return PERSON_M * math.sin(PITCH) * math.cos(PITCH) * H_PX / (h * VFOV)


def main():
    h = np.linspace(8, 40, 400)
    fig, ax = plt.subplots(figsize=(7.0, 4.6), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    ax.axhline(30, color=CAUTION, linewidth=1.2, linestyle="--", zorder=2)
    ax.plot(h, [person_px(x) for x in h], color=ACCENT, linewidth=2.4, zorder=3)

    for m in MARKS:
        px = person_px(m)
        ax.plot([m], [px], "o", ms=7, color=ACCENT, zorder=5)
        ax.annotate(f"{px:.0f}", (m, px), textcoords="offset points",
                    xytext=(0, 10), ha="center", fontsize=10,
                    fontweight="bold", color=INK)

    ax.annotate("30 px", (39.6, 30), textcoords="offset points", xytext=(0, 5),
                ha="right", fontsize=8.5, color=CAUTION)

    ax.set_xticks(MARKS)
    ax.set_xlim(8, 40)
    ax.set_ylim(0, 120)
    ax.set_xlabel("altitude (m)", fontsize=10.5, color=INK, labelpad=9)
    ax.set_ylabel("person height in image (pixels)",
                  fontsize=10.5, color=INK, labelpad=9)
    ax.tick_params(labelsize=9.5, colors=MUTED, length=0)
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    fig.text(0.055, 0.012,
             "Computed geometry: 640×384, HFOV 0.6 rad, 45° pitch, 1.7 m target · "
             "px = H·sin(p)·cos(p)·384 / (h·VFOV)\nDashed line marks the ~30 px "
             "practical floor for YOLO-family detectors.",
             fontsize=7, color=FAINT, linespacing=1.6)

    fig.tight_layout(rect=[0, 0.075, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")
    for m in MARKS:
        print(f"    {m:2d} m   {person_px(m):5.1f} px")


if __name__ == "__main__":
    main()
