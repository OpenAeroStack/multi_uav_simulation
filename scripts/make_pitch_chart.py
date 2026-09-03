#!/usr/bin/env python3
"""Optimum gimbal pitch for human detection.

Target pixel height goes as sin(p)*cos(p), which peaks at exactly 45 deg.
Plotted for the delivered camera at the altitude actually flown.

    python3 scripts/make_pitch_chart.py

Output: report/gimbal_pitch.png  (white background)
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "gimbal_pitch.png"

W_PX, H_PX, HFOV = 640, 384, 0.6
PERSON_M, ALT_M = 1.7, 30.0
VFOV = 2 * math.atan((H_PX / 2) / ((W_PX / 2) / math.tan(HFOV / 2)))

ACCENT, INK, MUTED, GRID = "#0B6E7F", "#141C26", "#5B6775", "#E6EAF0"


def person_px(pitch_deg, h=ALT_M):
    p = math.radians(pitch_deg)
    return PERSON_M * math.sin(p) * math.cos(p) * H_PX / (h * VFOV)


def main():
    p = np.linspace(5, 85, 600)
    fig, ax = plt.subplots(figsize=(7.0, 4.6), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    ax.plot(p, [person_px(x) for x in p], color=ACCENT, linewidth=2.4, zorder=3)

    peak = 45.0
    ax.vlines(peak, 0, person_px(peak), color=ACCENT, linewidth=1,
              linestyle=":", zorder=2)
    ax.plot([peak], [person_px(peak)], "o", ms=8, color=ACCENT, zorder=5)
    ax.annotate("45°", (peak, person_px(peak)), textcoords="offset points",
                xytext=(0, 12), ha="center", fontsize=13, fontweight="bold",
                color=INK)

    ax.set_xlim(5, 85)
    ax.set_ylim(0, 36)
    ax.set_xticks([15, 30, 45, 60, 75])
    ax.set_xlabel("gimbal pitch (degrees below horizontal)",
                  fontsize=10.5, color=INK, labelpad=9)
    ax.set_ylabel("person height in image (pixels)",
                  fontsize=10.5, color=INK, labelpad=9)
    ax.tick_params(labelsize=9.5, colors=MUTED, length=0)
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    fig.tight_layout()
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")
    for deg in (15, 30, 45, 60, 75):
        print(f"    {deg:2d}°  {person_px(deg):5.1f} px")


if __name__ == "__main__":
    main()
