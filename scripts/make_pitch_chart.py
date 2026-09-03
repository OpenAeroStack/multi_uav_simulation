#!/usr/bin/env python3
"""Why the camera sits at 45 deg: target size and dwell against gimbal pitch.

Both curves are exact geometry for the delivered camera. There is no measured
detection rate at any pitch other than 45 deg — every archived run used
0.7854 rad — so no detection curve is drawn against pitch. The single measured
rate is marked at the pitch it was flown at.

The 60 deg reference is the previous setting recorded in
models/iris_1_netns/model.sdf ("WAS: 1.0472 (60 deg)"), so the improvement
shown for the change to 45 deg is computed, not assumed.

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
VFOV_DEG = math.degrees(VFOV)

CURRENT_DEG, PREVIOUS_DEG = 45.0, 60.0

ACCENT, NEUTRAL, CAUTION = "#0B6E7F", "#8A94A1", "#B0430B"
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def person_px(pitch_deg, h=ALT_M):
    """Vertical target foreshortens by cos(p); slant range grows as 1/sin(p)."""
    p = math.radians(pitch_deg)
    return PERSON_M * math.sin(p) * math.cos(p) * H_PX / (h * VFOV)


def band(pitch_deg, h=ALT_M):
    """(near, far) ground distance covered by the frame, metres."""
    near = h / math.tan(math.radians(pitch_deg + VFOV_DEG / 2))
    far = h / math.tan(math.radians(pitch_deg - VFOV_DEG / 2))
    return near, far


def style(ax):
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")
    ax.tick_params(labelsize=8.5, colors=FAINT, length=0)
    ax.set_xlabel("gimbal pitch below horizontal (degrees)",
                  fontsize=9, color=MUTED, labelpad=6)
    ax.set_xticks([20, 30, 45, 60, 70])


def main():
    p = np.linspace(20, 72, 400)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.6, 4.6), dpi=150)
    fig.patch.set_facecolor("white")

    # ── Panel 1: target size peaks at 45 deg ────────────────────────────────
    ax1.set_facecolor("white")
    # Two altitudes: at 30 m no pitch clears the floor, at 20 m every sensible
    # one does. A single 30 m curve under a full-height band reads as "all bad".
    ax1.axhline(30, color=CAUTION, linewidth=1.2, linestyle="--", zorder=2)
    for alt, col, lbl in ((20.0, ACCENT, "20 m altitude"),
                          (30.0, NEUTRAL, "30 m altitude — flown")):
        ax1.plot(p, [person_px(x, alt) for x in p], color=col,
                 linewidth=2.2, zorder=3, label=lbl)
    style(ax1)
    ax1.set_ylim(0, 52)
    leg = ax1.legend(loc="lower center", fontsize=8.5, frameon=False,
                     ncol=1, handlelength=1.6, borderpad=0.2)
    for t in leg.get_texts():
        t.set_color(MUTED)
    ax1.set_xlim(20, 72)
    ax1.set_ylabel("standing person (pixels tall)",
                   fontsize=9, color=MUTED, labelpad=6)
    ax1.set_title("Target size peaks at 45°",
                  fontsize=11.5, fontweight="bold", color=INK, pad=12, loc="left")

    for deg in (CURRENT_DEG, PREVIOUS_DEG):
        for alt, col in ((20.0, ACCENT), (30.0, NEUTRAL)):
            ax1.plot([deg], [person_px(deg, alt)], "o", ms=6, color=col, zorder=5)
    ax1.annotate(f"45°\n{person_px(45, 20):.0f} / {person_px(45, 30):.0f} px",
                 (45, person_px(45, 20)), textcoords="offset points",
                 xytext=(0, 8), ha="center", fontsize=8.5,
                 fontweight="bold", color=INK, linespacing=1.5)
    ax1.annotate(f"60° previous\n{person_px(60, 20):.0f} / {person_px(60, 30):.0f} px",
                 (60, person_px(60, 20)), textcoords="offset points",
                 xytext=(10, 6), ha="left", fontsize=8.5, color=MUTED,
                 linespacing=1.5)

    gain = person_px(CURRENT_DEG) / person_px(PREVIOUS_DEG) - 1
    ax1.text(0.03, 0.93, "px ∝ sin(p)·cos(p)", transform=ax1.transAxes,
             fontsize=8.5, color=MUTED, family="monospace")
    # Lower-left: above the peak it collides with the 45° callout.
    ax1.text(0.03, 0.07, f"45° over 60°:  +{gain*100:.0f} % pixels",
             transform=ax1.transAxes, fontsize=8.5, color=ACCENT, fontweight="bold")
    ax1.annotate("~30 px floor", (71, 30), textcoords="offset points",
                 xytext=(-2, 4), ha="right", fontsize=7.5, color=CAUTION)

    # ── Panel 2: the ground the frame covers, and how far ahead ─────────────
    ax2.set_facecolor("white")
    near = [band(x)[0] for x in p]
    far = [band(x)[1] for x in p]
    ax2.fill_between(p, near, far, color=ACCENT, alpha=0.14, zorder=2,
                     label="visible band")
    ax2.plot(p, far, color=ACCENT, linewidth=1.8, zorder=3, label="far edge")
    ax2.plot(p, near, color=ACCENT, linewidth=1.8, linestyle="--", zorder=3,
             label="near edge — blind below")
    style(ax2)
    ax2.set_ylim(0, 100)
    ax2.set_xlim(20, 72)
    ax2.set_ylabel(f"ground distance ahead at {ALT_M:.0f} m (m)",
                   fontsize=9, color=MUTED, labelpad=6)
    ax2.set_title("Steeper pitch shrinks the visible band",
                  fontsize=11.5, fontweight="bold", color=INK, pad=12, loc="left")

    n45, f45 = band(CURRENT_DEG)
    ax2.plot([CURRENT_DEG, CURRENT_DEG], [n45, f45], color=INK,
             linewidth=1.4, zorder=4)
    ax2.annotate(f"45°: {n45:.0f}–{f45:.0f} m\n({f45-n45:.0f} m of band)",
                 (CURRENT_DEG, f45), textcoords="offset points", xytext=(8, 6),
                 fontsize=8.5, color=INK, linespacing=1.5)
    n60, f60 = band(PREVIOUS_DEG)
    ax2.annotate(f"60°: {n60:.0f}–{f60:.0f} m", (PREVIOUS_DEG, f60),
                 textcoords="offset points", xytext=(6, 8),
                 fontsize=8.5, color=MUTED)

    leg2 = ax2.legend(loc="upper right", fontsize=8.5, frameon=False,
                      handlelength=1.6, borderpad=0.2)
    for t in leg2.get_texts():
        t.set_color(MUTED)

    fig.text(0.055, 0.015,
             "Exact geometry for the delivered camera: 640×384, HFOV 0.6 rad, 1.7 m "
             "target; right-hand panel at 30 m. No run has been flown at any pitch other "
             "than 45°, so no measured\ndetection rate is plotted against pitch — at 45° "
             "UAV1 measured 14.0 % and 14.1 %. A steeper pitch also widens the "
             "COCO→aerial domain gap, which this geometry does not capture.",
             fontsize=6.8, color=FAINT, linespacing=1.6)

    fig.tight_layout(rect=[0, 0.075, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")
    for deg in (20, 30, 45, 60, 70):
        n, f = band(deg)
        print(f"    {deg:2d}°  person {person_px(deg):5.1f} px   "
              f"band {n:5.1f}-{f:6.1f} m ({f-n:5.1f} m wide)")


if __name__ == "__main__":
    main()
