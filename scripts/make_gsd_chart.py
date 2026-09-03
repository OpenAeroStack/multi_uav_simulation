#!/usr/bin/env python3
"""Ground Sampling Distance and person pixel height against altitude.

Two panels rather than one with two y-axes: GSD (cm/px) and pixel height are
different scales, and a dual-axis chart invites the reader to compare two
quantities that were never on the same scale.

Both curves come from the delivered camera configuration in
worlds/small_city_2uav_netns.world:  640x384, HFOV 0.6 rad, 45 deg pitch.

    python3 scripts/make_gsd_chart.py

Output: report/gsd_altitude.png  (white background)
"""
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "gsd_altitude.png"

# ── Camera, as configured in the world file ─────────────────────────────────
W_PX, H_PX = 640, 384
HFOV = 0.6                                   # rad
PITCH = math.radians(45)                     # from horizontal
PERSON_M = 1.7

F_PX = (W_PX / 2) / math.tan(HFOV / 2)       # focal length in pixels
VFOV = 2 * math.atan((H_PX / 2) / F_PX)      # 0.367 rad = 21.0 deg

# Same validated pair as the back-end chart: dE 18.1 normal, 14.4 worst-CVD.
ACCENT, NEUTRAL = "#0B6E7F", "#8A94A1"
CAUTION = "#B0430B"
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"

# Operating altitudes actually flown, from two_drone_mission.py
MARKS = [20, 30]


def gsd_cm(h):
    """Nadir-equivalent ground distance covered by one pixel, cm."""
    return 2 * h * math.tan(HFOV / 2) / W_PX * 100


def person_px(h):
    """Apparent height of a standing person, px.

    The sin*cos term is the oblique correction: a vertical target foreshortens
    by cos(pitch) while slant range grows as 1/sin(pitch). It peaks at 45 deg,
    which is why the camera is pitched there.
    """
    return PERSON_M * math.sin(PITCH) * math.cos(PITCH) * H_PX / (h * VFOV)


def style(ax):
    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")
    ax.tick_params(labelsize=8.5, colors=FAINT, length=0)
    ax.set_xlabel("altitude (m)", fontsize=9, color=MUTED, labelpad=6)


def main():
    h = np.linspace(5, 50, 400)
    gsd = np.array([gsd_cm(x) for x in h])
    px = np.array([person_px(x) for x in h])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9.2, 4.3), dpi=150)
    fig.patch.set_facecolor("white")

    # ── Panel 1: GSD, linear in altitude ────────────────────────────────────
    ax1.set_facecolor("white")
    ax1.plot(h, gsd, color=ACCENT, linewidth=2.2, zorder=3)
    style(ax1)
    ax1.set_ylim(0, 5.2)
    ax1.set_xlim(5, 50)
    ax1.set_ylabel("GSD (cm per pixel)", fontsize=9, color=MUTED, labelpad=6)
    ax1.set_title("Ground Sampling Distance\nrises linearly with altitude",
                  fontsize=11, fontweight="bold", color=INK, pad=12, loc="left")

    for m in MARKS:
        ax1.plot([m], [gsd_cm(m)], "o", ms=6, color=ACCENT, zorder=4)
        ax1.annotate(f"{m} m → {gsd_cm(m):.2f} cm/px", (m, gsd_cm(m)),
                     textcoords="offset points", xytext=(9, -3),
                     fontsize=8.5, color=INK, va="top")

    ax1.text(0.02, 0.93, "GSD = 2·h·tan(HFOV/2) / W", transform=ax1.transAxes,
             fontsize=8.5, color=MUTED, family="monospace")

    # ── Panel 2: person pixel height, inverse in altitude ───────────────────
    ax2.set_facecolor("white")
    ax2.axhspan(0, 30, color=CAUTION, alpha=0.09, zorder=1)
    ax2.axhline(30, color=CAUTION, linewidth=1.2, linestyle="--", zorder=2)
    ax2.plot(h, px, color=ACCENT, linewidth=2.2, zorder=3)
    style(ax2)
    ax2.set_ylim(0, 190)
    ax2.set_xlim(5, 50)
    ax2.set_ylabel("standing person (pixels tall)", fontsize=9, color=MUTED, labelpad=6)
    ax2.set_title("Person pixel height\nfalls as 1/altitude",
                  fontsize=11, fontweight="bold", color=INK, pad=12, loc="left")

    # Inside the band on the left, where the curve is far above it — the right
    # side collides with the 30 m annotation.
    ax2.text(6.5, 13, "detection unreliable\nbelow ~30 px",
             fontsize=8, color=CAUTION, ha="left", va="center", linespacing=1.5)

    for m in MARKS:
        ax2.plot([m], [person_px(m)], "o", ms=6, color=ACCENT, zorder=4)
        ax2.annotate(f"{m} m → {person_px(m):.0f} px", (m, person_px(m)),
                     textcoords="offset points", xytext=(9, 6),
                     fontsize=8.5, color=INK)

    ax2.text(0.60, 0.93, "px ≈ 889 / h", transform=ax2.transAxes,
             fontsize=8.5, color=MUTED, family="monospace")

    fig.text(0.055, 0.015,
             "Camera as flown: 640×384, HFOV 0.6 rad, 45° pitch, 1.7 m target. "
             "At 30 m (~30 px) UAV1 measured a 14.0 % per-frame hit rate on a static group.",
             fontsize=6.8, color=FAINT)

    fig.tight_layout(rect=[0, 0.045, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    w, hh = fig.get_size_inches()
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB  "
          f"{w*150:.0f}x{hh*150:.0f} px")
    print(f"  VFOV {math.degrees(VFOV):.2f}°   check: 30 m → "
          f"{gsd_cm(30):.2f} cm/px, {person_px(30):.1f} px")


if __name__ == "__main__":
    main()
