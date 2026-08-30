#!/usr/bin/env python3
"""Draw the coordinate-frame figures for docs/COORDINATE_FRAMES.pdf.

Separate from the PDF builder so the drawings can be tweaked and re-rendered
on their own:

    python3 scripts/make_frames_figures.py
"""
import math
import os

import matplotlib
matplotlib.use("Agg")                      # no display needed
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Circle, Rectangle

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "figures")
os.makedirs(OUT_DIR, exist_ok=True)

INK    = "#16202c"
ACCENT = "#1f5f8b"
WARN   = "#a8402b"
OK     = "#1c6b4d"
GREY   = "#8a97a6"


def save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=160, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(f"  wrote {path}")


def arrow(ax, x0, y0, x1, y1, color=INK, width=2.2, style="-|>"):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle=style,
                                 mutation_scale=16, lw=width, color=color,
                                 shrinkA=0, shrinkB=0))


# ── figure 1: the two frames, side by side ───────────────────────────────────
def fig_two_frames():
    fig, (a, b) = plt.subplots(1, 2, figsize=(9.2, 4.2))

    for ax in (a, b):
        ax.set_xlim(-1.4, 1.7); ax.set_ylim(-1.5, 1.6)
        ax.set_aspect("equal"); ax.axis("off")

    # left: what the WORLD FILE uses
    a.set_title("Gazebo world file\n(what you read from the .world)",
                fontsize=11, color=INK, pad=10)
    arrow(a, 0, 0, 1.1, 0, ACCENT)
    arrow(a, 0, 0, 0, 1.1, ACCENT)
    a.text(1.18, 0, "x", fontsize=13, color=ACCENT, va="center", weight="bold")
    a.text(0, 1.2, "y", fontsize=13, color=ACCENT, ha="center", weight="bold")
    a.text(1.18, -0.22, "NORTH", fontsize=9, color=GREY, va="center")
    a.text(0.05, 1.34, "WEST", fontsize=9, color=GREY, ha="left")
    a.text(0, -1.35, "z is UP, out of the page", fontsize=8.5,
           color=GREY, ha="center")

    # right: what the AUTOPILOT uses
    b.set_title("ArduPilot autopilot\n(what goto and GPS use)",
                fontsize=11, color=INK, pad=10)
    arrow(b, 0, 0, 0, 1.1, OK)
    arrow(b, 0, 0, 1.1, 0, OK)
    b.text(0, 1.2, "N", fontsize=13, color=OK, ha="center", weight="bold")
    b.text(1.18, 0, "E", fontsize=13, color=OK, va="center", weight="bold")
    b.text(0, -1.35, "D is DOWN, into the page", fontsize=8.5,
           color=GREY, ha="center")

    fig.text(0.5, -0.02,
             "north = gazebo_x        east = − gazebo_y",
             ha="center", fontsize=13, color=WARN, weight="bold",
             family="monospace")
    fig.text(0.5, -0.10,
             "set by  <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED>  "
             "in the model SDF",
             ha="center", fontsize=8.5, color=GREY)
    save(fig, "frames_two.png")


# ── figure 2: the 90-degree error, as a map ──────────────────────────────────
def fig_the_error():
    fig, ax = plt.subplots(figsize=(7.2, 6.4))

    spawn  = (-70.0, -22.0)      # gazebo x, y
    people = (40.5, -187.9)

    def to_map(gx, gy):
        """Plot in autopilot terms: east to the right, north up."""
        return -gy, gx            # east = -y, north = x

    sx, sy = to_map(*spawn)
    px, py = to_map(*people)
    wx, wy = sx + 110.5, sy - 165.9        # east/north swapped

    # correct route
    arrow(ax, sx, sy, px, py, OK, 2.8)
    # wrong route
    arrow(ax, sx, sy, wx, wy, WARN, 2.8)
    ax.plot(wx, wy, "x", ms=14, color=WARN, mew=3, zorder=6)

    # the gap between where it should have gone and where it went
    ax.annotate("", xy=(px, py), xytext=(wx, wy),
                arrowprops=dict(arrowstyle="<->", color=GREY, lw=1.3, ls=":"))

    ax.plot(sx, sy, "o", ms=12, color=ACCENT, zorder=6)
    ax.plot(px, py, "*", ms=22, color=OK, zorder=6)

    # labels, all placed clear of the lines
    ax.annotate("drone spawn\nGazebo (−70, −22)", xy=(sx, sy),
                xytext=(sx - 4, sy - 78), fontsize=9, color=ACCENT,
                ha="center", arrowprops=dict(arrowstyle="-", color=ACCENT, lw=1))
    ax.annotate("5 people\nGazebo (40.5, −187.9)", xy=(px, py),
                xytext=(px - 46, py + 34), fontsize=9, color=OK,
                ha="center", arrowprops=dict(arrowstyle="-", color=OK, lw=1))
    ax.annotate("CORRECT\n+165.9 E, +110.5 N\nbearing 56°",
                xy=((sx + px) / 2, (sy + py) / 2),
                xytext=(sx + 30, sy + 62), fontsize=9.5, color=OK,
                weight="bold", ha="center",
                arrowprops=dict(arrowstyle="-", color=OK, lw=1))
    ax.annotate("WHERE IT WENT\n+110.5 E, −165.9 N\nbearing 146°",
                xy=(wx, wy), xytext=(wx - 78, wy + 4), fontsize=9.5,
                color=WARN, weight="bold", ha="center",
                arrowprops=dict(arrowstyle="-", color=WARN, lw=1))
    ax.text((px + wx) / 2 + 10, (py + wy) / 2, "218 m\napart",
            fontsize=9.5, color=GREY, style="italic", va="center")

    ax.set_xlabel("east  (metres)", fontsize=10, color=INK)
    ax.set_ylabel("north  (metres)", fontsize=10, color=INK)
    ax.set_title("Same two numbers, axes swapped — 90° off",
                 fontsize=12.5, color=INK, pad=14)
    ax.grid(alpha=0.25, ls=":")
    ax.set_aspect("equal")
    ax.margins(0.22)
    save(fig, "frames_error.png")


# ── figure 3: world frame vs body frame ──────────────────────────────────────
def fig_world_vs_body():
    fig, ax = plt.subplots(figsize=(8.2, 4.8))
    ax.set_xlim(-10, 120); ax.set_ylim(-30, 55)
    ax.set_aspect("equal"); ax.axis("off")

    # ground
    ax.add_patch(Rectangle((-5, -26), 120, 22, color="#eef2f6"))
    ax.text(-3, -23, "ground", fontsize=8, color=GREY)

    # drone
    dx, dy = 20, 30
    ax.add_patch(Circle((dx, dy), 4.2, color=ACCENT, zorder=5))
    ax.text(dx, dy + 8, "drone", fontsize=9.5, color=ACCENT, ha="center")

    # world frame, fixed at the origin
    arrow(ax, 0, 0, 16, 0, OK, 2)
    arrow(ax, 0, 0, 0, 16, OK, 2)
    ax.text(18, 0, "E", fontsize=10, color=OK, va="center", weight="bold")
    ax.text(0, 18, "N", fontsize=10, color=OK, ha="center", weight="bold")
    ax.text(0, -8, "WORLD frame\nfixed, never moves", fontsize=8.5, color=OK)

    # body frame, tilted with the drone
    th = math.radians(-25)
    L = 16
    arrow(ax, dx, dy, dx + L * math.cos(th), dy + L * math.sin(th), WARN, 2)
    arrow(ax, dx, dy, dx - L * math.sin(th), dy + L * math.cos(th), WARN, 2)
    ax.text(dx + 20, dy - 8, "forward", fontsize=9, color=WARN)
    ax.text(dx - 14, dy + 18, "BODY frame\nturns with the drone",
            fontsize=8.5, color=WARN)

    # camera cone, 45 deg down and ahead
    cam_from = (dx + 3, dy - 2)
    for ang, ln in ((-52, 46), (-31, 74)):
        a = math.radians(ang)
        arrow(ax, *cam_from, cam_from[0] + ln * math.cos(a),
              cam_from[1] + ln * math.sin(a), GREY, 1.2, "-")
    ax.fill([cam_from[0],
             cam_from[0] + 46 * math.cos(math.radians(-52)),
             cam_from[0] + 74 * math.cos(math.radians(-31))],
            [cam_from[1],
             cam_from[1] + 46 * math.sin(math.radians(-52)),
             cam_from[1] + 74 * math.sin(math.radians(-31))],
            color=GREY, alpha=0.18)
    ax.text(74, 6, "camera sees HERE\n(body frame, 45° down)",
            fontsize=9, color=INK)

    ax.plot([31, 31], [-26, -22], color=GREY, lw=1)
    ax.plot([84, 84], [-26, -22], color=GREY, lw=1)
    ax.annotate("", xy=(31, -24), xytext=(84, -24),
                arrowprops=dict(arrowstyle="<->", color=INK, lw=1.4))
    ax.text(57, -20, "ground band ahead — never below",
            fontsize=9, color=INK, ha="center")

    ax.set_title("goto uses the WORLD frame · the camera uses the BODY frame",
                 fontsize=12, color=INK, pad=10)
    save(fig, "frames_world_body.png")


# ── figure 4: how to check, instead of computing ─────────────────────────────
def fig_verify():
    fig, ax = plt.subplots(figsize=(8.2, 3.6))
    ax.set_xlim(0, 100); ax.set_ylim(0, 42); ax.axis("off")

    boxes = [
        (4,  "1. put the vehicle at a\nKNOWN Gazebo pose", ACCENT),
        (30, "2. ask IT where it is\n/uavN/gps", ACCENT),
        (56, "3. compare with\nSITL --home", ACCENT),
        (82, "4. the difference IS\nthe mapping", OK),
    ]
    for x, label, col in boxes:
        ax.add_patch(Rectangle((x, 16), 16, 14, fc="white", ec=col, lw=1.6))
        ax.text(x + 8, 23, label, fontsize=8.4, color=col,
                ha="center", va="center")
    for x in (20, 46, 72):
        arrow(ax, x, 23, x + 8, 23, GREY, 1.6)

    ax.text(50, 8,
            "spawned at Gazebo (−70, −22)   →   reported 70 m SOUTH, 22 m EAST\n"
            "so  north = gazebo_x   and   east = − gazebo_y",
            fontsize=9.5, color=WARN, ha="center", family="monospace")
    ax.set_title("Ask the vehicle. Do not derive it.",
                 fontsize=12, color=INK, pad=8)
    save(fig, "frames_verify.png")


if __name__ == "__main__":
    fig_two_frames()
    fig_the_error()
    fig_world_vs_body()
    fig_verify()
