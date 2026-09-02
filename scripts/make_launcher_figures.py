#!/usr/bin/env python3
"""Draw the figures for docs/LAUNCH_2UAV_MECHANISM.pdf.

Separate from the PDF builder so drawings can be tweaked on their own:

    python3 scripts/make_launcher_figures.py
    python3 scripts/make_launcher_pdf.py
"""
import os

import matplotlib
matplotlib.use("Agg")                      # no display needed
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

OUT_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       "docs", "figures")
os.makedirs(OUT_DIR, exist_ok=True)

INK    = "#16202c"
ACCENT = "#1f5f8b"
WARN   = "#a8402b"
OK     = "#1c6b4d"
GREY   = "#8a97a6"
LIGHT  = "#eef3f7"
CAMBG  = "#eaf4ef"
RFBG   = "#fdf0ec"

plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8.5})


def save(fig, name):
    path = os.path.join(OUT_DIR, name)
    fig.savefig(path, dpi=170, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print("wrote", path)


def box(ax, x, y, w, h, label, sub="", fc=LIGHT, ec=ACCENT, fs=8.5, lw=1.2):
    ax.add_patch(FancyBboxPatch((x, y), w, h,
                                boxstyle="round,pad=0.012,rounding_size=0.02",
                                fc=fc, ec=ec, lw=lw, zorder=2))
    ax.text(x + w / 2, y + h / 2 + (0.055 if sub else 0), label,
            ha="center", va="center", fontsize=fs, color=INK,
            fontweight="bold", zorder=3)
    if sub:
        ax.text(x + w / 2, y + h / 2 - 0.062, sub, ha="center", va="center",
                fontsize=fs - 1.4, color=GREY, zorder=3)


def arrow(ax, p, q, color=ACCENT, style="-|>", lw=1.4, ls="-"):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle=style, mutation_scale=11,
                                 color=color, lw=lw, linestyle=ls,
                                 shrinkA=2, shrinkB=2, zorder=4))


# ── FIG 1: the three worlds, and the two paths ──────────────────────────────
def fig_overview():
    fig, ax = plt.subplots(figsize=(8.6, 5.0))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")

    ax.text(0.5, 0.975, "What the launcher builds",
            ha="center", fontsize=11.5, fontweight="bold", color=INK)

    # root namespace band (top)
    ax.add_patch(FancyBboxPatch((0.02, 0.70), 0.96, 0.235,
                                boxstyle="round,pad=0.008,rounding_size=0.02",
                                fc="white", ec=GREY, lw=1.0, ls="--", zorder=1))
    ax.text(0.045, 0.905, "ROOT NAMESPACE  (the real machine)",
            fontsize=7.6, color=GREY, fontweight="bold")

    box(ax, 0.05, 0.735, 0.19, 0.125, "Gazebo", "physics + world")
    box(ax, 0.28, 0.735, 0.19, 0.125, "ns-3", "simulated Wi-Fi")
    box(ax, 0.51, 0.735, 0.22, 0.125, "world_pos_publisher",
        "feeds ns-3 positions", fs=7.4)
    box(ax, 0.77, 0.735, 0.19, 0.125, "Raspberry Pi", "edge detector",
        fc=CAMBG, ec=OK, fs=8)

    # namespace band (bottom)
    box(ax, 0.05, 0.215, 0.19, 0.150, "uav1ns", "SITL 1 · 10.42.0.11", fs=8.5)
    box(ax, 0.28, 0.215, 0.19, 0.150, "uav2ns", "SITL 2 · 10.42.0.13", fs=8.5)
    box(ax, 0.60, 0.215, 0.36, 0.150, "gcsns",
        "micro-ROS agents · drone_bridge · mission", fs=8.5)

    # ── links, drawn in the clear band between the two rows ──
    arrow(ax, (0.145, 0.365), (0.145, 0.735), GREY, ls=":")
    ax.text(0.155, 0.50, "FDM physics\ndirect veth\nNOT impaired",
            fontsize=6.8, color=GREY, va="center", ha="left")

    arrow(ax, (0.375, 0.365), (0.375, 0.735), ACCENT)
    arrow(ax, (0.70, 0.365), (0.42, 0.735), ACCENT)
    ax.text(0.655, 0.615, "all radio traffic\ncrosses ns-3",
            fontsize=7.2, color=ACCENT, ha="center", va="center")

    arrow(ax, (0.865, 0.735), (0.80, 0.365), OK, ls="--")
    ax.text(0.885, 0.50, "camera VLAN\nunimpaired",
            fontsize=6.8, color=OK, ha="center", va="center")

    ax.text(0.5, 0.115, "A namespace is a private network stack: its own interfaces, routes, ports "
                        "and loopback.\nThese three can only reach each other through ns-3.",
            ha="center", fontsize=7.8, color=INK)
    ax.text(0.5, 0.035, "That split is the whole point. The CAMERA link is a cable and must stay clean; "
                       "the RADIO link\nis the thing under study and must cross the simulated channel.",
            ha="center", fontsize=7.6, color=INK, style="italic")
    save(fig, "launcher_overview.png")


# ── FIG 2: one packet's journey ─────────────────────────────────────────────
def fig_packet_path():
    fig, ax = plt.subplots(figsize=(8.8, 2.5))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(0.5, 0.93, "How one telemetry packet reaches the ground station",
            ha="center", fontsize=10.5, fontweight="bold", color=INK)

    names = ["SITL\n(uav1ns)", "veth1n", "veth1h", "br-uav1", "tap-uav1",
             "ns-3\nWi-Fi", "tap-gcs", "br-gcs", "veth0n", "agent\n(gcsns)"]
    n = len(names)
    w, gap = 0.077, 0.0175
    x0 = (1 - (n * w + (n - 1) * gap)) / 2
    for i, nm in enumerate(names):
        x = x0 + i * (w + gap)
        hot = nm.startswith("ns-3")
        box(ax, x, 0.36, w, 0.30, nm, fc=(RFBG if hot else LIGHT),
            ec=(WARN if hot else ACCENT), fs=6.6, lw=1.3 if hot else 1.0)
        if i < n - 1:
            arrow(ax, (x + w, 0.51), (x + w + gap, 0.51), GREY, lw=1.0)

    ax.text(0.5, 0.20, "Only the shaded hop applies delay, loss and fading. Everything else is\n"
                       "plain Linux plumbing whose job is to force the packet through it.",
            ha="center", fontsize=7.8, color=INK)
    ax.text(0.5, 0.045, "Remove any one link and the two namespaces simply cannot talk — "
                        "which is what makes the measurement honest.",
            ha="center", fontsize=7.4, color=GREY, style="italic")
    save(fig, "launcher_packet_path.png")


# ── FIG 3: the node-id map ──────────────────────────────────────────────────
def fig_node_map():
    fig, ax = plt.subplots(figsize=(8.4, 3.0))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(0.5, 0.94, "ns-3 node ids: aircraft and companion computers interleave",
            ha="center", fontsize=10.5, fontweight="bold", color=INK)

    rows = [
        ("node 0", "GCS",   "tap-gcs",  "10.42.0.10", "gcsns",  LIGHT, ACCENT),
        ("node 1", "SITL 1", "tap-uav1", "10.42.0.11", "uav1ns", LIGHT, ACCENT),
        ("node 2", "Pi 1",  "tap-uav2", "10.42.0.12", "hardware", CAMBG, OK),
        ("node 3", "SITL 2", "tap-uav3", "10.42.0.13", "uav2ns", LIGHT, ACCENT),
        ("node 4", "Pi 2",  "tap-uav4", "10.42.0.14", "hardware", CAMBG, OK),
    ]
    hdr = ["", "role", "TAP", "address", "lives in"]
    xs = [0.05, 0.20, 0.38, 0.56, 0.76]
    for x, h in zip(xs, hdr):
        ax.text(x, 0.80, h, fontsize=7.6, color=GREY, fontweight="bold")

    for i, (nid, role, tap, ip, where, fc, ec) in enumerate(rows):
        y = 0.685 - i * 0.125
        box(ax, 0.035, y, 0.115, 0.093, nid, fc=fc, ec=ec, fs=8)
        ax.text(xs[1], y + 0.046, role, fontsize=8.2, va="center",
                color=INK, fontweight="bold")
        ax.text(xs[2], y + 0.046, tap, fontsize=8, va="center", color=INK,
                family="DejaVu Sans Mono")
        ax.text(xs[3], y + 0.046, ip, fontsize=8, va="center", color=INK,
                family="DejaVu Sans Mono")
        ax.text(xs[4], y + 0.046, where, fontsize=8, va="center", color=GREY)

    ax.text(0.5, 0.045,
            "A companion computer sits on the SAME airframe as the aircraft above it,\n"
            "so node 2 mirrors node 1's position and node 4 mirrors node 3's.",
            ha="center", fontsize=7.8, color=INK, style="italic")
    save(fig, "launcher_node_map.png")


# ── FIG 4: the startup ladder ───────────────────────────────────────────────
def fig_stages():
    fig, ax = plt.subplots(figsize=(8.4, 6.0))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(0.5, 0.975, "Startup order — and why it cannot be shuffled",
            ha="center", fontsize=11, fontweight="bold", color=INK)

    stages = [
        ("0",   "Cleanup",            "kill stale processes, delete old namespaces"),
        ("1",   "Namespaces",         "gcsns, uav1ns, uav2ns + veth/bridge/TAP"),
        ("1c",  "Pi VLANs",           "camera VLAN 10 clean, radio VLAN 42/43 to ns-3"),
        ("2",   "Management link",    "SITL <-> Gazebo physics, bypasses ns-3"),
        ("3",   "ns-3",               "must own the TAPs before anything sends"),
        ("4",   "Gazebo",             "physics + FDM ports 9002/9012"),
        ("4b",  "Position publisher", "so ns-3 nodes move with the drones"),
        ("5",   "micro-ROS agents",   "must exist before SITL's DDS client dials in"),
        ("6",   "SITL x2",            "then settle 15 s for the AP_DDS handshake"),
        ("7",   "Readiness gates",    "REAL navsat messages on both aircraft"),
        ("8",   "drone_bridge x2",    "unique node names — see the warning box"),
    ]
    y = 0.925
    for num, title, why in stages:
        gate = num in ("3", "5", "7")
        box(ax, 0.045, y - 0.052, 0.072, 0.052, num,
            fc=(RFBG if gate else LIGHT), ec=(WARN if gate else ACCENT), fs=8)
        ax.text(0.135, y - 0.026, title, fontsize=8.8, va="center",
                color=INK, fontweight="bold")
        ax.text(0.375, y - 0.026, why, fontsize=8, va="center", color=GREY)
        if y > 0.28:
            arrow(ax, (0.081, y - 0.052), (0.081, y - 0.070), GREY, lw=0.9)
        y -= 0.070

    ax.text(0.5, 0.055,
            "Shaded steps are ordering constraints, not preferences: ns-3 must hold the TAPs\n"
            "before traffic flows, the agents must be listening before SITL dials in, and the\n"
            "gates must pass before a mission is allowed to arm anything.",
            ha="center", fontsize=7.8, color=INK)
    save(fig, "launcher_stages.png")


# ── FIG 5: the three data paths in flight ───────────────────────────────────
def fig_dataflow():
    fig, ax = plt.subplots(figsize=(8.6, 3.9))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.text(0.5, 0.955, "Three separate conversations happen at once",
            ha="center", fontsize=10.5, fontweight="bold", color=INK)

    box(ax, 0.02, 0.50, 0.17, 0.155, "mission", "in gcsns", fs=8)
    box(ax, 0.255, 0.50, 0.19, 0.155, "drone_bridge", "in gcsns", fs=8)
    box(ax, 0.545, 0.50, 0.17, 0.155, "SITL", "in uavNns", fs=8)
    box(ax, 0.80, 0.50, 0.17, 0.155, "Gazebo", "root ns", fs=8)

    # mission -> bridge
    arrow(ax, (0.19, 0.578), (0.255, 0.578), ACCENT)
    ax.text(0.2225, 0.700, "ROS 2", fontsize=6.8, color=ACCENT, ha="center")

    # bridge -> SITL (commands) and SITL -> bridge (telemetry)
    arrow(ax, (0.445, 0.605), (0.545, 0.605), WARN)
    ax.text(0.495, 0.700, "MAVLink TCP", fontsize=6.8, color=WARN, ha="center")
    arrow(ax, (0.545, 0.545), (0.445, 0.545), OK)
    ax.text(0.495, 0.432, "DDS navsat", fontsize=6.8, color=OK, ha="center")

    # SITL <-> Gazebo physics
    arrow(ax, (0.715, 0.578), (0.80, 0.578), GREY, ls=":")
    ax.text(0.7575, 0.700, "FDM", fontsize=6.8, color=GREY, ha="center")

    legend = [
        (WARN, "commands out (arm, takeoff, goto) travel MAVLink over TCP - through ns-3"),
        (OK,   "telemetry back (position, battery) travels DDS - also through ns-3"),
        (GREY, "flight physics uses a direct veth and NEVER touches ns-3"),
    ]
    for i, (c, t) in enumerate(legend):
        yy = 0.315 - i * 0.082
        ax.add_patch(FancyBboxPatch((0.055, yy), 0.020, 0.038,
                                    boxstyle="round,pad=0.004", fc=c, ec=c))
        ax.text(0.092, yy + 0.019, t, fontsize=7.8, va="center", color=INK)

    ax.text(0.5, 0.035,
            "Physics is deliberately exempt: impairing it would change how the aircraft FLIES,\n"
            "when the experiment is about how it COMMUNICATES.",
            ha="center", fontsize=7.6, color=INK, style="italic")
    save(fig, "launcher_dataflow.png")


if __name__ == "__main__":
    fig_overview()
    fig_packet_path()
    fig_node_map()
    fig_stages()
    fig_dataflow()
