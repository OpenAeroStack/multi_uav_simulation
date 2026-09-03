#!/usr/bin/env python3
"""Measured inference time per back-end on the Pi 4B.

Figures come from the bench_backends.py run recorded in docs/PI_SETUP.md
(median of 10 calls, correctness checked before speed).

    python3 scripts/make_bench_chart.py

Output: report/inference_backends.png  (white background)
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "inference_backends.png"

# docs/PI_SETUP.md — "Faster inference" benchmark table
BACKENDS = [
    ("OpenVINO\n(deployed)", 236),
    ("NCNN",                 270),
    ("MNN",                  342),
    ("PyTorch",             1027),
]
DEPLOYED = 0

# Validated pair on a white surface: OKLab dE 18.1, worst-CVD dE 14.4,
# contrast 5.91:1 and 3.07:1. A mid-slate neutral fails CVD (dE 1.6).
ACCENT, NEUTRAL = "#0B6E7F", "#8A94A1"
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def main():
    names = [b[0] for b in BACKENDS]
    values = [b[1] for b in BACKENDS]
    colors = [ACCENT if i == DEPLOYED else NEUTRAL for i in range(len(BACKENDS))]

    fig, ax = plt.subplots(figsize=(7.0, 4.6), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    bars = ax.bar(names, values, color=colors, width=0.6, zorder=3)

    for bar, ms in zip(bars, values):
        ax.annotate(f"{ms:,}", (bar.get_x() + bar.get_width() / 2, ms),
                    textcoords="offset points", xytext=(0, 7), ha="center",
                    va="bottom", fontsize=11, fontweight="bold", color=INK)

    ax.set_ylim(0, 1180)
    ax.set_yticks([0, 250, 500, 750, 1000])
    ax.set_ylabel("median inference time (ms)", fontsize=10.5, color=INK, labelpad=9)
    ax.tick_params(axis="y", labelsize=9.5, colors=MUTED, length=0)
    ax.tick_params(axis="x", labelsize=10, colors=INK, length=0, pad=7)
    for lbl in ax.get_xticklabels():
        lbl.set_linespacing(1.5)

    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    fig.text(0.055, 0.012,
             "Source: docs/PI_SETUP.md · bench_backends.py, median of 10 · Pi 4B "
             "Cortex-A72, FP32\nOpenVINO and MNN measured on YOLO11n, NCNN and PyTorch "
             "on YOLOv8n.",
             fontsize=7, color=FAINT, linespacing=1.6)

    fig.tight_layout(rect=[0, 0.075, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB")


if __name__ == "__main__":
    main()
