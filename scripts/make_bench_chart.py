#!/usr/bin/env python3
"""Bar chart of measured inference time per back-end on the Pi 4B.

Figures come from the bench_backends.py run recorded in docs/PI_SETUP.md
(median of 10 calls, correctness checked before speed).

    python3 scripts/make_bench_chart.py

Output: report/inference_backends.png  (white background, 1600x1100 @200dpi)
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "inference_backends.png"

# docs/PI_SETUP.md — "Faster inference" benchmark table
BACKENDS = [
    ("OpenVINO", "YOLO11n",  236, "4.35x"),
    ("NCNN",     "YOLOv8n",  270, "3.80x"),
    ("MNN",      "YOLO11n",  342, "3.00x"),
    ("PyTorch",  "YOLOv8n", 1027, "1.00x"),
]
DEPLOYED = "OpenVINO"

# Validated pair on a white surface: OKLab dE 18.1, worst-CVD dE 14.4,
# contrast 5.91:1 and 3.07:1. A mid-slate neutral fails CVD (dE 1.6).
ACCENT, NEUTRAL = "#0B6E7F", "#8A94A1"
INK, MUTED, FAINT, GRID = "#141C26", "#5B6775", "#8A94A1", "#E6EAF0"


def main():
    names = [b[0] for b in BACKENDS]
    values = [b[2] for b in BACKENDS]
    colors = [ACCENT if b[0] == DEPLOYED else NEUTRAL for b in BACKENDS]

    fig, ax = plt.subplots(figsize=(7.4, 4.9), dpi=150)
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    bars = ax.bar(names, values, color=colors, width=0.62, zorder=3)

    # Value + speedup above each bar; 4 bars, so labelling all is appropriate.
    for bar, (name, model, ms, mult) in zip(bars, BACKENDS):
        x = bar.get_x() + bar.get_width() / 2
        tag = f"{mult} · deployed" if name == DEPLOYED else mult
        ax.annotate(f"{ms:,} ms", (x, ms), textcoords="offset points",
                    xytext=(0, 20), ha="center", va="bottom",
                    fontsize=10.5, fontweight="bold", color=INK)
        ax.annotate(tag, (x, ms), textcoords="offset points",
                    xytext=(0, 6), ha="center", va="bottom",
                    fontsize=8, color=MUTED)

    # Model name under each category, so the mixed-model caveat is visible.
    ax.set_xticks(range(len(BACKENDS)))
    ax.set_xticklabels([f"{n}\n{m}" for n, m, _, _ in
                        [(b[0], b[1], b[2], b[3]) for b in BACKENDS]],
                       fontsize=10, color=INK)
    for lbl in ax.get_xticklabels():
        lbl.set_linespacing(1.6)

    ax.set_ylim(0, 1220)
    ax.set_yticks([0, 250, 500, 750, 1000])
    ax.set_ylabel("median inference time (ms)", fontsize=9.5, color=MUTED, labelpad=8)
    ax.tick_params(axis="y", labelsize=9, colors=FAINT, length=0)
    ax.tick_params(axis="x", length=0, pad=8)

    ax.yaxis.grid(True, color=GRID, linewidth=1, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right", "left"):
        ax.spines[side].set_visible(False)
    ax.spines["bottom"].set_color("#C9D1DA")

    ax.set_title("Inference back-end comparison — Raspberry Pi 4B",
                 fontsize=12.5, fontweight="bold", color=INK, pad=30, loc="left")
    ax.text(0, 1.018, "Median of 10 calls, lower is better. Teal = deployed on both edge nodes.",
            transform=ax.transAxes, fontsize=8.5, color=MUTED)

    fig.text(0.055, 0.015,
             "Source: docs/PI_SETUP.md · bench_backends.py · Cortex-A72, FP32. "
             "OpenVINO/MNN measured on YOLO11n, NCNN/PyTorch on YOLOv8n.",
             fontsize=6.8, color=FAINT)

    fig.tight_layout(rect=[0, 0.035, 1, 1])
    OUT.parent.mkdir(exist_ok=True)
    fig.savefig(OUT, facecolor="white", edgecolor="none")
    print(f"  {OUT.relative_to(ROOT)}  "
          f"{OUT.stat().st_size // 1024} KB  {fig.get_size_inches()[0]*200:.0f}"
          f"x{fig.get_size_inches()[1]*200:.0f} px")


if __name__ == "__main__":
    main()
