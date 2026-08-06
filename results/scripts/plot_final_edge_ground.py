#!/usr/bin/env python3
"""Generate publication-ready figures from definitive processed CSV files."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


REPO = Path(__file__).resolve().parents[2]
PROCESSED = REPO / 'results/final_edge_ground_comparison_primary/processed'
FIGURES = PROCESSED / 'figures'
RUN_SUMMARY = PROCESSED / 'run_summary.csv'
MODE_SUMMARY = PROCESSED / 'mode_summary.csv'
PAIRED_COMPARISON = PROCESSED / 'paired_comparison.csv'

EDGE_COLOR = '#2864a5'
GROUND_COLOR = '#d97706'
EDGE_STYLE = {'color': EDGE_COLOR, 'edgecolor': 'black', 'linewidth': 0.6}
GROUND_STYLE = {
    'color': GROUND_COLOR, 'edgecolor': 'black', 'linewidth': 0.6,
    'hatch': '//',
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def save(fig: plt.Figure, stem: str) -> None:
    fig.tight_layout()
    fig.savefig(FIGURES / f'{stem}.png', dpi=600, bbox_inches='tight')
    fig.savefig(FIGURES / f'{stem}.pdf', bbox_inches='tight')
    plt.close(fig)


def finish_axis(axis: plt.Axes, ylabel: str, title: str,
                top: float | None = None) -> None:
    axis.set_xlabel('NS-3 RNG run')
    axis.set_ylabel(ylabel)
    axis.set_title(title, pad=34)
    axis.set_ylim(bottom=0, top=top)
    axis.grid(axis='y', color='#b8b8b8', alpha=0.45, linewidth=0.6)
    axis.set_axisbelow(True)
    axis.legend(frameon=False, ncol=2, loc='upper center',
                bbox_to_anchor=(0.5, 1.12))


def mode_run(rows: list[dict[str, str]], rng: str, mode: str) -> dict[str, str]:
    return next(row for row in rows
                if row['rng_run'] == rng and row['actual_mode'] == mode)


def grouped(rows: list[dict[str, str]], field: str, stem: str, ylabel: str,
            title: str, percentage: bool = False) -> None:
    rngs = ('1', '2', '3')
    edge = [float(mode_run(rows, rng, 'edge')[field]) for rng in rngs]
    ground = [float(mode_run(rows, rng, 'ground')[field]) for rng in rngs]
    positions = np.arange(len(rngs))
    width = 0.36
    fig, axis = plt.subplots(figsize=(7.2, 4.8))
    edge_bars = axis.bar(positions - width / 2, edge, width,
                         label='Edge', **EDGE_STYLE)
    ground_bars = axis.bar(positions + width / 2, ground, width,
                           label='Ground', **GROUND_STYLE)
    axis.set_xticks(positions, [f'RNG {rng}' for rng in rngs])
    finish_axis(axis, ylabel, title, 1.05 if percentage else None)
    if percentage:
        axis.bar_label(edge_bars, labels=[f'{item:.0%}' for item in edge],
                       padding=3, fontsize=8.5)
        axis.bar_label(ground_bars, labels=[f'{item:.2%}' for item in ground],
                       padding=3, fontsize=8.5)
    else:
        axis.bar_label(edge_bars, fmt='%.1f', padding=3, fontsize=8.5)
        axis.bar_label(ground_bars, fmt='%.1f', padding=3, fontsize=8.5)
        axis.margins(y=0.14)
    save(fig, stem)


def ground_overhead(rows: list[dict[str, str]]) -> None:
    rngs = ('1', '2', '3')
    compression = [float(mode_run(rows, rng, 'ground')['mean_compression_ms'])
                   for rng in rngs]
    decode = [float(mode_run(rows, rng, 'ground')['mean_decode_ms'])
              for rng in rngs]
    positions = np.arange(len(rngs))
    width = 0.36
    fig, axis = plt.subplots(figsize=(7.2, 4.8))
    compression_bars = axis.bar(
        positions - width / 2, compression, width, label='JPEG compression',
        color='#6b46c1', edgecolor='black', linewidth=0.6)
    decode_bars = axis.bar(
        positions + width / 2, decode, width, label='JPEG decoding',
        color='#2f855a', edgecolor='black', linewidth=0.6, hatch='//')
    axis.set_xticks(positions, [f'RNG {rng}' for rng in rngs])
    finish_axis(axis, 'Mean time (ms)',
                'Ground JPEG Compression and Decoding Overhead')
    axis.bar_label(compression_bars, fmt='%.2f', padding=3, fontsize=8.5)
    axis.bar_label(decode_bars, fmt='%.2f', padding=3, fontsize=8.5)
    totals = [left + right for left, right in zip(compression, decode)]
    for position, total in zip(positions, totals):
        axis.text(position, max(compression[position], decode[position]) + 0.35,
                  f'total {total:.2f}', ha='center', va='bottom', fontsize=8)
    axis.margins(y=0.2)
    save(fig, 'ground_compression_decode_overhead_by_rng')


def paired_latency(rows: list[dict[str, str]]) -> None:
    fig, axis = plt.subplots(figsize=(6.4, 4.8))
    colors = ('#4c78a8', '#59a14f', '#b279a2')
    for row, color in zip(
            sorted(rows, key=lambda item: int(item['rng_run'])), colors):
        values = [float(row['edge_median_latency_ms']),
                  float(row['ground_median_latency_ms'])]
        axis.plot((0, 1), values, marker='o', markersize=6, linewidth=1.8,
                  color=color, label=f'RNG {row["rng_run"]}')
    axis.set_xticks((0, 1), ('Edge', 'Ground'))
    axis.set_ylabel('Official-window median latency (ms)')
    axis.set_title('Paired Median-Latency Comparison', pad=34)
    axis.set_ylim(bottom=0)
    axis.grid(axis='y', color='#b8b8b8', alpha=0.45, linewidth=0.6)
    axis.set_axisbelow(True)
    axis.legend(frameon=False, ncol=3, loc='upper center',
                bbox_to_anchor=(0.5, 1.12))
    axis.margins(y=0.15)
    save(fig, 'paired_median_latency_comparison')


def summary_value(rows: list[dict[str, str]], mode: str,
                  metric: str) -> dict[str, str]:
    return next(row for row in rows
                if row['mode'] == mode and row['metric'] == metric)


def mode_summary(mode_rows: list[dict[str, str]], run_rows: list[dict[str, str]],
                 metric: str, run_field: str, stem: str, ylabel: str,
                 title: str, percentage: bool = False) -> None:
    modes = ('edge', 'ground')
    means = [float(summary_value(mode_rows, mode, metric)['mean']) for mode in modes]
    fig, axis = plt.subplots(figsize=(6.4, 4.8))
    bars = axis.bar((0, 1), means, width=0.55,
                    color=(EDGE_COLOR, GROUND_COLOR), edgecolor='black',
                    linewidth=0.6)
    bars[1].set_hatch('//')
    jitter = (-0.08, 0.0, 0.08)
    for x_value, mode in enumerate(modes):
        values = [float(row[run_field]) for row in run_rows
                  if row['actual_mode'] == mode]
        axis.scatter([x_value + offset for offset in jitter], values,
                     color='black', s=24, zorder=3, label=None)
    axis.set_xticks((0, 1), ('Edge', 'Ground'))
    axis.set_ylabel(ylabel)
    axis.set_title(title, pad=12)
    axis.set_ylim(bottom=0, top=1.05 if percentage else None)
    axis.grid(axis='y', color='#b8b8b8', alpha=0.45, linewidth=0.6)
    axis.set_axisbelow(True)
    if percentage:
        axis.bar_label(bars, labels=[f'{item:.2%}' for item in means],
                       padding=3, fontsize=9)
    else:
        axis.bar_label(bars, fmt='%.1f', padding=3, fontsize=9)
        axis.margins(y=0.15)
    save(fig, stem)


def write_index() -> None:
    entries = [
        ('processed_frame_ratio_by_rng', 'Official processed-frame ratio by RNG',
         'run_summary.csv', 'Reliability: Edge local completion versus Ground complete compressed-frame processing.'),
        ('median_pipeline_latency_by_rng', 'Official-window median pipeline latency',
         'run_summary.csv', 'Latency comparison'),
        ('p95_pipeline_latency_by_rng', 'Official-window p95 pipeline latency',
         'run_summary.csv', 'Tail-latency comparison'),
        ('mean_inference_time_by_rng', 'Mean YOLO inference time',
         'run_summary.csv', 'Inference performance'),
        ('mean_cpu_utilization_by_rng', 'Mean monitored process CPU on the simulation host',
         'run_summary.csv', 'Host resource demand'),
        ('mean_rss_memory_by_rng', 'Mean monitored process RSS on the simulation host',
         'run_summary.csv', 'Host resource demand'),
        ('ground_compression_decode_overhead_by_rng', 'Ground JPEG compression and decoding overhead',
         'run_summary.csv', 'Image-processing overhead'),
        ('paired_median_latency_comparison', 'Within-RNG Edge-to-Ground median latency changes',
         'paired_comparison.csv', 'Paired latency comparison'),
        ('mode_processed_ratio_summary', 'Mode means with three run-level ratio values',
         'mode_summary.csv + run_summary.csv', 'Mode-level reliability summary'),
        ('mode_median_latency_summary', 'Mode means with three run-level median latencies',
         'mode_summary.csv + run_summary.csv', 'Mode-level latency summary'),
        ('d4_detection_accuracy_comparison', 'Precision, recall, F1, and exact-count rate for Edge/raw and Ground/JPEG q5',
         'phase_d_application processed D4 summary', 'Detection accuracy under compression'),
    ]
    lines = [
        '# Phase F figure index', '',
        'Every figure is available as a 600-DPI PNG and a vector PDF. '
        'Points in mode-level figures are complete-run values, not frame-level replicates.', '',
        '| Filename stem | Metric shown | Source CSV | Suggested report section / interpretation |',
        '| --- | --- | --- | --- |',
    ]
    lines.extend(f'| `{stem}` | {metric} | `{source}` | {interpretation} |'
                 for stem, metric, source, interpretation in entries)
    (FIGURES / 'README.md').write_text('\n'.join(lines) + '\n')


def main() -> None:
    FIGURES.mkdir(parents=True, exist_ok=True)
    run_rows = read_csv(RUN_SUMMARY)
    mode_rows = read_csv(MODE_SUMMARY)
    paired_rows = read_csv(PAIRED_COMPARISON)
    if len(run_rows) != 6 or len(paired_rows) != 3:
        raise ValueError('Expected six selected runs and three paired RNG rows')
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 10,
        'axes.labelsize': 11, 'axes.titlesize': 12,
        'xtick.labelsize': 10, 'ytick.labelsize': 10,
        'legend.fontsize': 9, 'pdf.fonttype': 42, 'ps.fonttype': 42,
    })
    grouped(run_rows, 'processed_frame_ratio',
            'processed_frame_ratio_by_rng', 'Processed-frame ratio',
            'Processed-Frame Completion by RNG Run\n'
            'Edge: local pipeline; Ground: complete compressed-frame processing',
            percentage=True)
    grouped(run_rows, 'median_pipeline_latency_ms',
            'median_pipeline_latency_by_rng', 'Median pipeline latency (ms)',
            'Median End-to-End Pipeline Latency')
    grouped(run_rows, 'p95_pipeline_latency_ms',
            'p95_pipeline_latency_by_rng', 'P95 pipeline latency (ms)',
            '95th-Percentile Pipeline Latency')
    grouped(run_rows, 'mean_inference_ms', 'mean_inference_time_by_rng',
            'Mean inference time (ms)', 'Mean YOLO Inference Time')
    grouped(run_rows, 'resource_cpu_mean_percent',
            'mean_cpu_utilization_by_rng', 'Mean CPU utilization (%)',
            'Mean Process CPU Utilization on Simulation Host')
    grouped(run_rows, 'resource_rss_mean_mb', 'mean_rss_memory_by_rng',
            'Mean RSS memory (MB)',
            'Mean Process Memory Usage on Simulation Host')
    ground_overhead(run_rows)
    paired_latency(paired_rows)
    mode_summary(mode_rows, run_rows, 'processed_frame_ratio',
                 'processed_frame_ratio', 'mode_processed_ratio_summary',
                 'Processed-frame ratio',
                 'Mode-Level Processed-Frame Completion', percentage=True)
    mode_summary(mode_rows, run_rows, 'median_latency_ms',
                 'median_pipeline_latency_ms', 'mode_median_latency_summary',
                 'Median pipeline latency (ms)',
                 'Mode-Level Median Pipeline Latency')
    write_index()
    print(f'Generated 10 PNG/PDF figure pairs in {FIGURES.relative_to(REPO)}')


if __name__ == '__main__':
    main()
