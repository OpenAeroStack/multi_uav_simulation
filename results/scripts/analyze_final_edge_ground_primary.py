#!/usr/bin/env python3
"""Build the definitive primary Edge-versus-Ground analysis."""

from __future__ import annotations

import csv
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Optional

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


REPO = Path(__file__).resolve().parents[2]
FINAL = REPO / 'results/final_edge_ground_comparison_primary'
PROCESSED = FINAL / 'processed'
FIGURES = PROCESSED / 'figures'
SELECTED = FINAL / 'selected_runs.csv'
D4_SUMMARY = (REPO / 'results/phase_d_application/processed/'
              'd4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv')

RUN_FIELDS = [
    'session', 'rng_run', 'actual_mode', 'run_id', 'official_sent_frames',
    'official_processed_frames', 'processed_frame_ratio',
    'startup_rows_excluded', 'official_analysis_rows', 'late_rows_excluded',
    'mean_detection_count', 'minimum_detection_count', 'maximum_detection_count',
    'mean_pipeline_latency_ms', 'median_pipeline_latency_ms',
    'p95_pipeline_latency_ms', 'minimum_pipeline_latency_ms',
    'maximum_pipeline_latency_ms', 'latency_rows_above_500_ms',
    'latency_rows_above_1000_ms', 'latency_rows_above_2000_ms',
    'mean_inference_ms', 'mean_compression_ms', 'mean_decode_ms',
    'timestamp_span_s', 'mean_interarrival_s',
    'resource_cpu_mean_percent', 'resource_cpu_peak_percent',
    'resource_rss_mean_mb', 'resource_rss_peak_mb',
    'gps_horizontal_drift_m', 'delivery_observations',
    'verification_status', 'validation_warnings',
]

LATENCY_FIELDS = [
    'session', 'rng_run', 'actual_mode', 'run_id', 'official_analysis_rows',
    'latency_rows_above_500_ms', 'latency_rows_above_1000_ms',
    'latency_rows_above_2000_ms', 'minimum_pipeline_latency_ms',
    'maximum_pipeline_latency_ms', 'validation_warnings',
]

MODE_METRICS = {
    'processed_frame_ratio': 'processed_frame_ratio',
    'mean_latency_ms': 'mean_pipeline_latency_ms',
    'median_latency_ms': 'median_pipeline_latency_ms',
    'p95_latency_ms': 'p95_pipeline_latency_ms',
    'mean_inference_ms': 'mean_inference_ms',
    'mean_detection_count': 'mean_detection_count',
    'cpu_mean_percent': 'resource_cpu_mean_percent',
    'cpu_peak_percent': 'resource_cpu_peak_percent',
    'rss_mean_mb': 'resource_rss_mean_mb',
    'rss_peak_mb': 'resource_rss_peak_mb',
    'mean_compression_ms': 'mean_compression_ms',
    'mean_decode_ms': 'mean_decode_ms',
}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fields: list[str], rows: Iterable[dict]) -> None:
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def artifact(relative: str) -> Path:
    path = FINAL / relative
    if not path.is_file():
        raise FileNotFoundError(f'Missing selected artifact: {relative}')
    return path


def value(text: str, key: str) -> Optional[str]:
    match = re.search(
        rf'^\s*{re.escape(key)}\s*(?::|=)\s*(.+)$', text, re.MULTILINE)
    return match.group(1).strip() if match else None


def integer(text: str, key: str) -> Optional[int]:
    raw = value(text, key)
    match = re.search(r'-?\d+', raw) if raw else None
    return int(match.group()) if match else None


def floating(text: str, key: str) -> Optional[float]:
    raw = value(text, key)
    match = re.search(r'-?(?:\d+(?:\.\d*)?|\.\d+)', raw) if raw else None
    return float(match.group()) if match else None


def parse_gps(text: str, heading: str) -> tuple[Optional[float], Optional[float]]:
    start = text.find(heading)
    if start < 0:
        return None, None
    block = text[start:]
    boundary = block.find('\nGPS message ', 1)
    if boundary >= 0:
        block = block[:boundary]
    lat = re.search(r'^latitude:\s*([-+0-9.eE]+)', block, re.MULTILINE)
    lon = re.search(r'^longitude:\s*([-+0-9.eE]+)', block, re.MULTILINE)
    return ((float(lat.group(1)) if lat else None),
            (float(lon.group(1)) if lon else None))


def haversine_m(a_lat: float, a_lon: float, b_lat: float, b_lon: float) -> float:
    radius = 6_371_000.0
    a_phi, b_phi = math.radians(a_lat), math.radians(b_lat)
    d_phi = math.radians(b_lat - a_lat)
    d_lon = math.radians(b_lon - a_lon)
    term = (math.sin(d_phi / 2) ** 2
            + math.cos(a_phi) * math.cos(b_phi) * math.sin(d_lon / 2) ** 2)
    return 2 * radius * math.asin(math.sqrt(term))


def numbers(rows: list[dict[str, str]], field: str,
            allow_negative: bool = False) -> list[float]:
    output = []
    for row in rows:
        try:
            number = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(number) and (allow_negative or number >= 0):
            output.append(number)
    return output


def percentile(items: list[float], fraction: float) -> Optional[float]:
    if not items:
        return None
    ordered = sorted(items)
    position = (len(ordered) - 1) * fraction
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def fmt(number: Optional[float], digits: int = 4) -> str:
    return '' if number is None else f'{number:.{digits}f}'


def resource_summary(path: Path) -> tuple[Optional[float], ...]:
    samples: dict[str, list[tuple[float, float]]] = defaultdict(list)
    with path.open() as handle:
        for line in handle:
            fields = line.split()
            if len(fields) < 16 or fields[1] not in ('AM', 'PM'):
                continue
            try:
                cpu, rss_kb = float(fields[8]), float(fields[13])
            except (ValueError, IndexError):
                continue
            samples[f'{fields[0]} {fields[1]}'].append((cpu, rss_kb))
    if not samples:
        return None, None, None, None
    cpu = [sum(item[0] for item in sample) for sample in samples.values()]
    rss = [sum(item[1] for item in sample) / 1024 for sample in samples.values()]
    return statistics.fmean(cpu), max(cpu), statistics.fmean(rss), max(rss)


def logged_mode(path: Path, source: str) -> Optional[str]:
    text = path.read_text(errors='replace')
    if source in ('detector', 'relay'):
        match = re.search(r'\b(EDGE|GROUND) mode\b', text)
        return match.group(1).lower() if match else None
    match = re.search(r'\bmode=(edge|ground)\b', text)
    return match.group(1) if match else None


def verify_configuration(registry: dict[str, str], metadata: str,
                         directory: Path) -> list[str]:
    expected = registry['actual_mode']
    observed = [logged_mode(directory / 'detector.log', 'detector'),
                logged_mode(directory / 'camera_relay.log', 'relay'),
                logged_mode(directory / 'metrics_logger.log', 'metrics')]
    failures = []
    if any(mode != expected for mode in observed):
        failures.append('mode_evidence_disagrees')
    checks = [
        (value(metadata, 'Run ID') == registry['run_id'], 'run_id_mismatch'),
        (value(metadata, 'Mode') == expected, 'metadata_mode_mismatch'),
        (integer(metadata, 'Expected NS-3 RNG run') == int(registry['rng_run']),
         'rng_mismatch'),
        (integer(metadata, 'Measurement duration') == 60, 'duration_mismatch'),
        (floating(metadata, 'Frame rate') == 1.0, 'frame_rate_mismatch'),
        (floating(metadata, 'Confidence threshold') == 0.25,
         'confidence_mismatch'),
        (integer(metadata, 'minimum_warmup_rows') == 9, 'warmup_mismatch'),
        (integer(metadata, 'post_warmup_settle_s') == 15, 'settling_mismatch'),
        (integer(metadata, 'stable_rows_required') == 5, 'stable_rows_mismatch'),
        (value(metadata, 'stable_interval_range_s') == '0.70-1.30',
         'stable_interval_mismatch'),
        (floating(metadata, 'Fixed target latitude') == 6.079430,
         'latitude_mismatch'),
        (floating(metadata, 'Fixed target longitude') == 80.193085,
         'longitude_mismatch'),
        (floating(metadata, 'Fixed target altitude') == 25.0,
         'altitude_mismatch'),
    ]
    for passed, label in checks:
        if not passed:
            failures.append(label)
    jpeg = value(metadata, 'Ground JPEG quality')
    if expected == 'ground' and jpeg != '5':
        failures.append('ground_jpeg_not_5')
    if expected == 'edge' and jpeg != 'n/a':
        failures.append('edge_jpeg_not_na')
    relay = (directory / 'camera_relay.log').read_text(errors='replace')
    if expected == 'ground' and 'JPEG quality=5' not in relay:
        failures.append('ground_relay_jpeg_evidence_missing')
    if None in parse_gps(metadata, 'GPS message before the run:'):
        failures.append('gps_before_missing')
    if None in parse_gps(metadata, 'GPS message after the run:'):
        failures.append('gps_after_missing')
    return failures


def analyze_run(registry: dict[str, str]) -> dict[str, str]:
    directory = FINAL / f'raw_links/rng_{registry["rng_run"]}/{registry["actual_mode"]}'
    for name in ('metrics.csv', 'metadata.txt', 'detector.log',
                 'metrics_logger.log', 'camera_relay.log', 'resources.txt'):
        if not (directory / name).is_file():
            raise FileNotFoundError(f'{registry["run_id"]}: missing {name}')
    metadata = (directory / 'metadata.txt').read_text()
    failures = verify_configuration(registry, metadata, directory)
    start = integer(metadata, 'official_start_csv_row')
    end = integer(metadata, 'official_end_csv_row')
    startup = integer(metadata, 'startup_rows_excluded')
    sent = integer(metadata, 'official measurement frames sent')
    processed = integer(metadata, 'official measurement frames processed')
    if None in (start, end, startup, sent, processed):
        raise ValueError(f'{registry["run_id"]}: incomplete official-window metadata')
    if start != int(registry['official_start_csv_row']) or end != int(
            registry['official_end_csv_row']):
        failures.append('manifest_boundary_mismatch')

    all_rows = read_csv(directory / 'metrics.csv')
    warnings = []
    if end > len(all_rows):
        failures.append('official_end_exceeds_csv_length')
    official = all_rows[start:min(end, len(all_rows))]
    if len(official) != processed:
        failures.append('official_analysis_rows_differ_from_metadata')
    late = max(0, len(all_rows) - end)
    detections = numbers(official, 'detection_count')
    latency = numbers(official, 'pipeline_latency_ms')
    inference = numbers(official, 'inference_ms')
    compression = numbers(official, 'compression_ms')
    decode = numbers(official, 'decode_ms')
    timestamps = numbers(official, 'timestamp_s', allow_negative=True)
    frames = numbers(official, 'frame_num')
    required_numeric = (
        'timestamp_s', 'frame_num', 'detection_count', 'pipeline_latency_ms',
        'inference_ms', 'compression_ms', 'decode_ms')
    if any(len(numbers(official, field, allow_negative=True)) != len(official)
           for field in required_numeric):
        failures.append('malformed_official_rows')
    intervals = [later - earlier for earlier, later in zip(timestamps, timestamps[1:])]
    if any(later <= earlier for earlier, later in zip(timestamps, timestamps[1:])):
        failures.append('official_timestamp_order_non_monotonic')
    if len(frames) != len(set(frames)):
        failures.append('duplicate_frame_numbers')
    mean_interval = statistics.fmean(intervals) if intervals else None
    if (registry['actual_mode'] == 'edge' and mean_interval is not None
            and not 0.8 <= mean_interval <= 1.2):
        warnings.append('edge_interarrival_outside_expected_range')
    if any(item > 2000 for item in latency):
        warnings.append('latency_exceeds_2000ms')
    observations = []
    if registry['actual_mode'] == 'ground' and processed < sent:
        observations.append('complete_frame_gaps_observed')
        if mean_interval is not None and mean_interval > 1.2:
            observations.append('mean_interarrival_above_nominal')
    before = parse_gps(metadata, 'GPS message before the run:')
    after = parse_gps(metadata, 'GPS message after the run:')
    drift = (haversine_m(before[0], before[1], after[0], after[1])
             if None not in before + after else None)
    cpu_mean, cpu_peak, rss_mean, rss_peak = resource_summary(
        directory / 'resources.txt')
    if cpu_mean is None:
        failures.append('resource_samples_missing')
    status = 'verified' if not failures else 'verification_failed'
    return {
        'session': registry['session'], 'rng_run': registry['rng_run'],
        'actual_mode': registry['actual_mode'], 'run_id': registry['run_id'],
        'official_sent_frames': str(sent),
        'official_processed_frames': str(processed),
        'processed_frame_ratio': fmt(processed / sent if sent else None),
        'startup_rows_excluded': str(startup),
        'official_analysis_rows': str(len(official)),
        'late_rows_excluded': str(late),
        'mean_detection_count': fmt(statistics.fmean(detections)),
        'minimum_detection_count': fmt(min(detections)),
        'maximum_detection_count': fmt(max(detections)),
        'mean_pipeline_latency_ms': fmt(statistics.fmean(latency)),
        'median_pipeline_latency_ms': fmt(statistics.median(latency)),
        'p95_pipeline_latency_ms': fmt(percentile(latency, 0.95)),
        'minimum_pipeline_latency_ms': fmt(min(latency)),
        'maximum_pipeline_latency_ms': fmt(max(latency)),
        'latency_rows_above_500_ms': str(sum(item > 500 for item in latency)),
        'latency_rows_above_1000_ms': str(sum(item > 1000 for item in latency)),
        'latency_rows_above_2000_ms': str(sum(item > 2000 for item in latency)),
        'mean_inference_ms': fmt(statistics.fmean(inference)),
        'mean_compression_ms': fmt(statistics.fmean(compression)
                                   if compression else None),
        'mean_decode_ms': fmt(statistics.fmean(decode) if decode else None),
        'timestamp_span_s': fmt(max(timestamps) - min(timestamps)),
        'mean_interarrival_s': fmt(mean_interval),
        'resource_cpu_mean_percent': fmt(cpu_mean),
        'resource_cpu_peak_percent': fmt(cpu_peak),
        'resource_rss_mean_mb': fmt(rss_mean),
        'resource_rss_peak_mb': fmt(rss_peak),
        'gps_horizontal_drift_m': fmt(drift),
        'delivery_observations': ';'.join(observations) or 'none',
        'verification_status': status,
        'validation_warnings': ';'.join(failures + warnings) or 'none',
    }


def mode_summary(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    output = []
    for mode in ('edge', 'ground'):
        selected = [row for row in rows if row['actual_mode'] == mode]
        for label, field in MODE_METRICS.items():
            values = [float(row[field]) for row in selected if row[field]]
            output.append({
                'mode': mode, 'metric': label, 'n': str(len(values)),
                'mean': fmt(statistics.fmean(values) if values else None),
                'sample_standard_deviation': fmt(
                    statistics.stdev(values) if len(values) > 1 else None),
                'median': fmt(statistics.median(values) if values else None),
                'minimum': fmt(min(values) if values else None),
                'maximum': fmt(max(values) if values else None),
            })
    return output


def paired(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    output = []
    for rng in ('1', '2', '3'):
        candidates = [row for row in rows if row['rng_run'] == rng]
        edge = next(row for row in candidates if row['actual_mode'] == 'edge')
        ground = next(row for row in candidates if row['actual_mode'] == 'ground')
        result = {
            'rng_run': rng, 'edge_run_id': edge['run_id'],
            'ground_run_id': ground['run_id'],
        }
        specifications = (
            ('processed_ratio', 'processed_frame_ratio', True),
            ('median_latency_ms', 'median_pipeline_latency_ms', False),
            ('p95_latency_ms', 'p95_pipeline_latency_ms', False),
            ('mean_inference_ms', 'mean_inference_ms', False),
            ('cpu_mean_percent', 'resource_cpu_mean_percent', False),
            ('rss_mean_mb', 'resource_rss_mean_mb', False),
        )
        for label, field, ratio in specifications:
            edge_value, ground_value = float(edge[field]), float(ground[field])
            result[f'edge_{label}'] = fmt(edge_value)
            result[f'ground_{label}'] = fmt(ground_value)
            difference = edge_value - ground_value if ratio else ground_value - edge_value
            difference_label = ('edge_advantage_percentage_points' if ratio
                                else f'ground_minus_edge_{label}')
            result[difference_label] = fmt(difference * 100 if ratio else difference)
        output.append(result)
    return output


def paired_fields() -> list[str]:
    return [
        'rng_run', 'edge_run_id', 'ground_run_id',
        'edge_processed_ratio', 'ground_processed_ratio',
        'edge_advantage_percentage_points', 'edge_median_latency_ms',
        'ground_median_latency_ms', 'ground_minus_edge_median_latency_ms',
        'edge_p95_latency_ms', 'ground_p95_latency_ms',
        'ground_minus_edge_p95_latency_ms', 'edge_mean_inference_ms',
        'ground_mean_inference_ms', 'ground_minus_edge_mean_inference_ms',
        'edge_cpu_mean_percent', 'ground_cpu_mean_percent',
        'ground_minus_edge_cpu_mean_percent', 'edge_rss_mean_mb',
        'ground_rss_mean_mb', 'ground_minus_edge_rss_mean_mb',
    ]


def save_grouped(rows: list[dict[str, str]], field: str, stem: str,
                 ylabel: str, ratio: bool = False) -> None:
    rngs = ['1', '2', '3']
    edge = [float(next(row for row in rows if row['rng_run'] == rng
                       and row['actual_mode'] == 'edge')[field]) for rng in rngs]
    ground = [float(next(row for row in rows if row['rng_run'] == rng
                         and row['actual_mode'] == 'ground')[field]) for rng in rngs]
    positions = np.arange(3)
    fig, axis = plt.subplots(figsize=(7.2, 4.6))
    width = 0.36
    edge_bars = axis.bar(positions - width / 2, edge, width, label='Edge',
                         color='#2864a5', edgecolor='black', linewidth=0.6)
    ground_bars = axis.bar(positions + width / 2, ground, width, label='Ground',
                           color='#d97706', edgecolor='black', linewidth=0.6,
                           hatch='//')
    axis.set_xticks(positions, [f'RNG {rng}' for rng in rngs])
    axis.set_ylabel(ylabel)
    axis.set_xlabel('NS-3 RNG run')
    axis.set_ylim(bottom=0, top=1.05 if ratio else None)
    axis.legend(frameon=False, ncol=2, loc='upper center',
                bbox_to_anchor=(0.5, 1.14))
    axis.grid(axis='y', color='#b8b8b8', alpha=0.45, linewidth=0.6)
    axis.set_axisbelow(True)
    label_format = '%.2f' if ratio else '%.1f'
    axis.bar_label(edge_bars, fmt=label_format, padding=3, fontsize=8)
    axis.bar_label(ground_bars, fmt=label_format, padding=3, fontsize=8)
    if not ratio:
        axis.margins(y=0.15)
    fig.tight_layout()
    fig.savefig(FIGURES / f'{stem}.png', dpi=600, bbox_inches='tight')
    fig.savefig(FIGURES / f'{stem}.pdf', bbox_inches='tight')
    plt.close(fig)


def save_ground_overhead(rows: list[dict[str, str]]) -> None:
    ground = sorted((row for row in rows if row['actual_mode'] == 'ground'),
                    key=lambda row: int(row['rng_run']))
    compression = [float(row['mean_compression_ms']) for row in ground]
    decode = [float(row['mean_decode_ms']) for row in ground]
    positions = np.arange(3)
    fig, axis = plt.subplots(figsize=(7.2, 4.6))
    width = 0.36
    compression_bars = axis.bar(
        positions - width / 2, compression, width, label='JPEG compression',
        color='#6b46c1', edgecolor='black', linewidth=0.6)
    decode_bars = axis.bar(
        positions + width / 2, decode, width, label='JPEG decode',
        color='#2f855a', edgecolor='black', linewidth=0.6, hatch='//')
    axis.set_xticks(positions, [f'RNG {row["rng_run"]}' for row in ground])
    axis.set_xlabel('NS-3 RNG run')
    axis.set_ylabel('Mean time (ms)')
    axis.set_ylim(bottom=0)
    axis.legend(frameon=False, ncol=2, loc='upper center',
                bbox_to_anchor=(0.5, 1.14))
    axis.grid(axis='y', color='#b8b8b8', alpha=0.45, linewidth=0.6)
    axis.set_axisbelow(True)
    axis.bar_label(compression_bars, fmt='%.2f', padding=3, fontsize=8)
    axis.bar_label(decode_bars, fmt='%.2f', padding=3, fontsize=8)
    axis.margins(y=0.15)
    fig.tight_layout()
    fig.savefig(FIGURES / 'ground_overhead_by_rng.png', dpi=600,
                bbox_inches='tight')
    fig.savefig(FIGURES / 'ground_overhead_by_rng.pdf', bbox_inches='tight')
    plt.close(fig)


def d4_combined(rows: list[dict[str, str]]) -> dict[str, str]:
    d4 = read_csv(D4_SUMMARY)
    edge_d4 = next(row for row in d4 if row['representation'] == 'edge_raw')
    ground_d4 = next(row for row in d4 if row['representation'] == 'ground_q5')
    edge_runs = [row for row in rows if row['actual_mode'] == 'edge']
    ground_runs = [row for row in rows if row['actual_mode'] == 'ground']
    edge_ratios = [float(row['processed_frame_ratio']) for row in edge_runs]
    ground_ratios = [float(row['processed_frame_ratio']) for row in ground_runs]
    edge_latency = [float(row['median_pipeline_latency_ms']) for row in edge_runs]
    ground_latency = [float(row['median_pipeline_latency_ms']) for row in ground_runs]
    return {
        'd4_edge_raw_precision': edge_d4['precision'],
        'd4_edge_raw_recall': edge_d4['recall'],
        'd4_edge_raw_f1': edge_d4['f1'],
        'd4_edge_raw_exact_count_accuracy': edge_d4['exact_count_accuracy'],
        'd4_ground_q5_precision': ground_d4['precision'],
        'd4_ground_q5_recall': ground_d4['recall'],
        'd4_ground_q5_f1': ground_d4['f1'],
        'd4_ground_q5_exact_count_accuracy': ground_d4['exact_count_accuracy'],
        'phase_f_edge_mean_processed_frame_ratio': fmt(statistics.fmean(edge_ratios)),
        'phase_f_ground_mean_processed_frame_ratio': fmt(statistics.fmean(ground_ratios)),
        'phase_f_edge_run_median_latencies_ms': ';'.join(fmt(item, 2) for item in edge_latency),
        'phase_f_edge_mean_of_run_medians_ms': fmt(statistics.fmean(edge_latency)),
        'phase_f_ground_run_median_latencies_ms': ';'.join(fmt(item, 2) for item in ground_latency),
        'phase_f_ground_mean_of_run_medians_ms': fmt(statistics.fmean(ground_latency)),
    }


def markdown_table(rows: list[dict[str, str]], fields: list[str]) -> str:
    lines = ['| ' + ' | '.join(fields) + ' |',
             '| ' + ' | '.join('---' for _ in fields) + ' |']
    lines.extend('| ' + ' | '.join(row.get(field, '') for field in fields) + ' |'
                 for row in rows)
    return '\n'.join(lines)


def combined_markdown(combined: dict[str, str]) -> str:
    rows = [
        {'measure': 'Precision', 'Edge/raw': combined['d4_edge_raw_precision'],
         'Ground/q5': combined['d4_ground_q5_precision'], 'phase': 'D4 accuracy'},
        {'measure': 'Recall', 'Edge/raw': combined['d4_edge_raw_recall'],
         'Ground/q5': combined['d4_ground_q5_recall'], 'phase': 'D4 accuracy'},
        {'measure': 'F1', 'Edge/raw': combined['d4_edge_raw_f1'],
         'Ground/q5': combined['d4_ground_q5_f1'], 'phase': 'D4 accuracy'},
        {'measure': 'Exact-count accuracy',
         'Edge/raw': combined['d4_edge_raw_exact_count_accuracy'],
         'Ground/q5': combined['d4_ground_q5_exact_count_accuracy'],
         'phase': 'D4 accuracy'},
        {'measure': 'Mean processed-frame ratio',
         'Edge/raw': combined['phase_f_edge_mean_processed_frame_ratio'],
         'Ground/q5': combined['phase_f_ground_mean_processed_frame_ratio'],
         'phase': 'Phase F system'},
        {'measure': 'Run median latency values (ms)',
         'Edge/raw': combined['phase_f_edge_run_median_latencies_ms'],
         'Ground/q5': combined['phase_f_ground_run_median_latencies_ms'],
         'phase': 'Phase F system'},
        {'measure': 'Mean of run median latencies (ms)',
         'Edge/raw': combined['phase_f_edge_mean_of_run_medians_ms'],
         'Ground/q5': combined['phase_f_ground_mean_of_run_medians_ms'],
         'phase': 'Phase F system'},
    ]
    introduction = ('# Combined D4 accuracy and Phase F system results\n\n'
                    'D4 evaluates detection accuracy against manually labelled frames. '
                    'Phase F evaluates live end-to-end system behavior; its detection '
                    'count is not precision or recall.\n\n')
    return introduction + markdown_table(rows, ['measure', 'Edge/raw', 'Ground/q5', 'phase']) + '\n'


def report(rows: list[dict[str, str]], modes: list[dict[str, str]],
           pairs: list[dict[str, str]], combined_md: str) -> str:
    compact = ['rng_run', 'actual_mode', 'run_id', 'official_sent_frames',
               'official_processed_frames', 'processed_frame_ratio',
               'median_pipeline_latency_ms', 'p95_pipeline_latency_ms',
               'mean_inference_ms', 'resource_cpu_mean_percent',
               'resource_rss_mean_mb', 'delivery_observations',
               'validation_warnings']
    mode_focus = [row for row in modes if row['metric'] in (
        'processed_frame_ratio', 'median_latency_ms', 'p95_latency_ms',
        'mean_inference_ms', 'cpu_mean_percent', 'rss_mean_mb')]
    return f"""# Definitive Edge-versus-Ground comparison

## 1. Experimental configuration

The fixed target was latitude 6.079430, longitude 80.193085, relative altitude 25.0 m, and externally commanded yaw 102.6 degrees. Relay rate was 1 Hz, the official window was 60 seconds, YOLO confidence was 0.25, debug publication was disabled by the runner, and Ground used JPEG quality 5.

## 2. Protocol revision and stabilization

Every selected run used at least nine processed warm-up rows, 15 seconds of additional settling, and a five-row stable-delivery gate with 0.70–1.30 second intervals. Official statistics use only metadata-recorded start/end boundaries. No official-window latency outlier was removed.

## 3. Final selected runs

{markdown_table(rows, compact)}

## 4. Provenance of RNG3 mode labels

RNG3 Run IDs are inverted: `f_edge_03_final_v2` is Ground and `f_ground_q5_03_final_v3` is Edge. Detector, relay, metrics, metadata, and JPEG evidence agree on those actual modes. Files were not renamed.

## 5. Per-run results

All six official slices contain exactly the metadata-recorded processed-frame count. Frames summarize their parent run; they are not treated as independent experimental repetitions.

## 6. Processed-frame reliability

{markdown_table(mode_focus, ['mode', 'metric', 'n', 'mean', 'sample_standard_deviation', 'median', 'minimum', 'maximum'])}

Edge ratio means local edge-pipeline completion. Ground ratio means complete compressed-frame reception and processing; it is not interchangeable with Edge wireless delivery.

In Ground mode, inter-arrival time is measured between successfully received and processed complete frames. Values above the nominal one-second relay interval are expected when complete compressed frames are lost. Increased Ground inter-arrival is therefore a delivery-performance observation rather than a provenance or experiment-validity failure.

## 7. Latency comparison

Median and p95 latency are reported per run and summarized across the three run-level values per mode. Official outliers remain included. `wireless_transit_ms` is not interpreted as guaranteed pure radio propagation delay.

## 8. Inference and image-processing overhead

Inference is reported for both modes. JPEG compression and decoding are Ground-specific overheads and are shown separately in `ground_overhead_by_rng`.

## 9. CPU and memory results

CPU and RSS aggregate the monitored detector and relay processes on the simulation host. They measure host process demand, not physical-UAV energy or battery consumption.

## 10. Combined D4 accuracy and Phase F system results

{combined_md.split(chr(10), 2)[2]}

D4 uses manually labelled frames for accuracy. Phase F measures live system completion and timing. Phase F `detection_count` is not precision, recall, or F1.

## 11. Main conclusions

The tables and paired plots show the observed reliability, latency, inference, and host-resource differences for these three controlled RNG pairs. Conclusions are descriptive: no significance test was performed with only three pairs.

## 12. Limitations

There are only three independent repetitions per mode, limiting generalization. Yaw was commanded externally rather than independently measured in each saved artifact. Host CPU/RSS cannot establish UAV power demand, and the available timing decomposition cannot isolate pure radio propagation delay.
"""


def main() -> None:
    PROCESSED.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)
    registry = read_csv(SELECTED)
    if len(registry) != 6:
        raise ValueError('Final manifest must contain exactly six runs')
    rows = [analyze_run(item) for item in registry]
    if any(row['verification_status'] == 'verification_failed' for row in rows):
        raise ValueError('A selected run failed configuration or provenance verification')
    modes = mode_summary(rows)
    pairs = paired(rows)
    write_csv(PROCESSED / 'run_summary.csv', RUN_FIELDS, rows)
    write_csv(PROCESSED / 'mode_summary.csv', [
        'mode', 'metric', 'n', 'mean', 'sample_standard_deviation',
        'median', 'minimum', 'maximum'], modes)
    write_csv(PROCESSED / 'paired_comparison.csv', paired_fields(), pairs)
    write_csv(PROCESSED / 'latency_validation.csv', LATENCY_FIELDS, rows)

    combined = d4_combined(rows)
    write_csv(PROCESSED / 'd4_phase_f_combined.csv', list(combined), [combined])
    combined_md = combined_markdown(combined)
    (PROCESSED / 'd4_phase_f_combined.md').write_text(combined_md)
    (PROCESSED / 'final_comparison_summary.md').write_text(
        report(rows, modes, pairs, combined_md))
    print('Generated definitive primary comparison outputs.')


if __name__ == '__main__':
    main()
