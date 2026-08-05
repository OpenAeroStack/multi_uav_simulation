#!/usr/bin/env python3
"""Audit and analyze only the six-run revised comparison registry."""

from __future__ import annotations

import csv
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Optional


REPO = Path(__file__).resolve().parents[2]
FINAL_DIR = REPO / 'results/final_edge_ground_comparison_revised'
SELECTED = FINAL_DIR / 'selected_runs.csv'
PROCESSED = FINAL_DIR / 'processed'

RUN_FIELDS = [
    'session', 'rng_run', 'expected_mode', 'observed_mode', 'run_id',
    'total_metrics_rows', 'official_start_csv_row', 'official_end_csv_row',
    'startup_rows_excluded', 'official_sent_frames',
    'official_processed_frames', 'processed_frame_ratio',
    'official_analysis_rows', 'late_rows_excluded',
    'mean_detection_count', 'min_detection_count', 'max_detection_count',
    'mean_pipeline_latency_ms', 'median_pipeline_latency_ms',
    'p95_pipeline_latency_ms', 'min_pipeline_latency_ms',
    'max_pipeline_latency_ms', 'mean_inference_ms', 'mean_compression_ms',
    'mean_decode_ms', 'timestamp_span_s', 'mean_interarrival_s',
    'gps_before_lat', 'gps_before_lon', 'gps_after_lat', 'gps_after_lon',
    'horizontal_drift_m', 'resource_cpu_mean_percent',
    'resource_cpu_peak_percent', 'resource_rss_mean_mb',
    'resource_rss_peak_mb', 'verification_status', 'eligible_for_comparison',
    'validation_warnings',
]

MODE_METRICS = [
    'processed_frame_ratio', 'mean_pipeline_latency_ms',
    'median_pipeline_latency_ms', 'p95_pipeline_latency_ms',
    'min_pipeline_latency_ms', 'max_pipeline_latency_ms',
    'mean_inference_ms', 'mean_compression_ms', 'mean_decode_ms',
    'mean_detection_count', 'resource_cpu_mean_percent',
    'resource_cpu_peak_percent', 'resource_rss_mean_mb',
    'resource_rss_peak_mb',
]

PAIR_METRICS = [
    'processed_frame_ratio', 'mean_pipeline_latency_ms',
    'median_pipeline_latency_ms', 'p95_pipeline_latency_ms',
    'mean_inference_ms', 'mean_detection_count',
    'resource_cpu_mean_percent', 'resource_cpu_peak_percent',
    'resource_rss_mean_mb', 'resource_rss_peak_mb',
    'mean_compression_ms', 'mean_decode_ms',
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fields: list[str], rows: Iterable[dict]) -> None:
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def resolve_curated(relative_path: str) -> Optional[Path]:
    if not relative_path:
        return None
    path = (FINAL_DIR / relative_path).resolve()
    return path if path.is_file() else None


def metadata_value(text: str, key: str) -> Optional[str]:
    match = re.search(
        rf'^\s*{re.escape(key)}\s*(?::|=)\s*(.+)$', text, re.MULTILINE)
    return match.group(1).strip() if match else None


def metadata_int(text: str, key: str) -> Optional[int]:
    value = metadata_value(text, key)
    if value is None:
        return None
    match = re.search(r'-?\d+', value)
    return int(match.group()) if match else None


def metadata_float(text: str, key: str) -> Optional[float]:
    value = metadata_value(text, key)
    if value is None:
        return None
    match = re.search(r'-?(?:\d+(?:\.\d*)?|\.\d+)', value)
    return float(match.group()) if match else None


def parse_gps(text: str, section: str) -> tuple[Optional[float], Optional[float]]:
    start = text.find(section)
    if start < 0:
        return None, None
    block = text[start:]
    next_section = block.find('\nGPS message ', 1)
    if next_section >= 0:
        block = block[:next_section]
    lat = re.search(r'^latitude:\s*([-+0-9.eE]+)', block, re.MULTILINE)
    lon = re.search(r'^longitude:\s*([-+0-9.eE]+)', block, re.MULTILINE)
    return ((float(lat.group(1)) if lat else None),
            (float(lon.group(1)) if lon else None))


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    value = (math.sin(dphi / 2.0) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(dlon / 2.0) ** 2)
    return 2.0 * radius * math.asin(math.sqrt(value))


def numeric(rows: list[dict[str, str]], field: str,
            nonnegative: bool = True) -> list[float]:
    values = []
    for row in rows:
        try:
            value = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value) and (not nonnegative or value >= 0):
            values.append(value)
    return values


def percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def fmt(value: Optional[float], digits: int = 4) -> str:
    return '' if value is None else f'{value:.{digits}f}'


def resource_summary(path: Optional[Path]) -> tuple[Optional[float], ...]:
    if path is None:
        return None, None, None, None
    samples: dict[str, list[tuple[float, float]]] = defaultdict(list)
    with path.open() as handle:
        for line in handle:
            parts = line.split()
            if len(parts) < 16 or parts[1] not in ('AM', 'PM'):
                continue
            try:
                cpu_percent = float(parts[8])
                rss_kb = float(parts[13])
            except (ValueError, IndexError):
                continue
            samples[f'{parts[0]} {parts[1]}'].append((cpu_percent, rss_kb))
    if not samples:
        return None, None, None, None
    cpu = [sum(item[0] for item in group) for group in samples.values()]
    rss = [sum(item[1] for item in group) / 1024.0 for group in samples.values()]
    return statistics.fmean(cpu), max(cpu), statistics.fmean(rss), max(rss)


def metadata_audit(registry: dict[str, str], metadata: str,
                   start: Optional[int], end: Optional[int],
                   sent: Optional[int], processed: Optional[int]) -> list[str]:
    problems = []
    expected_mode = registry['expected_mode']
    checks = (
        (metadata_value(metadata, 'Run ID') == registry['run_id'], 'run_id_mismatch'),
        (metadata_value(metadata, 'Mode') == expected_mode, 'mode_mismatch'),
        (metadata_int(metadata, 'Expected NS-3 RNG run') == int(registry['rng_run']),
         'rng_run_mismatch'),
        (metadata_int(metadata, 'Measurement duration') == 60, 'duration_not_60s'),
        (metadata_float(metadata, 'Frame rate') == 1.0, 'frame_rate_not_1hz'),
        (metadata_float(metadata, 'Confidence threshold') == 0.25,
         'confidence_not_0.25'),
        (metadata_float(metadata, 'Fixed target latitude') == 6.079430,
         'fixed_target_latitude_mismatch'),
        (metadata_float(metadata, 'Fixed target longitude') == 80.193085,
         'fixed_target_longitude_mismatch'),
        (metadata_float(metadata, 'Fixed target altitude') == 25.0,
         'fixed_target_altitude_mismatch'),
        (metadata_int(metadata, 'minimum_warmup_rows') == 9,
         'minimum_warmup_not_9'),
        (metadata_int(metadata, 'post_warmup_settle_s') == 15,
         'post_warmup_settle_not_15s'),
        (metadata_int(metadata, 'stable_rows_required') == 5,
         'stable_rows_required_not_5'),
        (metadata_value(metadata, 'stable_interval_range_s') == '0.70-1.30',
         'stable_interval_range_mismatch'),
        (start is not None, 'official_start_missing'),
        (end is not None, 'official_end_missing'),
        (sent is not None, 'official_sent_count_missing'),
        (processed is not None, 'official_processed_count_missing'),
    )
    for passed, label in checks:
        if not passed:
            problems.append(label)
    jpeg = metadata_value(metadata, 'Ground JPEG quality')
    if expected_mode == 'ground' and jpeg != '5':
        problems.append('ground_jpeg_quality_not_5')
    if expected_mode == 'edge' and jpeg != 'n/a':
        problems.append('edge_jpeg_quality_not_na')
    before = parse_gps(metadata, 'GPS message before the run:')
    after = parse_gps(metadata, 'GPS message after the run:')
    if None in before:
        problems.append('gps_before_missing')
    if None in after:
        problems.append('gps_after_missing')
    return problems


def empty_run(registry: dict[str, str], total_rows: int,
              warning: str) -> tuple[dict[str, str], dict[str, str]]:
    row = {field: '' for field in RUN_FIELDS}
    row.update({
        'session': registry['session'], 'rng_run': registry['rng_run'],
        'expected_mode': registry['expected_mode'],
        'observed_mode': registry['observed_mode'], 'run_id': registry['run_id'],
        'total_metrics_rows': str(total_rows),
        'verification_status': 'provenance_failure',
        'eligible_for_comparison': 'false', 'validation_warnings': warning,
    })
    latency = {
        'session': registry['session'], 'rng_run': registry['rng_run'],
        'expected_mode': registry['expected_mode'],
        'observed_mode': registry['observed_mode'], 'run_id': registry['run_id'],
        'official_analysis_rows': '', 'latency_rows_above_500_ms': '',
        'latency_rows_above_1000_ms': '', 'latency_rows_above_2000_ms': '',
        'maximum_latency_ms': '', 'validation_flags': warning,
    }
    return row, latency


def analyze_run(registry: dict[str, str]) -> tuple[dict[str, str], dict[str, str]]:
    metrics_path = resolve_curated(registry['metrics_csv'])
    if metrics_path is None:
        return empty_run(registry, 0, 'metrics_csv_missing')
    all_rows = read_csv(metrics_path)
    metadata_path = resolve_curated(registry['metadata_file'])
    if metadata_path is None:
        return empty_run(registry, len(all_rows),
                         'metadata_missing;official_boundaries_unavailable;resource_file_missing')

    metadata = metadata_path.read_text()
    observed_mode = metadata_value(metadata, 'Mode') or registry['observed_mode']
    start = metadata_int(metadata, 'official_start_csv_row')
    end = metadata_int(metadata, 'official_end_csv_row')
    sent = metadata_int(metadata, 'official measurement frames sent')
    processed = metadata_int(metadata, 'official measurement frames processed')
    provenance = metadata_audit(registry, metadata, start, end, sent, processed)
    warnings = []
    if start is None or end is None or sent is None or processed is None:
        return empty_run(registry, len(all_rows),
                         ';'.join(provenance or ['official_metadata_incomplete']))
    if end > len(all_rows):
        warnings.append('official_end_exceeds_csv_length')
    if start < 0 or end < start:
        return empty_run(registry, len(all_rows), 'invalid_official_row_boundaries')

    official_rows = all_rows[start:min(end, len(all_rows))]
    if len(official_rows) != processed:
        warnings.append('official_analysis_rows_differ_from_metadata')
    late = max(0, len(all_rows) - end)

    detections = numeric(official_rows, 'detection_count')
    pipeline = numeric(official_rows, 'pipeline_latency_ms')
    inference = numeric(official_rows, 'inference_ms')
    compression = numeric(official_rows, 'compression_ms')
    decode = numeric(official_rows, 'decode_ms')
    timestamps = numeric(official_rows, 'timestamp_s', nonnegative=False)
    frame_numbers = numeric(official_rows, 'frame_num')
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    if any(b <= a for a, b in zip(timestamps, timestamps[1:])):
        warnings.append('official_timestamp_order_non_monotonic')
    if len(frame_numbers) != len(set(frame_numbers)):
        warnings.append('duplicate_frame_numbers')
    mean_interval = statistics.fmean(intervals) if intervals else None
    if mean_interval is not None and not 0.8 <= mean_interval <= 1.2:
        warnings.append('mean_interarrival_outside_0.8_to_1.2s')
    if any(value > 2000 for value in pipeline):
        warnings.append('latency_exceeds_2000ms')

    before_lat, before_lon = parse_gps(metadata, 'GPS message before the run:')
    after_lat, after_lon = parse_gps(metadata, 'GPS message after the run:')
    drift = (haversine_m(before_lat, before_lon, after_lat, after_lon)
             if None not in (before_lat, before_lon, after_lat, after_lon) else None)
    cpu_mean, cpu_peak, rss_mean, rss_peak = resource_summary(
        resolve_curated(registry['resource_file']))
    if registry['resource_file'] and cpu_mean is None:
        warnings.append('resource_samples_unavailable')

    eligible = not provenance and registry['selection_verification'] == 'verified'
    status = ('verified' if eligible else 'provenance_failure')
    if eligible and warnings:
        status = 'verified_with_validation_warnings'
    row = {
        'session': registry['session'], 'rng_run': registry['rng_run'],
        'expected_mode': registry['expected_mode'], 'observed_mode': observed_mode,
        'run_id': registry['run_id'], 'total_metrics_rows': str(len(all_rows)),
        'official_start_csv_row': str(start), 'official_end_csv_row': str(end),
        'startup_rows_excluded': str(start),
        'official_sent_frames': str(sent),
        'official_processed_frames': str(processed),
        'processed_frame_ratio': fmt(processed / sent if sent else None),
        'official_analysis_rows': str(len(official_rows)),
        'late_rows_excluded': str(late),
        'mean_detection_count': fmt(statistics.fmean(detections) if detections else None),
        'min_detection_count': fmt(min(detections) if detections else None),
        'max_detection_count': fmt(max(detections) if detections else None),
        'mean_pipeline_latency_ms': fmt(statistics.fmean(pipeline) if pipeline else None),
        'median_pipeline_latency_ms': fmt(statistics.median(pipeline) if pipeline else None),
        'p95_pipeline_latency_ms': fmt(percentile(pipeline, 0.95)),
        'min_pipeline_latency_ms': fmt(min(pipeline) if pipeline else None),
        'max_pipeline_latency_ms': fmt(max(pipeline) if pipeline else None),
        'mean_inference_ms': fmt(statistics.fmean(inference) if inference else None),
        'mean_compression_ms': fmt(statistics.fmean(compression) if compression else None),
        'mean_decode_ms': fmt(statistics.fmean(decode) if decode else None),
        'timestamp_span_s': fmt(max(timestamps) - min(timestamps)
                                if len(timestamps) >= 2 else None),
        'mean_interarrival_s': fmt(mean_interval),
        'gps_before_lat': fmt(before_lat, 8), 'gps_before_lon': fmt(before_lon, 8),
        'gps_after_lat': fmt(after_lat, 8), 'gps_after_lon': fmt(after_lon, 8),
        'horizontal_drift_m': fmt(drift),
        'resource_cpu_mean_percent': fmt(cpu_mean),
        'resource_cpu_peak_percent': fmt(cpu_peak),
        'resource_rss_mean_mb': fmt(rss_mean),
        'resource_rss_peak_mb': fmt(rss_peak),
        'verification_status': status,
        'eligible_for_comparison': str(eligible).lower(),
        'validation_warnings': ';'.join(provenance + warnings),
    }
    latency = {
        'session': registry['session'], 'rng_run': registry['rng_run'],
        'expected_mode': registry['expected_mode'], 'observed_mode': observed_mode,
        'run_id': registry['run_id'],
        'official_analysis_rows': str(len(official_rows)),
        'latency_rows_above_500_ms': str(sum(value > 500 for value in pipeline)),
        'latency_rows_above_1000_ms': str(sum(value > 1000 for value in pipeline)),
        'latency_rows_above_2000_ms': str(sum(value > 2000 for value in pipeline)),
        'maximum_latency_ms': fmt(max(pipeline) if pipeline else None),
        'validation_flags': ';'.join(provenance + warnings) or 'none',
    }
    return row, latency


def eligible_values(rows: list[dict[str, str]], mode: str,
                    field: str) -> list[float]:
    values = []
    for row in rows:
        if (row['expected_mode'] != mode
                or row['eligible_for_comparison'] != 'true' or not row.get(field)):
            continue
        values.append(float(row[field]))
    return values


def make_mode_summary(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    output = []
    for mode in ('edge', 'ground'):
        for metric in MODE_METRICS:
            values = eligible_values(rows, mode, metric)
            output.append({
                'mode': mode, 'metric': metric, 'n': str(len(values)),
                'mean': fmt(statistics.fmean(values) if values else None),
                'sample_standard_deviation': fmt(
                    statistics.stdev(values) if len(values) >= 2 else None),
                'median': fmt(statistics.median(values) if values else None),
                'minimum': fmt(min(values) if values else None),
                'maximum': fmt(max(values) if values else None),
            })
    return output


def make_paired(rows: list[dict[str, str]]) -> tuple[list[str], list[dict[str, str]]]:
    fields = ['session', 'rng_run', 'edge_run_id', 'ground_run_id', 'pair_status']
    for metric in PAIR_METRICS:
        fields.extend([f'edge_{metric}', f'ground_{metric}',
                       f'ground_minus_edge_{metric}'])
    output = []
    for session in ('1', '2', '3'):
        selected = [row for row in rows if row['session'] == session]
        edge = next(row for row in selected if row['expected_mode'] == 'edge')
        ground = next(row for row in selected if row['expected_mode'] == 'ground')
        complete = (edge['eligible_for_comparison'] == 'true'
                    and ground['eligible_for_comparison'] == 'true')
        pair = {
            'session': session, 'rng_run': edge['rng_run'],
            'edge_run_id': edge['run_id'], 'ground_run_id': ground['run_id'],
            'pair_status': 'verified' if complete else 'not_comparable_provenance_failure',
        }
        for metric in PAIR_METRICS:
            edge_value = float(edge[metric]) if complete and edge.get(metric) else None
            ground_value = float(ground[metric]) if complete and ground.get(metric) else None
            pair[f'edge_{metric}'] = fmt(edge_value)
            pair[f'ground_{metric}'] = fmt(ground_value)
            pair[f'ground_minus_edge_{metric}'] = fmt(
                ground_value - edge_value
                if edge_value is not None and ground_value is not None else None)
        output.append(pair)
    return fields, output


def markdown_table(rows: list[dict[str, str]], fields: list[str]) -> str:
    lines = ['| ' + ' | '.join(fields) + ' |',
             '| ' + ' | '.join('---' for _ in fields) + ' |']
    for row in rows:
        lines.append('| ' + ' | '.join(row.get(field, '') for field in fields) + ' |')
    return '\n'.join(lines)


def make_report(rows: list[dict[str, str]], paired: list[dict[str, str]]) -> str:
    fields = ['rng_run', 'expected_mode', 'observed_mode', 'run_id',
              'official_analysis_rows', 'processed_frame_ratio',
              'mean_pipeline_latency_ms', 'p95_pipeline_latency_ms',
              'verification_status', 'validation_warnings']
    pair_fields = ['rng_run', 'edge_run_id', 'ground_run_id', 'pair_status']
    return f"""# Revised edge-versus-ground comparison

## Outcome

The requested six-run selection cannot currently support the intended three-by-three primary comparison. Only `f_edge_01_final_v4` has complete provenance matching its requested mode. Four run IDs contain the opposite saved mode, and `f_ground_q5_02_final_v3` has no metadata from which to recover official CSV boundaries. No run was silently relabeled and no official boundary was guessed.

## Per-run audit and diagnostics

{markdown_table(rows, fields)}

Statistics shown above use only `rows[official_start_csv_row:official_end_csv_row]`. Official-window outliers are retained. A provenance failure remains visible diagnostically but is excluded from mode-level and paired claims.

## Paired RNG status

{markdown_table(paired, pair_fields)}

There are no complete verified pairs, so no significance testing or comparative mode finding is reported. Each complete run—not each frame—is the intended independent experimental unit.

## Interpretation boundaries

- Detection count is not precision or recall; detection accuracy belongs to Phase D4.
- Edge completion is local edge-pipeline completion, not wireless image delivery.
- Ground completion is complete compressed-frame reception and processing.
- CPU and RSS measure simulation-host process demand, not physical-UAV energy.
- `wireless_transit_ms` is not treated as pure radio delay.
- Yaw 102.6 degrees was externally commanded, not measured per run in these artifacts.

The initial curated analysis is preserved in `../../final_edge_ground_comparison_initial_v1/`. Original raw artifacts remain unchanged.
"""


def main() -> None:
    PROCESSED.mkdir(parents=True, exist_ok=True)
    registry = read_csv(SELECTED)
    if len(registry) != 6 or any(row['is_primary'].lower() != 'true'
                                 for row in registry):
        raise ValueError('selected_runs.csv must contain exactly six primary records')
    analyzed = [analyze_run(row) for row in registry]
    run_rows = [item[0] for item in analyzed]
    latency_rows = [item[1] for item in analyzed]
    mode_rows = make_mode_summary(run_rows)
    paired_fields, paired_rows = make_paired(run_rows)
    write_csv(PROCESSED / 'run_summary.csv', RUN_FIELDS, run_rows)
    write_csv(PROCESSED / 'mode_summary.csv', [
        'mode', 'metric', 'n', 'mean', 'sample_standard_deviation',
        'median', 'minimum', 'maximum'], mode_rows)
    write_csv(PROCESSED / 'paired_comparison.csv', paired_fields, paired_rows)
    write_csv(PROCESSED / 'latency_validation.csv', [
        'session', 'rng_run', 'expected_mode', 'observed_mode', 'run_id',
        'official_analysis_rows', 'latency_rows_above_500_ms',
        'latency_rows_above_1000_ms', 'latency_rows_above_2000_ms',
        'maximum_latency_ms', 'validation_flags'], latency_rows)
    (PROCESSED / 'final_comparison_summary.md').write_text(
        make_report(run_rows, paired_rows))
    for name in ('run_summary.csv', 'mode_summary.csv', 'paired_comparison.csv',
                 'latency_validation.csv', 'final_comparison_summary.md'):
        print(f'Wrote {(PROCESSED / name).relative_to(REPO)}')


if __name__ == '__main__':
    main()
