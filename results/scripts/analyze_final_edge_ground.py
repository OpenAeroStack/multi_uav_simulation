#!/usr/bin/env python3
"""Analyze only the six curated edge-versus-ground comparison runs.

Each run is the independent experimental unit. Frame rows are used only to
summarize that run; mode-level summaries operate on the three run-level values.
"""

from __future__ import annotations

import csv
import math
import re
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Optional


REPO = Path(__file__).resolve().parents[2]
FINAL_DIR = REPO / 'results/final_edge_ground_comparison'
SELECTED = FINAL_DIR / 'selected_runs.csv'
PROCESSED = FINAL_DIR / 'processed'
RUN_SUMMARY = PROCESSED / 'run_summary.csv'
MODE_SUMMARY = PROCESSED / 'mode_summary.csv'
PAIRED = PROCESSED / 'paired_comparison.csv'
REPORT = PROCESSED / 'final_comparison_summary.md'

RUN_FIELDS = [
    'session', 'rng_run', 'mode', 'run_id', 'official_sent_frames',
    'official_processed_frames', 'processed_frame_ratio',
    'official_analysis_rows', 'late_rows_excluded',
    'mean_detection_count', 'min_detection_count', 'max_detection_count',
    'mean_pipeline_latency_ms', 'median_pipeline_latency_ms',
    'p95_pipeline_latency_ms', 'mean_inference_ms', 'mean_compression_ms',
    'mean_decode_ms', 'timestamp_span_s', 'mean_interarrival_s',
    'gps_before_lat', 'gps_before_lon', 'gps_after_lat', 'gps_after_lon',
    'horizontal_drift_m', 'resource_cpu_mean_percent',
    'resource_cpu_peak_percent', 'resource_rss_mean_mb',
    'resource_rss_peak_mb', 'verification_status',
]

MODE_METRICS = [
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


def resolve_curated(relative_path: str) -> Path:
    path = (FINAL_DIR / relative_path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f'Artifact not found: {relative_path}')
    return path


def metadata_value(text: str, key: str) -> Optional[str]:
    match = re.search(
        rf'^\s*{re.escape(key)}:\s*(.+)$', text, re.MULTILINE)
    return match.group(1).strip() if match else None


def metadata_int(text: str, key: str) -> Optional[int]:
    value = metadata_value(text, key)
    if value is None:
        return None
    match = re.search(r'-?\d+', value)
    return int(match.group()) if match else None


def parse_gps(text: str, section: str) -> tuple[Optional[float], Optional[float]]:
    start = text.find(section)
    if start < 0:
        return None, None
    block = text[start:]
    next_section = block.find('\nGPS message ', 1)
    if next_section >= 0:
        block = block[:next_section]
    lat_match = re.search(r'^latitude:\s*([-+0-9.eE]+)', block, re.MULTILINE)
    lon_match = re.search(r'^longitude:\s*([-+0-9.eE]+)', block, re.MULTILINE)
    return (
        float(lat_match.group(1)) if lat_match else None,
        float(lon_match.group(1)) if lon_match else None,
    )


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2)
    return 2.0 * radius * math.asin(math.sqrt(a))


def numeric(rows: list[dict[str, str]], field: str) -> list[float]:
    values = []
    for row in rows:
        try:
            value = float(row[field])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value) and value >= 0:
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
    return (ordered[lower]
            + (ordered[upper] - ordered[lower]) * (position - lower))


def fmt(value: Optional[float], digits: int = 4) -> str:
    return '' if value is None else f'{value:.{digits}f}'


def resource_summary(path: Path) -> tuple[Optional[float], ...]:
    """Aggregate detector+relay CPU and RSS at each pidstat timestamp."""
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
    rss = [sum(item[1] for item in group) / 1024.0
           for group in samples.values()]
    return statistics.fmean(cpu), max(cpu), statistics.fmean(rss), max(rss)


def analyze_run(registry: dict[str, str]) -> dict[str, str]:
    metrics_path = resolve_curated(registry['metrics_csv'])
    metadata_path = resolve_curated(registry['metadata_file'])
    resource_path = resolve_curated(registry['resource_file'])
    metadata = metadata_path.read_text()
    all_rows = read_csv(metrics_path)

    warmup = int(registry['warmup_processed_frames'])
    sent = metadata_int(metadata, 'official measurement frames sent')
    processed = metadata_int(metadata, 'official measurement frames processed')
    if sent is None or processed is None:
        raise ValueError(f'Missing official counts for {registry["run_id"]}')
    official_rows = all_rows[warmup:warmup + processed]
    late = max(0, len(all_rows) - warmup - len(official_rows))

    detections = numeric(official_rows, 'detection_count')
    pipeline = numeric(official_rows, 'pipeline_latency_ms')
    inference = numeric(official_rows, 'inference_ms')
    compression = numeric(official_rows, 'compression_ms')
    decode = numeric(official_rows, 'decode_ms')
    timestamps = numeric(official_rows, 'timestamp_s')
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:]) if b >= a]
    before_lat, before_lon = parse_gps(metadata, 'GPS message before the run:')
    after_lat, after_lon = parse_gps(metadata, 'GPS message after the run:')
    drift = (haversine_m(before_lat, before_lon, after_lat, after_lon)
             if None not in (before_lat, before_lon, after_lat, after_lon)
             else None)
    cpu_mean, cpu_peak, rss_mean, rss_peak = resource_summary(resource_path)

    status = registry['verification_status']
    if len(official_rows) != processed:
        status = 'warning: fewer CSV rows than recorded official count'

    return {
        'session': registry['session'],
        'rng_run': registry['rng_run'],
        'mode': registry['mode'],
        'run_id': registry['run_id'],
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
        'mean_inference_ms': fmt(statistics.fmean(inference) if inference else None),
        'mean_compression_ms': fmt(statistics.fmean(compression) if compression else None),
        'mean_decode_ms': fmt(statistics.fmean(decode) if decode else None),
        'timestamp_span_s': fmt(max(timestamps) - min(timestamps) if len(timestamps) >= 2 else None),
        'mean_interarrival_s': fmt(statistics.fmean(intervals) if intervals else None),
        'gps_before_lat': fmt(before_lat, 8),
        'gps_before_lon': fmt(before_lon, 8),
        'gps_after_lat': fmt(after_lat, 8),
        'gps_after_lon': fmt(after_lon, 8),
        'horizontal_drift_m': fmt(drift),
        'resource_cpu_mean_percent': fmt(cpu_mean),
        'resource_cpu_peak_percent': fmt(cpu_peak),
        'resource_rss_mean_mb': fmt(rss_mean),
        'resource_rss_peak_mb': fmt(rss_peak),
        'verification_status': status,
    }


def valid_run_values(rows: list[dict[str, str]], mode: str,
                     field: str) -> list[float]:
    values = []
    for row in rows:
        if row['mode'] != mode or not row.get(field):
            continue
        try:
            values.append(float(row[field]))
        except ValueError:
            pass
    return values


def make_mode_summary(run_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    output = []
    for mode in ('edge', 'ground'):
        for metric in MODE_METRICS:
            values = valid_run_values(run_rows, mode, metric)
            if not values:
                continue
            output.append({
                'mode': mode,
                'metric': metric,
                'n': str(len(values)),
                'mean': fmt(statistics.fmean(values)),
                'sample_standard_deviation': fmt(
                    statistics.stdev(values) if len(values) >= 2 else None),
                'median': fmt(statistics.median(values)),
                'minimum': fmt(min(values)),
                'maximum': fmt(max(values)),
            })
    return output


def make_paired(run_rows: list[dict[str, str]]) -> tuple[list[str], list[dict[str, str]]]:
    metrics = [
        'mean_pipeline_latency_ms', 'median_pipeline_latency_ms',
        'p95_pipeline_latency_ms', 'mean_inference_ms',
        'mean_detection_count', 'resource_cpu_mean_percent',
        'resource_cpu_peak_percent', 'resource_rss_mean_mb',
        'resource_rss_peak_mb', 'mean_compression_ms', 'mean_decode_ms',
    ]
    fields = ['session', 'rng_run', 'edge_run_id', 'ground_run_id',
              'edge_ratio', 'ground_ratio', 'ground_minus_edge_ratio',
              'edge_advantage_percentage_points']
    for metric in metrics:
        fields.extend([f'edge_{metric}', f'ground_{metric}',
                       f'ground_minus_edge_{metric}'])
    pairs = []
    for session in ('1', '2', '3'):
        session_rows = [row for row in run_rows if row['session'] == session]
        edge = next(row for row in session_rows if row['mode'] == 'edge')
        ground = next(row for row in session_rows if row['mode'] == 'ground')
        edge_ratio = float(edge['processed_frame_ratio'])
        ground_ratio = float(ground['processed_frame_ratio'])
        pair = {
            'session': session,
            'rng_run': edge['rng_run'],
            'edge_run_id': edge['run_id'],
            'ground_run_id': ground['run_id'],
            'edge_ratio': fmt(edge_ratio),
            'ground_ratio': fmt(ground_ratio),
            'ground_minus_edge_ratio': fmt(ground_ratio - edge_ratio),
            'edge_advantage_percentage_points': fmt(
                100.0 * (edge_ratio - ground_ratio)),
        }
        for metric in metrics:
            edge_value = float(edge[metric]) if edge.get(metric) else None
            ground_value = float(ground[metric]) if ground.get(metric) else None
            pair[f'edge_{metric}'] = fmt(edge_value)
            pair[f'ground_{metric}'] = fmt(ground_value)
            pair[f'ground_minus_edge_{metric}'] = fmt(
                ground_value - edge_value
                if edge_value is not None and ground_value is not None else None)
        pairs.append(pair)
    return fields, pairs


def markdown_table(rows: list[dict[str, str]], fields: list[str]) -> str:
    lines = ['| ' + ' | '.join(fields) + ' |',
             '| ' + ' | '.join('---' for _ in fields) + ' |']
    for row in rows:
        lines.append('| ' + ' | '.join(row.get(field, '') for field in fields) + ' |')
    return '\n'.join(lines)


def make_report(run_rows: list[dict[str, str]],
                mode_rows: list[dict[str, str]],
                paired_rows: list[dict[str, str]]) -> str:
    exclusions = read_csv(FINAL_DIR / 'exclusions.csv')
    run_table_fields = [
        'rng_run', 'mode', 'run_id', 'official_sent_frames',
        'official_processed_frames', 'processed_frame_ratio',
        'mean_pipeline_latency_ms', 'p95_pipeline_latency_ms',
        'mean_inference_ms', 'mean_detection_count',
    ]
    resource_fields = [
        'rng_run', 'mode', 'resource_cpu_mean_percent',
        'resource_cpu_peak_percent', 'resource_rss_mean_mb',
        'resource_rss_peak_mb',
    ]
    mode_focus = [row for row in mode_rows if row['metric'] in (
        'processed_frame_ratio', 'mean_pipeline_latency_ms',
        'p95_pipeline_latency_ms', 'mean_inference_ms',
        'mean_detection_count')]
    paired_focus = [{
        'rng_run': row['rng_run'],
        'edge_ratio': row['edge_ratio'],
        'ground_ratio': row['ground_ratio'],
        'edge_advantage_percentage_points': row['edge_advantage_percentage_points'],
        'ground_minus_edge_mean_pipeline_latency_ms': row['ground_minus_edge_mean_pipeline_latency_ms'],
    } for row in paired_rows]
    exclusion_rows = [{
        'run_id': row['run_id'], 'reason_excluded': row['reason_excluded']
    } for row in exclusions]

    return f"""# Final edge-versus-ground comparison

## 1. Experimental configuration

- Fixed pose: latitude 6.079430, longitude 80.193085, relative altitude 25.0 m.
- Yaw: 102.6 degrees, externally controlled by `goto_comparison_pose.py`.
- Input rate: 1 Hz; official measurement: 60 seconds; first 9 processed rows are warm-up.
- YOLO confidence: 0.25; debug images disabled; ground JPEG quality: 5.
- Three independent RNG sessions per mode. Each complete run, not each frame, is the independent experimental unit.

## 2. Selected runs

RNG 1 used edge then ground; RNG 2 used ground then edge; RNG 3 used edge then ground. Exact IDs and artifact paths are registered in `../selected_runs.csv`.

## 3. Per-run comparison

{markdown_table(run_rows, run_table_fields)}

## 4. Mode-level summary

These values summarize the three run-level values per mode; frames were not pooled as independent repetitions.

{markdown_table(mode_focus, ['mode', 'metric', 'n', 'mean', 'sample_standard_deviation', 'median', 'minimum', 'maximum'])}

## 5. Paired RNG comparison

Latency differences are ground minus edge. No formal significance test is reported for only three pairs.

{markdown_table(paired_focus, ['rng_run', 'edge_ratio', 'ground_ratio', 'edge_advantage_percentage_points', 'ground_minus_edge_mean_pipeline_latency_ms'])}

## 6. Resource comparison

CPU and RSS are summed across the monitored detector and relay processes at each pidstat sample, then summarized over the official window.

{markdown_table(run_rows, resource_fields)}

## 7. Main findings

- Every selected edge run completed 60/60 official frames in its local edge pipeline.
- Ground completed 56/60, 35/60 and 42/60 frames across RNG 1–3, showing session-dependent loss in the complete compressed-frame ground pipeline.
- Detection count differs between modes but is only the number of YOLO outputs per processed frame. It is not precision, recall, F1, or detection accuracy; those belong to Phase D4.
- Pipeline latency includes compression for ground. Compression and decoding are reported separately. `wireless_transit_ms` is not interpreted as pure network-only delay.

## 8. Limitations

- There are only three independent repetitions per mode, so generalization is limited.
- Frame-level summaries describe individual runs and are not treated as independent replicates.
- Saved NavSat altitude is interpreted according to the run metadata; relative-altitude provenance comes from the pose controller/runner, not by relabeling an AMSL field.
- Per-run yaw telemetry and the runner's DDS-match console line were not saved in the selected artifacts. Yaw was externally controlled at 102.6 degrees; endpoint matching is indirectly evidenced by successful post-gate frame flow.
- Resource files identify both processes as `python3`; aggregation therefore reports combined monitored pipeline resources, not process-specific attribution.

## 9. Excluded runs

{markdown_table(exclusion_rows, ['run_id', 'reason_excluded'])}

Raw and excluded artifacts remain unmodified in their original directories.
"""


def main() -> None:
    PROCESSED.mkdir(parents=True, exist_ok=True)
    registry = read_csv(SELECTED)
    if len(registry) != 6 or any(row['is_official'].lower() != 'true'
                                 for row in registry):
        raise ValueError('selected_runs.csv must contain exactly six official runs')
    run_rows = [analyze_run(row) for row in registry]
    mode_rows = make_mode_summary(run_rows)
    paired_fields, paired_rows = make_paired(run_rows)
    write_csv(RUN_SUMMARY, RUN_FIELDS, run_rows)
    write_csv(MODE_SUMMARY, [
        'mode', 'metric', 'n', 'mean', 'sample_standard_deviation',
        'median', 'minimum', 'maximum'], mode_rows)
    write_csv(PAIRED, paired_fields, paired_rows)
    REPORT.write_text(make_report(run_rows, mode_rows, paired_rows))
    print(f'Wrote {RUN_SUMMARY.relative_to(REPO)}')
    print(f'Wrote {MODE_SUMMARY.relative_to(REPO)}')
    print(f'Wrote {PAIRED.relative_to(REPO)}')
    print(f'Wrote {REPORT.relative_to(REPO)}')


if __name__ == '__main__':
    main()
