#!/usr/bin/env python3
"""Build combined D4/Phase F tables, figure, and report-ready drafts."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


REPO = Path(__file__).resolve().parents[2]
FINAL = REPO / 'results/final_edge_ground_comparison_primary'
PROCESSED = FINAL / 'processed'
FIGURES = PROCESSED / 'figures'
D4 = (REPO / 'results/phase_d_application/processed/'
      'd4_detection_summary_d4_raw_vs_q5_01_20260805_221850.csv')
RUNS = PROCESSED / 'run_summary.csv'
MODES = PROCESSED / 'mode_summary.csv'
PAIRS = PROCESSED / 'paired_comparison.csv'


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline='') as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, fields: list[str], rows: list[dict[str, str]]) -> None:
    with path.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def mode_stat(rows: list[dict[str, str]], mode: str,
              metric: str) -> dict[str, str]:
    return next(row for row in rows
                if row['mode'] == mode and row['metric'] == metric)


def markdown_table(rows: list[dict[str, str]], fields: list[str],
                   labels: list[str] | None = None) -> str:
    labels = labels or fields
    lines = ['| ' + ' | '.join(labels) + ' |',
             '| ' + ' | '.join('---' for _ in fields) + ' |']
    lines.extend('| ' + ' | '.join(row.get(field, '') for field in fields) + ' |'
                 for row in rows)
    return '\n'.join(lines)


def build_combined(d4_rows: list[dict[str, str]],
                   mode_rows: list[dict[str, str]]) -> list[dict[str, str]]:
    output = []
    mappings = (('edge', 'edge_raw', 'Edge/raw'),
                ('ground', 'ground_q5', 'Ground/JPEG q5'))
    for mode, representation, label in mappings:
        accuracy = next(row for row in d4_rows
                        if row['representation'] == representation)
        output.append({
            'mode': label,
            'precision': accuracy['precision'],
            'recall': accuracy['recall'],
            'f1': accuracy['f1'],
            'exact_count_rate': accuracy['exact_count_accuracy'],
            'evaluation_frames': accuracy['frames'],
            'image_representation': representation,
            'mean_processed_frame_ratio': mode_stat(
                mode_rows, mode, 'processed_frame_ratio')['mean'],
            'mean_of_run_median_latency_ms': mode_stat(
                mode_rows, mode, 'median_latency_ms')['mean'],
            'mean_p95_latency_ms': mode_stat(
                mode_rows, mode, 'p95_latency_ms')['mean'],
            'mean_inference_ms': mode_stat(
                mode_rows, mode, 'mean_inference_ms')['mean'],
            'mean_detection_count': mode_stat(
                mode_rows, mode, 'mean_detection_count')['mean'],
            'mean_cpu_percent': mode_stat(
                mode_rows, mode, 'cpu_mean_percent')['mean'],
            'mean_rss_mb': mode_stat(mode_rows, mode, 'rss_mean_mb')['mean'],
            'mean_compression_ms': mode_stat(
                mode_rows, mode, 'mean_compression_ms')['mean'],
            'mean_decode_ms': mode_stat(
                mode_rows, mode, 'mean_decode_ms')['mean'],
            'independent_runs': mode_stat(
                mode_rows, mode, 'processed_frame_ratio')['n'],
        })
    return output


def comparisons(combined: list[dict[str, str]]) -> dict[str, float]:
    edge, ground = combined
    return {
        'edge_completion_advantage_pp': 100 * (
            float(edge['mean_processed_frame_ratio'])
            - float(ground['mean_processed_frame_ratio'])),
        'ground_to_edge_median_latency_ratio': (
            float(ground['mean_of_run_median_latency_ms'])
            / float(edge['mean_of_run_median_latency_ms'])),
        'ground_minus_edge_median_latency_ms': (
            float(ground['mean_of_run_median_latency_ms'])
            - float(edge['mean_of_run_median_latency_ms'])),
        'edge_minus_ground_recall': float(edge['recall']) - float(ground['recall']),
        'edge_minus_ground_f1': float(edge['f1']) - float(ground['f1']),
        'edge_minus_ground_exact_count_rate': (
            float(edge['exact_count_rate']) - float(ground['exact_count_rate'])),
        'edge_minus_ground_mean_cpu_pp': (
            float(edge['mean_cpu_percent']) - float(ground['mean_cpu_percent'])),
        'ground_minus_edge_mean_rss_mb': (
            float(ground['mean_rss_mb']) - float(edge['mean_rss_mb'])),
        'ground_mean_compression_decode_ms': (
            float(ground['mean_compression_ms'])
            + float(ground['mean_decode_ms'])),
    }


def combined_markdown(combined: list[dict[str, str]],
                      comparison: dict[str, float]) -> str:
    accuracy_fields = ['mode', 'precision', 'recall', 'f1',
                       'exact_count_rate', 'evaluation_frames',
                       'image_representation']
    system_fields = ['mode', 'mean_processed_frame_ratio',
                     'mean_of_run_median_latency_ms', 'mean_p95_latency_ms',
                     'mean_inference_ms', 'mean_detection_count',
                     'mean_cpu_percent', 'mean_rss_mb', 'mean_compression_ms',
                     'mean_decode_ms', 'independent_runs']
    comparison_rows = [
        {'comparison': 'Edge completion advantage',
         'value': f'{comparison["edge_completion_advantage_pp"]:.2f} percentage points'},
        {'comparison': 'Ground-to-Edge mean run-median latency ratio',
         'value': f'{comparison["ground_to_edge_median_latency_ratio"]:.3f}×'},
        {'comparison': 'Ground minus Edge mean run-median latency',
         'value': f'{comparison["ground_minus_edge_median_latency_ms"]:.2f} ms'},
        {'comparison': 'Edge minus Ground recall',
         'value': f'{comparison["edge_minus_ground_recall"]:.6f}'},
        {'comparison': 'Edge minus Ground F1',
         'value': f'{comparison["edge_minus_ground_f1"]:.6f}'},
        {'comparison': 'Edge minus Ground exact-count rate',
         'value': f'{comparison["edge_minus_ground_exact_count_rate"]:.6f}'},
        {'comparison': 'Edge minus Ground mean CPU utilization',
         'value': f'{comparison["edge_minus_ground_mean_cpu_pp"]:.2f} percentage points'},
        {'comparison': 'Ground minus Edge mean RSS',
         'value': f'{comparison["ground_minus_edge_mean_rss_mb"]:.2f} MB'},
        {'comparison': 'Ground mean compression + decode overhead',
         'value': f'{comparison["ground_mean_compression_decode_ms"]:.2f} ms'},
    ]
    return f"""# Combined Phase D4 and Phase F results

## Detection accuracy from Phase D4

D4 evaluates detection accuracy against manually labelled frames.

{markdown_table(combined, accuracy_fields)}

## Live system performance from Phase F

Phase F evaluates live end-to-end behavior over three independent runs per mode. Phase F detection count is not an accuracy metric. Edge processed-frame ratio denotes local pipeline completion; Ground processed-frame ratio denotes complete compressed-frame reception and processing.

{markdown_table(combined, system_fields)}

## Key cross-mode comparisons

{markdown_table(comparison_rows, ['comparison', 'value'])}

Cross-mode values use the three run-level summaries rather than pooling individual frames. No significance test is reported for three independent runs per mode.
"""


def d4_figure(d4_rows: list[dict[str, str]]) -> None:
    edge = next(row for row in d4_rows if row['representation'] == 'edge_raw')
    ground = next(row for row in d4_rows if row['representation'] == 'ground_q5')
    metrics = ('precision', 'recall', 'f1', 'exact_count_accuracy')
    labels = ('Precision', 'Recall', 'F1', 'Exact-count rate')
    edge_values = [float(edge[field]) for field in metrics]
    ground_values = [float(ground[field]) for field in metrics]
    x_values = np.arange(len(metrics))
    width = 0.36
    fig, axis = plt.subplots(figsize=(7.4, 4.8))
    edge_bars = axis.bar(x_values - width / 2, edge_values, width,
                         label='Edge/raw', color='#2864a5',
                         edgecolor='black', linewidth=0.6)
    ground_bars = axis.bar(x_values + width / 2, ground_values, width,
                           label='Ground/JPEG q5', color='#d97706',
                           edgecolor='black', linewidth=0.6, hatch='//')
    axis.set_xticks(x_values, labels)
    axis.set_ylabel('Accuracy metric')
    axis.set_title('Detection Accuracy: Raw Edge Input vs JPEG Quality 5', pad=34)
    axis.set_ylim(0, 1.05)
    axis.grid(axis='y', color='#b8b8b8', alpha=0.45, linewidth=0.6)
    axis.set_axisbelow(True)
    axis.legend(frameon=False, ncol=2, loc='upper center',
                bbox_to_anchor=(0.5, 1.12))
    axis.bar_label(edge_bars, fmt='%.3f', padding=3, fontsize=8)
    axis.bar_label(ground_bars, fmt='%.3f', padding=3, fontsize=8)
    fig.tight_layout()
    fig.savefig(FIGURES / 'd4_detection_accuracy_comparison.png', dpi=600,
                bbox_inches='tight')
    fig.savefig(FIGURES / 'd4_detection_accuracy_comparison.pdf',
                bbox_inches='tight')
    plt.close(fig)


def figure_plan() -> str:
    entries = [
        ('1', 'processed_frame_ratio_by_rng',
         'Processed-frame completion for Edge local processing and Ground complete compressed-frame processing across the three RNG runs.',
         'Processed-Frame Reliability', 'Main report',
         'Edge completed every locally published official frame; Ground completion varied by RNG.'),
        ('2', 'median_pipeline_latency_by_rng',
         'Official-window median end-to-end pipeline latency for Edge and Ground processing.',
         'End-to-End Pipeline Latency', 'Main report',
         'Ground median latency was substantially higher in every RNG pair.'),
        ('3', 'paired_median_latency_comparison',
         'Within-RNG change in official-window median latency from Edge to Ground.',
         'End-to-End Pipeline Latency', 'Main report',
         'All three paired sessions show a latency increase for Ground processing.'),
        ('4', 'd4_detection_accuracy_comparison',
         'Phase D4 detection accuracy for raw Edge images and JPEG-quality-5 Ground images over 61 manually labelled frames.',
         'Detection Accuracy under Compression', 'Main report',
         'JPEG q5 retained precision but substantially reduced recall, F1 and exact-count rate.'),
        ('5', 'mean_cpu_utilization_by_rng',
         'Mean monitored process CPU utilization on the simulation host for each RNG run.',
         'CPU and Memory Utilization', 'Main report',
         'Edge processing required higher mean host CPU in all three pairs.'),
        ('A.1', 'p95_pipeline_latency_by_rng',
         'Official-window p95 pipeline latency by RNG run.',
         'Appendix: Tail Latency', 'Appendix',
         'Tail latency complements the median comparison and retains official-window extremes.'),
        ('A.2', 'mean_inference_time_by_rng',
         'Mean YOLO inference time by processing mode and RNG run.',
         'Appendix: Inference Timing', 'Appendix',
         'Inference time was similar between modes.'),
        ('A.3', 'mean_rss_memory_by_rng',
         'Mean monitored process RSS on the simulation host.',
         'Appendix: Host Resources', 'Appendix',
         'Memory demand was similar relative to the latency and reliability differences.'),
        ('A.4', 'ground_compression_decode_overhead_by_rng',
         'Mean JPEG compression and decoding time for Ground runs.',
         'Appendix: Image-Processing Overhead', 'Appendix',
         'Compression and decoding required only a few milliseconds.'),
        ('A.5', 'mode_processed_ratio_summary',
         'Mode mean processed-frame ratio with all three run-level observations.',
         'Appendix: Run-Level Summaries', 'Appendix',
         'Displays the run-level basis of the mode reliability summary.'),
        ('A.6', 'mode_median_latency_summary',
         'Mode mean of run median latencies with all three run-level observations.',
         'Appendix: Run-Level Summaries', 'Appendix',
         'Displays the run-level basis of the mode latency summary.'),
    ]
    rows = [{'figure': item[0], 'filename': item[1], 'caption': item[2],
             'subsection': item[3], 'placement': item[4], 'message': item[5]}
            for item in entries]
    return ('# Report figure plan\n\n'
            + markdown_table(rows, ['figure', 'filename', 'caption',
                                    'subsection', 'placement', 'message'],
                             ['Figure', 'Filename stem', 'Suggested caption',
                              'Report subsection', 'Placement', 'Main message'])
            + '\n')


def results_draft(mode_rows: list[dict[str, str]],
                  run_rows: list[dict[str, str]], combined: list[dict[str, str]],
                  comparison: dict[str, float]) -> str:
    edge, ground = combined
    ground_runs = sorted((row for row in run_rows if row['actual_mode'] == 'ground'),
                         key=lambda row: int(row['rng_run']))
    ground_completion = ', '.join(
        f'RNG {row["rng_run"]}: {100 * float(row["processed_frame_ratio"]):.2f}% '
        f'({row["official_processed_frames"]}/{row["official_sent_frames"]})'
        for row in ground_runs)
    def stat(mode: str, metric: str) -> tuple[float, float]:
        row = mode_stat(mode_rows, mode, metric)
        return float(row['mean']), float(row['sample_standard_deviation'])
    e_ratio, e_ratio_sd = stat('edge', 'processed_frame_ratio')
    g_ratio, g_ratio_sd = stat('ground', 'processed_frame_ratio')
    e_med, e_med_sd = stat('edge', 'median_latency_ms')
    g_med, g_med_sd = stat('ground', 'median_latency_ms')
    e_p95, e_p95_sd = stat('edge', 'p95_latency_ms')
    g_p95, g_p95_sd = stat('ground', 'p95_latency_ms')
    e_inf, e_inf_sd = stat('edge', 'mean_inference_ms')
    g_inf, g_inf_sd = stat('ground', 'mean_inference_ms')
    e_cpu, e_cpu_sd = stat('edge', 'cpu_mean_percent')
    g_cpu, g_cpu_sd = stat('ground', 'cpu_mean_percent')
    e_rss, e_rss_sd = stat('edge', 'rss_mean_mb')
    g_rss, g_rss_sd = stat('ground', 'rss_mean_mb')
    comp, comp_sd = stat('ground', 'mean_compression_ms')
    decode, decode_sd = stat('ground', 'mean_decode_ms')
    return f"""# Draft Results

## 4.X Experimental Configuration

Phase F compared Edge and Ground processing over three independent NS-3 RNG runs per mode. The UAV target pose, 1 Hz relay rate, 60-second official measurement interval, 0.25 detector confidence threshold, and revised startup-stabilization procedure were held constant. Ground used JPEG quality 5. Statistics were calculated per run over metadata-defined official windows and then summarized across the three runs; individual frames were not treated as independent repetitions.

## 4.X Detection Accuracy under Compression

The D4 evaluation used {edge['evaluation_frames']} manually labelled frames per representation. Edge/raw achieved precision {float(edge['precision']):.3f}, recall {float(edge['recall']):.3f}, F1 {float(edge['f1']):.3f}, and exact-count rate {float(edge['exact_count_rate']):.3f}. Ground/JPEG q5 achieved precision {float(ground['precision']):.3f}, recall {float(ground['recall']):.3f}, F1 {float(ground['f1']):.3f}, and exact-count rate {float(ground['exact_count_rate']):.3f}. Phase F detection counts are not accuracy measurements and are therefore not used to infer precision or recall.

## 4.X Processed-Frame Reliability

Edge local pipeline completion was {100*e_ratio:.2f}% ± {100*e_ratio_sd:.2f}% across the three runs. Ground complete compressed-frame reception and processing averaged {100*g_ratio:.2f}% ± {100*g_ratio_sd:.2f}%. Ground completion by session was {ground_completion}. The resulting Edge advantage in mean processed-frame completion was {comparison['edge_completion_advantage_pp']:.2f} percentage points.

## 4.X End-to-End Pipeline Latency

The mean of the three run-level median latencies was {e_med:.2f} ± {e_med_sd:.2f} ms for Edge and {g_med:.2f} ± {g_med_sd:.2f} ms for Ground. Ground therefore exceeded Edge by {comparison['ground_minus_edge_median_latency_ms']:.2f} ms and was {comparison['ground_to_edge_median_latency_ratio']:.2f} times the Edge value. Mean p95 latency was {e_p95:.2f} ± {e_p95_sd:.2f} ms for Edge and {g_p95:.2f} ± {g_p95_sd:.2f} ms for Ground. Median and p95 values describe different parts of the latency distribution, and official-window tail observations were retained.

## 4.X Inference and Image-Processing Overhead

Mean YOLO inference time was similar between modes: {e_inf:.2f} ± {e_inf_sd:.2f} ms for Edge and {g_inf:.2f} ± {g_inf_sd:.2f} ms for Ground. Ground JPEG compression averaged {comp:.2f} ± {comp_sd:.2f} ms and decoding averaged {decode:.2f} ± {decode_sd:.2f} ms, giving a combined mean overhead of {comparison['ground_mean_compression_decode_ms']:.2f} ms.

## 4.X CPU and Memory Utilization

Mean monitored CPU utilization on the simulation host was {e_cpu:.2f}% ± {e_cpu_sd:.2f}% for Edge and {g_cpu:.2f}% ± {g_cpu_sd:.2f}% for Ground. Edge was higher by {comparison['edge_minus_ground_mean_cpu_pp']:.2f} percentage points. Mean RSS was {e_rss:.2f} ± {e_rss_sd:.2f} MB for Edge and {g_rss:.2f} ± {g_rss_sd:.2f} MB for Ground, a Ground-minus-Edge difference of {comparison['ground_minus_edge_mean_rss_mb']:.2f} MB.

## 4.X Summary of Edge-versus-Ground Results

Under the tested configuration, Edge provided complete local frame processing and lower run-level median latency, while Ground used less host CPU but processed fewer complete frames. D4 further showed lower recall, F1, and exact-count rate for JPEG-quality-5 imagery than for raw imagery. These measurements describe separate accuracy and live-system evaluations and are not pooled into a single accuracy measure.
"""


def discussion_draft(combined: list[dict[str, str]],
                     comparison: dict[str, float]) -> str:
    edge, ground = combined
    return f"""# Draft Discussion

## Reliability and latency

Edge processing completed every locally published official frame under the tested configuration. This is consistent with the image stream remaining within the local Edge pipeline and avoiding complete-image delivery across the emulated wireless path. Ground processing achieved a mean completion ratio of {100*float(ground['mean_processed_frame_ratio']):.1f}%, indicating that some compressed frames were not received and processed completely. The longer Ground inter-arrival gaps are therefore treated as delivery-performance observations rather than provenance failures.

Ground's mean of run median latencies was {comparison['ground_to_edge_median_latency_ratio']:.2f} times the Edge value. JPEG compression and decoding together averaged only {comparison['ground_mean_compression_decode_ms']:.2f} ms, which suggests that these operations alone do not explain the {comparison['ground_minus_edge_median_latency_ms']:.2f} ms median-latency difference. The result is consistent with additional transport, scheduling, middleware, and pipeline waiting contributions. The available `wireless_transit_ms` field is not guaranteed to isolate pure radio propagation delay, so the latency gap should not be assigned to a single component.

## Detection accuracy and computation

YOLO inference time remained similar between modes, indicating that the inference workload itself was broadly comparable after an image reached the detector. However, D4 showed that JPEG quality 5 reduced recall from {float(edge['recall']):.3f} to {float(ground['recall']):.3f} and F1 from {float(edge['f1']):.3f} to {float(ground['f1']):.3f}. The Ground precision of {float(ground['precision']):.3f} occurred alongside low recall and does not indicate superior overall detection performance; it reflects few false positives among a much smaller set of detected positives.

## Reliability-resource trade-off

Edge required {comparison['edge_minus_ground_mean_cpu_pp']:.2f} percentage points more mean CPU on the simulation host. This indicates a compute-cost trade-off for its higher local completion and lower latency, but it is not a measurement of physical UAV energy use. Ground mean RSS exceeded Edge by only {comparison['ground_minus_edge_mean_rss_mb']:.2f} MB, suggesting relatively similar memory demand compared with the larger reliability and latency differences.

Overall, the results support Edge processing for time-sensitive perception under the tested configuration. Ground processing may remain suitable where aerial compute is constrained, communication conditions reliably support complete-frame delivery, sufficient image quality can be maintained, or centralized processing offers operational benefits. These findings do not establish universal superiority for either architecture.
"""


def limitations_draft() -> str:
    return """# Draft Limitations

The experiment used only three independent repetitions per processing mode, limiting the strength and generalizability of cross-mode comparisons. Testing used one fixed UAV position and externally commanded yaw, one scene, and one person-detection workload. The final Ground comparison used only JPEG quality 5 and one configured Wi-Fi scenario; other compression levels, traffic loads, channel conditions, and mobility patterns may produce different results.

The evaluation used SITL and network simulation rather than physical UAV flight. Edge and Ground processing were represented with namespaces on the same simulation host. CPU and RSS therefore measure simulation-host process demand, not onboard processor utilization, physical energy use, or battery endurance. No direct energy or battery measurement was collected. Furthermore, `wireless_transit_ms` is not guaranteed to represent pure radio propagation delay because the recorded interval may include other pipeline and middleware effects.

Phase D4 used a limited manually labelled image set, so its accuracy estimates may not represent other scenes, subjects, illumination, or viewpoints. The RNG3 Run IDs did not match their actual processing modes; however, actual mode was independently confirmed from detector, camera-relay, and metrics logs before analysis. These naming inconsistencies were retained transparently in the provenance record.
"""


def conclusion_draft(combined: list[dict[str, str]],
                     comparison: dict[str, float]) -> str:
    edge, ground = combined
    return f"""# Draft Conclusion

Under the tested configuration, Edge processing completed 100% of locally published official frames, whereas Ground completed approximately {100*float(ground['mean_processed_frame_ratio']):.1f}% of compressed frames on average. Edge also achieved substantially lower run-level median pipeline latency: the Ground mean was {comparison['ground_to_edge_median_latency_ratio']:.2f} times the Edge mean. YOLO inference time was similar between modes, and the {comparison['ground_mean_compression_decode_ms']:.2f} ms mean compression-plus-decoding overhead was insufficient by itself to account for the observed latency gap.

The D4 results showed that JPEG quality 5 substantially reduced recall and F1 relative to raw imagery, despite retaining high precision. Edge required higher mean CPU utilization on the simulation host, while RSS remained comparatively similar. These results indicate that Edge processing is the preferred architecture for latency-sensitive perception under the evaluated conditions. Ground processing may remain appropriate where aerial compute resources are constrained and communication reliability and image quality are sufficient for the application.
"""


def key_finding(combined: list[dict[str, str]], comparison: dict[str, float]) -> str:
    edge, ground = combined
    return (f'Across three independent runs per mode, Edge processing completed 100% '
            f'of locally published official frames, compared with '
            f'{100*float(ground["mean_processed_frame_ratio"]):.1f}% mean complete-frame '
            f'processing for Ground. The mean of run-level median latency was '
            f'{float(edge["mean_of_run_median_latency_ms"]):.1f} ms for Edge and '
            f'{float(ground["mean_of_run_median_latency_ms"]):.1f} ms for Ground, while '
            f'YOLO inference time remained similar. Ground JPEG compression and decoding '
            f'averaged only {comparison["ground_mean_compression_decode_ms"]:.1f} ms '
            f'combined. In the separately labelled D4 evaluation, JPEG quality 5 reduced '
            f'recall from {float(edge["recall"]):.3f} to {float(ground["recall"]):.3f} '
            f'and F1 from {float(edge["f1"]):.3f} to {float(ground["f1"]):.3f}. Edge '
            f'required higher simulation-host CPU, indicating a resource trade-off. '
            f'Under the tested conditions, Edge was preferable for latency-sensitive '
            f'perception, while Ground remains potentially suitable when communication '
            f'and image quality are sufficient or aerial compute is constrained.\n')


def value_sources() -> str:
    rows = [
        ('D4 precision, recall, F1, exact-count rate, frames', D4.relative_to(REPO),
         '`precision`, `recall`, `f1`, `exact_count_accuracy`, `frames`',
         'Rows `edge_raw` and `ground_q5`.'),
        ('Phase F per-RNG completion and official counts', RUNS.relative_to(REPO),
         '`processed_frame_ratio`, `official_processed_frames`, `official_sent_frames`',
         'One row per selected run.'),
        ('Mode completion mean and sample SD', MODES.relative_to(REPO),
         '`mean`, `sample_standard_deviation` where `metric=processed_frame_ratio`',
         'Three run-level values per mode.'),
        ('Median and p95 latency mean and sample SD', MODES.relative_to(REPO),
         '`mean`, `sample_standard_deviation` for `median_latency_ms` and `p95_latency_ms`',
         'Three run-level statistics per mode; no frame pooling.'),
        ('Inference, detection count, CPU, RSS, compression, decode', MODES.relative_to(REPO),
         '`mean`, `sample_standard_deviation` for the corresponding metric rows',
         'Three run-level values per mode.'),
        ('Within-RNG paired differences', PAIRS.relative_to(REPO),
         '`edge_*`, `ground_*`, and `ground_minus_edge_*` columns',
         'One paired row per RNG.'),
        ('Edge completion advantage', MODES.relative_to(REPO),
         '`mean` for `processed_frame_ratio`',
         '100 × (Edge mean − Ground mean).'),
        ('Ground-to-Edge latency ratio and latency difference', MODES.relative_to(REPO),
         '`mean` for `median_latency_ms`',
         'Ground mean ÷ Edge mean; Ground mean − Edge mean.'),
        ('Accuracy differences', D4.relative_to(REPO),
         '`recall`, `f1`, `exact_count_accuracy`',
         'Edge/raw value − Ground/q5 value.'),
        ('CPU and RSS differences', MODES.relative_to(REPO),
         '`mean` for `cpu_mean_percent` and `rss_mean_mb`',
         'Edge CPU − Ground CPU; Ground RSS − Edge RSS.'),
        ('Ground compression-plus-decode overhead', MODES.relative_to(REPO),
         '`mean` for `mean_compression_ms` and `mean_decode_ms`',
         'Sum of Ground run-level means.'),
    ]
    mapped = [{'reported_value': item[0], 'source_file': str(item[1]),
               'source_columns': item[2], 'derivation': item[3]} for item in rows]
    return ('# Report value sources\n\n'
            + markdown_table(mapped, ['reported_value', 'source_file',
                                      'source_columns', 'derivation']) + '\n')


def main() -> None:
    d4_rows = read_csv(D4)
    run_rows = read_csv(RUNS)
    mode_rows = read_csv(MODES)
    pair_rows = read_csv(PAIRS)
    if len(d4_rows) != 2 or len(run_rows) != 6 or len(pair_rows) != 3:
        raise ValueError('Unexpected D4 or Phase F source row count')
    combined = build_combined(d4_rows, mode_rows)
    comparison = comparisons(combined)
    combined_fields = list(combined[0])
    write_csv(PROCESSED / 'combined_d4_phase_f_results.csv',
              combined_fields, combined)
    (PROCESSED / 'combined_d4_phase_f_results.md').write_text(
        combined_markdown(combined, comparison))
    plt.rcParams.update({
        'font.family': 'DejaVu Sans', 'font.size': 10,
        'axes.labelsize': 11, 'axes.titlesize': 12,
        'xtick.labelsize': 9, 'ytick.labelsize': 10,
        'legend.fontsize': 9, 'pdf.fonttype': 42, 'ps.fonttype': 42,
    })
    d4_figure(d4_rows)
    (PROCESSED / 'report_figure_plan.md').write_text(figure_plan())
    (PROCESSED / 'report_results_draft.md').write_text(
        results_draft(mode_rows, run_rows, combined, comparison))
    (PROCESSED / 'report_discussion_draft.md').write_text(
        discussion_draft(combined, comparison))
    (PROCESSED / 'report_limitations_draft.md').write_text(limitations_draft())
    (PROCESSED / 'report_conclusion_draft.md').write_text(
        conclusion_draft(combined, comparison))
    (PROCESSED / 'key_finding.txt').write_text(key_finding(combined, comparison))
    (PROCESSED / 'report_value_sources.md').write_text(value_sources())
    print('Generated combined D4/Phase F report materials.')


if __name__ == '__main__':
    main()
