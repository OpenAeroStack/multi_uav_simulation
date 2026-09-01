#!/usr/bin/env python3
"""Generate FYP validation charts and a summary from the organized CSVs."""

import csv
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


RESULTS_DIR = Path(__file__).resolve().parent
PRIMARY_REAL_CSV = RESULTS_DIR / 'real_dropout_vs_distance_PRIMARY.csv'
SUPPLEMENTARY_REAL_CSV = (
    RESULTS_DIR
    / 'real_dropout_vs_distance_SUPPLEMENTARY_different_flight_0-110m.csv'
)
SIM_CSV = RESULTS_DIR / 'sim_snr_signal_vs_distance_0-28m.csv'
SNR_THRESHOLD_DB = 12.0

PRIMARY_REAL_TITLE = (
    'Real Flight: Message Dropout vs Distance '
    '(0-30m, source flight for simulated replica)'
)
SIM_SNR_TITLE = 'Simulated SNR vs Distance (along real flight path replica)'
SIM_SIGNAL_TITLE = 'Simulated Received Signal Strength vs Distance'
COMBINED_TITLE = (
    'Real Flight vs Simulated Link Prediction — Same Source Flight, 0-30m'
)
THRESHOLD_LABEL = 'Typical WiFi minimum reliable SNR (~12dB)'


def load_numeric_csv(path: Path, required_columns: set[str]) -> list[dict[str, float]]:
    if not path.is_file():
        raise FileNotFoundError(f'Required input CSV not found: {path}')

    with path.open(newline='', encoding='utf-8') as csv_file:
        reader = csv.DictReader(csv_file)
        actual_columns = set(reader.fieldnames or [])
        missing = required_columns - actual_columns
        if missing:
            raise ValueError(
                f'{path.name} is missing columns: {", ".join(sorted(missing))}')

        rows = []
        for line_number, row in enumerate(reader, start=2):
            try:
                rows.append({key: float(value) for key, value in row.items()})
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f'Non-numeric value in {path.name}, line {line_number}: {exc}') from exc

    if not rows:
        raise ValueError(f'Input CSV contains no data rows: {path}')
    return sorted(rows, key=lambda row: row['distance_bin_m'])


def style_distance_axis(axis, *, x_max: float = 30.0) -> None:
    axis.set_xlabel('Distance bin (m)')
    axis.set_xlim(0, x_max)
    axis.set_xticks(list(range(0, int(x_max) + 1, 5)))
    axis.grid(True, linestyle=':', linewidth=0.8, alpha=0.65)


def plot_real_dropout(axis, distances, dropout_pct, *, title: str) -> None:
    axis.plot(
        distances,
        dropout_pct,
        color='#1f77b4',
        marker='o',
        linewidth=2.2,
        markersize=6,
        label='Measured application-message dropout',
    )
    axis.set_title(title)
    axis.set_ylabel('Message dropout (%)')
    axis.set_ylim(-0.05, max(1.0, max(dropout_pct) * 1.15))
    style_distance_axis(axis)
    axis.legend(loc='upper left')


def plot_sim_snr(axis, distances, snr_db, *, title: str) -> None:
    axis.plot(
        distances,
        snr_db,
        color='#d62728',
        marker='o',
        linewidth=2.2,
        markersize=6,
        label='Simulated average SNR',
    )
    axis.axhline(
        SNR_THRESHOLD_DB,
        color='#444444',
        linestyle='--',
        linewidth=1.5,
        label=THRESHOLD_LABEL,
    )
    axis.set_title(title)
    axis.set_ylabel('Average SNR (dB)')
    axis.set_ylim(0, max(snr_db) * 1.12)
    style_distance_axis(axis)
    axis.legend(loc='best')


def save_figure(figure, filename: str) -> None:
    output_path = RESULTS_DIR / filename
    figure.savefig(output_path, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close(figure)
    print(f'Wrote {output_path}')


def write_summary(
    primary_rows,
    supplementary_rows,
    sim_rows,
) -> None:
    primary_dropout_pct = [row['dropout_ratio'] * 100.0 for row in primary_rows]
    supplementary_dropout_pct = [
        row['dropout_ratio'] * 100.0 for row in supplementary_rows
    ]
    sim_snr = [row['avg_snr_db'] for row in sim_rows]

    max_primary_dropout = max(primary_dropout_pct)
    max_supplementary_dropout = max(supplementary_dropout_pct)
    min_sim_snr = min(sim_snr)
    snr_at_max_distance = sim_rows[-1]['avg_snr_db']
    margin_at_max_distance = snr_at_max_distance - SNR_THRESHOLD_DB

    summary = f"""# Validation Summary

| Metric | Real Flight (Primary, 0-30m) | Simulation (0-28m) |
|---|---:|---:|
| Distance range tested | 0-30 m | 0-28 m |
| Max dropout observed | {max_primary_dropout:.2f}% | Excluded — packet counts are dominated by low-level 802.11 traffic |
| Min SNR observed | Not measured | {min_sim_snr:.2f} dB |
| SNR margin above 12dB threshold at max tested distance | Not measured | {margin_at_max_distance:.2f} dB ({snr_at_max_distance:.2f} dB − 12 dB) |
| One-line conclusion | No application-message dropout was observed over the replicated flight range. | Simulated SNR remained comfortably above the reference threshold over the same path. |

## Data source

Both primary datasets derive from the same real-world flight, `2026-08-31 18-03-14.tlog`. The real result comes from direct telemetry-log analysis. The simulated result comes from replaying that flight's exact GPS path through Gazebo, ArduPilot SITL, and ns-3.

## Supplementary data

`real_dropout_vs_distance_SUPPLEMENTARY_different_flight_0-110m.csv` is from a different flight and day. It is included only as supplementary context showing that the real-world link also stayed clean over a longer range; it is not part of the primary validation claim. Its maximum calculated dropout was {max_supplementary_dropout:.2f}% (the non-zero maximum occurs in the sparsely sampled 110 m bin).

## Limitations

- Simulated SNR and signal values assume 5180 MHz WiFi PHY parameters that were not independently verified against the real drone's actual radio hardware.
- Simulated dropout-rate figures were dominated by low-level 802.11 protocol traffic rather than application-layer telemetry and were therefore excluded from this comparison.
"""

    output_path = RESULTS_DIR / 'summary_table.md'
    output_path.write_text(summary, encoding='utf-8')
    print(f'Wrote {output_path}')


def main() -> None:
    primary_rows = load_numeric_csv(
        PRIMARY_REAL_CSV, {'distance_bin_m', 'dropout_ratio'})
    supplementary_rows = load_numeric_csv(
        SUPPLEMENTARY_REAL_CSV, {'distance_bin_m', 'dropout_ratio'})
    sim_rows = load_numeric_csv(
        SIM_CSV, {'distance_bin_m', 'avg_signal_dbm', 'avg_snr_db'})

    real_distance = [row['distance_bin_m'] for row in primary_rows]
    real_dropout_pct = [row['dropout_ratio'] * 100.0 for row in primary_rows]
    sim_distance = [row['distance_bin_m'] for row in sim_rows]
    sim_snr = [row['avg_snr_db'] for row in sim_rows]
    sim_signal = [row['avg_signal_dbm'] for row in sim_rows]

    figure, axis = plt.subplots(figsize=(10.5, 6.0), constrained_layout=True)
    plot_real_dropout(
        axis, real_distance, real_dropout_pct, title=PRIMARY_REAL_TITLE)
    save_figure(figure, 'real_dropout_vs_distance.png')

    figure, axis = plt.subplots(figsize=(10.5, 6.0), constrained_layout=True)
    plot_sim_snr(axis, sim_distance, sim_snr, title=SIM_SNR_TITLE)
    save_figure(figure, 'sim_snr_vs_distance.png')

    figure, axis = plt.subplots(figsize=(10.5, 6.0), constrained_layout=True)
    axis.plot(
        sim_distance,
        sim_signal,
        color='#2ca02c',
        marker='o',
        linewidth=2.2,
        markersize=6,
        label='Simulated average received signal',
    )
    axis.set_title(SIM_SIGNAL_TITLE)
    axis.set_ylabel('Average received signal strength (dBm)')
    style_distance_axis(axis)
    axis.legend(loc='best')
    save_figure(figure, 'sim_signal_vs_distance.png')

    figure, axes = plt.subplots(1, 2, figsize=(14.5, 5.6), constrained_layout=True)
    plot_real_dropout(
        axes[0], real_distance, real_dropout_pct,
        title='Measured dropout — source flight')
    plot_sim_snr(
        axes[1], sim_distance, sim_snr,
        title='Simulated SNR — replicated path')
    figure.suptitle(COMBINED_TITLE, fontsize=15, fontweight='bold')
    save_figure(figure, 'combined_comparison.png')

    write_summary(primary_rows, supplementary_rows, sim_rows)


if __name__ == '__main__':
    main()
