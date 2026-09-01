#!/usr/bin/env python3
"""Inspect both datasets and document schemas before comparison."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from common import (COLUMN_MAPPING, add_common_args, determine_alignment, number,
                    prepare_output, read_json, read_rows, validate_inputs, write_csv,
                    write_json)


TIME_CANDIDATES = ("elapsed_s", "elapsed_time_s", "log_timestamp_unix_s", "wall_time_unix_s")


def infer_column(values: list[str]) -> tuple[str, int, float | None, float | None]:
    nonempty = [value for value in values if value not in (None, "")]
    numeric = [number(value) for value in nonempty]
    valid = [value for value in numeric if math.isfinite(value)]
    dtype = "empty" if not nonempty else ("numeric" if len(valid) == len(nonempty) else "string")
    return dtype, len(values) - len(nonempty), (min(valid) if valid else None), (max(valid) if valid else None)


def inspect_directory(directory: Path, label: str) -> tuple[list[dict], list[str]]:
    output, report = [], []
    for path in sorted(directory.iterdir()):
        if path.suffix.lower() == ".csv":
            rows = read_rows(path)
            columns = list(rows[0]) if rows else []
            types, missing, ranges = {}, {}, {}
            for column in columns:
                dtype, absent, minimum, maximum = infer_column([row.get(column, "") for row in rows])
                types[column], missing[column] = dtype, absent
                if minimum is not None:
                    ranges[column] = {"min": minimum, "max": maximum}
            time_col = next((column for column in TIME_CANDIDATES if column in columns), "")
            times = np.asarray([number(row.get(time_col)) for row in rows]) if time_col else np.asarray([])
            times = times[np.isfinite(times)]
            first = float(np.min(times)) if len(times) else None
            last = float(np.max(times)) if len(times) else None
            duration = last - first if first is not None and last is not None else None
            rate = (len(times) - 1) / duration if duration and len(times) > 1 else None
            duplicates = int(len(times) - len(np.unique(times))) if len(times) else 0
            output.append({
                "filename": path.name, "file_type": "CSV", "rows": len(rows),
                "columns": json.dumps(columns), "inferred_types": json.dumps(types, sort_keys=True),
                "time_column": time_col, "first_timestamp": first, "last_timestamp": last,
                "duration_s": duration, "approx_sample_rate_hz": rate,
                "duplicate_timestamps": duplicates,
                "missing_values_by_column": json.dumps(missing, sort_keys=True),
                "numeric_ranges": json.dumps(ranges, sort_keys=True),
            })
            report.append(f"{path.name}: {len(rows)} rows, {len(columns)} columns, time={time_col or 'none'}, duration={duration}, rate={rate}, duplicate timestamps={duplicates}")
        elif path.suffix.lower() == ".json":
            data = read_json(path)
            output.append({
                "filename": path.name, "file_type": "JSON", "rows": "",
                "columns": json.dumps(list(data)), "inferred_types": "", "time_column": "",
                "first_timestamp": "", "last_timestamp": "", "duration_s": "",
                "approx_sample_rate_hz": "", "duplicate_timestamps": "",
                "missing_values_by_column": "", "numeric_ranges": "",
            })
            report.append(f"{path.name}: JSON metadata keys={list(data)}")
    return output, report


def run(real_dir: Path, sim_dir: Path, output: Path) -> None:
    fields = ["filename", "file_type", "rows", "columns", "inferred_types", "time_column",
              "first_timestamp", "last_timestamp", "duration_s", "approx_sample_rate_hz",
              "duplicate_timestamps", "missing_values_by_column", "numeric_ranges"]
    real_info, real_report = inspect_directory(real_dir, "real")
    sim_info, sim_report = inspect_directory(sim_dir, "simulation")
    write_csv(output / "inspection" / "real_files.csv", fields, real_info)
    write_csv(output / "inspection" / "sim_files.csv", fields, sim_info)

    mapping_rows = [{"signal": signal, **mapping} for signal, mapping in COLUMN_MAPPING.items()]
    write_csv(output / "inspection" / "schema_comparison.csv",
              ["signal", "real", "simulation", "status"], mapping_rows)
    write_json(output / "inspection" / "column_mapping.json", COLUMN_MAPPING)
    grouped = {status: [] for status in ("DIRECTLY_COMPARABLE", "COMPARABLE_AFTER_CONVERSION", "SIM_ONLY", "REAL_ONLY", "INSUFFICIENT_DATA")}
    for signal, mapping in COLUMN_MAPPING.items():
        grouped[mapping["status"]].append({"signal": signal, "real": mapping["real"], "simulation": mapping["simulation"]})
    write_json(output / "inspection" / "comparable_signals.json", grouped)

    alignment = determine_alignment(real_dir, sim_dir)
    write_json(output / "summary" / "time_alignment.json", alignment)
    real_meta, sim_meta = read_json(real_dir / "metadata.json"), read_json(sim_dir / "metadata.json")
    lines = [
        "REAL VS SIMULATION DATA QUALITY REPORT", "=" * 42, "",
        f"Real directory: {real_dir}", f"Simulation directory: {sim_dir}", "",
        "REAL FILES", "----------", *real_report, "", "SIMULATION FILES", "----------------", *sim_report, "",
        "CLOCKS AND FRAMES", "-----------------",
        f"Real primary clock: {real_meta['log_timeline']['primary_clock']}",
        "Simulation primary comparison clock: logger wall-time callback arrival / elapsed_time_s.",
        f"Real local frame metadata: {real_meta['coordinate_frame']}",
        f"Simulation frame metadata: {sim_meta['coordinate_frames']}", "",
        "ALIGNMENT", "---------", json.dumps(alignment, indent=2), "",
        "QUALITY/COMPARABILITY FINDINGS", "------------------------------",
        "- GPS latitude/longitude, relative altitude, measured groundspeed, GPS heading, and application telemetry interarrival are comparable after time alignment.",
        "- Both trajectories must be transformed from latitude/longitude into one common ENU frame; their recorded local frames have different origins/conventions.",
        "- Real vehicle_state.csv is asynchronous by source_message; blank fields are structural, not missing synchronized measurements.",
        "- The real flight is an AUTO mission with multiple mission items; the simulation is one GUIDED target. Only their common final geographic target is directly comparable.",
        "- Simulation SNR/signal are simulation-only. Three real RADIO/RADIO_STATUS rows are insufficient and are raw indicators, not verified dBm/SNR.",
        "- Simulation MAVLink sequence numbers are unavailable, so sequence-loss estimates cannot be compared.",
        "- Neither dataset contains defensible application packet loss, RTT, or throughput measurements.",
    ]
    (output / "inspection" / "data_quality_report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    add_common_args(parser)
    args = parser.parse_args()
    real_dir, sim_dir = validate_inputs(args.real_dir, args.sim_dir)
    output = prepare_output(args.output_dir)
    run(real_dir, sim_dir, output)
    print(f"Inspection outputs: {output / 'inspection'}")


if __name__ == "__main__":
    main()
