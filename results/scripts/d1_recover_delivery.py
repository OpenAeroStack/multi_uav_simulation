#!/usr/bin/env python3
"""
d1_recover_delivery.py — recovers the REAL delivery-vs-quality curve from
raw TAP snapshots already saved by d1_bandwidth_measurement.sh runs.

d1_bandwidth_measurement.sh's summary CSV only reports what was SENT
(tap-uav1 TX bytes) — that's offered bandwidth, not delivered bandwidth.
But every run's raw before/after snapshots already captured BOTH interfaces
in BOTH directions, so the real delivery ratio (tap-gcs RX vs tap-uav1 TX)
can be computed with zero new flying/testing — just re-reading files
already on disk.

Usage:
    python3 d1_recover_delivery.py [results_root]

Finds every tap_before_<label>_<timestamp>.txt / tap_after_<label>_<timestamp>.txt
pair in phase_d_application/raw/, pairs them by filename, and computes real
byte-level and packet-level delivery ratios for each.
"""
import glob
import os
import re
import sys
import csv


def parse_link_stats(text, iface):
    marker = f"--- {iface} ---"
    if marker not in text:
        return None
    block = text.split(marker, 1)[1]
    block = block.split("--- ", 1)[0]
    lines = block.strip().split("\n")
    stats = {}
    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith("RX:") or s.startswith("TX:"):
            direction = "rx" if s.startswith("RX:") else "tx"
            labels = s.replace(f"{direction.upper()}:", "").split()
            if i + 1 >= len(lines):
                continue
            values = lines[i + 1].strip().split()
            if len(labels) != len(values):
                print(f"  WARNING: {iface} {direction.upper()} label/value "
                      f"mismatch in this file — skipping that block.",
                      file=sys.stderr)
                continue
            for lab, val in zip(labels, values):
                try:
                    stats[f"{direction}_{lab}"] = int(val)
                except ValueError:
                    pass
    return stats


def main():
    results_root = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
        "~/FYP/multi_uav_simulation/results")
    raw_dir = os.path.join(results_root, "phase_d_application", "raw")
    processed_dir = os.path.join(results_root, "phase_d_application", "processed")
    os.makedirs(processed_dir, exist_ok=True)

    before_files = sorted(glob.glob(os.path.join(raw_dir, "tap_before_*.txt")))
    if not before_files:
        print(f"No tap_before_*.txt files found in {raw_dir}", file=sys.stderr)
        sys.exit(1)

    results = []
    for before_path in before_files:
        fname = os.path.basename(before_path)
        # tap_before_<label>_<YYYYMMDD>_<HHMMSS>.txt -> find matching "after"
        m = re.match(r"tap_before_(.+)_(\d{8}_\d{6})\.txt", fname)
        if not m:
            print(f"  Skipping unrecognised filename: {fname}", file=sys.stderr)
            continue
        label, ts = m.group(1), m.group(2)
        after_path = os.path.join(raw_dir, f"tap_after_{label}_{ts}.txt")
        if not os.path.isfile(after_path):
            print(f"  No matching 'after' file for {fname} — skipping.",
                  file=sys.stderr)
            continue

        before_text = open(before_path).read()
        after_text = open(after_path).read()

        uav1_before = parse_link_stats(before_text, "tap-uav1")
        uav1_after = parse_link_stats(after_text, "tap-uav1")
        gcs_before = parse_link_stats(before_text, "tap-gcs")
        gcs_after = parse_link_stats(after_text, "tap-gcs")

        required = [uav1_before, uav1_after, gcs_before, gcs_after]
        if any(s is None for s in required):
            print(f"  Incomplete stats for {label} ({ts}) — skipping.",
                  file=sys.stderr)
            continue

        sent_bytes = uav1_after["tx_bytes"] - uav1_before["tx_bytes"]
        sent_pkts = uav1_after.get("tx_packets", 0) - uav1_before.get("tx_packets", 0)
        arrived_bytes = gcs_after["rx_bytes"] - gcs_before["rx_bytes"]
        arrived_pkts = gcs_after.get("rx_packets", 0) - gcs_before.get("rx_packets", 0)

        if sent_bytes <= 0:
            print(f"  {label} ({ts}): zero/negative sent bytes — skipping "
                  f"(likely not a video-traffic run).", file=sys.stderr)
            continue

        byte_delivery_pct = 100.0 * arrived_bytes / sent_bytes
        pkt_delivery_pct = (100.0 * arrived_pkts / sent_pkts) if sent_pkts > 0 else float("nan")

        results.append({
            "label": label, "timestamp": ts,
            "sent_bytes": sent_bytes, "sent_pkts": sent_pkts,
            "arrived_bytes": arrived_bytes, "arrived_pkts": arrived_pkts,
            "byte_delivery_pct": byte_delivery_pct,
            "pkt_delivery_pct": pkt_delivery_pct,
        })

    if not results:
        print("No usable before/after pairs found.", file=sys.stderr)
        sys.exit(1)

    print(f"{'label':<20} {'sent(B)':>12} {'arrived(B)':>12} "
          f"{'byte deliv%':>12} {'pkt deliv%':>11}")
    for r in results:
        print(f"{r['label']:<20} {r['sent_bytes']:>12} {r['arrived_bytes']:>12} "
              f"{r['byte_delivery_pct']:>11.2f}% {r['pkt_delivery_pct']:>10.2f}%")

    out_csv = os.path.join(processed_dir, "d1_recovered_delivery.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "timestamp", "sent_bytes", "sent_pkts",
                    "arrived_bytes", "arrived_pkts", "byte_delivery_pct",
                    "pkt_delivery_pct"])
        for r in results:
            w.writerow([r["label"], r["timestamp"], r["sent_bytes"],
                        r["sent_pkts"], r["arrived_bytes"], r["arrived_pkts"],
                        f"{r['byte_delivery_pct']:.2f}",
                        f"{r['pkt_delivery_pct']:.2f}"])
    print(f"\nSaved: {out_csv}")


if __name__ == "__main__":
    main()