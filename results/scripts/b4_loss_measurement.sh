#!/bin/bash
# b4_loss_measurement.sh — Phase B4: packet loss vs distance, using real
# kernel TAP packet counters as an independent cross-check against iperf3's
# own self-reported loss numbers.
#
# Usage:
#   bash b4_loss_measurement.sh <label> [offered_rate] [duration_sec]
#
# Example (run at each of your 5 flyer-script points):
#   bash b4_loss_measurement.sh d01_58_5m 2M 10
#   bash b4_loss_measurement.sh d01_69_5m 2M 10
#   ...
#
# 2M matches the sub-ceiling rate from B3 that first revealed the baseline
# ~11% loss — using the same rate here lets you see directly whether that
# baseline loss varies with distance or stays flat like everything else in
# Phase B has so far.
#
# Run the iperf3 server in uav1ns BEFORE this (once, left running for the
# whole session):
#   sudo ip netns exec uav1ns iperf3 -s

set -uo pipefail

LABEL="${1:?usage: b4_loss_measurement.sh <label> [offered_rate] [duration_sec]}"
RATE="${2:-2M}"
DURATION="${3:-10}"
RESULTS_ROOT="${4:-$HOME/FYP/multi_uav_simulation/results}"

OUTDIR="$RESULTS_ROOT/phase_b_network"
mkdir -p "$OUTDIR/raw" "$OUTDIR/processed"

RAW_BEFORE="$OUTDIR/raw/tap_before_${LABEL}_$(date +%Y%m%d_%H%M%S).txt"
RAW_AFTER="$OUTDIR/raw/tap_after_${LABEL}_$(date +%Y%m%d_%H%M%S).txt"
RAW_IPERF="$OUTDIR/raw/iperf_${LABEL}_$(date +%Y%m%d_%H%M%S).txt"

echo "=== B4 loss measurement: $LABEL (offered=$RATE, duration=${DURATION}s) ==="

echo "Snapshotting TAP counters (before)..."
{
    echo "--- tap-gcs ---"
    sudo ip -s link show tap-gcs
    echo "--- tap-uav1 ---"
    sudo ip -s link show tap-uav1
} > "$RAW_BEFORE"

echo "Running iperf3 UDP test..."
sudo ip netns exec gcsns iperf3 -c 10.42.0.11 -u -b "$RATE" -t "$DURATION" \
    2>&1 | tee "$RAW_IPERF"

echo "Snapshotting TAP counters (after)..."
{
    echo "--- tap-gcs ---"
    sudo ip -s link show tap-gcs
    echo "--- tap-uav1 ---"
    sudo ip -s link show tap-uav1
} > "$RAW_AFTER"

python3 - "$LABEL" "$RATE" "$DURATION" "$RAW_BEFORE" "$RAW_AFTER" "$RAW_IPERF" "$OUTDIR/processed" << 'PYEOF'
import sys, re, os, csv

label, rate, duration, before_path, after_path, iperf_path, processed_dir = sys.argv[1:8]

def parse_link_stats(text, iface):
    """Extract the block for a specific interface, then parse RX/TX
    label lines matched by name (robust to column-order differences).
    Fails loudly (returns None, caller must check) on any label/value
    count mismatch rather than silently mis-pairing them — a silent
    mismatch here would produce a wrong-but-plausible-looking number,
    which is worse than an obvious failure."""
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
                print(f"WARNING: {iface} {direction.upper()} header found but "
                      f"no value line follows — skipping.", file=sys.stderr)
                continue
            values = lines[i+1].strip().split()
            if len(labels) != len(values):
                print(f"WARNING: {iface} {direction.upper()} label/value count "
                      f"mismatch ({len(labels)} labels vs {len(values)} values) "
                      f"— 'ip -s link show' output format may have changed. "
                      f"Refusing to guess the pairing; this interface's stats "
                      f"are INCOMPLETE for this snapshot.", file=sys.stderr)
                continue
            for lab, val in zip(labels, values):
                try:
                    stats[f"{direction}_{lab}"] = int(val)
                except ValueError:
                    print(f"WARNING: {iface} {direction}_{lab} value {val!r} "
                          f"is not an integer — skipped.", file=sys.stderr)
    return stats

before_text = open(before_path).read()
after_text = open(after_path).read()

gcs_before = parse_link_stats(before_text, "tap-gcs")
gcs_after  = parse_link_stats(after_text,  "tap-gcs")
uav1_before = parse_link_stats(before_text, "tap-uav1")
uav1_after  = parse_link_stats(after_text,  "tap-uav1")

required = [
    (gcs_before, "tx_packets", "tap-gcs before"),
    (gcs_after, "tx_packets", "tap-gcs after"),
    (uav1_before, "rx_packets", "tap-uav1 before"),
    (uav1_after, "rx_packets", "tap-uav1 after"),
]
missing = [f"{name} ({field})" for stats, field, name in required
           if stats is None or field not in stats]
if missing:
    print(f"\nERROR: required stats missing/unparseable: {', '.join(missing)}",
          file=sys.stderr)
    print("Refusing to compute a loss percentage from incomplete data. "
          "Check the raw before/after files and the warnings above.",
          file=sys.stderr)
    sys.exit(1)

# Traffic direction for this test: iperf3 client in gcsns -> server in
# uav1ns. Packets LEAVE the root/gcsns side via tap-gcs TX, and ARRIVE at
# uav1ns side via tap-uav1 RX. That's the real end-to-end path through ns-3.
tx_packets_delta = gcs_after.get("tx_packets", 0) - gcs_before.get("tx_packets", 0)
rx_packets_delta = uav1_after.get("rx_packets", 0) - uav1_before.get("rx_packets", 0)
tx_bytes_delta = gcs_after.get("tx_bytes", 0) - gcs_before.get("tx_bytes", 0)
rx_bytes_delta = uav1_after.get("rx_bytes", 0) - uav1_before.get("rx_bytes", 0)

if tx_packets_delta > 0:
    packet_loss_pct = 100.0 * (tx_packets_delta - rx_packets_delta) / tx_packets_delta
else:
    packet_loss_pct = float("nan")

if tx_bytes_delta > 0:
    byte_loss_pct = 100.0 * (tx_bytes_delta - rx_bytes_delta) / tx_bytes_delta
else:
    byte_loss_pct = float("nan")

# Also pull iperf3's own self-reported receiver loss for cross-check
iperf_text = open(iperf_path).read()
iperf_loss_pct = None
m = re.search(r'(\d+)/(\d+)\s*\(([\d.]+)%\)\s*receiver', iperf_text)
if m:
    iperf_loss_pct = float(m.group(3))

print(f"\n=== B4 result: {label} ===")
print(f"  tap-gcs TX packets sent    : {tx_packets_delta}")
print(f"  tap-uav1 RX packets arrived: {rx_packets_delta}")
print(f"  TAP-counter packet loss   : {packet_loss_pct:.2f}%")
print(f"  TAP-counter byte loss     : {byte_loss_pct:.2f}%")
if iperf_loss_pct is not None:
    print(f"  iperf3 self-reported loss : {iperf_loss_pct:.2f}%  "
          f"(cross-check — should roughly agree)")
else:
    print(f"  iperf3 self-reported loss : could not parse from output")

summary = os.path.join(processed_dir, "b4_loss_summary.csv")
exists = os.path.isfile(summary)
with open(summary, "a", newline="") as f:
    w = csv.writer(f)
    if not exists:
        w.writerow(["label", "offered_rate", "duration_s", "tx_packets",
                    "rx_packets", "tap_packet_loss_pct", "tap_byte_loss_pct",
                    "iperf_reported_loss_pct"])
    w.writerow([label, rate, duration, tx_packets_delta, rx_packets_delta,
                f"{packet_loss_pct:.2f}", f"{byte_loss_pct:.2f}",
                f"{iperf_loss_pct:.2f}" if iperf_loss_pct is not None else ""])
print(f"\n  Appended to: {summary}")
PYEOF