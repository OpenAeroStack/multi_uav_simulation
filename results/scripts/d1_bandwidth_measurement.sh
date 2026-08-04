#!/bin/bash
# d1_bandwidth_measurement.sh — Phase D1 (real bandwidth) + D3 (frame
# delivery rate), using real video/detection traffic — not synthetic
# iperf3 load like B3/B4 used to characterize the channel itself.
#
# Usage:
#   bash d1_bandwidth_measurement.sh <label> <duration_sec>
#
# Run this WHILE the actual detector (edge) or relay+detector (ground) is
# already running and settled — this script only snapshots before/after and
# waits; it doesn't start or stop anything itself.
#
# Example:
#   # Terminal 1: detector already running in uav1ns (edge mode)...
#   # Terminal 2:
#   bash d1_bandwidth_measurement.sh d1_edge_run1 60
#
#   # switch to ground mode (relay + detector), let it settle, then:
#   bash d1_bandwidth_measurement.sh d1_ground_run1 60

set -uo pipefail

LABEL="${1:?usage: d1_bandwidth_measurement.sh <label> <duration_sec>}"
DURATION="${2:-60}"
RESULTS_ROOT="${3:-$HOME/FYP/multi_uav_simulation/results}"

OUTDIR="$RESULTS_ROOT/phase_d_application"
mkdir -p "$OUTDIR/raw" "$OUTDIR/processed"

RAW_BEFORE="$OUTDIR/raw/tap_before_${LABEL}_$(date +%Y%m%d_%H%M%S).txt"
RAW_AFTER="$OUTDIR/raw/tap_after_${LABEL}_$(date +%Y%m%d_%H%M%S).txt"

echo "=== D1/D3 bandwidth measurement: $LABEL (window=${DURATION}s) ==="
echo "Make sure the real detector/relay traffic is ALREADY running and settled"
echo "before this starts — this script only measures, doesn't launch anything."
echo ""

echo "Snapshotting TAP counters (before)..."
{
    echo "--- tap-gcs ---"
    sudo ip -s link show tap-gcs
    echo "--- tap-uav1 ---"
    sudo ip -s link show tap-uav1
} > "$RAW_BEFORE"

echo "Waiting ${DURATION}s while real traffic flows..."
sleep "$DURATION"

echo "Snapshotting TAP counters (after)..."
{
    echo "--- tap-gcs ---"
    sudo ip -s link show tap-gcs
    echo "--- tap-uav1 ---"
    sudo ip -s link show tap-uav1
} > "$RAW_AFTER"

python3 - "$LABEL" "$DURATION" "$RAW_BEFORE" "$RAW_AFTER" "$OUTDIR/processed" << 'PYEOF'
import sys, os, csv

label, duration, before_path, after_path, processed_dir = sys.argv[1:6]
duration = float(duration)

def parse_link_stats(text, iface):
    """Same validated parser as b4_loss_measurement.sh — matches labels to
    values by name, fails loudly (returns None) rather than silently
    mis-pairing on any format mismatch."""
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
                      f"mismatch ({len(labels)} vs {len(values)}) — skipping.",
                      file=sys.stderr)
                continue
            for lab, val in zip(labels, values):
                try:
                    stats[f"{direction}_{lab}"] = int(val)
                except ValueError:
                    pass
    return stats

before_text = open(before_path).read()
after_text = open(after_path).read()

gcs_before = parse_link_stats(before_text, "tap-gcs")
gcs_after  = parse_link_stats(after_text,  "tap-gcs")
uav1_before = parse_link_stats(before_text, "tap-uav1")
uav1_after  = parse_link_stats(after_text,  "tap-uav1")

required = [
    (gcs_before, "tx_bytes", "tap-gcs before"),
    (gcs_after, "tx_bytes", "tap-gcs after"),
    (uav1_before, "tx_bytes", "tap-uav1 before"),
    (uav1_after, "tx_bytes", "tap-uav1 after"),
]
missing = [f"{name} ({field})" for stats, field, name in required
           if stats is None or field not in stats]
if missing:
    print(f"\nERROR: required stats missing/unparseable: {', '.join(missing)}",
          file=sys.stderr)
    sys.exit(1)

# D1 direction: this measures traffic in BOTH directions on the link, since
# unlike D8's ping test, real video traffic flows uav1ns -> gcsns (frames /
# detection results) while telemetry flows both ways. Report both.
gcs_tx_bytes = gcs_after["tx_bytes"] - gcs_before["tx_bytes"]
gcs_rx_bytes = gcs_after.get("rx_bytes", 0) - gcs_before.get("rx_bytes", 0)
uav1_tx_bytes = uav1_after["tx_bytes"] - uav1_before["tx_bytes"]
uav1_rx_bytes = uav1_after.get("rx_bytes", 0) - uav1_before.get("rx_bytes", 0)

gcs_tx_pkts = gcs_after.get("tx_packets", 0) - gcs_before.get("tx_packets", 0)
gcs_rx_pkts = gcs_after.get("rx_packets", 0) - gcs_before.get("rx_packets", 0)
uav1_tx_pkts = uav1_after.get("tx_packets", 0) - uav1_before.get("tx_packets", 0)
uav1_rx_pkts = uav1_after.get("rx_packets", 0) - uav1_before.get("rx_packets", 0)

# uav1 -> gcs direction (the direction that matters most: video/detection
# results leaving the UAV side)
uav1_to_gcs_mbps = (uav1_tx_bytes * 8) / (duration * 1e6)
# gcs -> uav1 direction (telemetry commands, MAVLink, etc.)
gcs_to_uav1_mbps = (gcs_tx_bytes * 8) / (duration * 1e6)

print(f"\n=== D1/D3 result: {label} (window={duration}s) ===")
print(f"  UAV1 -> GCS: {uav1_tx_bytes} bytes, {uav1_tx_pkts} packets  "
      f"({uav1_to_gcs_mbps:.4f} Mbps)")
print(f"  GCS -> UAV1: {gcs_tx_bytes} bytes, {gcs_tx_pkts} packets  "
      f"({gcs_to_uav1_mbps:.4f} Mbps)")
if uav1_tx_pkts > 0:
    print(f"  Mean packet size (UAV1->GCS): {uav1_tx_bytes/uav1_tx_pkts:.1f} bytes")
    print(f"  Effective packet rate (D3, UAV1->GCS): {uav1_tx_pkts/duration:.2f} pkts/sec")

summary = os.path.join(processed_dir, "d1_bandwidth_summary.csv")
exists = os.path.isfile(summary)
with open(summary, "a", newline="") as f:
    w = csv.writer(f)
    if not exists:
        w.writerow(["label", "duration_s", "uav1_to_gcs_bytes", "uav1_to_gcs_pkts",
                    "uav1_to_gcs_mbps", "gcs_to_uav1_bytes", "gcs_to_uav1_pkts",
                    "gcs_to_uav1_mbps", "mean_pkt_size_bytes", "pkt_rate_per_sec"])
    mean_pkt = uav1_tx_bytes/uav1_tx_pkts if uav1_tx_pkts > 0 else 0
    pkt_rate = uav1_tx_pkts/duration
    w.writerow([label, duration, uav1_tx_bytes, uav1_tx_pkts,
                f"{uav1_to_gcs_mbps:.4f}", gcs_tx_bytes, gcs_tx_pkts,
                f"{gcs_to_uav1_mbps:.4f}", f"{mean_pkt:.1f}", f"{pkt_rate:.2f}"])
print(f"\n  Appended to: {summary}")
PYEOF