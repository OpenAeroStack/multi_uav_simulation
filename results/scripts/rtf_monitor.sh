#!/bin/bash
# rtf_monitor.sh — Phase A1: sample Gazebo real-time factor for a fixed window.
#
# Usage:
#   bash rtf_monitor.sh <label> [duration_sec] [results_root]
#
# Example:
#   bash rtf_monitor.sh idle 60
#   bash rtf_monitor.sh edge_load 60
#   bash rtf_monitor.sh ground_load 60
#   bash rtf_monitor.sh mission_active 60
#
# Run this while the condition you're labelling is actually active.
# RTF below ~0.9 invalidates wall-clock timing results collected in that state.

set -uo pipefail

LABEL="${1:?usage: rtf_monitor.sh <label> [duration_sec] [results_root]}"
DURATION="${2:-60}"
RESULTS_ROOT="${3:-$HOME/FYP/multi_uav_simulation/results}"

OUTDIR="$RESULTS_ROOT/phase_a_apparatus"
RAW="$OUTDIR/raw/rtf_${LABEL}_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$OUTDIR/raw" "$OUTDIR/processed"

command -v gz >/dev/null 2>&1 || { echo "ERROR: 'gz' not found on PATH." >&2; exit 1; }

echo "Sampling RTF for ${DURATION}s (label: $LABEL)"
echo "Raw output: $RAW"

timeout "$DURATION" gz stats > "$RAW" 2>&1
# timeout returns 124 when it does its job; that's expected, not an error
rc=$?
if [[ $rc -ne 0 && $rc -ne 124 ]]; then
    echo "WARNING: gz stats exited with code $rc — check $RAW" >&2
fi

if [[ ! -s "$RAW" ]]; then
    echo "ERROR: no output captured. Is Gazebo running?" >&2
    exit 1
fi

python3 - "$RAW" "$LABEL" "$OUTDIR/processed" << 'PYEOF'
import re, sys, statistics as stats, os, csv

raw_path, label, processed_dir = sys.argv[1], sys.argv[2], sys.argv[3]

factors = []
with open(raw_path) as f:
    for line in f:
        m = re.search(r'Factor\[([\d.]+)\]', line)
        if m:
            factors.append(float(m.group(1)))

if not factors:
    print("ERROR: no Factor[...] values parsed. Check the raw log format.", file=sys.stderr)
    sys.exit(1)

mean = stats.mean(factors)
sd   = stats.stdev(factors) if len(factors) > 1 else 0.0

print(f"\n=== RTF summary: {label} ===")
print(f"  samples : {len(factors)}")
print(f"  mean    : {mean:.4f}")
print(f"  median  : {stats.median(factors):.4f}")
print(f"  stdev   : {sd:.4f}")
print(f"  min     : {min(factors):.4f}")
print(f"  max     : {max(factors):.4f}")

if mean < 0.9:
    print(f"\n  ** WARNING: mean RTF {mean:.3f} < 0.90 **")
    print("  Wall-clock timing results collected in this state are not trustworthy.")
else:
    print(f"\n  OK — RTF healthy, timing results from this state are usable.")

# Append to a running summary CSV across all labels
summary = os.path.join(processed_dir, "rtf_summary.csv")
exists = os.path.isfile(summary)
with open(summary, "a", newline="") as f:
    w = csv.writer(f)
    if not exists:
        w.writerow(["label", "n_samples", "mean", "median", "stdev", "min", "max", "raw_file"])
    w.writerow([label, len(factors), f"{mean:.4f}", f"{stats.median(factors):.4f}",
                f"{sd:.4f}", f"{min(factors):.4f}", f"{max(factors):.4f}",
                os.path.basename(raw_path)])
print(f"\n  Appended to: {summary}")
PYEOF