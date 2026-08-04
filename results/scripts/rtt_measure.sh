#!/bin/bash
# rtt_measure.sh — Phase A2 (emulation floor) and Phase B2 (RTT vs distance).
#
# Usage:
#   bash rtt_measure.sh <label> <from_namespace> <target_ip> [count] [results_root]
#
# PHASE A2 — emulation floor, three conditions:
#
#   1. Management veth, NS-3 NOT in path (pure kernel/veth cost):
#      bash rtt_measure.sh floor_mgmt_veth uav1ns 172.31.1.1 100
#
#   2. NS-3 in path, drone co-located with GCS (degenerate condition):
#      bash rtt_measure.sh floor_ns3_colocated gcsns 10.42.0.11 100
#
#   3. NS-3 in path, operational separation:
#      bash rtt_measure.sh floor_ns3_operational gcsns 10.42.0.11 100
#
#   (2) minus (1) = cost of the TAP/NS-3 emulation machinery
#   (3) minus (2) = cost genuinely attributable to modelled propagation
#
# PHASE B2 — RTT vs distance: fly to each separation, hold, then run e.g.
#      bash rtt_measure.sh dist_050m gcsns 10.42.0.11 100
#
# Record the actual drone separation for each B2 label in MANIFEST.md —
# the label is not self-documenting.

set -uo pipefail

LABEL="${1:?usage: rtt_measure.sh <label> <from_namespace> <target_ip> [count] [results_root]}"
NS="${2:?missing namespace (e.g. gcsns, uav1ns, or 'root')}"
TARGET="${3:?missing target IP}"
COUNT="${4:-100}"
RESULTS_ROOT="${5:-$HOME/FYP/multi_uav_simulation/results}"

# A2 lives in phase_a, B2 in phase_b — route by label prefix
if [[ "$LABEL" == floor_* ]]; then
    OUTDIR="$RESULTS_ROOT/phase_a_apparatus"
else
    OUTDIR="$RESULTS_ROOT/phase_b_network"
fi
mkdir -p "$OUTDIR/raw" "$OUTDIR/processed"

RAW="$OUTDIR/raw/rtt_${LABEL}_$(date +%Y%m%d_%H%M%S).log"

echo "Pinging $TARGET from $NS ($COUNT packets, 0.2s interval)"
echo "Label: $LABEL"
echo "Raw output: $RAW"
echo ""

if [[ "$NS" == "root" ]]; then
    ping -c "$COUNT" -i 0.2 "$TARGET" > "$RAW" 2>&1
else
    sudo ip netns exec "$NS" ping -c "$COUNT" -i 0.2 "$TARGET" > "$RAW" 2>&1
fi
rc=$?
if [[ $rc -ne 0 ]]; then
    echo "WARNING: ping exited with code $rc (100% loss counts as failure)." >&2
    echo "Raw log retained at $RAW for inspection." >&2
fi

if [[ ! -s "$RAW" ]]; then
    echo "ERROR: no output captured." >&2
    exit 1
fi

python3 - "$RAW" "$LABEL" "$NS" "$TARGET" "$OUTDIR/processed" << 'PYEOF'
import re, sys, os, csv, statistics as stats

raw_path, label, ns, target, processed_dir = sys.argv[1:6]

text = open(raw_path).read()

# Per-packet RTTs, so we can report percentiles (not just ping's own summary)
rtts = [float(m) for m in re.findall(r'time=([\d.]+)\s*ms', text)]

loss_pct = None
m = re.search(r'(\d+(?:\.\d+)?)%\s*packet loss', text)
if m:
    loss_pct = float(m.group(1))

transmitted = received = None
m = re.search(r'(\d+) packets transmitted, (\d+) received', text)
if m:
    transmitted, received = int(m.group(1)), int(m.group(2))

print(f"=== RTT summary: {label} ===")
print(f"  from      : {ns}  ->  {target}")
if transmitted is not None:
    print(f"  packets   : {received}/{transmitted} received")
if loss_pct is not None:
    print(f"  loss      : {loss_pct:.1f}%")

if not rtts:
    print("  No RTT samples parsed (all packets lost?).")
    sys.exit(0)

rtts_sorted = sorted(rtts)
def pct(p):
    idx = min(int(round(p / 100 * (len(rtts_sorted) - 1))), len(rtts_sorted) - 1)
    return rtts_sorted[idx]

mean = stats.mean(rtts)
sd   = stats.stdev(rtts) if len(rtts) > 1 else 0.0

print(f"  n samples : {len(rtts)}")
print(f"  mean      : {mean:.3f} ms")
print(f"  median    : {stats.median(rtts):.3f} ms")
print(f"  stdev     : {sd:.3f} ms")
print(f"  min / max : {min(rtts):.3f} / {max(rtts):.3f} ms")
print(f"  p95 / p99 : {pct(95):.3f} / {pct(99):.3f} ms")

summary = os.path.join(processed_dir, "rtt_summary.csv")
exists = os.path.isfile(summary)
with open(summary, "a", newline="") as f:
    w = csv.writer(f)
    if not exists:
        w.writerow(["label", "from_ns", "target", "n", "loss_pct", "mean_ms",
                    "median_ms", "stdev_ms", "min_ms", "max_ms", "p95_ms",
                    "p99_ms", "raw_file"])
    w.writerow([label, ns, target, len(rtts),
                f"{loss_pct:.1f}" if loss_pct is not None else "",
                f"{mean:.3f}", f"{stats.median(rtts):.3f}", f"{sd:.3f}",
                f"{min(rtts):.3f}", f"{max(rtts):.3f}",
                f"{pct(95):.3f}", f"{pct(99):.3f}",
                os.path.basename(raw_path)])
print(f"\n  Appended to: {summary}")

# Per-packet RTTs kept separately — needed for ECDF plots later
detail = os.path.join(processed_dir, f"rtt_samples_{label}.csv")
with open(detail, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["seq", "rtt_ms"])
    for i, v in enumerate(rtts, 1):
        w.writerow([i, f"{v:.3f}"])
print(f"  Per-packet samples: {detail}")
PYEOF