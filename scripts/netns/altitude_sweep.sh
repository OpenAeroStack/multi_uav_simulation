#!/bin/bash
# Fly the same mission at several altitudes, one run per altitude.
#
# Altitude is the only variable that changes, so the resulting runs support a
# detection-rate-vs-altitude curve. Two runs at different altitudes flown by
# DIFFERENT aircraft do not: they differ in mission geometry as well, and dwell
# time on target dominates pixel size (see report/detection_rate.png).
#
#   ./scripts/netns/rpi_init.sh
#   ./scripts/netns/sitl_init.sh --gui --view     # leave running
#   ./scripts/netns/detector_start.sh             # leave running
#   ./scripts/netns/altitude_sweep.sh             # this script
#
#   ALTITUDES="10 15 20 25 30"   metres, one run each
#   AIRCRAFT=UAV1                which aircraft flies; the others stay grounded
#   COOLDOWN=120                 seconds between runs, for the boards to cool

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

ALTITUDES="${ALTITUDES:-10 15 20 25 30}"
AIRCRAFT="${AIRCRAFT:-UAV1}"
COOLDOWN="${COOLDOWN:-120}"
LAND_WAIT="${LAND_WAIT:-60}"

say() { printf '\r%s\n' "$*"; }

# One aircraft only. Sweeping UAV1 down through UAV2's altitude would put them
# at the same height, and vertical separation is the only collision guard.
export DRONES_ONLY="$AIRCRAFT"

n=0
total=$(wc -w <<< "$ALTITUDES")
started=$(date +%s)
declare -a DONE=()

say ""
say "════════════════════════════════════════════════════════════"
say " ALTITUDE SWEEP — $AIRCRAFT at: $ALTITUDES m"
say "════════════════════════════════════════════════════════════"
say "  $total runs · ~2 min each · ${COOLDOWN}s cooldown between"
say "  Ctrl+C stops after the current run finishes archiving."
say ""

for alt in $ALTITUDES; do
    n=$((n + 1))
    say "────────────────────────────────────────────────────────────"
    say " RUN $n/$total   $AIRCRAFT @ ${alt} m"
    say "────────────────────────────────────────────────────────────"

    export "${AIRCRAFT}_ALTITUDE=$alt"

    if "$SCRIPT_DIR/run_missions.sh"; then
        DONE+=("${alt}m ok")
        say "  run $n/$total complete (${alt} m)"
    else
        DONE+=("${alt}m FAILED")
        say "  WARNING: run at ${alt} m failed — continuing with the rest" >&2
    fi

    # RTL is commanded but not waited on, so the aircraft is still descending
    # when the mission process exits. Arming again mid-descent fails.
    say "  waiting ${LAND_WAIT}s for the aircraft to land ..."
    sleep "$LAND_WAIT"

    if (( n < total )); then
        say "  cooling boards for ${COOLDOWN}s ..."
        sleep "$COOLDOWN"
    fi
    say ""
done

elapsed=$(( $(date +%s) - started ))
say "════════════════════════════════════════════════════════════"
say " SWEEP COMPLETE — $((elapsed / 60)) min $((elapsed % 60)) s"
say "════════════════════════════════════════════════════════════"
for d in "${DONE[@]}"; do say "   $d"; done
say ""
say "  Plot it:  python3 scripts/make_detection_chart.py"
say ""
