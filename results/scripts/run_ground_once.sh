#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RUN_ID="${1:?Usage: run_ground_once.sh RUN_ID RNG_RUN [DURATION]}"
RNG_RUN="${2:?Usage: run_ground_once.sh RUN_ID RNG_RUN [DURATION]}"
DURATION="${3:-60}"

exec bash "$SCRIPT_DIR/run_comparison_once.sh" \
  ground "$RUN_ID" "$RNG_RUN" "$DURATION"

