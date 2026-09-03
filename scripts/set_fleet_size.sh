#!/bin/bash
# set_fleet_size.sh
# -----------------
# Set the fleet size in config/fleet.yaml and pre-validate it.
#
# config/fleet.yaml is the single source of truth for the fleet, so resizing
# is one edit -- this just makes that edit safely (exactly one key, verified
# afterwards) and runs the generation checks before you spend time on a launch.
#
# Usage:
#   scripts/set_fleet_size.sh 3
#   scripts/set_fleet_size.sh 5 --no-validate

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FLEET_YAML="$PROJECT_DIR/config/fleet.yaml"

SIZE="$1"

if [[ -z "$SIZE" ]]; then
    echo "Usage: $(basename "$0") <num_uavs> [--no-validate]" >&2
    exit 1
fi

if ! [[ "$SIZE" =~ ^[0-9]+$ ]] || (( SIZE < 1 )); then
    echo "ERROR: fleet size must be a positive integer; got '$SIZE'" >&2
    exit 1
fi

if [[ ! -f "$FLEET_YAML" ]]; then
    echo "ERROR: $FLEET_YAML not found" >&2
    exit 1
fi

CURRENT="$(python3 -c "
import yaml
print(yaml.safe_load(open('$FLEET_YAML'))['fleet']['num_uavs'])
")"

# Rewrite only the num_uavs value under the fleet: block, so comments and
# every other key in the file survive untouched.
python3 - "$FLEET_YAML" "$SIZE" <<'PY'
import re
import sys

path, size = sys.argv[1], sys.argv[2]
text = open(path, encoding='utf-8').read()

pattern = re.compile(r'(^fleet:.*?^\s*num_uavs:\s*)(\d+)', re.S | re.M)
new, count = pattern.subn(lambda m: m.group(1) + size, text, count=1)

if count != 1:
    sys.exit(f'could not find fleet.num_uavs in {path}')

open(path, 'w', encoding='utf-8').write(new)
PY

APPLIED="$(python3 -c "
import yaml
print(yaml.safe_load(open('$FLEET_YAML'))['fleet']['num_uavs'])
")"

if [[ "$APPLIED" != "$SIZE" ]]; then
    echo "ERROR: wanted num_uavs=$SIZE but the file now reads $APPLIED" >&2
    exit 1
fi

echo "config/fleet.yaml: num_uavs $CURRENT -> $APPLIED"

if [[ "$2" == "--no-validate" ]]; then
    exit 0
fi

echo
python3 "$PROJECT_DIR/scripts/validate_fleet_scaling.py" --sizes "$SIZE"

echo
echo "Ready. Launch with:"
echo "    ./scripts/launch_city_dynamic_clustering.sh"
