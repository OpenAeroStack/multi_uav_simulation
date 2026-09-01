#!/usr/bin/env python3
"""
Verify generated AP_DDS parameter files against config/fleet.yaml.

Examples:
    python3 scripts/verify_dds_params.py
    python3 scripts/verify_dds_params.py --count 4
"""

from __future__ import annotations

import argparse
import ipaddress
import re
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: sudo apt install python3-yaml", file=sys.stderr)
    raise SystemExit(2)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "config" / "fleet.yaml"
PARAM_DIR = PROJECT_ROOT / "generated" / "params"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Verify generated AP_DDS parameter files."
    )
    parser.add_argument(
        "--count",
        type=int,
        default=None,
        help="Optional UAV-count override; otherwise use fleet.num_uavs.",
    )
    return parser.parse_args()


def parse_param_file(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()

        if not line or line.startswith("#"):
            continue

        parts = line.split()

        if len(parts) < 2:
            continue

        values[parts[0]] = parts[1]

    return values


def main() -> int:
    args = parse_args()

    with CONFIG.open("r", encoding="utf-8") as handle:
        cfg = yaml.safe_load(handle)

    n = (
        args.count
        if args.count is not None
        else int(cfg["fleet"]["num_uavs"])
    )

    if n < 1:
        print(f"ERROR: UAV count must be >= 1, got {n}", file=sys.stderr)
        return 2

    gcs_ip = ipaddress.ip_address(str(cfg["gcs"]["wireless_ip"]))
    octets = [str(x) for x in str(gcs_ip).split(".")]

    base = int(cfg["ports"]["dds_base"])
    stride = int(cfg["ports"].get("dds_stride", 1))

    dds = cfg.get("dds", {})
    expected_common = {
        "DDS_ENABLE": str(dds.get("enable", 1)),
        "DDS_IP0": octets[0],
        "DDS_IP1": octets[1],
        "DDS_IP2": octets[2],
        "DDS_IP3": octets[3],
        "DDS_DOMAIN_ID": str(dds.get("domain_id", 0)),
        "DDS_TIMEOUT_MS": str(dds.get("timeout_ms", 1000)),
        "DDS_MAX_RETRY": str(dds.get("max_retry", 0)),
        "DDS_USE_NS": str(dds.get("use_ns", 1)),
    }

    errors: list[str] = []

    expected_names = {f"uav{i}_dds.parm" for i in range(1, n + 1)}

    actual_names = {
        p.name
        for p in PARAM_DIR.glob("uav*_dds.parm")
        if re.fullmatch(r"uav\d+_dds\.parm", p.name)
    }

    stale = sorted(actual_names - expected_names)
    missing = sorted(expected_names - actual_names)

    for name in missing:
        errors.append(f"missing file: {PARAM_DIR / name}")

    for name in stale:
        errors.append(
            f"stale file present: {PARAM_DIR / name} "
            "(run generator with --clean)"
        )

    for i in range(1, n + 1):
        path = PARAM_DIR / f"uav{i}_dds.parm"

        if not path.is_file():
            continue

        values = parse_param_file(path)

        expected = dict(expected_common)
        expected["DDS_UDP_PORT"] = str(base + (i - 1) * stride)

        for key, expected_value in expected.items():
            actual_value = values.get(key)

            if actual_value != expected_value:
                errors.append(
                    f"UAV{i}: {key}: expected {expected_value}, "
                    f"got {actual_value!r}"
                )

    if errors:
        print("DDS verification FAILED")
        print("=======================")

        for error in errors:
            print(f"ERROR: {error}")

        return 1

    print("DDS verification PASSED")
    print("=======================")

    for i in range(1, n + 1):
        port = base + (i - 1) * stride
        print(
            f"UAV{i}: {gcs_ip}:{port} -> "
            f"generated/params/uav{i}_dds.parm"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
