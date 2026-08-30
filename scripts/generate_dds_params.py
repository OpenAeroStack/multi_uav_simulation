#!/usr/bin/env python3

import argparse
import ipaddress
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is not installed.")
    print("Install with:")
    print("  sudo apt install python3-yaml")
    sys.exit(1)


PROJECT_ROOT = Path(__file__).resolve().parents[1]

DEFAULT_CONFIG = PROJECT_ROOT / "config" / "fleet.yaml"
DEFAULT_OUTPUT = PROJECT_ROOT / "generated" / "params"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate ArduPilot DDS parameter files."
    )

    parser.add_argument(
        "--count",
        type=int,
        default=None,
        help="Override number of UAVs from fleet.yaml",
    )

    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="Path to fleet.yaml",
    )

    parser.add_argument(
        "--outdir",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Output directory",
    )

    parser.add_argument(
        "--clean",
        action="store_true",
        help="Remove old uav*_dds.parm files before generation",
    )

    return parser.parse_args()


def load_config(path):
    if not path.exists():
        raise FileNotFoundError(
            f"Fleet configuration does not exist: {path}"
        )

    with path.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    if not isinstance(config, dict):
        raise ValueError(
            f"Invalid YAML configuration: {path}"
        )

    return config


def main():
    args = parse_args()

    config_path = args.config.resolve()
    outdir = args.outdir.resolve()

    config = load_config(config_path)

    # ------------------------------------------------------------
    # Fleet size
    # ------------------------------------------------------------

    fleet = config.get("fleet", {})

    config_num_uavs = int(
        fleet.get("num_uavs", 0)
    )

    if args.count is not None:
        num_uavs = args.count
    else:
        num_uavs = config_num_uavs

    if num_uavs < 1:
        raise ValueError(
            f"Invalid UAV count: {num_uavs}"
        )

    # ------------------------------------------------------------
    # GCS DDS IP
    # ------------------------------------------------------------

    gcs = config.get("gcs", {})

    gcs_ip_string = str(
        gcs.get(
            "wireless_ip",
            "10.42.0.10"
        )
    )

    gcs_ip = ipaddress.ip_address(
        gcs_ip_string
    )

    if gcs_ip.version != 4:
        raise ValueError(
            "DDS generator currently expects IPv4"
        )

    ip_parts = str(gcs_ip).split(".")

    # ------------------------------------------------------------
    # Port configuration
    # ------------------------------------------------------------

    ports = config.get("ports", {})

    dds_base = int(
        ports.get(
            "dds_base",
            2019
        )
    )

    dds_stride = int(
        ports.get(
            "dds_stride",
            1
        )
    )

    if dds_stride < 1:
        raise ValueError(
            "dds_stride must be >= 1"
        )

    # ------------------------------------------------------------
    # DDS configuration
    # ------------------------------------------------------------

    dds = config.get("dds", {})

    dds_enable = int(
        dds.get(
            "enable",
            1
        )
    )

    domain_id = int(
        dds.get(
            "domain_id",
            0
        )
    )

    timeout_ms = int(
        dds.get(
            "timeout_ms",
            1000
        )
    )

    max_retry = int(
        dds.get(
            "max_retry",
            0
        )
    )

    use_ns = int(
        dds.get(
            "use_ns",
            1
        )
    )

    # ------------------------------------------------------------
    # Prepare output directory
    # ------------------------------------------------------------

    outdir.mkdir(
        parents=True,
        exist_ok=True
    )

    if args.clean:

        old_files = list(
            outdir.glob("uav*_dds.parm")
        )

        for old_file in old_files:
            old_file.unlink()

        if old_files:
            print(
                f"Removed {len(old_files)} old DDS parameter file(s)."
            )

    # ------------------------------------------------------------
    # Generate one parameter file per UAV
    # ------------------------------------------------------------

    generated = []

    for uav_id in range(
        1,
        num_uavs + 1
    ):

        dds_port = (
            dds_base
            + (uav_id - 1) * dds_stride
        )

        if dds_port > 65535:
            raise ValueError(
                f"Invalid DDS port for UAV{uav_id}: "
                f"{dds_port}"
            )

        output_file = (
            outdir
            / f"uav{uav_id}_dds.parm"
        )

        content = f"""DDS_ENABLE {dds_enable}
DDS_UDP_PORT {dds_port}

DDS_IP0 {ip_parts[0]}
DDS_IP1 {ip_parts[1]}
DDS_IP2 {ip_parts[2]}
DDS_IP3 {ip_parts[3]}

DDS_DOMAIN_ID {domain_id}
DDS_TIMEOUT_MS {timeout_ms}
DDS_MAX_RETRY {max_retry}
DDS_USE_NS {use_ns}
"""

        output_file.write_text(
            content,
            encoding="utf-8"
        )

        generated.append(
            (
                uav_id,
                dds_port,
                output_file
            )
        )

    # ------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------

    print()
    print("============================================================")
    print("DDS PARAMETER GENERATION COMPLETE")
    print("============================================================")

    print(f"Config        : {config_path}")
    print(f"Fleet config  : {config_num_uavs}")
    print(f"Requested N   : {num_uavs}")
    print(f"GCS DDS IP    : {gcs_ip}")
    print(f"DDS base      : {dds_base}")
    print(f"DDS stride    : {dds_stride}")
    print(f"Output dir    : {outdir}")

    print()
    print(
        f"{'UAV':<8}"
        f"{'DDS PORT':<12}"
        f"FILE"
    )

    print("-" * 70)

    for (
        uav_id,
        dds_port,
        output_file
    ) in generated:

        print(
            f"UAV{uav_id:<5}"
            f"{dds_port:<12}"
            f"{output_file}"
        )

    print()
    print(
        f"Generated {len(generated)} DDS parameter file(s)."
    )

    print("============================================================")


if __name__ == "__main__":

    try:
        main()

    except Exception as exc:

        print(
            f"ERROR: {exc}",
            file=sys.stderr
        )

        sys.exit(1)
