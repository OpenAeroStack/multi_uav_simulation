#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ipaddress
from pathlib import Path

import yaml


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path, default=None)
    p.add_argument("--project-root", type=Path, default=None)
    return p.parse_args()


def main():
    args = parse_args()

    project_root = (
        args.project_root.expanduser().resolve()
        if args.project_root
        else Path(__file__).resolve().parents[1]
    )

    config_path = (
        args.config.expanduser().resolve()
        if args.config
        else project_root / "config" / "fleet.yaml"
    )

    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    fleet = cfg["fleet"]
    network = cfg["network"]
    ports = cfg["ports"]
    defaults = cfg.get("defaults", {})
    overrides = cfg.get("uav_overrides", {})

    n = int(fleet["num_uavs"])
    if n < 1:
        raise ValueError("fleet.num_uavs must be >= 1")

    namespace_prefix = str(network.get("namespace_prefix", "uav"))
    start_ip = ipaddress.ip_address(
        str(network.get("uav_ip_start", "10.42.0.11"))
    )
    sim_template = str(
        network.get("sim_ip_template", "172.31.{id}.1")
    )

    dds_base = int(ports.get("dds_base", 2019))
    dds_stride = int(ports.get("dds_stride", 1))
    mav_base = int(ports.get("mavlink_base", 5760))
    mav_stride = int(ports.get("mavlink_stride", 10))
    gz_base = int(ports.get("gazebo_fdm_base", 9002))
    sitl_base = int(ports.get("sitl_fdm_base", 9003))
    fdm_stride = int(ports.get("fdm_stride", 10))

    default_alt = float(defaults.get("takeoff_altitude_m", 50.0))

    for uid in range(1, n + 1):
        k = uid - 1
        override = overrides.get(str(uid), {})
        if not isinstance(override, dict):
            override = {}

        takeoff_alt = float(
            override.get("takeoff_altitude_m", default_alt)
        )

        row = [
            str(uid),
            f"{namespace_prefix}{uid}",
            str(start_ip + k),
            str(dds_base + k * dds_stride),
            str(mav_base + k * mav_stride),
            str(gz_base + k * fdm_stride),
            str(sitl_base + k * fdm_stride),
            sim_template.format(id=uid),
            str(float(takeoff_alt)),
            str(
                project_root
                / "generated"
                / "params"
                / f"uav{uid}_dds.parm"
            ),
        ]

        print("\t".join(row))


if __name__ == "__main__":
    main()
