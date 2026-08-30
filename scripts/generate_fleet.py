#!/usr/bin/env python3

"""
Dynamic fleet configuration generator for multi_uav_simulation.

Reads:
    config/fleet.yaml

Generates:
    generated/fleet.json
    generated/params/uavN_dds.parm
    generated/worlds/spawn_points.json

Examples:

    python3 scripts/generate_fleet.py

Generate 5 UAVs without editing fleet.yaml:

    python3 scripts/generate_fleet.py --count 5

Clean old generated files first:

    python3 scripts/generate_fleet.py --clean

Use another config file:

    python3 scripts/generate_fleet.py \
        --config config/fleet.yaml \
        --outdir generated
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import shutil
import sys

from pathlib import Path
from typing import Any, Dict, List


# ============================================================
# PyYAML
# ============================================================

try:
    import yaml

except ImportError as exc:

    raise SystemExit(
        "\n"
        "ERROR: PyYAML is required.\n\n"
        "Install using:\n"
        "    sudo apt install python3-yaml\n\n"
        "or:\n"
        "    python3 -m pip install PyYAML\n"
    ) from exc


# ============================================================
# Utility functions
# ============================================================


def load_yaml(path: Path) -> Dict[str, Any]:

    if not path.is_file():

        raise FileNotFoundError(
            f"Config file not found: {path}"
        )

    with path.open(
        "r",
        encoding="utf-8"
    ) as file:

        data = yaml.safe_load(file)

    if not isinstance(data, dict):

        raise ValueError(
            "Top-level YAML value must be a mapping/object."
        )

    return data


def ensure_mapping(
    obj: Any,
    name: str
) -> Dict[str, Any]:

    if not isinstance(obj, dict):

        raise ValueError(
            f"'{name}' must be a mapping/object."
        )

    return obj


def as_int(
    value: Any,
    name: str
) -> int:

    try:

        result = int(value)

    except (TypeError, ValueError) as exc:

        raise ValueError(
            f"'{name}' must be an integer. "
            f"Received: {value!r}"
        ) from exc

    return result


def as_float(
    value: Any,
    name: str
) -> float:

    try:

        result = float(value)

    except (TypeError, ValueError) as exc:

        raise ValueError(
            f"'{name}' must be a number. "
            f"Received: {value!r}"
        ) from exc

    return result


# ============================================================
# Automatic Gazebo spawn-position generation
# ============================================================


def generated_spawn(
    uav_id: int,
    origin: List[float],
    spacing_m: float
) -> List[float]:

    """
    Automatically produce UAV starting locations.

    Example:

        origin = [-70, -22, 0]
        spacing = 5

    Results:

        UAV1 -> [-70, -22, 0]
        UAV2 -> [-70, -27, 0]
        UAV3 -> [-70, -17, 0]
        UAV4 -> [-70, -32, 0]
        UAV5 -> [-70, -12, 0]

    UAVs alternate on the -Y and +Y sides.
    """

    x, y, z = map(
        float,
        origin
    )

    if uav_id == 1:

        offset = 0.0

    else:

        ring = uav_id // 2

        if uav_id % 2 == 0:

            direction = -1.0

        else:

            direction = 1.0

        offset = (
            direction
            * ring
            * spacing_m
        )

    return [
        x,
        y + offset,
        z
    ]


# ============================================================
# Validation
# ============================================================


def validate_port(
    port: int,
    name: str
) -> None:

    if not 1 <= port <= 65535:

        raise ValueError(
            f"{name}={port} is outside "
            "valid TCP/UDP port range 1..65535."
        )


def validate_sysid(
    sysid: int
) -> None:

    if not 1 <= sysid <= 255:

        raise ValueError(
            f"MAVLink SYSID {sysid} is outside "
            "valid range 1..255."
        )


# ============================================================
# Fleet generator
# ============================================================


def make_fleet(
    config: Dict[str, Any],
    count_override: int | None = None
) -> Dict[str, Any]:

    # --------------------------------------------------------
    # Read configuration blocks
    # --------------------------------------------------------

    fleet_cfg = ensure_mapping(
        config.get("fleet", {}),
        "fleet"
    )

    gcs_cfg = ensure_mapping(
        config.get("gcs", {}),
        "gcs"
    )

    network_cfg = ensure_mapping(
        config.get("network", {}),
        "network"
    )

    ports_cfg = ensure_mapping(
        config.get("ports", {}),
        "ports"
    )

    gazebo_cfg = ensure_mapping(
        config.get("gazebo", {}),
        "gazebo"
    )

    dds_cfg = ensure_mapping(
        config.get("dds", {}),
        "dds"
    )

    defaults_cfg = ensure_mapping(
        config.get("defaults", {}),
        "defaults"
    )

    simulation_cfg = ensure_mapping(
        config.get("simulation", {}),
        "simulation"
    )

    overrides_cfg = ensure_mapping(
        config.get("uav_overrides", {}),
        "uav_overrides"
    )


    # --------------------------------------------------------
    # UAV count
    # --------------------------------------------------------

    if count_override is not None:

        num_uavs = as_int(
            count_override,
            "count_override"
        )

    else:

        num_uavs = as_int(
            fleet_cfg.get(
                "num_uavs",
                3
            ),
            "fleet.num_uavs"
        )


    if num_uavs < 1:

        raise ValueError(
            "fleet.num_uavs must be >= 1."
        )


    # Our design currently uses:
    #
    # UAV ID == MAVLink SYSID
    #
    # Therefore keep this safety check.
    if num_uavs > 255:

        raise ValueError(
            "fleet.num_uavs cannot exceed 255 "
            "while MAVLink SYSID == UAV ID."
        )


    # --------------------------------------------------------
    # Wireless network
    # --------------------------------------------------------

    subnet = ipaddress.ip_network(
        str(
            network_cfg.get(
                "wireless_subnet",
                "10.42.0.0/24"
            )
        ),
        strict=False
    )


    gcs_ip = ipaddress.ip_address(
        str(
            gcs_cfg.get(
                "wireless_ip",
                "10.42.0.10"
            )
        )
    )


    first_uav_ip = ipaddress.ip_address(
        str(
            network_cfg.get(
                "uav_ip_start",
                "10.42.0.11"
            )
        )
    )


    if gcs_ip not in subnet:

        raise ValueError(
            f"GCS IP {gcs_ip} "
            f"is not inside subnet {subnet}."
        )


    if first_uav_ip not in subnet:

        raise ValueError(
            f"First UAV IP {first_uav_ip} "
            f"is not inside subnet {subnet}."
        )


    def is_usable_host(
        address
    ) -> bool:

        if address not in subnet:

            return False

        if subnet.version == 4:

            return address not in (
                subnet.network_address,
                subnet.broadcast_address
            )

        return (
            address
            != subnet.network_address
        )


    if not is_usable_host(gcs_ip):

        raise ValueError(
            f"GCS IP {gcs_ip} "
            f"is not a usable host address "
            f"in {subnet}."
        )


    if not is_usable_host(first_uav_ip):

        raise ValueError(
            f"First UAV IP {first_uav_ip} "
            f"is not a usable host address "
            f"in {subnet}."
        )


    # --------------------------------------------------------
    # Ports
    # --------------------------------------------------------

    dds_base = as_int(
        ports_cfg.get(
            "dds_base",
            2019
        ),
        "ports.dds_base"
    )


    dds_stride = as_int(
        ports_cfg.get(
            "dds_stride",
            1
        ),
        "ports.dds_stride"
    )


    mav_base = as_int(
        ports_cfg.get(
            "mavlink_base",
            5760
        ),
        "ports.mavlink_base"
    )


    mav_stride = as_int(
        ports_cfg.get(
            "mavlink_stride",
            10
        ),
        "ports.mavlink_stride"
    )


    gz_fdm_base = as_int(
        ports_cfg.get(
            "gazebo_fdm_base",
            9002
        ),
        "ports.gazebo_fdm_base"
    )


    sitl_fdm_base = as_int(
        ports_cfg.get(
            "sitl_fdm_base",
            9003
        ),
        "ports.sitl_fdm_base"
    )


    fdm_stride = as_int(
        ports_cfg.get(
            "fdm_stride",
            10
        ),
        "ports.fdm_stride"
    )


    # --------------------------------------------------------
    # Gazebo defaults
    # --------------------------------------------------------

    origin = gazebo_cfg.get(
        "spawn_origin",
        [
            -70.0,
            -22.0,
            0.0
        ]
    )


    if (
        not isinstance(origin, list)
        or len(origin) != 3
    ):

        raise ValueError(
            "gazebo.spawn_origin must be "
            "[x, y, z]."
        )


    origin = [
        float(value)
        for value in origin
    ]


    spawn_spacing = as_float(
        gazebo_cfg.get(
            "spawn_spacing_m",
            5.0
        ),
        "gazebo.spawn_spacing_m"
    )


    if spawn_spacing <= 0:

        raise ValueError(
            "gazebo.spawn_spacing_m must be > 0."
        )


    namespace_prefix = str(
        network_cfg.get(
            "namespace_prefix",
            "uav"
        )
    )


    tap_prefix = str(
        network_cfg.get(
            "tap_prefix",
            "tap-uav"
        )
    )


    model_prefix = str(
        gazebo_cfg.get(
            "uav_prefix",
            "iris_"
        )
    )


    sim_ip_template = str(
        network_cfg.get(
            "sim_ip_template",
            "172.31.{id}.1"
        )
    )


    # --------------------------------------------------------
    # Mission defaults
    # --------------------------------------------------------

    default_takeoff = as_float(
        defaults_cfg.get(
            "takeoff_altitude_m",
            50.0
        ),
        "defaults.takeoff_altitude_m"
    )


    default_mission = str(
        defaults_cfg.get(
            "mission_profile",
            "generic"
        )
    )


    # ========================================================
    # Generate UAVs
    # ========================================================

    uavs: List[Dict[str, Any]] = []


    for uav_id in range(
        1,
        num_uavs + 1
    ):

        # ArduPilot instance is zero based.
        instance = (
            uav_id - 1
        )


        # ----------------------------------------------------
        # IDs
        # ----------------------------------------------------

        sysid = uav_id

        validate_sysid(
            sysid
        )


        # ----------------------------------------------------
        # Wireless IP
        # ----------------------------------------------------

        wireless_ip = (
            first_uav_ip
            + instance
        )


        if not is_usable_host(
            wireless_ip
        ):

            raise ValueError(

                f"Not enough usable IP addresses "
                f"in {subnet}.\n"

                f"UAV{uav_id} would require "
                f"{wireless_ip}."
            )


        # ----------------------------------------------------
        # Dynamic ports
        # ----------------------------------------------------

        dds_port = (
            dds_base
            + dds_stride
            * instance
        )


        mavlink_port = (
            mav_base
            + mav_stride
            * instance
        )


        gazebo_fdm_port = (
            gz_fdm_base
            + fdm_stride
            * instance
        )


        sitl_fdm_port = (
            sitl_fdm_base
            + fdm_stride
            * instance
        )


        validate_port(
            dds_port,
            f"UAV{uav_id} DDS UDP port"
        )


        validate_port(
            mavlink_port,
            f"UAV{uav_id} MAVLink TCP port"
        )


        validate_port(
            gazebo_fdm_port,
            f"UAV{uav_id} Gazebo FDM UDP port"
        )


        validate_port(
            sitl_fdm_port,
            f"UAV{uav_id} SITL FDM UDP port"
        )


        # ----------------------------------------------------
        # Per-UAV overrides
        # ----------------------------------------------------

        override = overrides_cfg.get(
            str(uav_id),
            overrides_cfg.get(
                uav_id,
                {}
            )
        )


        if override is None:

            override = {}


        override = ensure_mapping(
            override,
            f"uav_overrides.{uav_id}"
        )


        # ----------------------------------------------------
        # Spawn position
        # ----------------------------------------------------

        spawn = override.get(
            "spawn",
            generated_spawn(
                uav_id,
                origin,
                spawn_spacing
            )
        )


        if (
            not isinstance(spawn, list)
            or len(spawn) != 3
        ):

            raise ValueError(
                f"uav_overrides.{uav_id}.spawn "
                "must be [x, y, z]."
            )


        spawn = [
            float(value)
            for value in spawn
        ]


        # ----------------------------------------------------
        # Takeoff altitude
        # ----------------------------------------------------

        takeoff_altitude = as_float(

            override.get(
                "takeoff_altitude_m",
                default_takeoff
            ),

            (
                f"uav_overrides."
                f"{uav_id}."
                f"takeoff_altitude_m"
            )
        )


        # ----------------------------------------------------
        # Mission profile
        # ----------------------------------------------------

        mission_profile = str(

            override.get(
                "mission_profile",
                default_mission
            )
        )


        # ----------------------------------------------------
        # Gazebo/SITL simulation address
        # ----------------------------------------------------

        try:

            sim_ip = sim_ip_template.format(
                id=uav_id,
                index=instance
            )

        except (
            KeyError,
            IndexError,
            ValueError
        ) as exc:

            raise ValueError(

                "network.sim_ip_template "
                "may use only "
                "{id} and {index}."

            ) from exc


        # ----------------------------------------------------
        # Complete UAV description
        # ----------------------------------------------------

        uav = {

            # Logical UAV number.
            "id": uav_id,

            # MAVLink identity.
            "sysid": sysid,

            # NS-3 ID.
            #
            # node 0 = GCS
            # node 1 = UAV1
            # ...
            "ns3_id": uav_id,

            # ArduPilot SITL instance.
            "instance": instance,

            # Linux namespace.
            "namespace":
                f"{namespace_prefix}{uav_id}",

            # TAP device.
            "tap":
                f"{tap_prefix}{uav_id}",

            # NS-3 wireless IP.
            "wireless_ip":
                str(wireless_ip),

            # Gazebo/SITL management address.
            "sim_ip":
                sim_ip,

            # AP_DDS / micro-ROS port.
            "dds_port":
                dds_port,

            # MAVLink TCP listener.
            "mavlink_port":
                mavlink_port,

            # Gazebo FDM UDP port.
            "gazebo_fdm_port":
                gazebo_fdm_port,

            # SITL FDM UDP port.
            "sitl_fdm_port":
                sitl_fdm_port,

            # Gazebo model name.
            "gazebo_model":
                f"{model_prefix}{uav_id}",

            # Gazebo initial position.
            "spawn":
                spawn,

            # Mission takeoff altitude.
            "takeoff_altitude_m":
                takeoff_altitude,

            # Mission type.
            "mission_profile":
                mission_profile,

            # Generated ArduPilot parameter file.
            "dds_param_file":
                f"params/uav{uav_id}_dds.parm"
        }


        uavs.append(
            uav
        )


    # ========================================================
    # Complete manifest
    # ========================================================

    gcs_id = as_int(
        gcs_cfg.get(
            "id",
            0
        ),
        "gcs.id"
    )


    manifest: Dict[str, Any] = {

        "schema_version": 1,

        "num_uavs":
            num_uavs,

        # GCS + UAVs
        "num_radio_nodes":
            num_uavs + 1,

        # Number of undirected wireless pairs:
        #
        # (N + 1) * N / 2
        #
        # N=3 -> 6
        # N=4 -> 10
        "expected_link_pairs":
            (
                (num_uavs + 1)
                * num_uavs
                // 2
            ),


        # ----------------------------------------------------
        # GCS
        # ----------------------------------------------------

        "gcs": {

            "id":
                gcs_id,

            "ns3_id":
                gcs_id,

            "namespace":
                str(
                    gcs_cfg.get(
                        "namespace",
                        "gcsns"
                    )
                ),

            "tap":
                str(
                    gcs_cfg.get(
                        "tap",
                        "tap-gcs"
                    )
                ),

            "wireless_ip":
                str(gcs_ip),

            "gazebo_model":
                str(
                    gcs_cfg.get(
                        "gazebo_model",
                        "gcs"
                    )
                ),

            "antenna_height_m":
                as_float(
                    gcs_cfg.get(
                        "antenna_height_m",
                        2.9
                    ),
                    "gcs.antenna_height_m"
                )
        },


        # ----------------------------------------------------
        # Network information
        # ----------------------------------------------------

        "network": {

            "wireless_subnet":
                str(subnet),

            "uav_ip_start":
                str(first_uav_ip)
        },


        # ----------------------------------------------------
        # DDS settings
        # ----------------------------------------------------

        "dds": {

            "domain_id":
                as_int(
                    dds_cfg.get(
                        "domain_id",
                        0
                    ),
                    "dds.domain_id"
                ),

            "timeout_ms":
                as_int(
                    dds_cfg.get(
                        "timeout_ms",
                        1000
                    ),
                    "dds.timeout_ms"
                ),

            "max_retry":
                as_int(
                    dds_cfg.get(
                        "max_retry",
                        0
                    ),
                    "dds.max_retry"
                ),

            "use_ns":
                as_int(
                    dds_cfg.get(
                        "use_ns",
                        1
                    ),
                    "dds.use_ns"
                )
        },


        # ----------------------------------------------------
        # NS-3/channel settings
        # ----------------------------------------------------

        "simulation":
            simulation_cfg,


        # ----------------------------------------------------
        # UAVs
        # ----------------------------------------------------

        "uavs":
            uavs
    }


    validate_manifest(
        manifest
    )


    return manifest


# ============================================================
# Check generated resources are unique
# ============================================================


def validate_manifest(
    manifest: Dict[str, Any]
) -> None:

    uavs = manifest["uavs"]


    def assert_unique(
        field: str
    ) -> None:

        values = [
            uav[field]
            for uav in uavs
        ]

        if len(values) != len(set(values)):

            raise ValueError(

                f"Generated UAV field "
                f"'{field}' contains duplicates:\n"
                f"{values}"
            )


    fields_that_must_be_unique = [

        "id",

        "sysid",

        "ns3_id",

        "instance",

        "namespace",

        "tap",

        "wireless_ip",

        "sim_ip",

        "dds_port",

        "mavlink_port",

        "gazebo_fdm_port",

        "sitl_fdm_port",

        "gazebo_model"
    ]


    for field in fields_that_must_be_unique:

        assert_unique(
            field
        )


    gcs = manifest["gcs"]


    # Current project convention:
    #
    # node 0 = GCS
    if (
        gcs["id"] != 0
        or gcs["ns3_id"] != 0
    ):

        raise ValueError(

            "Current project convention "
            "requires GCS ID and "
            "NS-3 ID to equal 0."
        )


    uav_ips = {
        uav["wireless_ip"]
        for uav in uavs
    }


    if (
        gcs["wireless_ip"]
        in uav_ips
    ):

        raise ValueError(
            "GCS wireless IP collides "
            "with UAV wireless IP."
        )


    uav_taps = {
        uav["tap"]
        for uav in uavs
    }


    if (
        gcs["tap"]
        in uav_taps
    ):

        raise ValueError(
            "GCS TAP collides "
            "with UAV TAP."
        )


    uav_namespaces = {
        uav["namespace"]
        for uav in uavs
    }


    if (
        gcs["namespace"]
        in uav_namespaces
    ):

        raise ValueError(
            "GCS namespace collides "
            "with UAV namespace."
        )


# ============================================================
# Generate DDS parameter files
# ============================================================


def render_dds_params(
    manifest: Dict[str, Any],
    params_dir: Path
) -> None:

    params_dir.mkdir(
        parents=True,
        exist_ok=True
    )


    gcs_ip = ipaddress.ip_address(
        manifest["gcs"]["wireless_ip"]
    )


    if gcs_ip.version != 4:

        raise ValueError(
            "DDS parameter generator "
            "currently expects IPv4."
        )


    ip_parts = (
        str(gcs_ip)
        .split(".")
    )


    dds_cfg = manifest["dds"]


    for uav in manifest["uavs"]:

        text = (

            "DDS_ENABLE 1\n"

            f"DDS_UDP_PORT "
            f"{uav['dds_port']}\n"

            f"DDS_IP0 "
            f"{ip_parts[0]}\n"

            f"DDS_IP1 "
            f"{ip_parts[1]}\n"

            f"DDS_IP2 "
            f"{ip_parts[2]}\n"

            f"DDS_IP3 "
            f"{ip_parts[3]}\n"

            f"DDS_DOMAIN_ID "
            f"{dds_cfg['domain_id']}\n"

            f"DDS_TIMEOUT_MS "
            f"{dds_cfg['timeout_ms']}\n"

            f"DDS_MAX_RETRY "
            f"{dds_cfg['max_retry']}\n"

            f"DDS_USE_NS "
            f"{dds_cfg['use_ns']}\n"
        )


        output_file = (

            params_dir
            / f"uav{uav['id']}_dds.parm"
        )


        output_file.write_text(
            text,
            encoding="utf-8"
        )


# ============================================================
# Generate Gazebo spawn manifest
#
# We are NOT generating the complete Gazebo world yet.
#
# That comes in the Gazebo-dynamic-world step.
# ============================================================


def render_spawn_manifest(
    manifest: Dict[str, Any],
    worlds_dir: Path
) -> None:

    worlds_dir.mkdir(
        parents=True,
        exist_ok=True
    )


    data = {

        "gcs": {

            "id":
                manifest["gcs"]["id"],

            "model":
                manifest["gcs"]["gazebo_model"],

            "antenna_height_m":
                manifest["gcs"]["antenna_height_m"]
        },


        "uavs": [

            {

                "id":
                    uav["id"],

                "model":
                    uav["gazebo_model"],

                "spawn":
                    uav["spawn"],

                "gazebo_fdm_port":
                    uav["gazebo_fdm_port"]

            }

            for uav
            in manifest["uavs"]
        ]
    }


    output_file = (

        worlds_dir
        / "spawn_points.json"
    )


    output_file.write_text(

        json.dumps(
            data,
            indent=2
        )
        + "\n",

        encoding="utf-8"
    )


# ============================================================
# Save main fleet manifest
# ============================================================


def write_manifest(
    manifest: Dict[str, Any],
    output_dir: Path
) -> Path:

    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )


    output_file = (

        output_dir
        / "fleet.json"
    )


    output_file.write_text(

        json.dumps(
            manifest,
            indent=2
        )
        + "\n",

        encoding="utf-8"
    )


    return output_file


# ============================================================
# Terminal output
# ============================================================


def print_summary(
    manifest: Dict[str, Any]
) -> None:

    print()

    print(
        f"Fleet: "
        f"{manifest['num_uavs']} UAV(s), "
        f"{manifest['num_radio_nodes']} radio nodes, "
        f"{manifest['expected_link_pairs']} "
        f"wireless link pairs"
    )

    print()


    print(

        f"{'UAV':<5}"

        f"{'IP':<16}"

        f"{'DDS':<7}"

        f"{'MAV':<7}"

        f"{'GZ-FDM':<9}"

        f"{'SITL-FDM':<11}"

        f"{'TAP':<14}"

        f"{'MODEL':<12}"
    )


    print(
        "-" * 81
    )


    for uav in manifest["uavs"]:

        print(

            f"{uav['id']:<5}"

            f"{uav['wireless_ip']:<16}"

            f"{uav['dds_port']:<7}"

            f"{uav['mavlink_port']:<7}"

            f"{uav['gazebo_fdm_port']:<9}"

            f"{uav['sitl_fdm_port']:<11}"

            f"{uav['tap']:<14}"

            f"{uav['gazebo_model']:<12}"
        )


# ============================================================
# Command-line arguments
# ============================================================


def parse_args() -> argparse.Namespace:

    parser = argparse.ArgumentParser(

        description=(

            "Generate resolved configuration "
            "for a dynamic multi-UAV fleet."
        )
    )


    parser.add_argument(

        "--config",

        default="config/fleet.yaml",

        help=(

            "Input YAML configuration. "
            "Default: config/fleet.yaml"
        )
    )


    parser.add_argument(

        "--outdir",

        default="generated",

        help=(

            "Output directory. "
            "Default: generated"
        )
    )


    parser.add_argument(

        "--count",

        type=int,

        default=None,

        help=(

            "Temporarily override "
            "fleet.num_uavs "
            "without modifying fleet.yaml."
        )
    )


    parser.add_argument(

        "--clean",

        action="store_true",

        help=(

            "Delete existing generated "
            "directory before generation."
        )
    )


    return parser.parse_args()


# ============================================================
# Main
# ============================================================


def main() -> int:

    args = parse_args()


    config_path = Path(
        args.config
    ).resolve()


    output_dir = Path(
        args.outdir
    ).resolve()


    try:

        # ----------------------------------------------------
        # Load YAML
        # ----------------------------------------------------

        config = load_yaml(
            config_path
        )


        # ----------------------------------------------------
        # Optional clean
        # ----------------------------------------------------

        if (
            args.clean
            and output_dir.exists()
        ):

            shutil.rmtree(
                output_dir
            )


        # ----------------------------------------------------
        # Generate fleet
        # ----------------------------------------------------

        manifest = make_fleet(

            config,

            count_override=args.count
        )


        # ----------------------------------------------------
        # Write main manifest
        # ----------------------------------------------------

        manifest_file = write_manifest(

            manifest,

            output_dir
        )


        # ----------------------------------------------------
        # DDS parameters
        # ----------------------------------------------------

        render_dds_params(

            manifest,

            output_dir
            / "params"
        )


        # ----------------------------------------------------
        # Gazebo spawn information
        # ----------------------------------------------------

        render_spawn_manifest(

            manifest,

            output_dir
            / "worlds"
        )


        # ----------------------------------------------------
        # Terminal summary
        # ----------------------------------------------------

        print_summary(
            manifest
        )


        print()

        print(
            f"Created: "
            f"{manifest_file}"
        )


        print(
            f"Created: "
            f"{output_dir / 'params'}"
        )


        print(
            f"Created: "
            f"{output_dir / 'worlds' / 'spawn_points.json'}"
        )


        print()

        print(
            "Fleet generation completed successfully."
        )


        return 0


    except (
        OSError,
        ValueError,
        KeyError
    ) as exc:

        print(

            f"\nERROR: {exc}\n",

            file=sys.stderr
        )

        return 1


if __name__ == "__main__":

    raise SystemExit(
        main()
    )