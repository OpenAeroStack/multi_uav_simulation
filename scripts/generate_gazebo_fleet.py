#!/usr/bin/env python3
"""
generate_gazebo_fleet.py

Generate a Gazebo Classic world for the fleet size selected in config/fleet.yaml.

Behaviour:
- Keeps the existing city world/environment unchanged.
- Removes fixed top-level Gazebo models named iris_<number>.
- Re-inserts iris_1 ... iris_N using spawn positions from fleet.yaml.
- Keeps the existing GCS and obstacle plugin unchanged.
- If iris_4 / iris_5 model directories do not yet exist, clones iris_1
  and updates the ArduPilot FDM port numbers using the fleet port mapping.

The script is intended for startup-time fleet sizes, not runtime hot-add/remove.
"""

from __future__ import annotations

import argparse
import copy
import re
import shutil
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


IRIS_RE = re.compile(r"^iris_(\d+)$")


def load_config(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    if not isinstance(cfg, dict):
        raise ValueError(f"Invalid YAML root in {path}")

    return cfg


def selected_count(cfg: dict, override: int | None) -> int:
    if override is not None:
        n = int(override)
    else:
        n = int(cfg["fleet"]["num_uavs"])

    if n < 1:
        raise ValueError("num_uavs must be >= 1")

    return n


def port_values(cfg: dict, uid: int) -> tuple[int, int]:
    ports = cfg["ports"]
    k = uid - 1

    gazebo_fdm = int(ports["gazebo_fdm_base"]) + k * int(ports["fdm_stride"])
    sitl_fdm = int(ports["sitl_fdm_base"]) + k * int(ports["fdm_stride"])

    return gazebo_fdm, sitl_fdm


def fallback_spawn(cfg: dict, uid: int) -> list[float]:
    """
    Generic fallback only.

    Explicit uav_overrides are preferred for UAV1..UAV5. The fallback places
    UAVs on the same Y-axis staging line around UAV1.
    """
    gazebo = cfg["gazebo"]
    origin = [float(v) for v in gazebo["spawn_origin"]]
    spacing = float(gazebo["spawn_spacing_m"])

    x, y, z = origin

    if uid == 1:
        return [x, y, z]

    # Alternating line:
    # UAV2 = -1 spacing
    # UAV3 = +1 spacing
    # UAV4 = -2 spacing
    # UAV5 = +2 spacing
    side_index = (uid - 2) // 2 + 1
    direction = -1.0 if uid % 2 == 0 else 1.0

    return [x, y + direction * side_index * spacing, z]


def spawn_for(cfg: dict, uid: int) -> list[float]:
    overrides = cfg.get("uav_overrides", {}) or {}
    override = overrides.get(str(uid), {}) or {}

    if "spawn" in override:
        spawn = override["spawn"]

        if not isinstance(spawn, list) or len(spawn) != 3:
            raise ValueError(
                f"uav_overrides.{uid}.spawn must contain [x, y, z]"
            )

        return [float(v) for v in spawn]

    return fallback_spawn(cfg, uid)


def sim_ip_prefix(cfg: dict, uid: int) -> str:
    """
    Return the first three octets (with trailing dot) of the Gazebo/SITL
    simulation-network address for ``uid`` e.g. "172.31.4.".

    This is the address family the ArduPilot Gazebo plugin must listen on so
    that a SITL instance running inside network namespace ``uavN`` can reach it.
    """
    template = cfg["network"]["sim_ip_template"]
    addr = template.format(id=uid)
    return addr.rsplit(".", 1)[0] + "."


def patch_text_file(
    path: Path,
    uid: int,
    source_gazebo_port: int,
    source_sitl_port: int,
    target_gazebo_port: int,
    target_sitl_port: int,
    source_sim_prefix: str = "",
    target_sim_prefix: str = "",
) -> tuple[int, int]:
    try:
        original = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        return 0, 0

    updated = original

    # Rename any explicit template model references.
    updated = updated.replace("iris_1", f"iris_{uid}")

    # Per-UAV camera sensor name and ROS namespace in the template
    # ("iris1_camera", "<namespace>/uav1</namespace>").
    updated = updated.replace("iris1_camera", f"iris{uid}_camera")
    updated = updated.replace("/uav1<", f"/uav{uid}<")

    # Remap the Gazebo/SITL simulation-network address so the ArduPilot
    # plugin listens on the namespace-reachable interface (172.31.<uid>.x).
    if source_sim_prefix and target_sim_prefix:
        updated = updated.replace(source_sim_prefix, target_sim_prefix)

    gazebo_count = updated.count(str(source_gazebo_port))
    sitl_count = updated.count(str(source_sitl_port))

    updated = updated.replace(
        str(source_gazebo_port),
        str(target_gazebo_port),
    )
    updated = updated.replace(
        str(source_sitl_port),
        str(target_sitl_port),
    )

    if updated != original:
        path.write_text(updated, encoding="utf-8")

    return gazebo_count, sitl_count


def ensure_model(
    cfg: dict,
    project_models_dir: Path,
    uid: int,
    force: bool,
) -> None:
    target = project_models_dir / f"iris_{uid}"

    if uid == 1:
        return

    target_sim_prefix = sim_ip_prefix(cfg, uid)
    source_sim_prefix = sim_ip_prefix(cfg, 1)

    # Preserve an existing model directory only when it already carries the
    # namespace-reachable FDM listen address (172.31.<uid>.1). A stale model
    # that points the ArduPilot plugin at 127.0.0.1 is unreachable from the
    # SITL instance inside network namespace uav<uid>, so it must be rebuilt.
    if target.exists() and not force:
        sdf = target / "model.sdf"
        try:
            sdf_text = sdf.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            sdf_text = ""

        if f"<listen_addr>{target_sim_prefix}1</listen_addr>" in sdf_text:
            return

        print(
            f"[gazebo-model] iris_{uid}: existing model does not listen on "
            f"{target_sim_prefix}1 (namespace-unreachable); regenerating."
        )

    template = project_models_dir / "iris_1"

    if not template.is_dir():
        raise FileNotFoundError(
            f"Template Gazebo model does not exist: {template}"
        )

    if target.exists():
        shutil.rmtree(target)

    shutil.copytree(template, target)

    source_gazebo_port, source_sitl_port = port_values(cfg, 1)
    target_gazebo_port, target_sitl_port = port_values(cfg, uid)

    total_gazebo_replacements = 0
    total_sitl_replacements = 0

    for path in target.rglob("*"):
        if not path.is_file():
            continue

        g_count, s_count = patch_text_file(
            path,
            uid,
            source_gazebo_port,
            source_sitl_port,
            target_gazebo_port,
            target_sitl_port,
            source_sim_prefix,
            target_sim_prefix,
        )

        total_gazebo_replacements += g_count
        total_sitl_replacements += s_count

    if total_gazebo_replacements == 0:
        shutil.rmtree(target, ignore_errors=True)
        raise RuntimeError(
            f"Could not find template Gazebo FDM port "
            f"{source_gazebo_port} inside {template}. "
            "The model layout must be inspected before safely generating "
            f"iris_{uid}."
        )

    print(
        f"[gazebo-model] iris_{uid}: "
        f"Gazebo FDM {source_gazebo_port}->{target_gazebo_port}, "
        f"SITL FDM {source_sitl_port}->{target_sitl_port} "
        f"(SITL replacements={total_sitl_replacements})"
    )


def local_name(tag) -> str:
    # XML comments are also preserved by ElementTree.
    # Comment nodes do not have a normal string tag.
    if not isinstance(tag, str):
        return ""

    if "}" in tag:
        return tag.rsplit("}", 1)[1]

    return tag


def find_world(root: ET.Element) -> ET.Element:
    if local_name(root.tag) == "world":
        return root

    for elem in root.iter():
        if local_name(elem.tag) == "world":
            return elem

    raise RuntimeError("No <world> element found in SDF template")


def remove_existing_uavs(world: ET.Element) -> None:
    for child in list(world):
        if local_name(child.tag) != "model":
            continue

        name = child.attrib.get("name", "")

        if IRIS_RE.fullmatch(name):
            world.remove(child)


def find_gcs_insert_index(world: ET.Element) -> int:
    children = list(world)

    for index, child in enumerate(children):
        if (
            local_name(child.tag) == "model"
            and child.attrib.get("name") == "gcs"
        ):
            return index

    # If GCS is absent, put UAVs near the end.
    return len(children)


def build_uav_model(uid: int, spawn: list[float]) -> ET.Element:
    model = ET.Element("model", {"name": f"iris_{uid}"})

    pose = ET.SubElement(model, "pose")
    pose.text = " ".join(f"{value:g}" for value in spawn) + " 0 0 0"

    include = ET.SubElement(model, "include")

    uri = ET.SubElement(include, "uri")
    uri.text = f"model://iris_{uid}"

    return model


def indent_tree(tree: ET.ElementTree) -> None:
    # Python 3.9+
    ET.indent(tree, space="  ")


def main() -> int:
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--config",
        default="config/fleet.yaml",
        help="fleet.yaml path",
    )

    parser.add_argument(
        "--template",
        default="worlds/city_3uav.world",
        help="Existing city world used as environment template",
    )

    parser.add_argument(
        "--output",
        default=None,
        help="Generated world path",
    )

    parser.add_argument(
        "--models-dir",
        default="models",
        help="Project Gazebo models directory",
    )

    parser.add_argument(
        "--count",
        type=int,
        default=None,
        help="Optional fleet-size override",
    )

    parser.add_argument(
        "--force-models",
        action="store_true",
        help="Regenerate iris_2..iris_N from iris_1 (normally not needed)",
    )

    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    template_path = Path(args.template).resolve()
    models_dir = Path(args.models_dir).resolve()

    cfg = load_config(config_path)
    n = selected_count(cfg, args.count)

    if args.output:
        output_path = Path(args.output).resolve()
    else:
        output_path = (
            config_path.parent.parent
            / "generated"
            / "worlds"
            / f"city_{n}uav.world"
        ).resolve()

    if not template_path.is_file():
        raise FileNotFoundError(
            f"Gazebo world template not found: {template_path}"
        )

    # Ensure every referenced model exists.
    for uid in range(1, n + 1):
        ensure_model(
            cfg,
            models_dir,
            uid,
            force=(args.force_models and uid > 1),
        )

    parser_xml = ET.XMLParser(
        target=ET.TreeBuilder(insert_comments=True)
    )

    tree = ET.parse(template_path, parser=parser_xml)
    root = tree.getroot()
    world = find_world(root)

    remove_existing_uavs(world)

    insert_at = find_gcs_insert_index(world)

    generated = []

    for uid in range(1, n + 1):
        spawn = spawn_for(cfg, uid)
        model = build_uav_model(uid, spawn)

        world.insert(insert_at, model)
        insert_at += 1

        gazebo_port, sitl_port = port_values(cfg, uid)

        generated.append(
            (
                uid,
                spawn,
                gazebo_port,
                sitl_port,
            )
        )

    output_path.parent.mkdir(parents=True, exist_ok=True)

    indent_tree(tree)

    tree.write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
    )

    print()
    print("=" * 68)
    print("DYNAMIC GAZEBO WORLD GENERATED")
    print("=" * 68)
    print(f"Fleet size : {n}")
    print(f"Template   : {template_path}")
    print(f"Output     : {output_path}")
    print()
    print("UAV   SPAWN (x,y,z)                    GZ-FDM   SITL-FDM")
    print("-" * 68)

    for uid, spawn, gazebo_port, sitl_port in generated:
        print(
            f"UAV{uid:<2} "
            f"{str(spawn):<32} "
            f"{gazebo_port:<8} "
            f"{sitl_port}"
        )

    print("=" * 68)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(
            f"ERROR: {exc}",
            file=sys.stderr,
        )
        raise SystemExit(1)
