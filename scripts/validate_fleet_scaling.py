#!/usr/bin/env python3
"""
validate_fleet_scaling.py

Prove that changing `fleet.num_uavs` in config/fleet.yaml is the only edit a
new fleet size needs.

For each requested size this runs the whole generation pipeline into a scratch
directory and checks the artefacts it produces, without starting Gazebo, SITL,
ns-3 or ROS. It is a fast pre-flight for `launch_city_dynamic_clustering.sh`:
if this passes, the launcher has everything it needs; if it fails, the launcher
would have failed later and more expensively.

Checks per fleet size
---------------------
  addressing   every wireless IP, sim IP, TAP, namespace, DDS port, MAVLink
               port and both FDM ports are unique, and the spawn points are
               distinct
  params       one DDS .parm per UAV, each with the DDS_UDP_PORT the fleet
               map assigns it
  models       models/iris_N exists for every UAV and its ArduPilotPlugin
               points at the netns-reachable 172.31.N.1/.2 pair on the right
               ports -- a model left on 127.0.0.1 makes SITL block forever
               waiting for its first FDM frame
  world        the generated world contains exactly N iris models, each
               referencing its own model:// URI
  mission      city_mission assigns a role to every UAV in the fleet

Usage
-----
    scripts/validate_fleet_scaling.py                # sweep 1..6
    scripts/validate_fleet_scaling.py --sizes 3 5 8
    scripts/validate_fleet_scaling.py --sizes 5 --keep   # leave artefacts
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]

# Fields that must be unique across the fleet, and what to call them when the
# uniqueness check fails.
UNIQUE_FIELDS = (
    'wireless_ip',
    'sim_ip',
    'tap',
    'namespace',
    'dds_port',
    'mavlink_port',
    'gazebo_fdm_port',
    'sitl_fdm_port',
    'gazebo_model',
)


class Failure(Exception):
    """One failed check, reported against a fleet size."""


def run(command: list[str], cwd: Path = PROJECT_ROOT) -> str:
    result = subprocess.run(
        command, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=180,
    )
    if result.returncode != 0:
        raise Failure(
            f'{" ".join(command)} exited {result.returncode}\n'
            f'{result.stdout.strip()}')
    return result.stdout


# ── individual checks ────────────────────────────────────────────────────────

def check_addressing(fleet: dict) -> list[str]:
    uavs = fleet['uavs']
    notes = []

    if len(uavs) != fleet['num_uavs']:
        raise Failure(
            f'fleet.json says num_uavs={fleet["num_uavs"]} '
            f'but lists {len(uavs)} UAVs')

    for field in UNIQUE_FIELDS:
        values = [u[field] for u in uavs]
        if len(set(values)) != len(values):
            duplicates = sorted(
                {v for v in values if values.count(v) > 1})
            raise Failure(f'duplicate {field}: {duplicates}')

    spawns = [tuple(u['spawn']) for u in uavs]
    if len(set(spawns)) != len(spawns):
        raise Failure('two UAVs share a spawn point')

    # ns-3 addresses the fleet out of one /24 with the GCS at .10.
    gcs_ip = fleet['gcs']['wireless_ip']
    if gcs_ip in {u['wireless_ip'] for u in uavs}:
        raise Failure(f'a UAV was given the GCS address {gcs_ip}')

    notes.append(f'{len(uavs)} UAVs, no address or port collisions')
    return notes


def check_params(fleet: dict, outdir: Path) -> list[str]:
    for uav in fleet['uavs']:
        path = outdir / 'params' / f'uav{uav["id"]}_dds.parm'
        if not path.is_file():
            raise Failure(f'missing DDS parameter file {path.name}')

        text = path.read_text()
        match = re.search(r'^DDS_UDP_PORT\s+(\d+)', text, re.M)
        if not match:
            raise Failure(f'{path.name} has no DDS_UDP_PORT')
        if int(match.group(1)) != uav['dds_port']:
            raise Failure(
                f'{path.name} has DDS_UDP_PORT {match.group(1)} '
                f'but the fleet map assigns {uav["dds_port"]}')

    return [f'{len(fleet["uavs"])} DDS parameter files match the port map']


def check_models(fleet: dict) -> list[str]:
    for uav in fleet['uavs']:
        uid = uav['id']
        sdf = PROJECT_ROOT / 'models' / f'iris_{uid}' / 'model.sdf'
        if not sdf.is_file():
            raise Failure(f'models/iris_{uid}/model.sdf does not exist')

        text = sdf.read_text()

        # Gazebo runs in the root namespace and each SITL inside netns uavN,
        # so the plugin has to use the veth pair, never loopback.
        expected = [
            (f'<listen_addr>172.31.{uid}.1</listen_addr>', 'listen_addr'),
            (f'<fdm_addr>172.31.{uid}.2</fdm_addr>', 'fdm_addr'),
            (f'<fdm_port_in>{uav["gazebo_fdm_port"]}</fdm_port_in>',
             'fdm_port_in'),
            (f'<fdm_port_out>{uav["sitl_fdm_port"]}</fdm_port_out>',
             'fdm_port_out'),
        ]
        for needle, label in expected:
            if needle not in text:
                raise Failure(
                    f'models/iris_{uid}/model.sdf has the wrong {label} '
                    f'(expected {needle})')

        if '127.0.0.1' in text:
            raise Failure(
                f'models/iris_{uid}/model.sdf still points the FDM at '
                f'127.0.0.1; SITL would block forever waiting for Gazebo')

    return [f'{len(fleet["uavs"])} iris models use netns-reachable FDM addresses']


def check_world(fleet: dict, world: Path) -> list[str]:
    if not world.is_file():
        raise Failure(f'world file {world} was not generated')

    text = world.read_text()
    found = sorted(int(m) for m in re.findall(r'<uri>model://iris_(\d+)</uri>',
                                              text))
    expected = [u['id'] for u in fleet['uavs']]

    if found != expected:
        raise Failure(
            f'world contains iris models {found}, expected {expected}')

    return [f'world spawns exactly {len(expected)} iris models']


def check_mission_roles(size: int) -> list[str]:
    """Every UAV must be given a mission worker by city_mission."""
    source = (PROJECT_ROOT / 'ros2' / 'uav_controller' / 'uav_controller'
              / 'city_mission.py').read_text()

    named = len(re.findall(r'def _mission_uav(\d+)\(self\)', source))
    if 'def _mission_follower' not in source:
        raise Failure('city_mission has no generic follower worker')

    lower_bound = re.search(r'self\.num_uavs < (\d+)', source)
    if lower_bound and size < int(lower_bound.group(1)):
        raise Failure(f'city_mission rejects num_uavs={size}')

    if re.search(r'num_uavs not in \(', source):
        raise Failure(
            'city_mission still has a hardcoded set of supported fleet sizes')

    roles = min(size, named)
    followers = max(0, size - named)
    return [f'{roles} named role(s) + {followers} generic follower(s)']


# ── driver ───────────────────────────────────────────────────────────────────

def validate(size: int, keep: bool) -> tuple[bool, list[str]]:
    scratch = Path(tempfile.mkdtemp(prefix=f'fleetcheck{size}-'))
    notes: list[str] = []
    try:
        run(['python3', 'scripts/generate_fleet.py',
             '--count', str(size), '--outdir', str(scratch), '--clean'])

        fleet = json.loads((scratch / 'fleet.json').read_text())
        notes += check_addressing(fleet)

        run(['python3', 'scripts/generate_dds_params.py',
             '--count', str(size), '--outdir', str(scratch / 'params'),
             '--clean'])
        notes += check_params(fleet, scratch)

        world = scratch / f'city_{size}uav.world'
        run(['python3', 'scripts/generate_gazebo_fleet.py',
             '--config', 'config/fleet.yaml',
             '--template', 'worlds/city_3uav.world',
             '--output', str(world),
             '--models-dir', 'models',
             '--count', str(size)])
        notes += check_models(fleet)
        notes += check_world(fleet, world)
        notes += check_mission_roles(size)
        return True, notes

    except Failure as exc:
        notes.append(str(exc))
        return False, notes
    finally:
        if keep:
            notes.append(f'artefacts kept in {scratch}')
        else:
            shutil.rmtree(scratch, ignore_errors=True)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Check that the fleet pipeline scales to any num_uavs.')
    parser.add_argument('--sizes', type=int, nargs='+',
                        default=[1, 2, 3, 4, 5, 6],
                        help='fleet sizes to validate (default: 1..6)')
    parser.add_argument('--keep', action='store_true',
                        help='keep generated artefacts for inspection')
    args = parser.parse_args()

    print('=' * 66)
    print('FLEET SCALING VALIDATION')
    print('=' * 66)
    print(f'Sizes: {", ".join(str(s) for s in args.sizes)}')
    print()

    failures = []
    for size in args.sizes:
        ok, notes = validate(size, args.keep)
        print(f'{"PASS" if ok else "FAIL"}  N={size}')
        for note in notes:
            for line in str(note).splitlines():
                print(f'        {line}')
        print()
        if not ok:
            failures.append(size)

    print('=' * 66)
    if failures:
        print(f'FAILED for fleet size(s): '
              f'{", ".join(str(s) for s in failures)}')
        return 1

    print(f'All {len(args.sizes)} fleet size(s) validated.')
    print('config/fleet.yaml `fleet.num_uavs` is the only edit a resize needs.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
