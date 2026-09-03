#!/usr/bin/env python3
"""
summarize_run.py

Summarise the last simulation run from its logs, so runs at different fleet
sizes can be compared on the same terms instead of by scrolling.

Reads (all optional -- whatever is present gets reported):

    /tmp/city_mission.log              waypoints, link loss, completion
    /tmp/dynamic_cluster_manager.log   elections and relay decisions
    /tmp/micro_ros_agent_uavN.log      DDS session health per UAV
    /tmp/ns3_link_validation.csv       per-link SNR

Usage:
    scripts/summarize_run.py
    scripts/summarize_run.py --save results/run_5uav.txt
"""

from __future__ import annotations

import argparse
import csv
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

MISSION_LOG = Path('/tmp/city_mission.log')
CLUSTER_LOG = Path('/tmp/dynamic_cluster_manager.log')
LINK_CSV = Path('/tmp/ns3_link_validation.csv')
AGENT_LOG = '/tmp/micro_ros_agent_uav{}.log'

UAV = re.compile(r'\[UAV(\d+)\]')


def mission_section(out: list[str]) -> set[int]:
    """Per-UAV waypoint and link-loss tally. Returns the UAV ids seen."""
    if not MISSION_LOG.is_file():
        out.append(f'  {MISSION_LOG} not found')
        return set()

    reached = defaultdict(int)
    assumed = defaultdict(int)
    lost = defaultdict(int)
    recovered = defaultdict(int)
    gaveup = defaultdict(int)
    timeout = defaultdict(int)
    complete = set()
    landed = set()
    seen = set()

    for line in MISSION_LOG.read_text(errors='replace').splitlines():
        match = UAV.search(line)
        if not match:
            continue
        uid = int(match.group(1))
        seen.add(uid)

        if '✓ Reached' in line:
            reached[uid] += 1
        elif 'assuming reached' in line:
            assumed[uid] += 1
        elif 'LINK LOST en route' in line or 'Link lost near' in line:
            lost[uid] += 1
        elif 'Link recovered' in line:
            recovered[uid] += 1
        elif 'Giving up on' in line:
            gaveup[uid] += 1
        elif 'Timeout reaching' in line:
            timeout[uid] += 1

        if 'Mission complete' in line:
            complete.add(uid)
        if 'Touched down' in line or 'Landed and disarmed' in line:
            landed.add(uid)

    if not seen:
        out.append('  no per-UAV mission lines found')
        return set()

    out.append(f'  {"uav":>4} {"reached":>8} {"assumed":>8} {"lost":>5} '
               f'{"recov":>6} {"gaveup":>7} {"t/out":>6} {"landed":>7} '
               f'{"done":>5}')
    for uid in sorted(seen):
        out.append(
            f'  {uid:>4} {reached[uid]:>8} {assumed[uid]:>8} {lost[uid]:>5} '
            f'{recovered[uid]:>6} {gaveup[uid]:>7} {timeout[uid]:>6} '
            f'{"yes" if uid in landed else "NO":>7} '
            f'{"yes" if uid in complete else "NO":>5}')

    missing = sorted(seen - complete)
    out.append('')
    if missing:
        out.append(f'  INCOMPLETE: UAV{", UAV".join(str(m) for m in missing)}')
    else:
        out.append(f'  all {len(seen)} UAVs completed')

    text = MISSION_LOG.read_text(errors='replace')
    if 'CITY MISSION COMPLETE' in text:
        out.append('  mission reported: COMPLETE')
    else:
        out.append('  mission reported: did not reach the completion banner')

    return seen


def cluster_section(out: list[str]) -> None:
    if not CLUSTER_LOG.is_file():
        out.append(f'  {CLUSTER_LOG} not found')
        return

    elections = []
    relays = []
    for line in CLUSTER_LOG.read_text(errors='replace').splitlines():
        if 'Epoch' in line and 'primary=' in line:
            elections.append(line.split(']')[-1].strip())
        if 'Relay:' in line:
            relays.append(line.split('Relay:')[-1].strip())

    out.append(f'  elections logged : {len(elections)}')
    for entry in elections[:6]:
        out.append(f'    {entry}')
    if len(elections) > 6:
        out.append(f'    ... {len(elections) - 6} more')

    out.append(f'  relay changes    : {len(relays)}')
    for entry in relays:
        out.append(f'    {entry}')
    if not relays:
        out.append('    (none -- every UAV held a usable direct link)')


def dds_section(out: list[str], uavs: set[int]) -> None:
    if not uavs:
        uavs = set(range(1, 6))

    any_found = False
    for uid in sorted(uavs):
        path = Path(AGENT_LOG.format(uid))
        if not path.is_file():
            continue
        any_found = True
        text = path.read_text(errors='replace')
        established = text.count('session established')
        re_established = text.count('session re-established')
        participants = text.count('participant created')
        publishers = text.count('create_publisher')

        health = 'ok'
        if participants and not publishers:
            health = 'STALLED after participant'
        elif re_established:
            health = f'{re_established} session drop(s)'
        elif not established:
            health = 'never connected'

        out.append(f'  UAV{uid}: established={established} '
                   f'participants={participants} -> {health}')

    if not any_found:
        out.append('  no micro_ros_agent logs found')


def link_section(out: list[str]) -> None:
    if not LINK_CSV.is_file():
        out.append(f'  {LINK_CSV} not found')
        return

    per_link = defaultdict(list)
    try:
        with LINK_CSV.open() as handle:
            for row in csv.DictReader(handle):
                key = (int(row['node_a']), int(row['node_b']))
                per_link[key].append(float(row['snr_db']))
    except (KeyError, ValueError) as exc:
        out.append(f'  could not parse {LINK_CSV.name}: {exc}')
        return

    if not per_link:
        out.append('  no link samples recorded')
        return

    out.append('  node 0 = GCS. Links to the GCS decide whether telemetry '
               'flows.')
    out.append(f'  {"link":>8} {"mean":>7} {"min":>7} {"max":>7} '
               f'{"samples":>8}')
    for (a, b), values in sorted(per_link.items()):
        if a != 0:
            continue
        out.append(f'  {f"GCS-{b}":>8} {statistics.mean(values):>7.1f} '
                   f'{min(values):>7.1f} {max(values):>7.1f} '
                   f'{len(values):>8}')

    weak = [f'GCS-UAV{b}' for (a, b), v in sorted(per_link.items())
            if a == 0 and statistics.mean(v) < 15.0]
    out.append('')
    if weak:
        out.append(f'  below the 15 dB relay threshold on average: '
                   f'{", ".join(weak)}')
    else:
        out.append('  every GCS link averaged above the 15 dB relay threshold')


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Summarise the most recent simulation run.')
    parser.add_argument('--save', type=Path,
                        help='also write the summary to this file')
    args = parser.parse_args()

    out: list[str] = []
    out.append('=' * 68)
    out.append('RUN SUMMARY')
    out.append('=' * 68)

    out.append('')
    out.append('MISSION')
    uavs = mission_section(out)

    out.append('')
    out.append('CLUSTERING AND RELAY')
    cluster_section(out)

    out.append('')
    out.append('DDS SESSION HEALTH')
    dds_section(out, uavs)

    out.append('')
    out.append('GCS LINK QUALITY (ns-3)')
    link_section(out)

    out.append('')
    out.append('=' * 68)

    report = '\n'.join(out)
    print(report)

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        args.save.write_text(report + '\n', encoding='utf-8')
        print(f'\nsaved to {args.save}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
