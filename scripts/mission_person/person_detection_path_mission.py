#!/usr/bin/env python3
"""Fly UAV1 through the person-detection observation path.

Sequence:
  1. Arm/take off when necessary.
  2. Fly to the comparison pose and hold there.
  3. Measure the stopped UAV's heading and fly 20 m straight forward.
  4. Fly 10 m back along the same path.
  5. Hold, then command RTL.

Run this script inside ``gcsns`` after the single-UAV pipeline is ready.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
from std_srvs.srv import Trigger


# Reuse the tested telemetry, takeoff, goto, yaw and arrival implementation
# used by scripts/mission/goto_comparison_pose.py.
MISSION_DIR = Path(__file__).resolve().parents[1] / 'mission'
sys.path.insert(0, str(MISSION_DIR))
from goto_comparison_pose import (  # noqa: E402
    ComparisonPoseMission,
    DEFAULT_ALTITUDE_M,
    DEFAULT_LATITUDE,
    DEFAULT_LONGITUDE,
)


EARTH_RADIUS_M = 6_371_000.0


def offset_gps(latitude: float, longitude: float, distance_m: float,
               bearing_deg: float) -> tuple[float, float]:
    """Return a GPS point offset by distance along a compass bearing."""
    angular_distance = distance_m / EARTH_RADIUS_M
    bearing = math.radians(bearing_deg)
    latitude_1 = math.radians(latitude)
    longitude_1 = math.radians(longitude)

    latitude_2 = math.asin(
        math.sin(latitude_1) * math.cos(angular_distance)
        + math.cos(latitude_1) * math.sin(angular_distance)
        * math.cos(bearing)
    )
    longitude_2 = longitude_1 + math.atan2(
        math.sin(bearing) * math.sin(angular_distance)
        * math.cos(latitude_1),
        math.cos(angular_distance)
        - math.sin(latitude_1) * math.sin(latitude_2),
    )
    return math.degrees(latitude_2), math.degrees(longitude_2)


class PersonPathMission(ComparisonPoseMission):
    """Comparison-pose flight controls plus timed holds and RTL."""

    def __init__(self, uav_id: int) -> None:
        super().__init__(uav_id)
        self.rtl_client = self.create_client(Trigger, f'/uav{uav_id}/rtl')

    def hold(self, duration_s: float, label: str) -> None:
        """Hold the active target while continuing ROS callbacks/timers."""
        self.get_logger().info(f'{label}: holding for {duration_s:.0f}s')
        deadline = time.monotonic() + duration_s
        next_status = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            remaining = max(0.0, deadline - time.monotonic())
            if time.monotonic() >= next_status:
                self.get_logger().info(
                    f'{label}: {remaining:.0f}s remaining')
                next_status = time.monotonic() + 5.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Person-detection path: comparison pose, forward/back, RTL')
    parser.add_argument('--uav', type=int, default=1)
    parser.add_argument('--lat', type=float, default=DEFAULT_LATITUDE)
    parser.add_argument('--lon', type=float, default=DEFAULT_LONGITUDE)
    parser.add_argument('--alt', type=float, default=DEFAULT_ALTITUDE_M)
    parser.add_argument('--first-hold', type=float, default=20.0)
    parser.add_argument('--final-hold', type=float, default=20.0)
    parser.add_argument('--forward-distance', type=float, default=40.0)
    parser.add_argument('--back-distance', type=float, default=10.0)
    parser.add_argument('--horizontal-tolerance', type=float, default=1.5)
    parser.add_argument('--vertical-tolerance', type=float, default=1.0)
    parser.add_argument('--stable-seconds', type=float, default=3.0)
    parser.add_argument('--waypoint-timeout', type=float, default=180.0)
    parser.add_argument('--takeoff-alt', type=float, default=10.0)
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.uav != 1:
        raise ValueError('Only --uav 1 is currently supported')
    numeric_names = (
        'lat', 'lon', 'alt', 'first_hold', 'final_hold',
        'forward_distance', 'back_distance', 'horizontal_tolerance',
        'vertical_tolerance', 'stable_seconds', 'waypoint_timeout',
        'takeoff_alt',
    )
    for name in numeric_names:
        if not math.isfinite(getattr(args, name)):
            raise ValueError(f'--{name.replace("_", "-")} must be finite')
    positive_names = tuple(name for name in numeric_names
                           if name not in ('lat', 'lon'))
    if any(getattr(args, name) <= 0.0 for name in positive_names):
        raise ValueError('distances, durations, tolerances and altitudes must be positive')
    if args.back_distance >= args.forward_distance:
        raise ValueError('--back-distance must be less than --forward-distance')
    if not -90.0 <= args.lat <= 90.0:
        raise ValueError('--lat must be between -90 and 90')
    if not -180.0 <= args.lon <= 180.0:
        raise ValueError('--lon must be between -180 and 180')


def reach_target(node: PersonPathMission, latitude: float, longitude: float,
                 args: argparse.Namespace, label: str,
                 yaw_command: float | None = None) -> bool:
    yaw_text = ('unchanged' if yaw_command is None
                else f'held at {yaw_command:.1f}deg')
    node.get_logger().info(
        f'{label}: lat={latitude:.7f}, lon={longitude:.7f}, '
        f'alt={args.alt:.1f}m (yaw {yaw_text})')
    node.set_target(latitude, longitude, args.alt, yaw_command)
    return node.wait_until_stable(
        latitude, longitude, args.alt, None,
        args.horizontal_tolerance, args.vertical_tolerance,
        args.stable_seconds, args.waypoint_timeout,
    )


def run_mission(args: argparse.Namespace) -> int:
    rclpy.init()
    node = PersonPathMission(args.uav)
    try:
        if not node.wait_for_state(require_heading=True, timeout=30.0):
            node.get_logger().error('Timed out waiting for UAV telemetry')
            return 1
        if not node.wait_for_command_interfaces(require_yaw=True):
            node.get_logger().error('Flight-control interfaces unavailable')
            return 1
        if not node.rtl_client.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f'/uav{args.uav}/rtl is unavailable')
            return 1

        if not node.armed and not node.call_trigger(
                node.arm_client, f'/uav{args.uav}/arm', 50.0):
            return 1
        if node.relative_altitude < 1.0:
            if not node.call_trigger(
                    node.takeoff_client, f'/uav{args.uav}/takeoff', 90.0):
                return 1
            if not node.wait_for_altitude(args.takeoff_alt, 45.0):
                return 1

        node.get_logger().info('PERSON-DETECTION PATH MISSION STARTED')
        node.get_logger().info(
            f'Path: comparison pose -> {args.forward_distance:.0f}m forward '
            f'-> {args.back_distance:.0f}m back -> RTL')

        if not reach_target(node, args.lat, args.lon, args,
                            'Step 1 - comparison pose'):
            return 1
        node.hold(args.first_hold, 'Step 1 - comparison pose')

        # Measure heading only after the vehicle has stopped and completed its
        # hold. This makes "forward" follow its actual pose without rotating it.
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.heading_deg is None:
            node.get_logger().error('No heading available at comparison pose')
            return 1
        forward_bearing = node.heading_deg
        forward_lat, forward_lon = offset_gps(
            args.lat, args.lon, args.forward_distance, forward_bearing)
        final_lat, final_lon = offset_gps(
            forward_lat, forward_lon, args.back_distance,
            (forward_bearing + 180.0) % 360.0)
        node.get_logger().info(
            f'Using stopped UAV heading {forward_bearing:.1f}deg as forward; '
            'holding that pose through the forward and backward legs')

        if not reach_target(node, forward_lat, forward_lon, args,
                            'Step 2 - forward point', forward_bearing):
            return 1

        if not reach_target(node, final_lat, final_lon, args,
                            'Step 3 - 10m back point', forward_bearing):
            return 1
        node.hold(args.final_hold, 'Step 3 - final observation point')

        node.set_target(None, None, args.alt, None)
        node.get_logger().info('Step 4 - commanding RTL')
        if not node.call_trigger(
                node.rtl_client, f'/uav{args.uav}/rtl', 30.0):
            return 1
        node.get_logger().info('MISSION COMPLETE - RTL accepted')
        return 0
    except KeyboardInterrupt:
        node.get_logger().warning(
            'Mission interrupted; RTL was not commanded automatically')
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    try:
        args = parse_args()
        validate_args(args)
    except ValueError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    return run_mission(args)


if __name__ == '__main__':
    raise SystemExit(main())
