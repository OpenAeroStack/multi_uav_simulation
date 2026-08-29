#!/usr/bin/env python3
"""Safely stage three UAVs, move them to fixed comparison poses, and hold."""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger


EARTH_RADIUS_M = 6_371_000.0
FINAL_POSES = {
    1: {'lat': 6.079429, 'lon': 80.193085, 'alt': 25.0, 'yaw': None},
    2: {'lat': 6.079339, 'lon': 80.193222, 'alt': 25.0, 'yaw': 0.0},
    3: {'lat': 6.079609, 'lon': 80.193214, 'alt': 25.0, 'yaw': 180.0},
}

AP_DDS_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def haversine_m(lat1: float, lon1: float,
                lat2: float, lon2: float) -> float:
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2)
         * math.sin(dlambda / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def angular_error_deg(current: float, target: float) -> float:
    """Smallest absolute difference between two compass headings."""
    return abs((current - target + 180.0) % 360.0 - 180.0)


def move_gps(lat: float, lon: float, distance_m: float,
             bearing_deg: float) -> tuple[float, float]:
    """Same spherical destination calculation used by waypoint_finder.py."""
    bearing = math.radians(bearing_deg)
    lat1, lon1 = math.radians(lat), math.radians(lon)
    angular = distance_m / EARTH_RADIUS_M
    lat2 = math.asin(
        math.sin(lat1) * math.cos(angular)
        + math.cos(lat1) * math.sin(angular) * math.cos(bearing))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(angular) * math.cos(lat1),
        math.cos(angular) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def quaternion_to_compass_heading_deg(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    enu_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return (90.0 - enu_yaw_deg) % 360.0


def gps_to_local_xy(lat: float, lon: float,
                    origin_lat: float, origin_lon: float) -> tuple[float, float]:
    mean_lat = math.radians((lat + origin_lat) / 2.0)
    x = EARTH_RADIUS_M * math.radians(lon - origin_lon) * math.cos(mean_lat)
    y = EARTH_RADIUS_M * math.radians(lat - origin_lat)
    return x, y


def point_segment_distance(point: tuple[float, float],
                           start: tuple[float, float],
                           end: tuple[float, float]) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0,
                     ((px - ax) * dx + (py - ay) * dy) / length_squared))
    closest = (ax + t * dx, ay + t * dy)
    return math.hypot(px - closest[0], py - closest[1])


def orientation(a: tuple[float, float], b: tuple[float, float],
                c: tuple[float, float]) -> float:
    return ((b[0] - a[0]) * (c[1] - a[1])
            - (b[1] - a[1]) * (c[0] - a[0]))


def segments_intersect(a: tuple[float, float], b: tuple[float, float],
                       c: tuple[float, float], d: tuple[float, float]) -> bool:
    eps = 1e-9
    o1, o2 = orientation(a, b, c), orientation(a, b, d)
    o3, o4 = orientation(c, d, a), orientation(c, d, b)
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and \
       ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    return False


def segment_distance(a: tuple[float, float], b: tuple[float, float],
                     c: tuple[float, float], d: tuple[float, float]) -> float:
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    )


def segment_closest_approach(
        a: tuple[float, float], b: tuple[float, float],
        c: tuple[float, float], d: tuple[float, float]
) -> tuple[float, tuple[float, float], tuple[float, float], float, float]:
    """Return distance, closest points, and progress fractions on AB and CD."""
    ux, uy = b[0] - a[0], b[1] - a[1]
    vx, vy = d[0] - c[0], d[1] - c[1]
    wx, wy = a[0] - c[0], a[1] - c[1]
    aa, bb, cc = ux * ux + uy * uy, ux * vx + uy * vy, vx * vx + vy * vy
    dd, ee = ux * wx + uy * wy, vx * wx + vy * wy
    denominator = aa * cc - bb * bb
    small = 1e-12

    if aa <= small and cc <= small:
        s = t = 0.0
    elif aa <= small:
        s, t = 0.0, max(0.0, min(1.0, ee / cc))
    elif cc <= small:
        s, t = max(0.0, min(1.0, -dd / aa)), 0.0
    else:
        s_numerator = denominator
        t_numerator = denominator
        if denominator <= small:
            s_numerator = 0.0
            s_denominator = 1.0
            t_numerator = ee
            t_denominator = cc
        else:
            s_numerator = bb * ee - cc * dd
            t_numerator = aa * ee - bb * dd
            s_denominator = t_denominator = denominator
            if s_numerator < 0.0:
                s_numerator = 0.0
                t_numerator, t_denominator = ee, cc
            elif s_numerator > s_denominator:
                s_numerator = s_denominator
                t_numerator, t_denominator = ee + bb, cc
        if t_numerator < 0.0:
            t_numerator = 0.0
            if -dd < 0.0:
                s_numerator = 0.0
            elif -dd > aa:
                s_numerator = s_denominator
            else:
                s_numerator, s_denominator = -dd, aa
        elif t_numerator > t_denominator:
            t_numerator = t_denominator
            if -dd + bb < 0.0:
                s_numerator = 0.0
            elif -dd + bb > aa:
                s_numerator = s_denominator
            else:
                s_numerator, s_denominator = -dd + bb, aa
        s = 0.0 if abs(s_numerator) <= small else s_numerator / s_denominator
        t = 0.0 if abs(t_numerator) <= small else t_numerator / t_denominator

    point_ab = (a[0] + s * ux, a[1] + s * uy)
    point_cd = (c[0] + t * vx, c[1] + t * vy)
    return math.dist(point_ab, point_cd), point_ab, point_cd, s, t


def project_progress(point: tuple[float, float], start: tuple[float, float],
                     end: tuple[float, float]) -> tuple[float, float]:
    """Return unclamped segment fraction and along-track distance in metres."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return 1.0, 0.0
    fraction = ((point[0] - start[0]) * dx
                + (point[1] - start[1]) * dy) / length_squared
    return fraction, fraction * math.sqrt(length_squared)


@dataclass
class Target:
    latitude: float
    longitude: float
    altitude: float
    yaw_deg: Optional[float]


@dataclass
class PathGeometry:
    origin_latitude: float
    origin_longitude: float
    staging_xy: dict[int, tuple[float, float]]
    final_xy: dict[int, tuple[float, float]]
    uav3_conflict_xy: tuple[float, float]
    uav1_conflict_xy: tuple[float, float]
    conflict_distance: float
    uav3_conflict_fraction: float
    uav1_conflict_fraction: float


class VehicleState:
    """State and ROS interfaces for one source-confirmed UAV namespace."""

    def __init__(self, node: Node, uav_id: int) -> None:
        self.node = node
        self.uav_id = uav_id
        self.latitude: Optional[float] = None
        self.longitude: Optional[float] = None
        self.relative_altitude: Optional[float] = None
        self.armed: Optional[bool] = None
        self.heading_deg: Optional[float] = None
        self.target: Optional[Target] = None

        ns = f'/uav{uav_id}'
        ap_ns = f'/ap/v{uav_id}'
        node.create_subscription(
            NavSatFix, f'{ap_ns}/navsat', self._gps_callback, AP_DDS_QOS)
        node.create_subscription(
            PoseStamped, f'{ap_ns}/pose/filtered',
            self._pose_callback, AP_DDS_QOS)
        node.create_subscription(
            Float32, f'{ns}/rel_alt', self._altitude_callback, 10)
        node.create_subscription(Bool, f'{ns}/armed', self._armed_callback, 10)
        self.goto_publisher = node.create_publisher(GeoPoint, f'{ns}/goto', 10)
        self.yaw_publisher = node.create_publisher(
            GlobalPosition, f'{ap_ns}/cmd_gps_pose', 10)
        self.arm_client = node.create_client(Trigger, f'{ns}/arm')
        self.takeoff_client = node.create_client(Trigger, f'{ns}/takeoff')

    def _gps_callback(self, msg: NavSatFix) -> None:
        if (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                and (msg.latitude != 0.0 or msg.longitude != 0.0)):
            self.latitude, self.longitude = msg.latitude, msg.longitude

    def _pose_callback(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        if all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)):
            self.heading_deg = quaternion_to_compass_heading_deg(msg)

    def _altitude_callback(self, msg: Float32) -> None:
        if math.isfinite(msg.data):
            self.relative_altitude = float(msg.data)

    def _armed_callback(self, msg: Bool) -> None:
        self.armed = bool(msg.data)

    def state_ready(self, require_heading: bool) -> bool:
        return (self.latitude is not None and self.longitude is not None
                and self.relative_altitude is not None
                and self.armed is not None
                and (not require_heading or self.heading_deg is not None))

    def interfaces_ready(self, require_yaw: bool) -> bool:
        return (self.arm_client.service_is_ready()
                and self.takeoff_client.service_is_ready()
                and self.node.count_subscribers(f'/uav{self.uav_id}/goto') >= 1
                and (not require_yaw or self.node.count_subscribers(
                    f'/ap/v{self.uav_id}/cmd_gps_pose') >= 1))

    def publish_target(self) -> None:
        if self.target is None:
            return
        position = GeoPoint()
        position.latitude = self.target.latitude
        position.longitude = self.target.longitude
        position.altitude = self.target.altitude
        self.goto_publisher.publish(position)
        if self.target.yaw_deg is not None:
            yaw = GlobalPosition()
            yaw.header.stamp = self.node.get_clock().now().to_msg()
            yaw.header.frame_id = 'map'
            yaw.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
            yaw.latitude = self.target.latitude
            yaw.longitude = self.target.longitude
            yaw.altitude = self.target.altitude
            yaw.type_mask = (
                GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY
                | GlobalPosition.IGNORE_VZ | GlobalPosition.IGNORE_AFX
                | GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ
                | GlobalPosition.IGNORE_YAW_RATE)
            yaw.yaw = math.radians(self.target.yaw_deg)
            self.yaw_publisher.publish(yaw)


class ThreeUAVComparisonMission(Node):
    def __init__(self, dry_run: bool) -> None:
        super().__init__('goto_three_uav_comparison_poses')
        self.dry_run = dry_run
        self.vehicles = {uav: VehicleState(self, uav) for uav in (1, 2, 3)}
        self.create_timer(2.0, self.refresh_targets)

    def refresh_targets(self) -> None:
        # This guard is an explicit invariant: dry-run never publishes motion.
        if self.dry_run:
            return
        for vehicle in self.vehicles.values():
            vehicle.publish_target()

    def spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def print_readiness(self, require_heading: bool,
                        require_yaw: bool) -> None:
        print('UAV | State | Services/goto/yaw')
        for uav, vehicle in self.vehicles.items():
            state = 'READY' if vehicle.state_ready(require_heading) else 'NOT READY'
            interfaces = ('READY' if vehicle.interfaces_ready(require_yaw)
                          else 'NOT READY')
            print(f'{uav:>3} | {state:<9} | {interfaces}')

    def wait_for_all_ready(self, require_heading: bool,
                           require_yaw: bool, timeout: float) -> bool:
        def ready() -> bool:
            return all(v.state_ready(require_heading)
                       and v.interfaces_ready(require_yaw)
                       for v in self.vehicles.values())

        ok = self.spin_until(ready, timeout)
        self.print_readiness(require_heading, require_yaw)
        return ok

    def print_command_ownership(self) -> None:
        """Report competing command publishers discovered at mission start."""
        print('Command publisher ownership:')
        for uav in (1, 2, 3):
            for topic in (f'/uav{uav}/goto', f'/ap/v{uav}/cmd_gps_pose'):
                endpoints = self.get_publishers_info_by_topic(topic)
                others = [
                    f'{endpoint.node_namespace.rstrip("/")}/'
                    f'{endpoint.node_name}'
                    for endpoint in endpoints
                    if endpoint.node_name != self.get_name()
                ]
                status = ('none' if not others else ', '.join(others))
                print(f'  {topic}: other publishers = {status}')
                if others:
                    self.get_logger().warning(
                        f'Competing command publisher(s) on {topic}: {status}')

    def call_all(self, attribute: str, label: str,
                 timeout: float) -> bool:
        futures = {}
        for uav, vehicle in self.vehicles.items():
            client = getattr(vehicle, attribute)
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'UAV{uav} {label} service unavailable')
                return False
            futures[uav] = client.call_async(Trigger.Request())

        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(f.done() for f in futures.values()):
                break
        success = True
        for uav, future in futures.items():
            if not future.done():
                self.get_logger().error(f'UAV{uav} {label} timed out')
                success = False
                continue
            response = future.result()
            if response is None or not response.success:
                message = response.message if response is not None else 'no response'
                self.get_logger().error(f'UAV{uav} {label} failed: {message}')
                success = False
            else:
                print(f'UAV{uav} {label}: {response.message}')
        return success

    def assign_targets(self, targets: dict[int, Target],
                       publish_now: bool = True) -> None:
        for uav, target in targets.items():
            self.vehicles[uav].target = target
        if publish_now and not self.dry_run:
            # All assignments happen first; publications follow immediately in
            # one loop, avoiding sequential wait-for-arrival behavior.
            for uav in (1, 2, 3):
                if uav in targets:
                    self.vehicles[uav].publish_target()

    def current_targets(self, altitude: float,
                        yaw_by_uav: dict[int, Optional[float]]) -> dict[int, Target]:
        return {
            uav: Target(vehicle.latitude, vehicle.longitude, altitude,
                        yaw_by_uav[uav])
            for uav, vehicle in self.vehicles.items()
        }

    def wait_until_all_stable(
            self, targets: dict[int, Target], horizontal_tolerance: float,
            vertical_tolerance: float, stable_seconds: float,
            timeout: float, active_uavs=(1, 2, 3),
            minimum_separation: Optional[float] = None,
            yaw_tolerance: Optional[float] = None) -> bool:
        stable_since: dict[int, Optional[float]] = {
            uav: None for uav in active_uavs}
        deadline = time.monotonic() + timeout
        last_status = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            rows = []
            for uav in active_uavs:
                vehicle, target = self.vehicles[uav], targets[uav]
                if vehicle.latitude is None or vehicle.longitude is None \
                        or vehicle.relative_altitude is None:
                    stable_since[uav] = None
                    rows.append((uav, math.inf, math.inf, None, None,
                                 math.inf, 0.0))
                    continue
                horizontal = haversine_m(
                    vehicle.latitude, vehicle.longitude,
                    target.latitude, target.longitude)
                vertical = abs(vehicle.relative_altitude - target.altitude)
                yaw_error = (angular_error_deg(
                    vehicle.heading_deg, target.yaw_deg)
                    if target.yaw_deg is not None
                    and vehicle.heading_deg is not None else math.inf)
                yaw_ok = (yaw_tolerance is None or target.yaw_deg is None
                          or yaw_error <= yaw_tolerance)
                if horizontal <= horizontal_tolerance \
                        and vertical <= vertical_tolerance and yaw_ok:
                    stable_since[uav] = stable_since[uav] or now
                else:
                    stable_since[uav] = None
                stable_for = (now - stable_since[uav]
                              if stable_since[uav] is not None else 0.0)
                rows.append((uav, horizontal, vertical,
                             vehicle.heading_deg, target.yaw_deg,
                             yaw_error, stable_for))
            if now - last_status >= 1.0:
                separations = self.actual_separations()
                closest_actual = min(separations.values())
                print('UAV | horiz | vert | current yaw | target yaw | yaw err | stable')
                for (uav, horizontal, vertical, current_yaw, target_yaw,
                     yaw_error, stable_for) in rows:
                    nearest = min(
                        distance for pair, distance in separations.items()
                        if uav in pair)
                    current_text = (f'{current_yaw:.1f}'
                                    if current_yaw is not None else 'n/a')
                    target_text = (f'{target_yaw:.1f}'
                                   if target_yaw is not None else 'PRESERVE')
                    error_text = (f'{yaw_error:.1f}'
                                  if math.isfinite(yaw_error) else '-')
                    print(f'{uav:>3} | {horizontal:>5.2f} m | '
                          f'{vertical:>4.2f} m | {current_text:>11} | '
                          f'{target_text:>10} | {error_text:>7} | '
                          f'{stable_for:>5.1f} s  (nearest={nearest:.2f} m)')
                if minimum_separation is not None \
                        and closest_actual < minimum_separation:
                    self.get_logger().error(
                        'SAFETY WARNING: actual pairwise separation is below '
                        f'{minimum_separation:.2f} m; commanded targets remain '
                        'unchanged')
                last_status = now
            if all(stable_since[uav] is not None
                   and now - stable_since[uav] >= stable_seconds
                   for uav in active_uavs):
                return True
        return False

    def hold_forever(self, message: str) -> None:
        last_message = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_message >= 10.0:
                print(message)
                last_message = now

    def current_xy(self, uav: int, geometry: PathGeometry) -> tuple[float, float]:
        vehicle = self.vehicles[uav]
        return gps_to_local_xy(
            vehicle.latitude, vehicle.longitude,
            geometry.origin_latitude, geometry.origin_longitude)

    def actual_separations(self) -> dict[tuple[int, int], float]:
        distances = {}
        for a, b in itertools.combinations((1, 2, 3), 2):
            va, vb = self.vehicles[a], self.vehicles[b]
            distances[(a, b)] = haversine_m(
                va.latitude, va.longitude, vb.latitude, vb.longitude)
        return distances

    def print_separations(self, distances: dict[tuple[int, int], float]) -> None:
        print('Actual horizontal separation: ' + ', '.join(
            f'UAV{a}/UAV{b}={distance:.2f} m'
            for (a, b), distance in distances.items()))

    def separation_safe(self, minimum_separation: float) -> bool:
        distances = self.actual_separations()
        self.print_separations(distances)
        if min(distances.values()) < minimum_separation:
            self.get_logger().error(
                'SAFETY WARNING: actual pairwise separation is below '
                f'{minimum_separation:.2f} m; no further UAVs will be released')
            return False
        return True

    def uav2_release_clearance(self, final: dict[int, Target],
                               geometry: PathGeometry) -> tuple[float, dict[int, float]]:
        uav2_start = self.current_xy(2, geometry)
        clearances = {}
        for other in (1, 3):
            other_start = self.current_xy(other, geometry)
            clearances[other] = segment_distance(
                uav2_start, geometry.final_xy[2],
                other_start, geometry.final_xy[other])
        return min(clearances.values()), clearances

    def staggered_final_release(
            self, staging: dict[int, Target], final: dict[int, Target],
            geometry: PathGeometry, minimum_separation: float,
            timeout: float) -> bool:
        print('PHASE 4A — UAV3 RELEASED')
        self.assign_targets({3: final[3]}, publish_now=True)
        deadline = time.monotonic() + timeout
        last_status = 0.0
        path3_length = math.dist(
            geometry.staging_xy[3], geometry.final_xy[3])
        conflict_along = geometry.uav3_conflict_fraction * path3_length

        if geometry.conflict_distance >= minimum_separation:
            print('  New UAV1/UAV3 paths have no minimum-separation conflict; '
                  'UAV1 may be released without waiting for a crossing.')
        else:
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                now = time.monotonic()
                current3 = self.current_xy(3, geometry)
                fraction, progress = project_progress(
                    current3, geometry.staging_xy[3], geometry.final_xy[3])
                conflict_distance = math.dist(
                    current3, geometry.uav3_conflict_xy)
                if now - last_status >= 1.0:
                    print(f'  UAV3 progress = {progress:.1f} m / '
                          f'{path3_length:.1f} m')
                    print('  distance to UAV1/UAV3 conflict region = '
                          f'{conflict_distance:.1f} m')
                    if not self.separation_safe(minimum_separation):
                        return False
                    last_status = now
                cleared = (fraction > geometry.uav3_conflict_fraction
                           and progress >= conflict_along + minimum_separation
                           and conflict_distance >= minimum_separation)
                if cleared:
                    print('UAV3 cleared UAV1 conflict region.')
                    break
            else:
                self.get_logger().error(
                    'Timed out waiting for UAV3 conflict clearance')
                return False

        print('PHASE 4B — UAV1 RELEASED')
        self.assign_targets({1: final[1]}, publish_now=True)
        last_status = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            clearance, by_uav = self.uav2_release_clearance(final, geometry)
            if now - last_status >= 1.0:
                print('  UAV2 remaining-path clearance: '
                      f'UAV1={by_uav[1]:.2f} m, UAV3={by_uav[3]:.2f} m')
                if not self.separation_safe(minimum_separation):
                    return False
                last_status = now
            if clearance >= minimum_separation:
                print('PHASE 4C — UAV2 RELEASED')
                self.assign_targets({2: final[2]}, publish_now=True)
                return True
        self.get_logger().error('Timed out waiting for a safe UAV2 release')
        return False

    def heading_correction_maneuver(
            self, final: dict[int, Target], distance_m: float,
            horizontal_tolerance: float, vertical_tolerance: float,
            stable_seconds: float, timeout: float,
            minimum_separation: float) -> bool:
        """Use short north/south legs to establish final travel headings."""
        temp2_lat, temp2_lon = move_gps(
            final[2].latitude, final[2].longitude, distance_m, 180.0)
        temp3_lat, temp3_lon = move_gps(
            final[3].latitude, final[3].longitude, distance_m, 0.0)
        temporary = {
            1: final[1],
            2: Target(temp2_lat, temp2_lon, final[2].altitude, None),
            3: Target(temp3_lat, temp3_lon, final[3].altitude, None),
        }
        print('PHASE 6A — HEADING-CORRECTION TEMPORARY LEGS')
        print(f'  UAV1 HOLD: {final[1].latitude:.6f}, {final[1].longitude:.6f}')
        print(f'  UAV2 SOUTH {distance_m:.1f} m: '
              f'{temporary[2].latitude:.6f}, {temporary[2].longitude:.6f}')
        print(f'  UAV3 NORTH {distance_m:.1f} m: '
              f'{temporary[3].latitude:.6f}, {temporary[3].longitude:.6f}')
        self.assign_targets({2: temporary[2], 3: temporary[3]}, publish_now=True)
        if not self.wait_until_all_stable(
                temporary, horizontal_tolerance, vertical_tolerance,
                stable_seconds, timeout,
                minimum_separation=minimum_separation):
            return False

        print('PHASE 6B — RETURN TO FINAL GPS TARGETS')
        self.assign_targets({2: final[2], 3: final[3]}, publish_now=True)
        return self.wait_until_all_stable(
            final, horizontal_tolerance, vertical_tolerance, stable_seconds,
            timeout, minimum_separation=minimum_separation)


def build_path_geometry(staging: dict[int, Target],
                        final: dict[int, Target]) -> PathGeometry:
    all_points = [(t.latitude, t.longitude)
                  for t in (*staging.values(), *final.values())]
    origin_lat = sum(p[0] for p in all_points) / len(all_points)
    origin_lon = sum(p[1] for p in all_points) / len(all_points)
    xy_staging = {u: gps_to_local_xy(t.latitude, t.longitude,
                                     origin_lat, origin_lon)
                  for u, t in staging.items()}
    xy_final = {u: gps_to_local_xy(t.latitude, t.longitude,
                                   origin_lat, origin_lon)
                for u, t in final.items()}
    conflict = segment_closest_approach(
        xy_staging[3], xy_final[3], xy_staging[1], xy_final[1])
    return PathGeometry(
        origin_lat, origin_lon, xy_staging, xy_final,
        conflict[1], conflict[2], conflict[0], conflict[3], conflict[4])


def xy_to_gps(x: float, y: float, origin_lat: float,
              origin_lon: float) -> tuple[float, float]:
    latitude = origin_lat + math.degrees(y / EARTH_RADIUS_M)
    longitude = origin_lon + math.degrees(
        x / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat))))
    return latitude, longitude


def safety_report(staging: dict[int, Target], final: dict[int, Target],
                  minimum_separation: float) -> tuple[bool, PathGeometry]:
    geometry = build_path_geometry(staging, final)
    safe = True
    print('Safety geometry (horizontal metres):')
    for a, b in itertools.combinations((1, 2, 3), 2):
        initial = haversine_m(
            staging[a].latitude, staging[a].longitude,
            staging[b].latitude, staging[b].longitude)
        final_distance = haversine_m(
            final[a].latitude, final[a].longitude,
            final[b].latitude, final[b].longitude)
        path_distance = segment_distance(
            geometry.staging_xy[a], geometry.final_xy[a],
            geometry.staging_xy[b], geometry.final_xy[b])
        print(f'  UAV{a}/UAV{b}: staging={initial:.2f} m '
              f'final={final_distance:.2f} m '
              f'path-segment minimum={path_distance:.2f} m')
        if min(initial, final_distance) < minimum_separation:
            print(f'  ABORT: UAV{a}/UAV{b} staging/final position falls below '
                  f'{minimum_separation:.2f} m separation')
            safe = False
        elif path_distance < minimum_separation:
            print(f'  CONFLICT: UAV{a}/UAV{b} simultaneous paths require '
                  'staggered release coordination')

    conflict_lat, conflict_lon = xy_to_gps(
        *geometry.uav3_conflict_xy,
        geometry.origin_latitude, geometry.origin_longitude)
    path3_length = math.dist(geometry.staging_xy[3], geometry.final_xy[3])
    conflict_along = geometry.uav3_conflict_fraction * path3_length
    print('UAV1/UAV3 closest-approach region:')
    print(f'  approximate GPS = {conflict_lat:.9f}, {conflict_lon:.9f}')
    print(f'  closest path approach = {geometry.conflict_distance:.2f} m')
    print(f'  UAV3 distance from staging to conflict = {conflict_along:.2f} m')
    if geometry.conflict_distance < minimum_separation:
        print('  UAV3 release threshold = conflict progress + '
              f'{minimum_separation:.2f} m clearance')
    else:
        print('  no UAV1 release wait required: closest approach exceeds '
              f'{minimum_separation:.2f} m')

    uav2_clearances = {}
    for other in (1, 3):
        uav2_clearances[other] = segment_distance(
            geometry.staging_xy[2], geometry.final_xy[2],
            geometry.staging_xy[other], geometry.final_xy[other])
    print('UAV2 initial release safety analysis: '
          f'versus UAV1={uav2_clearances[1]:.2f} m, '
          f'versus UAV3={uav2_clearances[3]:.2f} m')
    if min(uav2_clearances.values()) < minimum_separation:
        print('  UAV2 must wait for dynamic remaining-path clearance.')
    else:
        print('  UAV2 paths meet the configured separation at staging.')
    return safe, geometry


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Stage three UAVs, move to fixed comparison poses, and hold.')
    parser.add_argument(
        '--alt', type=float, default=None,
        help='optional common altitude override (default: calibrated 25.0 m)')
    parser.add_argument('--staging-distance', type=float, default=10.0)
    parser.add_argument('--horizontal-tolerance', type=float, default=1.0)
    parser.add_argument('--vertical-tolerance', type=float, default=0.75)
    parser.add_argument('--stable-seconds', type=float, default=5.0)
    parser.add_argument('--minimum-separation', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=1000.0)
    parser.add_argument('--yaw-tolerance', type=float, default=5.0)
    parser.add_argument('--dry-run', action='store_true')
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    positive = ('staging_distance', 'horizontal_tolerance',
                'vertical_tolerance', 'stable_seconds',
                'minimum_separation', 'timeout', 'yaw_tolerance')
    for name in positive:
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'--{name.replace("_", "-")} must be finite and positive')
    if args.alt is not None and (
            not math.isfinite(args.alt) or args.alt <= 0.0):
        raise ValueError('--alt must be finite and positive')


def mission_plan(initial: dict[int, Target], staging: dict[int, Target],
                 final: dict[int, Target], staging_distance: float) -> None:
    print('Initial:')
    for uav in (1, 2, 3):
        t = initial[uav]
        print(f'  UAV{uav} = {t.latitude:.6f}, {t.longitude:.6f}, '
              f'alt={t.altitude:.1f}')
    print('Staging:')
    print('  UAV1 = HOLD')
    print(f'  UAV2 = {staging_distance:.1f} m SOUTH -> '
          f'{staging[2].latitude:.6f}, {staging[2].longitude:.6f}')
    print(f'  UAV3 = {staging_distance:.1f} m NORTH -> '
          f'{staging[3].latitude:.6f}, {staging[3].longitude:.6f}')
    print('FINAL GPS TARGETS (translation phase; yaw not commanded)')
    for uav in (1, 2, 3):
        t = final[uav]
        print(f'  UAV{uav} = {t.latitude:.6f}, {t.longitude:.6f}, '
              f'{t.altitude:.1f} m')
    print('HEADING-CORRECTION MANEUVER')
    print('  UAV2 = 10 m SOUTH, then return NORTH to final GPS target')
    print('  UAV3 = 10 m NORTH, then return SOUTH to final GPS target')
    print('Final pairwise distances:')
    for a, b in itertools.combinations((1, 2, 3), 2):
        distance = haversine_m(final[a].latitude, final[a].longitude,
                               final[b].latitude, final[b].longitude)
        print(f'  UAV{a}/UAV{b}: {distance:.2f} m')
def run_mission(args: argparse.Namespace) -> int:
    rclpy.init()
    node = ThreeUAVComparisonMission(dry_run=args.dry_run)
    airborne = False
    try:
        calibrated_altitudes = {pose['alt'] for pose in FINAL_POSES.values()}
        if len(calibrated_altitudes) != 1:
            raise ValueError('all calibrated poses must use one staging altitude')
        mission_alt = (next(iter(calibrated_altitudes))
                       if args.alt is None else args.alt)
        # Position targets are sufficient; heading feedback and yaw command
        # discovery are not required by this mission.
        require_heading = False
        require_yaw = False

        print('PHASE 0 — STATE / INTERFACE CHECK')
        if not node.wait_for_all_ready(
                require_heading, require_yaw, min(args.timeout, 60.0)):
            node.get_logger().error('Not all three UAV interfaces are ready')
            return 1
        node.print_command_ownership()

        # Translation and staging use position-only targets.
        staging_yaw_by_uav = {uav: None for uav in (1, 2, 3)}

        initial = node.current_targets(mission_alt, staging_yaw_by_uav)
        staging = dict(initial)
        staging[2] = Target(*move_gps(
            initial[2].latitude, initial[2].longitude,
            args.staging_distance, 180.0), mission_alt,
            staging_yaw_by_uav[2])
        staging[3] = Target(*move_gps(
            initial[3].latitude, initial[3].longitude,
            args.staging_distance, 0.0), mission_alt,
            staging_yaw_by_uav[3])
        final = {
            uav: Target(
                pose['lat'], pose['lon'],
                mission_alt if args.alt is not None else pose['alt'],
                None)
            for uav, pose in FINAL_POSES.items()
        }
        mission_plan(initial, staging, final, args.staging_distance)

        if args.dry_run:
            print('DRY RUN: no arm/takeoff call and no goto/yaw publication.')
            safe, geometry = safety_report(
                staging, final, args.minimum_separation)
            if geometry.conflict_distance < args.minimum_separation:
                print('Expected release sequence: UAV3 -> UAV1 after conflict '
                      'clearance -> UAV2 after dynamic path clearance.')
            else:
                print('Expected release sequence: UAV3 -> UAV1 immediately '
                      '(paths clear) -> UAV2 after dynamic path clearance.')
            print('DRY RUN RESULT: ' + ('geometry accepted' if safe else 'geometry rejected'))
            return 0 if safe else 1

        print('PHASE 1 — ARM AND TAKEOFF')
        if not node.call_all('arm_client', 'arm', min(args.timeout, 60.0)):
            return 1
        if not node.call_all('takeoff_client', 'takeoff', min(args.timeout, 120.0)):
            return 1
        airborne = True

        # Trigger takeoff uses the bridge's configured/default altitude (10 m).
        # Command a vertical climb at each current horizontal position to 25 m.
        altitude_holds = node.current_targets(
            mission_alt, staging_yaw_by_uav)
        node.assign_targets(altitude_holds)
        if not node.wait_until_all_stable(
                altitude_holds, max(2.0, args.horizontal_tolerance),
                args.vertical_tolerance, args.stable_seconds, args.timeout):
            node.hold_forever('Altitude staging failed; holding last safe targets.')
            return 1

        print('PHASE 2 — SAFE STAGING SEPARATION')
        # Recompute from the actual post-climb positions, never guessed GPS.
        staging = node.current_targets(mission_alt, staging_yaw_by_uav)
        staging[2] = Target(*move_gps(
            node.vehicles[2].latitude, node.vehicles[2].longitude,
            args.staging_distance, 180.0), mission_alt,
            staging_yaw_by_uav[2])
        staging[3] = Target(*move_gps(
            node.vehicles[3].latitude, node.vehicles[3].longitude,
            args.staging_distance, 0.0), mission_alt,
            staging_yaw_by_uav[3])
        print(f'UAV1 staging HOLD: {staging[1].latitude:.6f}, '
              f'{staging[1].longitude:.6f}')
        print(f'UAV2 staging SOUTH: {staging[2].latitude:.6f}, '
              f'{staging[2].longitude:.6f}')
        print(f'UAV3 staging NORTH: {staging[3].latitude:.6f}, '
              f'{staging[3].longitude:.6f}')

        # UAV2 keeps its altitude-hold target while UAV3 stages first. Do not
        # assign UAV2's south target yet: the 2-second refresh timer would
        # otherwise publish it before UAV3 had stabilized.
        node.assign_targets({1: staging[1], 3: staging[3]}, publish_now=True)
        if not node.wait_until_all_stable(
                staging, args.horizontal_tolerance, args.vertical_tolerance,
                args.stable_seconds, args.timeout, active_uavs=(3,)):
            node.hold_forever('UAV3 staging failed; holding staging targets.')
            return 1
        node.assign_targets({2: staging[2]}, publish_now=True)
        if not node.wait_until_all_stable(
                staging, args.horizontal_tolerance, args.vertical_tolerance,
                args.stable_seconds, args.timeout, active_uavs=(2,)):
            node.hold_forever('UAV2 staging failed; holding staging targets.')
            return 1
        if not node.wait_until_all_stable(
                staging, args.horizontal_tolerance, args.vertical_tolerance,
                args.stable_seconds, args.timeout):
            node.hold_forever('Formation staging failed; holding staging targets.')
            return 1

        print('PHASE 3 — COLLISION / PATH SAFETY CHECK')
        safe, geometry = safety_report(
            staging, final, args.minimum_separation)
        if not safe:
            node.hold_forever(
                'Safety check rejected final paths; holding staging formation.')
            return 1

        print('PHASE 4 — STAGGERED CONCURRENT FINAL MOVE')
        if not node.staggered_final_release(
                staging, final, geometry, args.minimum_separation,
                args.timeout):
            node.hold_forever(
                'Final release coordination stopped; holding last targets.')
            return 1

        print('PHASE 5 — FINAL STABILITY CHECK')
        if not node.wait_until_all_stable(
                final, args.horizontal_tolerance, args.vertical_tolerance,
                args.stable_seconds, args.timeout,
                minimum_separation=args.minimum_separation):
            node.hold_forever(
                'Final position stability timed out; holding final targets.')
            return 1

        if not node.heading_correction_maneuver(
                final, 10.0, args.horizontal_tolerance,
                args.vertical_tolerance, args.stable_seconds, args.timeout,
                args.minimum_separation):
            node.assign_targets(final, publish_now=True)
            node.hold_forever(
                'Heading-correction maneuver failed; returning to and holding '
                'final targets.')
            return 1

        print('=========================================')
        print('THREE-UAV COMPARISON FORMATION READY')
        print('=========================================')
        print('FINAL CALIBRATED EXPERIMENT POSES')
        for uav in (1, 2, 3):
            target = final[uav]
            print(f'\nUAV{uav}')
            print(f'lat = {target.latitude:.6f}')
            print(f'lon = {target.longitude:.6f}')
            print(f'alt = {target.altitude:.1f} m')
            print('yaw = not commanded (travel direction provides correction)')

        print('PHASE 7 — HOLD')
        node.hold_forever('Holding three-UAV comparison formation...')
        return 0
    except KeyboardInterrupt:
        print('\nMission node stopped; no land or disarm command was issued.')
        return 0
    except Exception as exc:  # preserve safe target refresh after airborne fault
        node.get_logger().error(f'Mission error: {exc}')
        if airborne and rclpy.ok():
            node.hold_forever('Mission fault; holding last commanded safe targets.')
        return 1
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
