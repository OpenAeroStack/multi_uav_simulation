#!/usr/bin/env python3
"""Move UAV1 to the fixed edge-versus-ground comparison pose and hold it.

Run inside ``gcsns``:

```bash
sudo ip netns exec gcsns sudo -H -u multi_uav bash -lc '
source /opt/ros/humble/setup.bash
source /home/multi_uav/FYP/multi_uav_simulation/ros2/install/setup.bash
python3 /home/multi_uav/FYP/multi_uav_simulation/scripts/mission/goto_comparison_pose.py \
  --uav 1 \
  --lat 6.079430 \
  --lon 80.193085 \
  --alt 25.0 \
  --preserve-yaw
'
```

Explicit-yaw example:

```bash
python3 scripts/mission/goto_comparison_pose.py \
  --uav 1 \
  --lat 6.079430 \
  --lon 80.193085 \
  --alt 25.0 \
  --yaw-deg 0.0
```

Position commands reuse drone_bridge's ``/uav1/goto`` GeoPoint interface;
its altitude is relative to home.  That interface ignores yaw, so heading is
sent separately through ArduPilot's existing ``/ap/v1/cmd_gps_pose``
GlobalPosition interface.  Heading feedback comes from the ENU quaternion on
``/ap/v1/pose/filtered`` and is reported as compass heading (0 degrees north,
increasing clockwise), matching the MAVLink/GlobalPosition yaw convention.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
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


DEFAULT_LATITUDE = 6.079430
DEFAULT_LONGITUDE = 80.193085
DEFAULT_ALTITUDE_M = 25.0
DEFAULT_YAW_DEG = 0.0
DEFAULT_YAW_TOLERANCE_DEG = 5.0
EARTH_RADIUS_M = 6_371_000.0

AP_DDS_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Return great-circle distance in metres between nearby GPS points."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2)
         * math.sin(dlambda / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def quaternion_to_compass_heading_deg(msg: PoseStamped) -> float:
    """Convert ROS ENU quaternion yaw to heading: north=0, clockwise positive."""
    q = msg.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    enu_yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    return (90.0 - enu_yaw_deg) % 360.0


def angular_error_deg(current: float, target: float) -> float:
    """Return the smallest absolute heading difference in degrees."""
    return abs((current - target + 180.0) % 360.0 - 180.0)


class ComparisonPoseMission(Node):
    """Use the existing bridge and ArduPilot DDS interfaces for UAV1."""

    def __init__(self, uav_id: int) -> None:
        super().__init__('goto_comparison_pose')
        self.uav_id = uav_id
        ns = f'/uav{uav_id}'
        ap_ns = f'/ap/v{uav_id}'

        self.latitude: Optional[float] = None
        self.longitude: Optional[float] = None
        self.relative_altitude = 0.0
        self.armed: Optional[bool] = None
        self.heading_deg: Optional[float] = None

        self.target_latitude: Optional[float] = None
        self.target_longitude: Optional[float] = None
        self.target_altitude: Optional[float] = None
        self.target_yaw_deg: Optional[float] = None

        self.create_subscription(
            NavSatFix, f'{ap_ns}/navsat', self._gps_callback, AP_DDS_QOS)
        self.create_subscription(
            PoseStamped, f'{ap_ns}/pose/filtered',
            self._pose_callback, AP_DDS_QOS)
        self.create_subscription(
            Float32, f'{ns}/rel_alt', self._altitude_callback, 10)
        self.create_subscription(Bool, f'{ns}/armed', self._armed_callback, 10)

        self.goto_publisher = self.create_publisher(
            GeoPoint, f'{ns}/goto', 10)
        self.yaw_publisher = self.create_publisher(
            GlobalPosition, f'{ap_ns}/cmd_gps_pose', 10)
        self.arm_client = self.create_client(Trigger, f'{ns}/arm')
        self.takeoff_client = self.create_client(Trigger, f'{ns}/takeoff')

        # Match waypoint_finder.py: refresh the commanded target every 2 s.
        self.create_timer(2.0, self.publish_target)

    def _gps_callback(self, msg: NavSatFix) -> None:
        if (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)
                and (msg.latitude != 0.0 or msg.longitude != 0.0)):
            self.latitude = msg.latitude
            self.longitude = msg.longitude

    def _pose_callback(self, msg: PoseStamped) -> None:
        values = (
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        )
        if all(math.isfinite(value) for value in values):
            self.heading_deg = quaternion_to_compass_heading_deg(msg)

    def _altitude_callback(self, msg: Float32) -> None:
        if math.isfinite(msg.data):
            self.relative_altitude = float(msg.data)

    def _armed_callback(self, msg: Bool) -> None:
        self.armed = bool(msg.data)

    def wait_for_state(self, require_heading: bool, timeout: float) -> bool:
        """Wait for valid GPS/state data and, when needed, attitude."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            state_ready = (self.latitude is not None
                           and self.longitude is not None
                           and self.armed is not None)
            if state_ready and (not require_heading
                                or self.heading_deg is not None):
                return True
        return False

    def wait_for_command_interfaces(self, require_yaw: bool,
                                    timeout: float = 15.0) -> bool:
        """Confirm bridge services and command subscribers are discovered."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            services_ready = (self.arm_client.service_is_ready()
                              and self.takeoff_client.service_is_ready())
            goto_ready = self.count_subscribers(
                f'/uav{self.uav_id}/goto') >= 1
            yaw_ready = (not require_yaw or self.count_subscribers(
                f'/ap/v{self.uav_id}/cmd_gps_pose') >= 1)
            if services_ready and goto_ready and yaw_ready:
                return True
        return False

    def call_trigger(self, client: object, name: str,
                     timeout: float) -> bool:
        """Call a bridge Trigger service with a bounded response wait."""
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'Command service unavailable: {name}')
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not future.done():
            self.get_logger().error(f'{name} timed out after {timeout:.0f}s')
            return False
        response = future.result()
        if response is None:
            self.get_logger().error(f'{name} returned no response')
            return False
        self.get_logger().info(
            f'{name}: success={response.success} {response.message}')
        return bool(response.success)

    def set_target(self, latitude: float, longitude: float, altitude: float,
                   yaw_deg: Optional[float]) -> None:
        self.target_latitude = latitude
        self.target_longitude = longitude
        self.target_altitude = altitude
        self.target_yaw_deg = yaw_deg
        self.publish_target()

    def publish_target(self) -> None:
        """Refresh relative-altitude position and optional fixed heading."""
        if self.target_latitude is None or self.target_longitude is None:
            return
        position = GeoPoint()
        position.latitude = self.target_latitude
        position.longitude = self.target_longitude
        position.altitude = float(self.target_altitude)
        self.goto_publisher.publish(position)

        if self.target_yaw_deg is not None:
            yaw = GlobalPosition()
            yaw.header.stamp = self.get_clock().now().to_msg()
            yaw.header.frame_id = 'map'
            yaw.coordinate_frame = GlobalPosition.FRAME_GLOBAL_REL_ALT
            yaw.latitude = self.target_latitude
            yaw.longitude = self.target_longitude
            yaw.altitude = float(self.target_altitude)
            yaw.type_mask = (
                GlobalPosition.IGNORE_VX
                | GlobalPosition.IGNORE_VY
                | GlobalPosition.IGNORE_VZ
                | GlobalPosition.IGNORE_AFX
                | GlobalPosition.IGNORE_AFY
                | GlobalPosition.IGNORE_AFZ
                | GlobalPosition.IGNORE_YAW_RATE
            )
            yaw.yaw = math.radians(self.target_yaw_deg)
            self.yaw_publisher.publish(yaw)

    def wait_for_altitude(self, target_altitude: float,
                          timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.relative_altitude >= target_altitude * 0.85:
                return True
        self.get_logger().error(
            f'Altitude timeout: current={self.relative_altitude:.1f}m, '
            f'target={target_altitude:.1f}m')
        return False

    def wait_until_stable(
            self, latitude: float, longitude: float, altitude: float,
            yaw_deg: Optional[float], horizontal_tolerance: float,
            vertical_tolerance: float, stable_seconds: float,
            timeout: float) -> bool:
        """Require every pose tolerance continuously for stable_seconds."""
        deadline = time.monotonic() + timeout
        stable_since: Optional[float] = None
        last_status = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latitude is None or self.longitude is None:
                stable_since = None
                continue

            horizontal_error = haversine_m(
                self.latitude, self.longitude, latitude, longitude)
            vertical_error = abs(self.relative_altitude - altitude)
            yaw_error = (angular_error_deg(self.heading_deg, yaw_deg)
                         if yaw_deg is not None
                         and self.heading_deg is not None else None)
            yaw_ok = yaw_deg is None or (
                yaw_error is not None
                and yaw_error <= DEFAULT_YAW_TOLERANCE_DEG)
            inside = (horizontal_error <= horizontal_tolerance
                      and vertical_error <= vertical_tolerance and yaw_ok)
            now = time.monotonic()
            if inside:
                stable_since = stable_since or now
            else:
                stable_since = None
            stable_for = now - stable_since if stable_since is not None else 0.0

            if now - last_status >= 1.0:
                yaw_text = f'{yaw_error:.1f} deg' if yaw_error is not None else 'n/a'
                print(f'Target:  lat={latitude:.6f} lon={longitude:.6f} '
                      f'alt={altitude:.1f} m')
                print(f'Current: lat={self.latitude:.6f} '
                      f'lon={self.longitude:.6f} '
                      f'alt={self.relative_altitude:.1f} m')
                print(f'Error:   horizontal={horizontal_error:.2f} m '
                      f'vertical={vertical_error:.2f} m yaw={yaw_text}')
                print(f'Stable:  {stable_for:.1f} / {stable_seconds:.1f} seconds')
                last_status = now
            if stable_for >= stable_seconds:
                return True

        self.get_logger().error(
            f'Comparison pose was not stable within {timeout:.0f} seconds')
        return False

    def hold_forever(self) -> None:
        """Keep timers and subscriptions alive without landing on exit."""
        last_message = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            now = time.monotonic()
            if now - last_message >= 5.0:
                print('Holding comparison pose; Ctrl+C stops this mission node '
                      'without landing or disarming.')
                last_message = now


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Move UAV1 to the fixed comparison pose and hold it.')
    parser.add_argument('--uav', type=int, default=1)
    parser.add_argument('--lat', type=float, default=DEFAULT_LATITUDE)
    parser.add_argument('--lon', type=float, default=DEFAULT_LONGITUDE)
    parser.add_argument('--alt', type=float, default=DEFAULT_ALTITUDE_M)
    yaw_group = parser.add_mutually_exclusive_group()
    yaw_group.add_argument('--yaw-deg', type=float, default=None)
    yaw_group.add_argument('--preserve-yaw', action='store_true')
    parser.add_argument('--horizontal-tolerance', type=float, default=1.0)
    parser.add_argument('--vertical-tolerance', type=float, default=0.75)
    parser.add_argument('--stable-seconds', type=float, default=5.0)
    parser.add_argument('--timeout', type=float, default=1000.0)
    parser.add_argument('--takeoff-alt', type=float, default=10.0)
    parser.add_argument('--print-current-yaw', action='store_true')
    return parser.parse_args()


def validate_args(args: argparse.Namespace) -> None:
    if args.uav != 1:
        raise ValueError('Only --uav 1 is currently supported')
    for name in ('lat', 'lon', 'alt', 'horizontal_tolerance',
                 'vertical_tolerance', 'stable_seconds', 'timeout',
                 'takeoff_alt'):
        value = getattr(args, name)
        if not math.isfinite(value):
            raise ValueError(f'--{name.replace("_", "-")} must be finite')
    if args.lat == 0.0 and args.lon == 0.0:
        raise ValueError('target latitude/longitude cannot both be zero')
    if not -90.0 <= args.lat <= 90.0:
        raise ValueError('--lat must be between -90 and 90')
    if not -180.0 <= args.lon <= 180.0:
        raise ValueError('--lon must be between -180 and 180')
    if any(getattr(args, name) <= 0.0 for name in (
            'alt', 'horizontal_tolerance', 'vertical_tolerance',
            'stable_seconds', 'timeout', 'takeoff_alt')):
        raise ValueError('altitudes, tolerances, stable time, and timeout must be positive')
    if args.yaw_deg is not None and not math.isfinite(args.yaw_deg):
        raise ValueError('--yaw-deg must be finite')


def run_mission(args: argparse.Namespace) -> int:
    rclpy.init()
    node = ComparisonPoseMission(args.uav)
    try:
        require_heading = (args.print_current_yaw or args.preserve_yaw
                           or args.yaw_deg is not None)
        if not node.wait_for_state(require_heading=require_heading,
                                   timeout=min(args.timeout, 30.0)):
            node.get_logger().error(
                'Timed out waiting for valid GPS, vehicle state, or heading')
            return 1

        if args.print_current_yaw:
            assert node.heading_deg is not None
            print(f'Current UAV{args.uav} yaw: {node.heading_deg:.1f} degrees')
            print(f'Use: --yaw-deg {node.heading_deg:.1f}')
            return 0

        if args.preserve_yaw:
            target_yaw = node.heading_deg
            print('Yaw mode: preserve yaw measured before movement')
            print(f'Preserved yaw: {target_yaw:.1f} degrees')
        else:
            target_yaw = (DEFAULT_YAW_DEG if args.yaw_deg is None
                          else args.yaw_deg) % 360.0
            print(f'Target yaw: {target_yaw:.1f} degrees')

        if not node.wait_for_command_interfaces(
                require_yaw=target_yaw is not None):
            node.get_logger().error(
                'Flight-control command interfaces are unavailable')
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

        node.set_target(args.lat, args.lon, args.alt, target_yaw)
        if not node.wait_until_stable(
                args.lat, args.lon, args.alt, target_yaw,
                args.horizontal_tolerance, args.vertical_tolerance,
                args.stable_seconds, args.timeout):
            return 1

        print('COMPARISON POSE READY')
        print(f'lat = {args.lat:.6f}')
        print(f'lon = {args.lon:.6f}')
        print(f'relative altitude = {args.alt:.1f} m')
        print(f'yaw = {target_yaw:.1f} degrees')
        print('The UAV will continue holding this pose.')
        node.hold_forever()
        return 0
    except KeyboardInterrupt:
        print('\nMission node stopped; UAV was not landed or disarmed.')
        return 0
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
