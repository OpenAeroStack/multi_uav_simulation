#!/usr/bin/env python3
"""
uav1_patrol_mission.py
-----------------------
Box patrol around a fixed GPS point. UAV1 only. No detection — navigation only.

Sequence:
  1. Arm + takeoff
  2. Climb to PATROL_ALTITUDE explicitly (independent of drone_bridge's own
     takeoff_altitude param)
  3. Fly the 4 box corners (BOX_SIZE_M x BOX_SIZE_M) around CENTER_LAT/LON,
     LAPS times
  4. Return to the center point
  5. RTL

Uses drone_bridge's existing interfaces only:
  /uav1/arm      (std_srvs/Trigger)
  /uav1/takeoff  (std_srvs/Trigger)
  /uav1/goto     (geographic_msgs/GeoPoint)   -- relative altitude
  /uav1/gps      (sensor_msgs/NavSatFix)      -- arrival confirmation
  /uav1/rel_alt  (std_msgs/Float32)           -- climb confirmation
  /uav1/rtl      (std_srvs/Trigger)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


# ── Mission parameters ───────────────────────────────────────────────────────
CENTER_LAT        = 6.079429
CENTER_LON        = 80.193174
BOX_SIZE_M        = 25.0     # full side length of the box
PATROL_ALTITUDE   = 25.0     # metres, relative. WAS 25.0 - raised to test
                             # coverage: detections were clustering at the box
                             # centre and at corner 1 only, so the camera was
                             # rarely looking at the subjects from the
                             # perimeter. Higher altitude widens the ground
                             # footprint per frame. Trade-off: subjects shrink,
                             # roughly 53 px at 25 m to ~33 px at 40 m, close to
                             # what YOLOv8n can still resolve.
LAPS              = 2
ARRIVAL_RADIUS_M  = 2.5      # horizontal distance counted as "arrived"
WAYPOINT_TIMEOUT  = 40.0     # seconds to wait per waypoint before giving up
CLIMB_TIMEOUT     = 30.0     # seconds to wait for patrol altitude


def meters_to_latlon_offset(d_north_m, d_east_m, ref_lat_deg):
    """Convert a local metre offset (north, east) to a (dlat, dlon) delta."""
    lat_per_m = 1.0 / 111320.0
    lon_per_m = 1.0 / (111320.0 * math.cos(math.radians(ref_lat_deg)))
    return d_north_m * lat_per_m, d_east_m * lon_per_m


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def build_box_corners(center_lat, center_lon, size_m):
    """Clockwise box: NW -> NE -> SE -> SW."""
    half = size_m / 2.0
    ne_offsets = [(half, -half), (half, half), (-half, half), (-half, -half)]
    corners = []
    for d_north, d_east in ne_offsets:
        dlat, dlon = meters_to_latlon_offset(d_north, d_east, center_lat)
        corners.append((center_lat + dlat, center_lon + dlon))
    return corners


class PatrolMission(Node):
    def __init__(self):
        super().__init__('patrol_mission')
        self.uav_id = 1
        ns = f'/uav{self.uav_id}'

        self.lat = None
        self.lon = None
        self.rel_alt = 0.0

        qos = QoSProfile(depth=10,
                          reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(NavSatFix, f'{ns}/gps', self._on_gps, qos)
        self.create_subscription(Float32, f'{ns}/rel_alt', self._on_alt, qos)

        self.goto_pub = self.create_publisher(GeoPoint, f'{ns}/goto', 10)

        self.arm_cli     = self.create_client(Trigger, f'{ns}/arm')
        self.takeoff_cli = self.create_client(Trigger, f'{ns}/takeoff')
        self.rtl_cli     = self.create_client(Trigger, f'{ns}/rtl')

    def _on_gps(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _on_alt(self, msg):
        self.rel_alt = msg.data

    def call_trigger(self, client, name):
        self.get_logger().info(f'Waiting for service {name}...')
        client.wait_for_service()
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        self.get_logger().info(f'{name} -> success={res.success}  {res.message}')
        return res.success

    def goto(self, lat, lon, alt):
        msg = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = alt
        self.goto_pub.publish(msg)

    def wait_for_altitude(self, target_alt, timeout=CLIMB_TIMEOUT):
        self.get_logger().info(f'Climbing to {target_alt}m...')
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.rel_alt >= target_alt * 0.9:
                self.get_logger().info(f'  -> reached {self.rel_alt:.1f}m')
                return True
        self.get_logger().warn(f'  -> altitude timeout at {self.rel_alt:.1f}m')
        return False

    def wait_until_arrived(self, lat, lon, timeout=WAYPOINT_TIMEOUT):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.lat is not None:
                d = haversine_m(self.lat, self.lon, lat, lon)
                if d <= ARRIVAL_RADIUS_M:
                    self.get_logger().info(f'  -> arrived (within {d:.1f}m)')
                    return True
        self.get_logger().warn('  -> waypoint timeout, continuing anyway')
        return False


def main():
    rclpy.init()
    node = PatrolMission()
    try:
        if not node.call_trigger(node.arm_cli, '/uav1/arm'):
            node.get_logger().error('Arm failed — aborting mission.')
            return
        if not node.call_trigger(node.takeoff_cli, '/uav1/takeoff'):
            node.get_logger().error('Takeoff failed — aborting mission.')
            return

        while node.lat is None:
            rclpy.spin_once(node, timeout_sec=0.3)
        node.goto(node.lat, node.lon, PATROL_ALTITUDE)
        node.wait_for_altitude(PATROL_ALTITUDE)

        corners = build_box_corners(CENTER_LAT, CENTER_LON, BOX_SIZE_M)

        for lap in range(1, LAPS + 1):
            node.get_logger().info(f'=== Lap {lap}/{LAPS} ===')
            for i, (lat, lon) in enumerate(corners, start=1):
                node.get_logger().info(
                    f'Corner {i}/4 -> ({lat:.6f}, {lon:.6f})')
                node.goto(lat, lon, PATROL_ALTITUDE)
                node.wait_until_arrived(lat, lon)

        node.get_logger().info('Patrol complete — returning to center...')
        node.goto(CENTER_LAT, CENTER_LON, PATROL_ALTITUDE)
        node.wait_until_arrived(CENTER_LAT, CENTER_LON)

        node.get_logger().info('Switching to RTL...')
        node.call_trigger(node.rtl_cli, '/uav1/rtl')

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()