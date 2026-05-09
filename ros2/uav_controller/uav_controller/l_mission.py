"""
l_mission.py
------------
L-shaped autonomous mission for UAV1.

Mission profile:
    1. Arm + takeoff to 10m       (via /uav1/takeoff service)
    2. Fly 50m North  at 10m alt  (via /uav1/goto topic)
    3. Fly 50m West   at 30m alt  (via /uav1/goto topic)
    4. RTL                        (via /uav1/rtl service)

Requires drone_bridge to be running first.

Usage:
    ros2 run uav_controller l_mission
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool
from geographic_msgs.msg import GeoPoint


# ── GPS math ──────────────────────────────────────────────────────────────────

def move_gps(lat, lon, distance_m, bearing_deg):
    """Move from (lat,lon) by distance_m in bearing_deg direction."""
    R = 6_371_000.0
    bearing = math.radians(bearing_deg)
    lat1    = math.radians(lat)
    lon1    = math.radians(lon)
    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance_m / R) +
        math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
        math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))


# ── Mission node ──────────────────────────────────────────────────────────────

class LMission(Node):

    def __init__(self):
        super().__init__('l_mission')

        self.declare_parameter('uav_id',          1)
        self.declare_parameter('waypoint_radius', 2.0)

        self.uav_id = self.get_parameter('uav_id').value
        self.wp_r   = self.get_parameter('waypoint_radius').value
        ns          = f'/uav{self.uav_id}'

        # ── state ─────────────────────────────────────────────────────────────
        self.lat     = None
        self.lon     = None
        self.rel_alt = 0.0
        self.mode    = '---'
        self.armed   = False

        # ── subscribers ───────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, f'{ns}/gps',     self._cb_gps,   10)
        self.create_subscription(Float32,   f'{ns}/rel_alt', self._cb_alt,   10)
        self.create_subscription(String,    f'{ns}/mode',    self._cb_mode,  10)
        self.create_subscription(Bool,      f'{ns}/armed',   self._cb_armed, 10)

        # ── services ──────────────────────────────────────────────────────────
        self._cli_takeoff = self.create_client(Trigger, f'{ns}/takeoff')
        self._cli_rtl     = self.create_client(Trigger, f'{ns}/rtl')

        # ── goto publisher → drone_bridge handles the MAVLink send ────────────
        self._pub_goto = self.create_publisher(GeoPoint, f'{ns}/goto', 10)

        threading.Thread(target=self._run_mission, daemon=True).start()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_gps(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _cb_alt(self, msg):
        self.rel_alt = msg.data

    def _cb_mode(self, msg):
        self.mode = msg.data

    def _cb_armed(self, msg):
        self.armed = msg.data

    # ── helpers ───────────────────────────────────────────────────────────────

    def _call_service(self, client, label, timeout=60.0):
        self.get_logger().info(f'  → {label}')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'Service {label} not available — is drone_bridge running?')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'{label} — no response')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info(f'  ✓ {result.message}')
        else:
            self.get_logger().error(f'  ✗ {result.message}')
        return result.success

    def _goto(self, lat, lon, alt_m):
        """Publish goto — drone_bridge forwards this via MAVLink."""
        msg = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = float(alt_m)
        self._pub_goto.publish(msg)

    def _fly_to(self, lat, lon, alt_m, label, timeout=90):
        self.get_logger().info(
            f'  → Flying to {label} '
            f'({lat:.6f}, {lon:.6f}, alt={alt_m}m)')

        deadline  = time.time() + timeout
        last_send = 0

        while time.time() < deadline:
            # Resend every 2s to keep ArduPilot on track
            if time.time() - last_send > 2.0:
                self._goto(lat, lon, alt_m)
                last_send = time.time()

            if self.lat is not None:
                dist    = haversine_m(self.lat, self.lon, lat, lon)
                alt_err = abs(self.rel_alt - alt_m)
                if dist < self.wp_r and alt_err < 2.0:
                    self.get_logger().info(
                        f'  ✓ Reached {label} '
                        f'(dist={dist:.1f}m  alt={self.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)

        self.get_logger().warn(
            f'  ⚠ Timeout at {label} (alt={self.rel_alt:.1f}m)')
        return False

    # ── mission ───────────────────────────────────────────────────────────────

    def _run_mission(self):
        # Wait for GPS
        self.get_logger().info('[LMission] Waiting for GPS from drone_bridge...')
        while self.lat is None:
            time.sleep(0.3)

        self.get_logger().info(
            f'[LMission] GPS ready — home: ({self.lat:.6f}, {self.lon:.6f})')
        self.get_logger().info('[LMission] Waiting 15s for EKF to settle...')
        time.sleep(15.0)

        self.get_logger().info('')
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info('  L-SHAPED MISSION START')
        self.get_logger().info('  Takeoff 10m → North 50m → West 50m@30m → RTL')
        self.get_logger().info('════════════════════════════════════════')

        home_lat = self.lat
        home_lon = self.lon

        wp1_lat, wp1_lon = move_gps(home_lat, home_lon, 50.0, 0.0)
        wp2_lat, wp2_lon = move_gps(wp1_lat,  wp1_lon,  50.0, 270.0)

        self.get_logger().info(f'  Home : ({home_lat:.6f}, {home_lon:.6f})')
        self.get_logger().info(
            f'  WP1  : ({wp1_lat:.6f}, {wp1_lon:.6f})  50m North  alt=10m')
        self.get_logger().info(
            f'  WP2  : ({wp2_lat:.6f}, {wp2_lon:.6f})  50m West   alt=30m')

        # Step 1 — Takeoff
        self.get_logger().info('')
        self.get_logger().info('[ STEP 1 ] Arm + Takeoff to 10m')
        if not self._call_service(self._cli_takeoff, '/takeoff'):
            self.get_logger().error('Takeoff failed — aborting')
            return
        time.sleep(3.0)

        # Step 2 — North 50m at 10m
        self.get_logger().info('')
        self.get_logger().info('[ STEP 2 ] Fly North 50m at 10m altitude')
        self._fly_to(wp1_lat, wp1_lon, 10.0, 'WP1 (North 50m)')
        time.sleep(3.0)

        # Step 3 — West 50m at 30m
        self.get_logger().info('')
        self.get_logger().info('[ STEP 3 ] Fly West 50m climbing to 30m')
        self._fly_to(wp2_lat, wp2_lon, 30.0, 'WP2 (West 50m @ 30m)')
        time.sleep(3.0)

        # Step 4 — RTL
        self.get_logger().info('')
        self.get_logger().info('[ STEP 4 ] RTL')
        self._call_service(self._cli_rtl, '/rtl')

        self.get_logger().info('')
        self.get_logger().info('════════════════════════════════════════')
        self.get_logger().info('  L-MISSION COMPLETE')
        self.get_logger().info('════════════════════════════════════════')


def main(args=None):
    rclpy.init(args=args)
    node = LMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()