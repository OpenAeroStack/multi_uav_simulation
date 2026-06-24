"""
single_uav_mission.py
---------------------
Single-drone L-shaped survey mission at Faculty of Engineering,
University of Ruhuna, Hapugala, Galle, Sri Lanka.

GPS Origin: 6.0792673°N, 80.1921607°E
  (Front entrance, Administration Building)

Mission profile:
  1. Takeoff to 20m
     (altitude set via drone_bridge takeoff_altitude param in launch script)

  2. Fly 40m North  → toward Main Gate / Guard Room area

  3. Turn right (east) — fly 40m east
     (right turn from North = East, bearing 90°)

  4. RTL — return to launch and land

Waypoint positions (approximate Gazebo coords from GPS origin):
  P1 North: X=0,   Y=+40  (Main Gate direction)
  P2 east:  X=-40, Y=+40  

Waypoint arrival threshold: 3m horizontal + 3m altitude

Usage:
    Run AFTER bridge shows: ✓ DDS GPS flowing
    ros2 run uav_controller single_uav_mission
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool
from geographic_msgs.msg import GeoPoint


# ── GPS helpers ───────────────────────────────────────────────────────────────

def move_gps(lat, lon, distance_m, bearing_deg):
    """Move a GPS point by distance_m in the direction of bearing_deg."""
    R       = 6_371_000.0
    bearing = math.radians(bearing_deg)
    lat1    = math.radians(lat)
    lon1    = math.radians(lon)
    lat2    = math.asin(
        math.sin(lat1) * math.cos(distance_m / R) +
        math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))
    lon2    = lon1 + math.atan2(
        math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
        math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def haversine_m(lat1, lon1, lat2, lon2):
    """Horizontal distance in metres between two GPS points."""
    R    = 6_371_000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a    = (math.sin(dphi / 2) ** 2 +
            math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── Mission config ────────────────────────────────────────────────────────────

TAKEOFF_ALT  = 20.0   # m — must match takeoff_altitude param in launch script
CRUISE_ALT   = 20.0   # m — patrol altitude (stay at takeoff height)
LEG1_DIST    = 40.0   # m — distance North
LEG2_DIST    = 40.0   # m — distance West (left turn from North)
WP_RADIUS    =  3.0   # m — horizontal arrival threshold
ALT_TOL      =  3.0   # m — altitude arrival tolerance
HOLD_TIME    =  3.0   # s — pause at each waypoint before continuing


# ── Drone state ───────────────────────────────────────────────────────────────

class DroneState:
    def __init__(self):
        self.lat     = None
        self.lon     = None
        self.rel_alt = 0.0
        self.mode    = '---'
        self.armed   = False
        self.gps_ok  = False


# ── Mission node ──────────────────────────────────────────────────────────────

class SingleUAVMission(Node):

    def __init__(self):
        super().__init__('single_uav_mission')

        self.state = DroneState()

        # ── subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(NavSatFix, '/uav1/gps',     self._cb_gps,   10)
        self.create_subscription(Float32,   '/uav1/rel_alt', self._cb_alt,   10)
        self.create_subscription(String,    '/uav1/mode',    self._cb_mode,  10)
        self.create_subscription(Bool,      '/uav1/armed',   self._cb_armed, 10)

        # ── service clients ───────────────────────────────────────────────────
        self.takeoff_client = self.create_client(Trigger, '/uav1/takeoff')
        self.rtl_client     = self.create_client(Trigger, '/uav1/rtl')

        # ── goto publisher ────────────────────────────────────────────────────
        self.goto_pub = self.create_publisher(GeoPoint, '/uav1/goto', 10)

        # kick off mission in background thread so rclpy.spin() stays free
        import threading
        threading.Thread(target=self._run_mission, daemon=True).start()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_gps(self, msg):
        s        = self.state
        s.lat    = msg.latitude
        s.lon    = msg.longitude
        s.gps_ok = (msg.latitude != 0.0 or msg.longitude != 0.0)

    def _cb_alt(self, msg):
        self.state.rel_alt = msg.data

    def _cb_mode(self, msg):
        self.state.mode = msg.data

    def _cb_armed(self, msg):
        self.state.armed = msg.data

    # ── helpers ───────────────────────────────────────────────────────────────

    def _call_service(self, client, label, timeout=90.0):
        self.get_logger().info(f'  [UAV1] -> {label}')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'  [UAV1] {label} service not available')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'  [UAV1] no response from {label}')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info(f'  [UAV1] OK — {result.message}')
        else:
            self.get_logger().error(f'  [UAV1] FAIL — {result.message}')
        return result.success

    def _goto(self, lat, lon, alt_m):
        msg           = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = float(alt_m)
        self.goto_pub.publish(msg)

    def _fly_to(self, lat, lon, alt_m, label, timeout=120):
        """
        Publish goto repeatedly until the drone reaches the waypoint
        or timeout is exceeded.
        """
        self.get_logger().info(
            f'  [UAV1] -> {label} ({lat:.6f}, {lon:.6f}, {alt_m:.0f}m)')
        deadline  = time.time() + timeout
        last_send = 0.0

        while time.time() < deadline:
            # resend goto every 2 s so ArduPilot doesn't drift
            if time.time() - last_send > 2.0:
                self._goto(lat, lon, alt_m)
                last_send = time.time()

            s = self.state
            if s.lat is not None:
                dist    = haversine_m(s.lat, s.lon, lat, lon)
                alt_err = abs(s.rel_alt - alt_m)
                if dist < WP_RADIUS and alt_err < ALT_TOL:
                    self.get_logger().info(
                        f'  [UAV1] Reached {label} '
                        f'(dist={dist:.1f}m  alt={s.rel_alt:.1f}m)')
                    return True

            time.sleep(0.5)

        self.get_logger().warn(f'  [UAV1] Timeout reaching {label} — continuing')
        return False

    # ── mission ───────────────────────────────────────────────────────────────

    def _run_mission(self):
        s = self.state

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║   Faculty of Engineering — Single UAV L-Shape Mission   ║')
        self.get_logger().info('║   University of Ruhuna, Hapugala, Galle, Sri Lanka      ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  Takeoff 20m → 40m North → 40m West (left turn) → RTL  ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')

        # ── wait for GPS fix ──────────────────────────────────────────────────
        self.get_logger().info('[Mission] Waiting for GPS fix...')
        while s.lat is None:
            time.sleep(0.3)
        self.get_logger().info(
            f'[Mission] GPS ready — home: ({s.lat:.6f}, {s.lon:.6f})')

        # allow subscriptions to stabilise
        self.get_logger().info('[Mission] Waiting 15s for bridge to stabilise...')
        time.sleep(15.0)

        # record home position after stabilisation
        home_lat = s.lat
        home_lon = s.lon
        self.get_logger().info(
            f'[Mission] Home locked: ({home_lat:.6f}, {home_lon:.6f})')

        # ── Phase 1: Takeoff ──────────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── Phase 1: Takeoff ─────────────────────────────────────')
        if not self._call_service(self.takeoff_client, '/uav1/takeoff', timeout=90):
            self.get_logger().error('[Mission] Takeoff failed — aborting')
            return

        # wait until we are airborne at cruise altitude
        self.get_logger().info(f'[Mission] Climbing to {TAKEOFF_ALT}m...')
        while s.rel_alt < TAKEOFF_ALT - ALT_TOL:
            time.sleep(0.5)
        self.get_logger().info(f'[Mission] Airborne at {s.rel_alt:.1f}m')
        time.sleep(2.0)   # brief stabilisation pause

        # ── Phase 2: Fly 40m North ────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── Phase 2: Fly 40m North ───────────────────────────────')
        p1_lat, p1_lon = move_gps(home_lat, home_lon, LEG1_DIST, 0.0)
        self._fly_to(p1_lat, p1_lon, CRUISE_ALT, 'P1 North (40m)')

        self.get_logger().info(f'[Mission] Holding {HOLD_TIME}s at P1')
        time.sleep(HOLD_TIME)

        # ── Phase 3: Turn left (West) — fly 40m West ──────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── Phase 3: Turn right → Fly 40m East ───────────────────')
        # from P1, bearing 270° = West (left turn from North)

        # Phase 3 — was 270.0 (West), now 90.0 (East)
        p2_lat, p2_lon = move_gps(p1_lat, p1_lon, LEG2_DIST, 90.0)
        self._fly_to(p2_lat, p2_lon, CRUISE_ALT, 'P2 East (40m from P1)')

        self.get_logger().info(f'[Mission] Holding {HOLD_TIME}s at P2')
        time.sleep(HOLD_TIME)

        # ── Phase 4: RTL ──────────────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('── Phase 4: RTL ─────────────────────────────────────────')
        if not self._call_service(self.rtl_client, '/uav1/rtl'):
            # fallback: publish home position manually if RTL service fails
            self.get_logger().warn('[Mission] RTL service failed — sending goto home')
            self._fly_to(home_lat, home_lon, TAKEOFF_ALT, 'Home (manual RTL)')

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║  MISSION COMPLETE  ✅                    ║')
        self.get_logger().info('║  Takeoff → 40m N → 40m W → RTL done    ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SingleUAVMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()