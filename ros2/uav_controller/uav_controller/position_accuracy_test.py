"""
position_accuracy_test.py
--------------------------
Validates ArduPilot SITL position accuracy by commanding
a single drone to known GPS waypoints and measuring arrival error.

Commanded waypoints are computed using move_gps() from home position.
Actual arrival position is read from /uav1/gps topic.
Error = haversine distance between commanded and actual position.

Usage:
    ros2 run uav_controller position_accuracy_test
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


# ── GPS helpers ───────────────────────────────────────────────────────────────

def move_gps(lat, lon, distance_m, bearing_deg):
    R = 6_371_000.0
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    lat2 = math.asin(
        math.sin(lat1) * math.cos(distance_m / R) +
        math.cos(lat1) * math.sin(distance_m / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(distance_m / R) * math.cos(lat1),
        math.cos(distance_m / R) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── Test config ───────────────────────────────────────────────────────────────

TAKEOFF_ALT  = 20.0   # metres
TEST_ALT     = 20.0   # fly all waypoints at this altitude
WP_RADIUS    = 3.0    # arrival threshold metres
HOVER_TIME   = 3.0    # seconds to hover at each waypoint before recording

# 5 test waypoints — North, East, South, West, diagonal
# Each 50m from home position
TEST_BEARINGS = [
    (0.0,   'North  50m'),
    (90.0,  'East   50m'),
    (180.0, 'South  50m'),
    (270.0, 'West   50m'),
    (45.0,  'NE     50m'),
]
TEST_DISTANCE = 50.0  # metres


# ── Node ──────────────────────────────────────────────────────────────────────

class PositionAccuracyTest(Node):

    def __init__(self):
        super().__init__('position_accuracy_test')

        self.lat     = None
        self.lon     = None
        self.rel_alt = 0.0
        self.mode    = '---'

        self.create_subscription(NavSatFix, '/uav1/gps',
            lambda msg: self._set_gps(msg), 10)
        self.create_subscription(Float32, '/uav1/rel_alt',
            lambda msg: setattr(self, 'rel_alt', msg.data), 10)
        self.create_subscription(String, '/uav1/mode',
            lambda msg: setattr(self, 'mode', msg.data), 10)

        self.takeoff_client = self.create_client(Trigger, '/uav1/takeoff')
        self.rtl_client     = self.create_client(Trigger, '/uav1/rtl')
        self.goto_pub       = self.create_publisher(GeoPoint, '/uav1/goto', 10)

        self.results = []

        threading.Thread(target=self._run_test, daemon=True).start()

    def _set_gps(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _call_service(self, client, label, timeout=90.0):
        self.get_logger().info(f'  -> {label}')
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return False
        return future.result().success

    def _goto(self, lat, lon, alt):
        msg = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = float(alt)
        self.goto_pub.publish(msg)

    def _fly_to(self, lat, lon, alt, label, timeout=90):
        deadline  = time.time() + timeout
        last_send = 0
        while time.time() < deadline:
            if time.time() - last_send > 2.0:
                self._goto(lat, lon, alt)
                last_send = time.time()
            if self.lat is not None:
                dist    = haversine_m(self.lat, self.lon, lat, lon)
                alt_err = abs(self.rel_alt - alt)
                if dist < WP_RADIUS and alt_err < 3.0:
                    return True
            time.sleep(0.3)
        return False

    def _run_test(self):
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║   ArduPilot SITL Position Accuracy Test  ║')
        self.get_logger().info('║   5 waypoints × 50m distance             ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')
        self.get_logger().info('')

        # Wait for GPS
        self.get_logger().info('Waiting for GPS...')
        while self.lat is None:
            time.sleep(0.3)
        self.get_logger().info(f'GPS ready: ({self.lat:.6f}, {self.lon:.6f})')
        self.get_logger().info('Waiting 30s for EKF...')
        time.sleep(30.0)

        home_lat = self.lat
        home_lon = self.lon

        # Takeoff
        self.get_logger().info(f'Takeoff to {TAKEOFF_ALT}m...')
        if not self._call_service(self.takeoff_client, '/takeoff'):
            self.get_logger().error('Takeoff failed')
            return
        time.sleep(5.0)

        # Fly to each test waypoint
        for bearing, label in TEST_BEARINGS:
            wp_lat, wp_lon = move_gps(home_lat, home_lon,
                                       TEST_DISTANCE, bearing)

            self.get_logger().info(f'\nFlying to {label}...')
            self.get_logger().info(
                f'  Commanded: ({wp_lat:.6f}, {wp_lon:.6f})')

            reached = self._fly_to(wp_lat, wp_lon, TEST_ALT, label)

            if reached:
                # Hover briefly then record actual position
                time.sleep(HOVER_TIME)
                actual_lat = self.lat
                actual_lon = self.lon
                actual_alt = self.rel_alt

                error_m = haversine_m(
                    actual_lat, actual_lon, wp_lat, wp_lon)
                alt_error = abs(actual_alt - TEST_ALT)

                self.get_logger().info(
                    f'  Actual:    ({actual_lat:.6f}, {actual_lon:.6f})')
                self.get_logger().info(
                    f'  Position error: {error_m:.2f}m')
                self.get_logger().info(
                    f'  Altitude error: {alt_error:.2f}m')

                self.results.append({
                    'label':     label,
                    'commanded': (wp_lat, wp_lon),
                    'actual':    (actual_lat, actual_lon),
                    'error_m':   error_m,
                    'alt_err':   alt_error,
                })
            else:
                self.get_logger().warn(f'  Timeout reaching {label}')
                self.results.append({
                    'label':   label,
                    'error_m': None,
                    'alt_err': None,
                })

        # RTL
        self.get_logger().info('\nAll waypoints visited. RTL...')
        self._call_service(self.rtl_client, '/rtl')

        # Print results table
        self._print_results()

    def _print_results(self):
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════╗')
        self.get_logger().info('║           POSITION ACCURACY RESULTS                  ║')
        self.get_logger().info('╠══════════════════════════════════════════╦═══════════╣')
        self.get_logger().info('║  Waypoint              Horiz Error (m)   Alt Err (m) ║')
        self.get_logger().info('╠══════════════════════════════════════════╦═══════════╣')

        valid = [r for r in self.results if r['error_m'] is not None]
        for r in self.results:
            if r['error_m'] is not None:
                self.get_logger().info(
                    f"║  {r['label']:<20}  {r['error_m']:>6.2f} m"
                    f"          {r['alt_err']:>5.2f} m   ║")
            else:
                self.get_logger().info(
                    f"║  {r['label']:<20}  TIMEOUT               ║")

        if valid:
            mean_err = sum(r['error_m'] for r in valid) / len(valid)
            max_err  = max(r['error_m'] for r in valid)
            min_err  = min(r['error_m'] for r in valid)
            mean_alt = sum(r['alt_err'] for r in valid) / len(valid)

            errors = [r['error_m'] for r in valid]
            std_err = math.sqrt(
                sum((e - mean_err)**2 for e in errors) / len(errors))

            self.get_logger().info('╠══════════════════════════════════════════════════════╣')
            self.get_logger().info(f'║  Mean position error:   {mean_err:>6.2f} m                   ║')
            self.get_logger().info(f'║  Std deviation:         {std_err:>6.2f} m                   ║')
            self.get_logger().info(f'║  Min error:             {min_err:>6.2f} m                   ║')
            self.get_logger().info(f'║  Max error:             {max_err:>6.2f} m                   ║')
            self.get_logger().info(f'║  Mean altitude error:   {mean_alt:>6.2f} m                   ║')
            self.get_logger().info('╠══════════════════════════════════════════════════════╣')
            self.get_logger().info('║  Reference (ArduPilot SITL uBlox M8N model):         ║')
            self.get_logger().info('║  Expected GPS CEP: ~2.5m                             ║')
            self.get_logger().info('║  WP_RADIUS threshold: 3.0m                           ║')
            self.get_logger().info('╚══════════════════════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = PositionAccuracyTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()