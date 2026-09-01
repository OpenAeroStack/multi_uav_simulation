"""
mech_workshop_validation_mission.py
------------------------------------
Single-UAV real-flight replication mission for the mech_workshop_validation world.

Replicates key parameters from real flight log 2026-08-31 18-03-14.tlog:
  - Takeoff altitude: 3.09m (real average altitude during flight)
  - Target: real max-distance point in the log (28.0m from true launch pad)
  - Groundspeed: 0.35 m/s (real average "moving" speed from the log —
    this flight was a slow hover/loiter test, not a fast cruise)

GPS/world origin:        6.0773722N, 80.1907552E (true launch pad)
Launch pad (true home):  6.0773722N, 80.1907552E
Target (max-distance pt): 6.0771224N, 80.1907863E

Usage:
    ros2 run uav_controller mech_workshop_validation_mission
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool
from geographic_msgs.msg import GeoPoint


# ── Real flight reference points ──────────────────────────────────────────
HOME_LAT, HOME_LON = 6.0773722, 80.1907552   # true launch pad
LANDING_LAT, LANDING_LON = 6.0771224, 80.1907863  # real max-distance point (28.0m)

TAKEOFF_ALT = 3.09    # m — real average altitude from log
HOLD_TIME   = 20.0    # s — hold at target before returning
WP_RADIUS   = 0.75    # m arrival threshold for the validation endpoint
HOME_RADIUS = 3.0     # m maximum permitted launch-position error

WAYPOINT_SPEED = 0.35  # m/s — real "moving" groundspeed from log
CONFIG_REVISION = 'enu-ned-wp-spd-v2'


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


class DroneState:
    def __init__(self):
        self.lat = None
        self.lon = None
        self.rel_alt = 0.0
        self.mode = '---'
        self.armed = False


class MechWorkshopValidationMission(Node):

    def __init__(self):
        super().__init__('mech_workshop_validation_mission')

        self.state = DroneState()

        self.create_subscription(NavSatFix, '/uav1/gps', self._gps_cb, 10)
        self.create_subscription(Float32, '/uav1/rel_alt', self._alt_cb, 10)
        self.create_subscription(String, '/uav1/mode', self._mode_cb, 10)
        self.create_subscription(Bool, '/uav1/armed', self._armed_cb, 10)

        self.takeoff_client = self.create_client(Trigger, '/uav1/takeoff')
        self.land_client = self.create_client(Trigger, '/uav1/land')
        self.goto_pub = self.create_publisher(GeoPoint, '/uav1/goto', 10)

        self.get_logger().info('Waiting 3s before starting mission thread...')
        self.create_timer(3.0, self._start_once)
        self._started = False

    def _start_once(self):
        if self._started:
            return
        self._started = True
        import threading
        threading.Thread(target=self._run, daemon=True).start()

    # ── callbacks ──────────────────────────────────────────────────────────
    def _gps_cb(self, msg):
        self.state.lat = msg.latitude
        self.state.lon = msg.longitude

    def _alt_cb(self, msg):
        self.state.rel_alt = msg.data

    def _mode_cb(self, msg):
        self.state.mode = msg.data

    def _armed_cb(self, msg):
        self.state.armed = msg.data

    # ── helpers ────────────────────────────────────────────────────────────
    def _call_service(self, client, label, timeout=90.0):
        self.get_logger().info(f'-> {label}')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'{label} not available')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error('no response')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info(f'OK {result.message}')
        else:
            self.get_logger().error(f'FAIL {result.message}')
        return result.success

    def _goto(self, lat, lon, alt_m):
        msg = GeoPoint()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = float(alt_m)
        self.goto_pub.publish(msg)

    def _fly_to(self, lat, lon, alt_m, label, speed_mps, timeout=300):
        """
        Sends repeated goto commands (ArduPilot GUIDED mode uses its own
        internal waypoint speed, not a per-command speed parameter via this
        simple goto interface). speed_mps is used only to compute a
        realistic timeout and to log expected vs actual progress —
        actual cruise speed is governed by WP_SPD on this ArduPilot build.
        """
        dist_start = haversine_m(self.state.lat, self.state.lon, lat, lon) \
            if self.state.lat is not None else None
        expected_time = dist_start / speed_mps if dist_start else None
        self.get_logger().info(
            f'-> {label} ({lat:.7f}, {lon:.7f}, {alt_m}m) '
            f'dist={dist_start:.1f}m expected~{expected_time:.0f}s at {speed_mps}m/s'
            if dist_start else f'-> {label}')

        deadline = time.time() + timeout
        last_send = 0
        while time.time() < deadline:
            if time.time() - last_send > 2.0:
                self._goto(lat, lon, alt_m)
                last_send = time.time()
            if self.state.lat is not None:
                dist = haversine_m(self.state.lat, self.state.lon, lat, lon)
                alt_err = abs(self.state.rel_alt - alt_m)
                if dist < WP_RADIUS and alt_err < 1.5:
                    self.get_logger().info(
                        f'Reached {label} (dist={dist:.1f}m alt={self.state.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)
        self.get_logger().warn(f'Timeout reaching {label}')
        return False

    # ── mission ────────────────────────────────────────────────────────────
    def _run(self):
        self.get_logger().info('')
        self.get_logger().info('=== Mech Workshop Validation Mission ===')
        self.get_logger().info(f'Configuration revision: {CONFIG_REVISION}')
        self.get_logger().info(
            f'Replicating real flight 2026-08-31 18-03-14.tlog: '
            f'takeoff={TAKEOFF_ALT}m, target=28.0m away, speed~{WAYPOINT_SPEED}m/s')

        self.get_logger().info('Waiting for GPS...')
        while self.state.lat is None:
            time.sleep(0.3)
        self.get_logger().info('GPS ready. Waiting 10s for system stabilization...')
        time.sleep(10.0)

        home_error = haversine_m(
            self.state.lat, self.state.lon, HOME_LAT, HOME_LON)
        if home_error > HOME_RADIUS:
            self.get_logger().error(
                f'Initial GPS is {home_error:.1f}m from the configured launch '
                f'pad (maximum {HOME_RADIUS:.1f}m); aborting mission. Check '
                'the Gazebo UAV pose and SITL HOME_GPS.')
            return
        self.get_logger().info(
            f'Launch position verified ({home_error:.2f}m from HOME_GPS)')

        # Phase 1 — takeoff to real average altitude
        self.get_logger().info(f'Phase 1 — Takeoff to {TAKEOFF_ALT}m')
        if not self._call_service(self.takeoff_client, '/takeoff', 90):
            self.get_logger().error('Takeoff failed, aborting mission.')
            return
        time.sleep(2.0)

        # Phase 2 — fly to the real max-distance point at real groundspeed
        self.get_logger().info('Phase 2 — Flying to real max-distance point (28.0m)')
        if not self._fly_to(LANDING_LAT, LANDING_LON, TAKEOFF_ALT,
                            'Landing Point', WAYPOINT_SPEED):
            self.get_logger().error(
                'Target was not reached; landing in place and aborting mission.')
            self._call_service(self.land_client, '/uav1/land')
            return

        # Phase 3 — hold
        self.get_logger().info(f'Phase 3 — Holding {HOLD_TIME}s at max distance')
        time.sleep(HOLD_TIME)

        # Recheck after the hold so LAND is only requested over the specified
        # real-flight landing coordinate, not at a drifted hold position.
        landing_error = haversine_m(
            self.state.lat, self.state.lon, LANDING_LAT, LANDING_LON)
        if landing_error >= WP_RADIUS:
            self.get_logger().warn(
                f'Drifted {landing_error:.1f}m from landing point; repositioning')
            if not self._fly_to(LANDING_LAT, LANDING_LON, TAKEOFF_ALT,
                                'Landing Point', WAYPOINT_SPEED):
                self.get_logger().error(
                    'Could not regain landing point; landing in place for safety.')
                self._call_service(self.land_client, '/uav1/land')
                return

        # Phase 4 — land vertically at the specified max-distance point
        landing_error = haversine_m(
            self.state.lat, self.state.lon, LANDING_LAT, LANDING_LON)
        self.get_logger().info(
            f'Phase 4 — Land at ({LANDING_LAT:.7f}, {LANDING_LON:.7f}); '
            f'current error={landing_error:.2f}m')
        self._call_service(self.land_client, '/uav1/land')

        self.get_logger().info('')
        self.get_logger().info('=== MISSION COMPLETE ===')


def main(args=None):
    rclpy.init(args=args)
    node = MechWorkshopValidationMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT may already have shut down the default ROS context.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
