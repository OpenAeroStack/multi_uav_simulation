"""
faculty_mission.py
------------------
3-drone building sweep surveillance mission over
Faculty of Engineering, University of Ruhuna.

GPS Origin: 6.07778°N, 80.20722°E (Admin Building front yard)

Cluster roles:
  UAV1 → Member: sweeps Dept Electrical + Mechanical area
  UAV2 → CLUSTER HEAD: hovers over Admin Building, collects reports
  UAV3 → Member: sweeps Dept Civil + Lecture Halls area

Mission phases:
  1. All 3 takeoff to 30m         ← barrier sync
  2. UAV2 moves to cluster head position (center of campus)
     UAV1 begins Electrical dept sweep
     UAV3 begins Civil dept sweep   ← barrier sync (all at first WP)
  3. UAV1 moves to Mechanical dept
     UAV3 moves to Lecture Halls
     UAV2 continues hovering        ← barrier sync (all at second WP)
  4. All hold 5 seconds             ← barrier sync
  5. All RTL simultaneously

Usage:
    ros2 run uav_controller faculty_mission
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String, Bool
from geographic_msgs.msg import GeoPoint


# ── GPS origin (world 0,0,0) ──────────────────────────────────────────────────
# Faculty of Engineering, University of Ruhuna
ORIGIN_LAT = 6.07778
ORIGIN_LON = 80.20722

# ── Waypoints (approximate GPS from Gazebo world coordinates) ─────────────────
# Gazebo X = East, Y = North in ENU frame
# 1 degree lat ≈ 111,320 m, 1 degree lon ≈ 111,320 * cos(lat) m
# At lat 6.07778: cos(6.07778°) ≈ 0.9944
# So 1 degree lon ≈ 110,700 m
#
# Conversion: gps_lat = ORIGIN_LAT + (Y_gazebo / 111320)
#             gps_lon = ORIGIN_LON + (X_gazebo / 110700)

def gazebo_to_gps(x_m, y_m):
    """Convert Gazebo ENU metres offset to GPS coordinates."""
    lat = ORIGIN_LAT + (y_m / 111320.0)
    lon = ORIGIN_LON + (x_m / 110700.0)
    return lat, lon


# Campus waypoints (Gazebo X,Y offsets in metres from Admin Building)
# These are approximate — tune after first test flight
WP = {
    # UAV2 cluster head — center of campus above Admin Building
    'cluster_head':     gazebo_to_gps(  0,   30),   # central campus

    # UAV1 sweep — Electrical + Mechanical (left/north side of campus)
    'electrical':       gazebo_to_gps(-60,   80),   # Dept Electrical
    'mechanical':       gazebo_to_gps(-40,  120),   # Dept Mechanical

    # UAV3 sweep — Civil + Lecture Halls (right/east side of campus)
    'civil':            gazebo_to_gps( 80,   60),   # Dept Civil
    'lecture_halls':    gazebo_to_gps( 90,  110),   # Lecture Halls 1+2
}

# ── Mission config ────────────────────────────────────────────────────────────
TAKEOFF_ALT   = 30.0   # m — clear building rooftops
SWEEP_ALT     = 25.0   # m — members fly lower during sweep
HEAD_ALT      = 35.0   # m — cluster head flies higher for comms coverage
HOLD_TIME     = 8.0    # seconds
WP_RADIUS     = 3.0    # metres arrival threshold

N_DRONES = 3

barrier_takeoff = threading.Barrier(N_DRONES)
barrier_sweep1  = threading.Barrier(N_DRONES)
barrier_sweep2  = threading.Barrier(N_DRONES)
barrier_hold    = threading.Barrier(N_DRONES)

errors      = []
errors_lock = threading.Lock()


# ── GPS helpers ───────────────────────────────────────────────────────────────

def haversine_m(lat1, lon1, lat2, lon2):
    R = 6_371_000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── Per-drone state ───────────────────────────────────────────────────────────

class DroneState:
    def __init__(self):
        self.lat     = None
        self.lon     = None
        self.rel_alt = 0.0
        self.mode    = '---'
        self.armed   = False
        self.gps_ok  = False


# ── Mission node ──────────────────────────────────────────────────────────────

class FacultyMission(Node):

    def __init__(self):
        super().__init__('faculty_mission')

        self.states          = {}
        self.takeoff_clients = {}
        self.rtl_clients     = {}
        self.goto_pubs       = {}

        for uid in [1, 2, 3]:
            ns = f'/uav{uid}'
            s  = DroneState()
            self.states[uid] = s

            self.create_subscription(
                NavSatFix, f'{ns}/gps',
                self._make_gps_cb(uid), 10)
            self.create_subscription(
                Float32, f'{ns}/rel_alt',
                self._make_alt_cb(uid), 10)
            self.create_subscription(
                String, f'{ns}/mode',
                self._make_mode_cb(uid), 10)
            self.create_subscription(
                Bool, f'{ns}/armed',
                self._make_armed_cb(uid), 10)

            self.takeoff_clients[uid] = self.create_client(
                Trigger, f'{ns}/takeoff')
            self.rtl_clients[uid] = self.create_client(
                Trigger, f'{ns}/rtl')
            self.goto_pubs[uid] = self.create_publisher(
                GeoPoint, f'{ns}/goto', 10)

        # Cluster status publisher — cluster head publishes summary here
        self.pub_cluster = self.create_publisher(
            String, '/cluster/status', 10)

        threading.Thread(target=self._run_all, daemon=True).start()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _make_gps_cb(self, uid):
        def cb(msg):
            s = self.states[uid]
            s.lat    = msg.latitude
            s.lon    = msg.longitude
            s.gps_ok = (msg.latitude != 0.0 or msg.longitude != 0.0)
        return cb

    def _make_alt_cb(self, uid):
        def cb(msg): self.states[uid].rel_alt = msg.data
        return cb

    def _make_mode_cb(self, uid):
        def cb(msg): self.states[uid].mode = msg.data
        return cb

    def _make_armed_cb(self, uid):
        def cb(msg): self.states[uid].armed = msg.data
        return cb

    # ── helpers ───────────────────────────────────────────────────────────────

    def _call_service(self, client, uid, label, timeout=90.0):
        self.get_logger().info(f'  [UAV{uid}] → {label}')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'  [UAV{uid}] {label} not available')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'  [UAV{uid}] {label} no response')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info(f'  [UAV{uid}] ✓ {result.message}')
        else:
            self.get_logger().error(f'  [UAV{uid}] ✗ {result.message}')
        return result.success

    def _goto(self, uid, lat, lon, alt_m):
        msg = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = float(alt_m)
        self.goto_pubs[uid].publish(msg)

    def _fly_to(self, uid, lat, lon, alt_m, label, timeout=120):
        self.get_logger().info(
            f'  [UAV{uid}] → Flying to {label} '
            f'({lat:.6f}, {lon:.6f}, {alt_m}m)')
        deadline  = time.time() + timeout
        last_send = 0
        while time.time() < deadline:
            if time.time() - last_send > 2.0:
                self._goto(uid, lat, lon, alt_m)
                last_send = time.time()
            s = self.states[uid]
            if s.lat is not None:
                dist    = haversine_m(s.lat, s.lon, lat, lon)
                alt_err = abs(s.rel_alt - alt_m)
                if dist < WP_RADIUS and alt_err < 3.0:
                    self.get_logger().info(
                        f'  [UAV{uid}] ✓ Reached {label} '
                        f'(dist={dist:.1f}m alt={s.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)
        self.get_logger().warn(f'  [UAV{uid}] ⚠ Timeout at {label}')
        return False

    def _publish_cluster_status(self):
        """Cluster head (UAV2) publishes aggregated status."""
        s1 = self.states[1]
        s2 = self.states[2]
        s3 = self.states[3]
        msg = String()
        msg.data = (
            f'CLUSTER_STATUS | '
            f'HEAD=UAV2({s2.lat:.5f},{s2.lon:.5f},{s2.rel_alt:.1f}m) | '
            f'M1=UAV1({s1.lat:.5f},{s1.lon:.5f},{s1.rel_alt:.1f}m) | '
            f'M2=UAV3({s3.lat:.5f},{s3.lon:.5f},{s3.rel_alt:.1f}m)'
        )
        self.pub_cluster.publish(msg)

    # ── per-drone missions ────────────────────────────────────────────────────

    def _mission_uav1(self):
        """UAV1 — Member: sweeps Dept Electrical then Mechanical."""
        uid = 1
        s   = self.states[uid]
        try:
            self.get_logger().info(f'[UAV1] Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info(f'[UAV1] GPS ready. Waiting 15s for EKF...')
            time.sleep(15.0)

            # Takeoff
            self.get_logger().info('[UAV1] Phase 1 — Takeoff to 30m')
            if not self._call_service(
                    self.takeoff_clients[1], 1, '/takeoff', 90):
                raise RuntimeError('UAV1 takeoff failed')
            time.sleep(2.0)
            barrier_takeoff.wait()

            # Sweep 1 — Dept Electrical
            self.get_logger().info('[UAV1] Phase 2 — Sweeping Dept Electrical')
            lat, lon = WP['electrical']
            self._fly_to(1, lat, lon, SWEEP_ALT, 'Dept Electrical')
            barrier_sweep1.wait()

            # Sweep 2 — Dept Mechanical
            self.get_logger().info('[UAV1] Phase 3 — Sweeping Dept Mechanical')
            lat, lon = WP['mechanical']
            self._fly_to(1, lat, lon, SWEEP_ALT, 'Dept Mechanical')
            barrier_sweep2.wait()

            # Hold
            self.get_logger().info(f'[UAV1] Phase 4 — Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            # RTL
            self.get_logger().info('[UAV1] Phase 5 — RTL')
            self._call_service(self.rtl_clients[1], 1, '/rtl')
            self.get_logger().info('[UAV1] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV1] ERROR: {e}')
            with errors_lock: errors.append((1, str(e)))

    def _mission_uav2(self):
        """UAV2 — CLUSTER HEAD: hovers center, publishes cluster status."""
        uid = 2
        s   = self.states[uid]
        try:
            self.get_logger().info('[UAV2] CLUSTER HEAD — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info('[UAV2] GPS ready. Waiting 15s for EKF...')
            time.sleep(15.0)

            # Takeoff
            self.get_logger().info('[UAV2] Phase 1 — Takeoff to 30m')
            if not self._call_service(
                    self.takeoff_clients[2], 2, '/takeoff', 90):
                raise RuntimeError('UAV2 takeoff failed')
            time.sleep(2.0)
            barrier_takeoff.wait()

            # Move to cluster head position (higher altitude for coverage)
            self.get_logger().info(
                '[UAV2] Phase 2 — Moving to cluster head position')
            lat, lon = WP['cluster_head']
            self._fly_to(2, lat, lon, HEAD_ALT, 'Cluster Head Position')

            # Start publishing cluster status every 2 seconds
            self.get_logger().info(
                '[UAV2] Publishing cluster status on /cluster/status')
            cluster_timer = self.create_timer(2.0, self._publish_cluster_status)

            barrier_sweep1.wait()

            # Hold at cluster head position while members do sweep 2
            self.get_logger().info('[UAV2] Phase 3 — Holding at cluster head')
            barrier_sweep2.wait()

            # Hold
            self.get_logger().info(f'[UAV2] Phase 4 — Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            # RTL
            cluster_timer.cancel()
            self.get_logger().info('[UAV2] Phase 5 — RTL')
            self._call_service(self.rtl_clients[2], 2, '/rtl')
            self.get_logger().info('[UAV2] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV2] ERROR: {e}')
            with errors_lock: errors.append((2, str(e)))

    def _mission_uav3(self):
        """UAV3 — Member: sweeps Dept Civil then Lecture Halls."""
        uid = 3
        s   = self.states[uid]
        try:
            self.get_logger().info('[UAV3] Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info('[UAV3] GPS ready. Waiting 15s for EKF...')
            time.sleep(15.0)

            # Takeoff
            self.get_logger().info('[UAV3] Phase 1 — Takeoff to 30m')
            if not self._call_service(
                    self.takeoff_clients[3], 3, '/takeoff', 90):
                raise RuntimeError('UAV3 takeoff failed')
            time.sleep(2.0)
            barrier_takeoff.wait()

            # Sweep 1 — Dept Civil
            self.get_logger().info('[UAV3] Phase 2 — Sweeping Dept Civil')
            lat, lon = WP['civil']
            self._fly_to(3, lat, lon, SWEEP_ALT, 'Dept Civil')
            barrier_sweep1.wait()

            # Sweep 2 — Lecture Halls
            self.get_logger().info('[UAV3] Phase 3 — Sweeping Lecture Halls')
            lat, lon = WP['lecture_halls']
            self._fly_to(3, lat, lon, SWEEP_ALT, 'Lecture Halls')
            barrier_sweep2.wait()

            # Hold
            self.get_logger().info(f'[UAV3] Phase 4 — Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            # RTL
            self.get_logger().info('[UAV3] Phase 5 — RTL')
            self._call_service(self.rtl_clients[3], 3, '/rtl')
            self.get_logger().info('[UAV3] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV3] ERROR: {e}')
            with errors_lock: errors.append((3, str(e)))

    # ── launch all threads ────────────────────────────────────────────────────

    def _run_all(self):
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════╗')
        self.get_logger().info('║  Faculty of Engineering — UAV Surveillance Demo  ║')
        self.get_logger().info('║  University of Ruhuna, Hapugala, Galle           ║')
        self.get_logger().info('║                                                  ║')
        self.get_logger().info('║  UAV1 → Member: Electrical + Mechanical sweep    ║')
        self.get_logger().info('║  UAV2 → CLUSTER HEAD: Admin Building hover       ║')
        self.get_logger().info('║  UAV3 → Member: Civil + Lecture Halls sweep      ║')
        self.get_logger().info('╚══════════════════════════════════════════════════╝')
        self.get_logger().info('')

        threads = [
            threading.Thread(target=self._mission_uav1, name='UAV1', daemon=True),
            threading.Thread(target=self._mission_uav2, name='UAV2', daemon=True),
            threading.Thread(target=self._mission_uav3, name='UAV3', daemon=True),
        ]

        for t in threads: t.start()
        for t in threads: t.join()

        self.get_logger().info('')
        if errors:
            self.get_logger().error('╔══════════════════════════════════════╗')
            self.get_logger().error('║  MISSION FINISHED WITH ERRORS        ║')
            for uid, err in errors:
                self.get_logger().error(f'║  UAV{uid}: {err}')
            self.get_logger().error('╚══════════════════════════════════════╝')
        else:
            self.get_logger().info('╔══════════════════════════════════════╗')
            self.get_logger().info('║  FACULTY SURVEILLANCE COMPLETE  ✅   ║')
            self.get_logger().info('╚══════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = FacultyMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
