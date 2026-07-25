"""
faculty_mission.py
------------------
3-drone clustering surveillance mission at Faculty of Engineering,
University of Ruhuna, Hapugala, Galle, Sri Lanka.

GPS Origin: 6.0792673°N, 80.1921607°E
  (Front entrance, Administration Building)

Campus bounds (terrain mesh): ~382m (E-W) × 535m (N-S)
All patrol waypoints stay within ~150m of origin — safely inside campus.

Cluster roles:
  UAV1 → CLUSTER HEAD: hovers at 60m above campus center,
                        aggregates and publishes member reports
  UAV2 → Member North: patrols Main Gate / Guard Room area
  UAV3 → Member South: patrols Electrical Dept / Civil Dept area

Mission phases:
  1. All 3 takeoff simultaneously — vertically separated:
       UAV1 → 45m,  UAV2 → 35m,  UAV3 → 40m
       (set via drone_bridge takeoff_altitude param in launch script)

  2. barrier_takeoff: all airborne
     UAV1 climbs to 60m (head hover — stays here all mission)
     UAV2 flies North ~130m @ 50m  ← toward Main Gate / Guard Room
     UAV3 flies South ~130m @ 50m  ← toward Electrical / Civil Dept

  3. barrier_patrol1: all reached first waypoints
     UAV2 flies NW ~100m @ 50m  ← sweeps toward Library / Bo Tree
     UAV3 flies SW ~100m @ 50m  ← sweeps toward Mech Workshop

  4. barrier_patrol2: all reached second waypoints
     All hold 5 seconds

  5. barrier_hold → all RTL simultaneously

Cluster status published to /cluster/status every 2 seconds.
Camera feeds: /uav2/camera/image_raw, /uav3/camera/image_raw

Campus reference (Gazebo coords):
  North sector:  Main Gate (X=49, Y=38), Guard Room (X=37, Y=46)
  South sector:  Electrical Dept (X=2, Y=-110), Civil Dept (X=-92, Y=-131)

Usage:
    Run AFTER all 3 bridges show: ✓ DDS GPS flowing
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


# ── GPS helpers ───────────────────────────────────────────────────────────────

def move_gps(lat, lon, distance_m, bearing_deg):
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
    R    = 6_371_000.0
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a    = (math.sin(dphi / 2) ** 2 +
            math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── Mission config ────────────────────────────────────────────────────────────

HEAD_ALT      = 60.0   # m — cluster head hover altitude
PATROL_ALT    = 50.0   # m — member patrol altitude
PATROL_DIST_1 = 130.0  # m — first waypoint distance from origin
PATROL_DIST_2 = 100.0  # m — second waypoint distance from first
HOLD_TIME     =   5.0  # s  — hold at final waypoint before RTL
WP_RADIUS     =   3.0  # m  — arrival threshold

N_DRONES = 3

barrier_takeoff = threading.Barrier(N_DRONES)
barrier_patrol1 = threading.Barrier(N_DRONES)
barrier_patrol2 = threading.Barrier(N_DRONES)
barrier_hold    = threading.Barrier(N_DRONES)

errors      = []
errors_lock = threading.Lock()


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

            self.create_subscription(NavSatFix, f'{ns}/gps',
                self._make_gps_cb(uid),   10)
            self.create_subscription(Float32,   f'{ns}/rel_alt',
                self._make_alt_cb(uid),   10)
            self.create_subscription(String,    f'{ns}/mode',
                self._make_mode_cb(uid),  10)
            self.create_subscription(Bool,      f'{ns}/armed',
                self._make_armed_cb(uid), 10)

            self.takeoff_clients[uid] = self.create_client(
                Trigger, f'{ns}/takeoff')
            self.rtl_clients[uid]     = self.create_client(
                Trigger, f'{ns}/rtl')
            self.goto_pubs[uid]       = self.create_publisher(
                GeoPoint, f'{ns}/goto', 10)

        self.pub_cluster = self.create_publisher(String, '/cluster/status', 10)

        threading.Thread(target=self._run_all, daemon=True).start()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _make_gps_cb(self, uid):
        def cb(msg):
            s        = self.states[uid]
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
        self.get_logger().info(f'  [UAV{uid}] -> {label}')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f'  [UAV{uid}] {label} not available')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'  [UAV{uid}] no response')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info(f'  [UAV{uid}] OK {result.message}')
        else:
            self.get_logger().error(f'  [UAV{uid}] FAIL {result.message}')
        return result.success

    def _goto(self, uid, lat, lon, alt_m):
        msg           = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = float(alt_m)
        self.goto_pubs[uid].publish(msg)

    def _fly_to(self, uid, lat, lon, alt_m, label, timeout=120):
        self.get_logger().info(
            f'  [UAV{uid}] -> {label} ({lat:.6f}, {lon:.6f}, {alt_m}m)')
        deadline  = time.time() + timeout
        last_send = 0.0
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
                        f'  [UAV{uid}] Reached {label} '
                        f'(dist={dist:.1f}m alt={s.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)
        self.get_logger().warn(f'  [UAV{uid}] Timeout reaching {label}')
        return False

    def _publish_cluster_status(self):
        s = self.states
        if any(s[uid].lat is None for uid in [1, 2, 3]):
            return
        msg      = String()
        msg.data = (
            f'CLUSTER_STATUS | '
            f'HEAD=UAV1(alt={s[1].rel_alt:.1f}m mode={s[1].mode}) | '
            f'M1=UAV2(lat={s[2].lat:.5f} lon={s[2].lon:.5f} alt={s[2].rel_alt:.1f}m) | '
            f'M2=UAV3(lat={s[3].lat:.5f} lon={s[3].lon:.5f} alt={s[3].rel_alt:.1f}m)'
        )
        self.pub_cluster.publish(msg)

    # ── per-drone missions ────────────────────────────────────────────────────

    def _mission_uav1(self):
        """UAV1 — CLUSTER HEAD. Takes off to 45m, climbs to 60m, hovers."""
        uid = 1
        s   = self.states[uid]
        try:
            self.get_logger().info('[UAV1] CLUSTER HEAD — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info('[UAV1] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            self.get_logger().info('[UAV1] Phase 1 — Takeoff to 45m')
            if not self._call_service(self.takeoff_clients[1], 1, '/takeoff', 90):
                raise RuntimeError('UAV1 takeoff failed')
            time.sleep(1.0)
            barrier_takeoff.wait()

            self.get_logger().info(f'[UAV1] Climbing to {HEAD_ALT}m cluster head position')
            self._fly_to(1, home_lat, home_lon, HEAD_ALT, 'Head Hover (60m)')

            cluster_timer = self.create_timer(2.0, self._publish_cluster_status)
            self.get_logger().info('[UAV1] ✓ Broadcasting /cluster/status every 2s')
            self.get_logger().info('[UAV1] Hovering — waiting for members to complete patrol...')

            barrier_patrol1.wait()
            barrier_patrol2.wait()

            self.get_logger().info(f'[UAV1] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            cluster_timer.cancel()
            self.get_logger().info('[UAV1] RTL')
            self._call_service(self.rtl_clients[1], 1, '/rtl')
            self.get_logger().info('[UAV1] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV1] ERROR: {e}')
            with errors_lock: errors.append((1, str(e)))

    def _mission_member(self, uid, bearing1, bearing2, label):
        """
        Member drone patrol.
          bearing1: bearing from origin to P1
          bearing2: bearing from P1 to P2
          label:    sector name e.g. 'North'
        Takeoff altitude set via drone_bridge param in launch script.
        """
        s = self.states[uid]
        try:
            self.get_logger().info(f'[UAV{uid}] Member ({label}) — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info(f'[UAV{uid}] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            self.get_logger().info(f'[UAV{uid}] Phase 1 — Takeoff')
            if not self._call_service(self.takeoff_clients[uid], uid, '/takeoff', 90):
                raise RuntimeError(f'UAV{uid} takeoff failed')
            time.sleep(1.0)
            barrier_takeoff.wait()

            # Phase 2 — first patrol waypoint
            p1_lat, p1_lon = move_gps(home_lat, home_lon, PATROL_DIST_1, bearing1)
            self.get_logger().info(f'[UAV{uid}] Phase 2 — {label} P1 (bearing {bearing1}°)')
            self._fly_to(uid, p1_lat, p1_lon, PATROL_ALT, f'{label} P1')
            barrier_patrol1.wait()

            # Phase 3 — second patrol waypoint from P1
            p2_lat, p2_lon = move_gps(p1_lat, p1_lon, PATROL_DIST_2, bearing2)
            self.get_logger().info(f'[UAV{uid}] Phase 3 — {label} P2 (bearing {bearing2}° from P1)')
            self._fly_to(uid, p2_lat, p2_lon, PATROL_ALT, f'{label} P2')
            barrier_patrol2.wait()

            self.get_logger().info(f'[UAV{uid}] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            self.get_logger().info(f'[UAV{uid}] RTL')
            self._call_service(self.rtl_clients[uid], uid, '/rtl')
            self.get_logger().info(f'[UAV{uid}] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV{uid}] ERROR: {e}')
            with errors_lock: errors.append((uid, str(e)))

    # ── launch all threads ────────────────────────────────────────────────────

    def _run_all(self):
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════════════╗')
        self.get_logger().info('║   Faculty of Engineering — Campus Surveillance Mission  ║')
        self.get_logger().info('║   University of Ruhuna, Hapugala, Galle, Sri Lanka      ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  UAV1 → CLUSTER HEAD (60m hover above campus center)   ║')
        self.get_logger().info('║  UAV2 → Member North: Main Gate / Guard Room            ║')
        self.get_logger().info('║  UAV3 → Member South: Electrical Dept / Civil Dept      ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  Takeoff separation: UAV1=45m  UAV2=35m  UAV3=40m      ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  Monitor: ros2 topic echo /cluster/status                ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info('[Mission] Waiting 15s for ROS2 subscriptions to connect...')
        time.sleep(15.0)

        threads = [
            threading.Thread(
                target=self._mission_uav1,
                name='UAV1-HEAD', daemon=True),
            threading.Thread(
                target=self._mission_member,
                # UAV2 North: origin → North 130m, then NW 100m
                # Covers: Main Gate (Y=+38), Guard Room (Y=+46)
                args=(2, 0.0, 315.0, 'North'),
                name='UAV2-North', daemon=True),
            threading.Thread(
                target=self._mission_member,
                # UAV3 South: origin → South 130m, then SW 100m
                # Covers: Electrical Dept (Y=-110), Civil Dept (Y=-131)
                args=(3, 180.0, 225.0, 'South'),
                name='UAV3-South', daemon=True),
        ]

        for t in threads: t.start()
        for t in threads: t.join()

        self.get_logger().info('')
        if errors:
            self.get_logger().error('╔══════════════════════════════════════════╗')
            self.get_logger().error('║  MISSION FINISHED WITH ERRORS            ║')
            for uid, err in errors:
                self.get_logger().error(f'║  UAV{uid}: {err}')
            self.get_logger().error('╚══════════════════════════════════════════╝')
        else:
            self.get_logger().info('╔══════════════════════════════════════════╗')
            self.get_logger().info('║  FACULTY CAMPUS MISSION COMPLETE  ✅     ║')
            self.get_logger().info('╚══════════════════════════════════════════╝')


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