"""
faculty_dynamic_mission.py
---------------------------
Dynamic-clustering variant of faculty_mission.py for the 3-drone
surveillance mission at Faculty of Engineering, University of Ruhuna,
Hapugala, Galle, Sri Lanka.

Unlike faculty_mission.py, this node has NO fixed "UAV1 is cluster head"
role. Cluster leadership is elected independently and continuously by
dynamic_cluster_manager over /cluster/primary_ch — this mission does not
read that topic and does not assert or assume which UAV currently holds
the primary role. Flight behaviour here is fixed at launch and identical
regardless of which UAV dynamic_cluster_manager elects as primary at any
given moment; the two nodes coordinate only in the sense that they run
concurrently, not through any topic this mission consumes.

GPS Origin: 6.0792673°N, 80.1921607°E
  (Front entrance, Administration Building)

Campus bounds (terrain mesh): ~382m (E-W) × 535m (N-S)
All patrol waypoints stay within ~150m of origin — safely inside campus.

Flight plan (same geometry as faculty_mission.py, no head/member labels):
  UAV1 → Center: hovers at 60m above campus center
  UAV2 → North: patrols Main Gate / Guard Room area
  UAV3 → South: patrols Electrical Dept / Civil Dept area

Mission phases:
  1. All 3 takeoff simultaneously — vertically separated:
       UAV1 → 45m,  UAV2 → 35m,  UAV3 → 40m
       (set via drone_bridge takeoff_altitude param in launch script)

  2. barrier_takeoff: all airborne
     UAV1 climbs to 60m and holds position (Center)
     UAV2 flies North ~130m @ 50m  ← toward Main Gate / Guard Room
     UAV3 flies South ~130m @ 50m  ← toward Electrical / Civil Dept

  3. barrier_patrol1: all reached first waypoints
     UAV1 stays at Center
     UAV2 flies NW ~100m @ 50m  ← sweeps toward Library / Bo Tree
     UAV3 flies SW ~100m @ 50m  ← sweeps toward Mech Workshop

  4. barrier_patrol2: all reached second waypoints
     All hold 5 seconds

  5. barrier_hold → all RTL simultaneously

No /cluster/status is published by this node — it is not consumed by
dynamic_cluster_manager or scripts/viz_dashboard_stable_topology.py.
Camera feeds: /uav2/camera/image_raw, /uav3/camera/image_raw

Campus reference (Gazebo coords):
  North sector:  Main Gate (X=49, Y=38), Guard Room (X=37, Y=46)
  South sector:  Electrical Dept (X=2, Y=-110), Civil Dept (X=-92, Y=-131)

Usage:
    Run AFTER all 3 bridges show: ✓ DDS GPS flowing
    Intended to run alongside: ros2 run uav_controller dynamic_cluster_manager
    ros2 run uav_controller faculty_dynamic_mission
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

HEAD_ALT      = 60.0   # m — center-hover altitude
PATROL_ALT    = 50.0   # m — patrol altitude
PATROL_DIST_1 = 130.0  # m — first waypoint distance from origin
PATROL_DIST_2 = 100.0  # m — second waypoint distance from first
HOLD_TIME     =   5.0  # s  — hold at final waypoint before RTL
WP_RADIUS     =   3.0  # m  — arrival threshold

N_DRONES = 3

# uid: (dist1, bearing1, dist2, bearing2, alt, label)
# UAV1 uses zero-distance legs to hold its takeoff position at HEAD_ALT —
# same physical flight geometry as faculty_mission.py's head hover, just
# expressed through the same generic per-drone function as UAV2/UAV3
# instead of a UAV1-only method.
PATROL_PLAN = {
    1: (0.0,   0.0, 0.0,   0.0, HEAD_ALT,   'Center'),
    2: (PATROL_DIST_1, 0.0,   PATROL_DIST_2, 315.0, PATROL_ALT, 'North'),
    3: (PATROL_DIST_1, 180.0, PATROL_DIST_2, 225.0, PATROL_ALT, 'South'),
}

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

class FacultyDynamicMission(Node):

    def __init__(self):
        super().__init__('faculty_dynamic_mission')

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

    # ── per-drone mission ─────────────────────────────────────────────────────

    def _mission_patrol(self, uid, dist1, bearing1, dist2, bearing2, alt, label):
        """
        Generic per-drone patrol, used identically for UAV1/2/3 — no
        uav_id-based branching. UAV1's plan (dist1=dist2=0) reduces this
        to holding its takeoff position at `alt`; UAV2/UAV3 fly the same
        two-leg bearing patrol as in faculty_mission.py.
        """
        s = self.states[uid]
        try:
            self.get_logger().info(f'[UAV{uid}] {label} — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info(f'[UAV{uid}] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            self.get_logger().info(f'[UAV{uid}] Phase 1 — Takeoff')
            if not self._call_service(self.takeoff_clients[uid], uid, '/takeoff', 90):
                raise RuntimeError(f'UAV{uid} takeoff failed')
            time.sleep(1.0)
            barrier_takeoff.wait()

            # Phase 2 — first waypoint
            p1_lat, p1_lon = move_gps(home_lat, home_lon, dist1, bearing1)
            self.get_logger().info(f'[UAV{uid}] Phase 2 — {label} P1 (bearing {bearing1}°)')
            self._fly_to(uid, p1_lat, p1_lon, alt, f'{label} P1')
            barrier_patrol1.wait()

            # Phase 3 — second waypoint from P1
            p2_lat, p2_lon = move_gps(p1_lat, p1_lon, dist2, bearing2)
            self.get_logger().info(f'[UAV{uid}] Phase 3 — {label} P2 (bearing {bearing2}° from P1)')
            self._fly_to(uid, p2_lat, p2_lon, alt, f'{label} P2')
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
        self.get_logger().info('║   Faculty of Engineering — Dynamic Clustering Mission   ║')
        self.get_logger().info('║   University of Ruhuna, Hapugala, Galle, Sri Lanka      ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  Cluster head elected dynamically by                    ║')
        self.get_logger().info('║  dynamic_cluster_manager over /cluster/primary_ch —     ║')
        self.get_logger().info('║  this mission does not read that topic.                 ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  UAV1 → Center: 60m hover above campus center           ║')
        self.get_logger().info('║  UAV2 → North: Main Gate / Guard Room                    ║')
        self.get_logger().info('║  UAV3 → South: Electrical Dept / Civil Dept              ║')
        self.get_logger().info('║                                                          ║')
        self.get_logger().info('║  Takeoff separation: UAV1=45m  UAV2=35m  UAV3=40m      ║')
        self.get_logger().info('╚══════════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info('[Mission] Waiting 15s for ROS2 subscriptions to connect...')
        time.sleep(15.0)

        threads = [
            threading.Thread(
                target=self._mission_patrol,
                args=(uid, *PATROL_PLAN[uid]),
                name=f'UAV{uid}-{PATROL_PLAN[uid][-1]}', daemon=True)
            for uid in [1, 2, 3]
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
            self.get_logger().info('║  FACULTY DYNAMIC-CLUSTERING MISSION COMPLETE  ✅  ║')
            self.get_logger().info('╚══════════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = FacultyDynamicMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
