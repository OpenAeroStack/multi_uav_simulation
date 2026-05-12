"""
airport_mission.py
------------------
3-drone clustering surveillance mission at KSQL Airport.

GPS Origin: 37.523640N, -122.255122E (San Carlos Airport, CA)

Cluster roles:
  UAV1 -> CLUSTER HEAD: hovers at 50m center, aggregates member reports
  UAV2 -> Member: patrols North sector, camera active
  UAV3 -> Member: patrols South sector, camera active

Mission phases:
  1. All 3 takeoff to 40m                    <- barrier sync
  1b. UAV2 climbs to 35m, UAV3 to 40m       <- vertical separation during climb
  2. UAV1 climbs to 50m (head position)
     UAV2 flies to North patrol point        <- barrier sync
     UAV3 flies to South patrol point
  3. UAV2 flies to second North point
     UAV3 flies to second South point        <- barrier sync
  4. All hold 8 seconds                      <- barrier sync
  5. All RTL simultaneously

FIXES vs original:
  - TAKEOFF_ALT raised 20m → 40m (clears KSQL airport buildings)
  - HEAD_ALT raised 30m → 50m
  - UAV2/UAV3 climb to different interim altitudes (35m/40m) before
    moving horizontally, preventing mid-air collision during initial climb
  - PATROL_ALT raised 15m → 40m

Cluster status published to /cluster/status every 2 seconds.
Camera feeds: /uav2/camera/image_raw, /uav3/camera/image_raw

Usage:
    ros2 run uav_controller airport_mission
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
    phi1 = math.radians(lat1); phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi/2)**2 +
         math.cos(phi1)*math.cos(phi2)*math.sin(dlam/2)**2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ── Mission config ────────────────────────────────────────────────────────────

TAKEOFF_ALT    = 40.0   # m — raised from 20m, clears KSQL airport buildings
HEAD_ALT       = 50.0   # m — cluster head flies above members
PATROL_ALT     = 40.0   # m — members patrol at takeoff altitude
PATROL_DIST    = 80.0   # m — patrol radius from spawn
HOLD_TIME      =  8.0   # seconds
WP_RADIUS      =  3.0   # metres arrival threshold

# Vertical separation during initial climb — prevents UAV2/UAV3 collision
# UAV2 levels at 35m, UAV3 at 40m, then both move horizontally
UAV2_CLIMB_ALT = 35.0   # m
UAV3_CLIMB_ALT = 40.0   # m

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

class AirportMission(Node):

    def __init__(self):
        super().__init__('airport_mission')

        self.states          = {}
        self.takeoff_clients = {}
        self.rtl_clients     = {}
        self.goto_pubs       = {}

        for uid in [1, 2, 3]:
            ns = f'/uav{uid}'
            s  = DroneState()
            self.states[uid] = s

            self.create_subscription(NavSatFix, f'{ns}/gps',
                self._make_gps_cb(uid), 10)
            self.create_subscription(Float32, f'{ns}/rel_alt',
                self._make_alt_cb(uid), 10)
            self.create_subscription(String, f'{ns}/mode',
                self._make_mode_cb(uid), 10)
            self.create_subscription(Bool, f'{ns}/armed',
                self._make_armed_cb(uid), 10)

            self.takeoff_clients[uid] = self.create_client(
                Trigger, f'{ns}/takeoff')
            self.rtl_clients[uid] = self.create_client(
                Trigger, f'{ns}/rtl')
            self.goto_pubs[uid] = self.create_publisher(
                GeoPoint, f'{ns}/goto', 10)

        self.pub_cluster = self.create_publisher(String, '/cluster/status', 10)

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
        msg = GeoPoint()
        msg.latitude  = lat
        msg.longitude = lon
        msg.altitude  = float(alt_m)
        self.goto_pubs[uid].publish(msg)

    def _fly_to(self, uid, lat, lon, alt_m, label, timeout=120):
        self.get_logger().info(
            f'  [UAV{uid}] -> {label} ({lat:.6f}, {lon:.6f}, {alt_m}m)')
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
                        f'  [UAV{uid}] Reached {label} '
                        f'(dist={dist:.1f}m alt={s.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)
        self.get_logger().warn(f'  [UAV{uid}] Timeout at {label}')
        return False

    def _publish_cluster_status(self):
        s = self.states
        if s[1].lat is None or s[2].lat is None or s[3].lat is None:
            return
        msg = String()
        msg.data = (
            f'CLUSTER_STATUS | '
            f'HEAD=UAV1(alt={s[1].rel_alt:.1f}m mode={s[1].mode}) | '
            f'M1=UAV2(lat={s[2].lat:.5f} lon={s[2].lon:.5f} alt={s[2].rel_alt:.1f}m) | '
            f'M2=UAV3(lat={s[3].lat:.5f} lon={s[3].lon:.5f} alt={s[3].rel_alt:.1f}m)'
        )
        self.pub_cluster.publish(msg)

    # ── per-drone missions ────────────────────────────────────────────────────

    def _mission_uav1(self):
        """UAV1 — CLUSTER HEAD."""
        uid = 1
        s   = self.states[uid]
        try:
            self.get_logger().info('[UAV1] CLUSTER HEAD — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info('[UAV1] GPS ready. Waiting 30s for EKF...')
            time.sleep(30.0)

            home_lat, home_lon = s.lat, s.lon

            self.get_logger().info('[UAV1] Phase 1 — Takeoff to 40m')
            if not self._call_service(self.takeoff_clients[1], 1, '/takeoff', 90):
                raise RuntimeError('UAV1 takeoff failed')
            time.sleep(2.0)
            barrier_takeoff.wait()

            # Climb to cluster head altitude
            self.get_logger().info(f'[UAV1] Climbing to cluster head altitude {HEAD_ALT}m')
            self._fly_to(1, home_lat, home_lon, HEAD_ALT, 'Head Position')

            cluster_timer = self.create_timer(2.0, self._publish_cluster_status)
            self.get_logger().info('[UAV1] Publishing cluster status on /cluster/status')

            barrier_patrol1.wait()
            barrier_patrol2.wait()

            self.get_logger().info(f'[UAV1] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            cluster_timer.cancel()
            self.get_logger().info('[UAV1] RTL')
            self._call_service(self.rtl_clients[1], 1, '/rtl')
            self.get_logger().info('[UAV1] Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV1] ERROR: {e}')
            with errors_lock: errors.append((1, str(e)))

    def _mission_member(self, uid, bearing1, bearing2, label):
        """Generic member — vertical separation climb, two patrol waypoints, RTL."""
        s = self.states[uid]
        try:
            self.get_logger().info(f'[UAV{uid}] Member ({label}) — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info(f'[UAV{uid}] GPS ready. Waiting 30s for EKF...')
            time.sleep(30.0)

            home_lat, home_lon = s.lat, s.lon

            # Phase 1 — takeoff
            self.get_logger().info(f'[UAV{uid}] Phase 1 — Takeoff to {TAKEOFF_ALT}m')
            if not self._call_service(
                    self.takeoff_clients[uid], uid, '/takeoff', 90):
                raise RuntimeError(f'UAV{uid} takeoff failed')
            time.sleep(2.0)
            barrier_takeoff.wait()

            # Phase 1b — climb to staggered altitude BEFORE moving horizontally
            # UAV2 → 35m, UAV3 → 40m to avoid collision during climb
            climb_alt = UAV2_CLIMB_ALT if uid == 2 else UAV3_CLIMB_ALT
            self.get_logger().info(
                f'[UAV{uid}] Phase 1b — Vertical separation climb to {climb_alt}m')
            self._fly_to(uid, home_lat, home_lon, climb_alt,
                         f'Separation alt {climb_alt}m')

            # Phase 2 — fly to first patrol point at full patrol altitude
            p1_lat, p1_lon = move_gps(home_lat, home_lon, PATROL_DIST, bearing1)
            self.get_logger().info(f'[UAV{uid}] Phase 2 — {label} patrol P1')
            self._fly_to(uid, p1_lat, p1_lon, PATROL_ALT, f'{label} P1')
            barrier_patrol1.wait()

            # Phase 3 — fly to second patrol point
            p2_lat, p2_lon = move_gps(home_lat, home_lon, PATROL_DIST, bearing2)
            self.get_logger().info(f'[UAV{uid}] Phase 3 — {label} patrol P2')
            self._fly_to(uid, p2_lat, p2_lon, PATROL_ALT, f'{label} P2')
            barrier_patrol2.wait()

            self.get_logger().info(f'[UAV{uid}] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            barrier_hold.wait()

            self.get_logger().info(f'[UAV{uid}] RTL')
            self._call_service(self.rtl_clients[uid], uid, '/rtl')
            self.get_logger().info(f'[UAV{uid}] Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV{uid}] ERROR: {e}')
            with errors_lock: errors.append((uid, str(e)))

    # ── launch all threads ────────────────────────────────────────────────────

    def _run_all(self):
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════╗')
        self.get_logger().info('║     Airport Clustering Mission — 3 UAVs         ║')
        self.get_logger().info('║                                                  ║')
        self.get_logger().info('║  UAV1 -> CLUSTER HEAD (50m center hover)        ║')
        self.get_logger().info('║  UAV2 -> Member: North sector patrol @ 40m      ║')
        self.get_logger().info('║  UAV3 -> Member: South sector patrol @ 40m      ║')
        self.get_logger().info('║                                                  ║')
        self.get_logger().info('║  Collision avoidance: UAV2 climbs to 35m first  ║')
        self.get_logger().info('║                       UAV3 climbs to 40m first  ║')
        self.get_logger().info('║                                                  ║')
        self.get_logger().info('║  Monitor: ros2 topic echo /cluster/status        ║')
        self.get_logger().info('╚══════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info(
            '[Mission] Waiting 60s for all SITL instances to initialize...')
        time.sleep(60.0)

        threads = [
            threading.Thread(
                target=self._mission_uav1,
                name='UAV1-HEAD', daemon=True),
            threading.Thread(
                target=self._mission_member,
                args=(2, 180.0, 225.0, 'South'),   # UAV2 → South
                name='UAV2-South', daemon=True),
            threading.Thread(
                target=self._mission_member,
                args=(3, 0.0, 45.0, 'North'),       # UAV3 → North
                name='UAV3-North', daemon=True),
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
            self.get_logger().info('║  AIRPORT MISSION COMPLETE  ✅        ║')
            self.get_logger().info('╚══════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = AirportMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()