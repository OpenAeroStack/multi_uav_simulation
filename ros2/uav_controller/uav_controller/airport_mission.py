"""
airport_mission.py
------------------
3-drone clustering surveillance mission at KSQL Airport.

GPS Origin: 37.523640N, -122.255122E (San Carlos Airport, CA)

Cluster roles:
  UAV1 -> CLUSTER HEAD: hovers at 50m center, aggregates member reports
  UAV2 -> Member: patrols South sector, camera active
  UAV3 -> Member: patrols North sector, camera active

Mission phases:
  1. All 3 takeoff simultaneously
       UAV1 → 40m, UAV2 → 35m, UAV3 → 40m   <- vertical separation from ground
       (takeoff altitudes set in launch script per bridge param)
  2. barrier_takeoff: all airborne
     UAV1 climbs to 50m (head position)
     UAV2 flies South 150m @ 40m             <- barrier_patrol1
     UAV3 flies North 150m @ 40m
  3. UAV2 flies SW 150m @ 40m               <- barrier_patrol2
     UAV3 flies NE 150m @ 40m
  4. All hold 5 seconds                      <- barrier_hold
  5. All RTL simultaneously

Collision avoidance:
  UAV2 takeoff altitude = 35m  (set via drone_bridge param in launch script)
  UAV3 takeoff altitude = 40m  (set via drone_bridge param in launch script)
  They are vertically separated from liftoff — no mid-air crossing.

Cluster status published to /cluster/status every 2 seconds.
Camera feeds: /uav2/camera/image_raw, /uav3/camera/image_raw

Launch script drone_bridge params required:
  UAV1: takeoff_altitude:=40.0
  UAV2: takeoff_altitude:=35.0   ← must be lower than UAV3
  UAV3: takeoff_altitude:=40.0

Usage:
    Run AFTER all 3 bridges show: ✓ DDS GPS flowing
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

HEAD_ALT       = 50.0   # m — cluster head altitude
PATROL_ALT     = 40.0   # m — members patrol altitude
PATROL_DIST    = 150.0  # m — patrol radius from spawn (increased from 80m)
HOLD_TIME      =  5.0   # seconds — hold at final waypoint before RTL
WP_RADIUS      =  3.0   # metres arrival threshold

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
        self.snr     = None


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
            self.create_subscription(Float32, f'{ns}/snr',
                self._make_snr_cb(uid), 10)
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
    def _make_snr_cb(self, uid):
        def cb(msg): self.states[uid].snr = msg.data
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
        """UAV1 — CLUSTER HEAD. Hovers at 50m, broadcasts cluster status."""
        uid = 1
        s   = self.states[uid]
        try:
            self.get_logger().info('[UAV1] CLUSTER HEAD — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info('[UAV1] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            self.get_logger().info('[UAV1] Phase 1 — Takeoff to 40m')
            if not self._call_service(self.takeoff_clients[1], 1, '/takeoff', 90):
                raise RuntimeError('UAV1 takeoff failed')
            time.sleep(1.0)
            barrier_takeoff.wait()

            # Climb to cluster head altitude and hover
            self.get_logger().info(f'[UAV1] Climbing to {HEAD_ALT}m cluster head position')
            self._fly_to(1, home_lat, home_lon, HEAD_ALT, 'Head Position')

            # Start broadcasting cluster status
            cluster_timer = self.create_timer(2.0, self._publish_cluster_status)
            self.get_logger().info('[UAV1] ✓ Broadcasting /cluster/status every 2s')

            # Wait at barriers while members patrol
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
        Member drone — takeoff (altitude set by bridge param), patrol two
        waypoints, RTL.

        Collision avoidance is handled by drone_bridge takeoff_altitude param:
          UAV2: takeoff_altitude=35.0  → lands at 35m after takeoff
          UAV3: takeoff_altitude=40.0  → lands at 40m after takeoff
        They are vertically separated from liftoff with no crossing.
        """
        s = self.states[uid]
        try:
            self.get_logger().info(f'[UAV{uid}] Member ({label}) — Waiting for GPS...')
            while s.lat is None: time.sleep(0.3)
            self.get_logger().info(f'[UAV{uid}] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            # Phase 1 — takeoff to altitude defined in launch script bridge param
            # UAV2 → 35m, UAV3 → 40m, already separated vertically
            self.get_logger().info(f'[UAV{uid}] Phase 1 — Takeoff')
            if not self._call_service(
                    self.takeoff_clients[uid], uid, '/takeoff', 90):
                raise RuntimeError(f'UAV{uid} takeoff failed')
            time.sleep(1.0)
            barrier_takeoff.wait()
            # UAV2 is at 35m, UAV3 is at 40m — 5m vertical gap, safe to move

            # Phase 2 — fly to first patrol point at PATROL_ALT
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
            self.get_logger().info(f'[UAV{uid}] ✅ Mission complete')

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
        self.get_logger().info('║  UAV2 -> Member: South sector patrol @ 40m      ║')
        self.get_logger().info('║  UAV3 -> Member: North sector patrol @ 40m      ║')
        self.get_logger().info('║                                                  ║')
        self.get_logger().info('║  Collision avoidance via takeoff altitude:       ║')
        self.get_logger().info('║    UAV2 takeoff=35m, UAV3 takeoff=40m           ║')
        self.get_logger().info('║    (set in launch script bridge params)          ║')
        self.get_logger().info('║                                                  ║')
        self.get_logger().info('║  Monitor: ros2 topic echo /cluster/status        ║')
        self.get_logger().info('╚══════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info(
            '[Mission] Waiting 15s for ROS2 subscriptions to connect...')
        time.sleep(15.0)

        threads = [
            threading.Thread(
                target=self._mission_uav1,
                name='UAV1-HEAD', daemon=True),
            threading.Thread(
                target=self._mission_member,
                args=(2, 180.0, 225.0, 'South'),   # UAV2 → South then SW
                name='UAV2-South', daemon=True),
            threading.Thread(
                target=self._mission_member,
                args=(3, 0.0, 45.0, 'North'),       # UAV3 → North then NE
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
            self.get_logger().info('║  AIRPORT MISSION COMPLETE            ║')
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