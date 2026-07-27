"""
city_mission.py
---------------
3-drone clustering surveillance mission in small city world.

GPS Origin: 37.3382N, -121.8863W (San Jose, CA)

Cluster roles:
  UAV1 -> CLUSTER HEAD: flies to city centre, circles for coverage,
          then hovers as relay node while members patrol
  UAV2 -> Member: flies to pond side, sweeps the water area
  UAV3 -> Member: flies to mountain side, sweeps elevated terrain

Mission phases:
  1. All 3 takeoff simultaneously from stage area
       UAV1 → 60m, UAV2 → 40m, UAV3 → 50m  (vertical separation)
  2. barrier_takeoff: all airborne confirmed
  3. UAV1 flies to city centre and begins circle patrol
     UAV2 flies to pond area
     UAV3 flies to mountain area
  4. barrier_positions: all at patrol positions
  5. UAV1 completes circle, transitions to hover (relay mode)
     UAV2 sweeps pond zone (P1 → P2)
     UAV3 sweeps mountain zone (P1 → P2)
  6. barrier_sweep: all sweeps complete
  7. All hold 10s at final positions
  8. All RTL simultaneously — UAV1 last (keeps relay active)

Collision avoidance:
  UAV1=60m, UAV2=40m, UAV3=50m — vertically separated from liftoff

Camera relay through cluster head:
  /uav1/camera/image_raw → /cluster/cam/uav1
  /uav2/camera/image_raw → /cluster/cam/uav2
  /uav3/camera/image_raw → /cluster/cam/uav3

Cluster status: /cluster/status every 2s

Usage:
    ros2 run uav_controller city_mission
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, Image
from std_msgs.msg import Float32, String, Bool
from geographic_msgs.msg import GeoPoint
from pathlib import Path
from rclpy.executors import ExternalShutdownException
import yaml

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

# ── Hardcoded GPS targets (converted from Gazebo world XYZ) ──────────────────
# Spawn GPS (from drone_bridge DDS log): 37.337570, -121.886055
# Spawn world coords: x=-56, y=-22
# Conversion: 1 degree lat ≈ 111320m, 1 degree lon ≈ 88600m at lat 37.34°

# City centre — fast_food_93 area (world x=28, y=-14)
CITY_LAT = 37.338562
CITY_LON = -121.886223

# Pond/pier area (world x=199, y=-17)
POND_LAT = 37.340176
POND_LON = -121.887016

# Mountain area (world x=209, y=-154)
MTN_LAT  = 37.340092
MTN_LON  = -121.884521

# Altitudes
HEAD_ALT     = 60.0   # m — cluster head altitude
POND_ALT     = 40.0   # m — UAV2 pond patrol altitude
MTN_ALT      = 50.0   # m — UAV3 mountain patrol altitude

# Circle patrol (UAV1 over city)
CIRCLE_RADIUS = 50.0  # m — covers city block
CIRCLE_POINTS = 8     # waypoints

# Sweep leg distance for members
SWEEP_DIST   = 50.0   # m per leg

HOLD_TIME    = 10.0   # s
WP_RADIUS    =  4.0   # m arrival threshold



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

class FleetMission(Node):

    def __init__(self):
        super().__init__('fleet_mission')

        self.declare_parameter('num_uavs', 3)
        self.declare_parameter(
            'mission_file',
            'config/mission_plans.yaml',
        )

        self.num_uavs = int(
            self.get_parameter('num_uavs').value
        )

        self.mission_file = str(
            self.get_parameter('mission_file').value
        )

        if self.num_uavs < 3 or self.num_uavs > 6:
            raise ValueError(
                'num_uavs must be between 3 and 6'
            )

        self.uav_ids = list(
            range(1, self.num_uavs + 1)
        )

        self.mission_config = self._load_mission_config(
            self.mission_file
        )

        self._validate_mission_config()

        self.barrier_takeoff = threading.Barrier(
            self.num_uavs
        )
        self.barrier_positions = threading.Barrier(
            self.num_uavs
        )
        self.barrier_sweep = threading.Barrier(
            self.num_uavs
        )
        self.barrier_hold = threading.Barrier(
            self.num_uavs
        )

        self.errors = []
        self.errors_lock = threading.Lock()

        self.states = {}
        self.takeoff_clients = {}
        self.rtl_clients = {}
        self.goto_pubs = {}
        self.pub_cam = {}
        

        for uid in self.uav_ids:
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

        # Cluster status publisher
        self.pub_cluster = self.create_publisher(String, '/cluster/status', 10)

        # Camera relay
        self.pub_cam = {
            1: self.create_publisher(Image, '/cluster/cam/uav1', 10),
            2: self.create_publisher(Image, '/cluster/cam/uav2', 10),
            3: self.create_publisher(Image, '/cluster/cam/uav3', 10),
        }
        for uid in self.uav_ids:
            ns = f'/uav{uid}'
            state = DroneState()
            self.states[uid] = state

            self.create_subscription(
                NavSatFix,
                f'{ns}/gps',
                self._make_gps_cb(uid),
                10,
            )

            self.create_subscription(
                Float32,
                f'{ns}/rel_alt',
                self._make_alt_cb(uid),
                10,
            )

            self.create_subscription(
                String,
                f'{ns}/mode',
                self._make_mode_cb(uid),
                10,
            )

            self.create_subscription(
                Bool,
                f'{ns}/armed',
                self._make_armed_cb(uid),
                10,
            )

            self.takeoff_clients[uid] = self.create_client(
                Trigger,
                f'{ns}/takeoff',
            )

            self.rtl_clients[uid] = self.create_client(
                Trigger,
                f'{ns}/rtl',
            )

            self.goto_pubs[uid] = self.create_publisher(
                GeoPoint,
                f'{ns}/goto',
                10,
            )

            self.pub_cam[uid] = self.create_publisher(
                Image,
                f'/cluster/cam/uav{uid}',
                10,
            )

            self.create_subscription(
                Image,
                f'/uav{uid}/camera/image_raw',
                self._make_cam_cb(uid),
                10,
            )
        self.get_logger().info(
            'Camera relay active: '
            '/uav{1,2,3}/camera/image_raw → /cluster/cam/uav{1,2,3}')

        threading.Thread(target=self._run_all, daemon=True).start()

    def _load_mission_config(self, mission_file):
        path = Path(mission_file).expanduser().resolve()

        if not path.is_file():
            raise FileNotFoundError(
                f'Mission file not found: {path}'
            )

        with path.open('r', encoding='utf-8') as file:
            config = yaml.safe_load(file)

        if not isinstance(config, dict):
            raise ValueError(
                'Mission configuration must be a YAML dictionary'
            )

        self.get_logger().info(
            f'Loaded mission configuration: {path}'
        )

        return config


    def _validate_mission_config(self):
        uav_plans = self.mission_config.get('uavs')

        if not isinstance(uav_plans, dict):
            raise ValueError(
                'Mission configuration has no valid "uavs" section'
            )

        for uid in self.uav_ids:
            plan = uav_plans.get(uid)

            if plan is None:
                # Handles YAML files where keys were written as strings.
                plan = uav_plans.get(str(uid))

            if not isinstance(plan, dict):
                raise ValueError(
                    f'UAV{uid} mission plan is missing'
                )

            enabled_from = int(
                plan.get('enabled_from_fleet_size', uid)
            )

            if self.num_uavs < enabled_from:
                raise ValueError(
                    f'UAV{uid} requires fleet size '
                    f'{enabled_from} or greater'
                )

            waypoints = plan.get('waypoints', [])

            if not waypoints:
                raise ValueError(
                    f'UAV{uid} has no configured waypoints'
                )

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

    def _make_cam_cb(self, uid):
        def cb(msg): self.pub_cam[uid].publish(msg)
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

    def _fly_to(self, uid, lat, lon, alt_m, label, timeout=180):
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
                        f'  [UAV{uid}] ✓ Reached {label} '
                        f'(dist={dist:.1f}m alt={s.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)
        self.get_logger().warn(f'  [UAV{uid}] Timeout reaching {label}')
        return False

    def _publish_cluster_status(self):
        if any(
            self.states[uid].lat is None
            for uid in self.uav_ids
        ):
            return

        status_parts = []

        for uid in self.uav_ids:
            state = self.states[uid]

            status_parts.append(
                f'UAV{uid}('
                f'lat={state.lat:.5f} '
                f'lon={state.lon:.5f} '
                f'alt={state.rel_alt:.1f}m '
                f'mode={state.mode})'
            )

        msg = String()
        msg.data = (
            'FLEET_STATUS | '
            + ' | '.join(status_parts)
        )

        self.pub_cluster.publish(msg)
    def _circle_patrol(self, uid, centre_lat, centre_lon,
                       radius_m, alt_m, n_points):
        """Fly a circle of n_points waypoints around centre."""
        self.get_logger().info(
            f'  [UAV{uid}] Starting city circle patrol '
            f'(r={radius_m}m, {n_points} waypoints)')
        for i in range(n_points):
            bearing = (360.0 / n_points) * i
            wp_lat, wp_lon = move_gps(
                centre_lat, centre_lon, radius_m, bearing)
            self._fly_to(uid, wp_lat, wp_lon, alt_m,
                         f'Circle WP{i+1}/{n_points} ({bearing:.0f}°)')
        self.get_logger().info(f'  [UAV{uid}] ✓ City circle complete')

    # ── per-drone missions ────────────────────────────────────────────────────

    def _mission_uav1(self):
        """
        UAV1 — CLUSTER HEAD
        Phase A: Fly to city centre
        Phase B: Circle patrol over city (8 waypoints, r=60m)
        Phase C: Hover at city centre as relay node until members done
        Phase D: RTL last (keeps relay active longest)
        """
        uid = 1
        s   = self.states[uid]
        cluster_timer = None
        try:
            self.get_logger().info('[UAV1] CLUSTER HEAD — Waiting for GPS...')
            while s.lat is None:
                time.sleep(0.3)
            self.get_logger().info('[UAV1] GPS ready. Waiting 15s for system...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            # Phase 1 — takeoff
            self.get_logger().info('[UAV1] Phase 1 — Takeoff to 60m')
            if not self._call_service(
                    self.takeoff_clients[1], 1, '/takeoff', 90):
                raise RuntimeError('UAV1 takeoff failed')
            time.sleep(1.0)
            self.self.barrier_takeoff.wait()

            # Phase 2 — fly to city centre
            self.get_logger().info('[UAV1] Phase 2 — Flying to city centre')
            self._fly_to(1, CITY_LAT, CITY_LON, HEAD_ALT, 'City Centre')

            # Start cluster status broadcast
            cluster_timer = self.create_timer(
                2.0, self._publish_cluster_status)
            self.get_logger().info(
                '[UAV1] ✓ Broadcasting /cluster/status every 2s')

            self.self.barrier_positions.wait()

            # Phase 3 — circle patrol over city
            self.get_logger().info('[UAV1] Phase 3 — City circle patrol')
            self._circle_patrol(
                1, CITY_LAT, CITY_LON,
                CIRCLE_RADIUS, HEAD_ALT, CIRCLE_POINTS)
            
            # Phase 4 — hover at city centre as relay node
            self.get_logger().info(
                '[UAV1] Phase 4 — Hovering at city centre (relay mode)')
            self._fly_to(1, CITY_LAT, CITY_LON, HEAD_ALT,
                         'City Centre Hover')

            self.self.barrier_sweep.wait()

            # Phase 5 — hold then RTL last
            self.get_logger().info(f'[UAV1] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            self.self.barrier_hold.wait()

            # Members RTL first — wait 5s then CH follows
            time.sleep(5.0)
            self.get_logger().info('[UAV1] RTL (last — relay stays active)')
            self._call_service(self.rtl_clients[1], 1, '/rtl')
            self.get_logger().info('[UAV1] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV1] ERROR: {e}')
            with self.errors_lock:
                self.self.errors.append((1, str(e)))
        finally:
            if cluster_timer:
                cluster_timer.cancel()

    def _mission_uav2(self):
        """
        UAV2 — Member, pond side
        Flies to pond area, sweeps north then south leg
        """
        uid = 2
        s   = self.states[uid]
        try:
            self.get_logger().info('[UAV2] Member (Pond) — Waiting for GPS...')
            while s.lat is None:
                time.sleep(0.3)
            self.get_logger().info('[UAV2] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            # Phase 1 — takeoff
            self.get_logger().info('[UAV2] Phase 1 — Takeoff to 40m')
            if not self._call_service(
                    self.takeoff_clients[2], 2, '/takeoff', 90):
                raise RuntimeError('UAV2 takeoff failed')
            time.sleep(1.0)
            self.self.barrier_takeoff.wait()

            # Phase 2 — fly to pond area
            self.get_logger().info('[UAV2] Phase 2 — Flying to pond area')
            self._fly_to(2, POND_LAT, POND_LON, POND_ALT, 'Pond Area')

            self.self.barrier_positions.wait()

            # Phase 3 — sweep pond zone
            # Leg 1: north edge of pond
            self.get_logger().info('[UAV2] Phase 3 — Pond sweep leg 1 (north)')
            p1_lat, p1_lon = move_gps(POND_LAT, POND_LON, SWEEP_DIST, 0.0)
            self._fly_to(2, p1_lat, p1_lon, POND_ALT, 'Pond North Edge')

            self.get_logger().info('[UAV2] Phase 3 — Pond sweep leg 2 (south)')
            p2_lat, p2_lon = move_gps(POND_LAT, POND_LON, SWEEP_DIST, 180.0)
            self._fly_to(2, p2_lat, p2_lon, POND_ALT, 'Pond South Edge')

            self._fly_to(2, POND_LAT, POND_LON, POND_ALT, 'Pond Centre')

            self.self.barrier_sweep.wait()

            self.get_logger().info(f'[UAV2] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            self.self.barrier_hold.wait()

            self.get_logger().info('[UAV2] RTL')
            self._call_service(self.rtl_clients[2], 2, '/rtl')
            self.get_logger().info('[UAV2] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV2] ERROR: {e}')
            with self.errors_lock:
                self.self.errors.append((2, str(e)))

    def _mission_uav3(self):
        """
        UAV3 — Member, mountain side
        Flies to mountain area, sweeps east then west leg
        """
        uid = 3
        s   = self.states[uid]
        try:
            self.get_logger().info(
                '[UAV3] Member (Mountain) — Waiting for GPS...')
            while s.lat is None:
                time.sleep(0.3)
            self.get_logger().info('[UAV3] GPS ready. Waiting 15s...')
            time.sleep(15.0)

            home_lat, home_lon = s.lat, s.lon

            # Phase 1 — takeoff
            self.get_logger().info('[UAV3] Phase 1 — Takeoff to 50m')
            if not self._call_service(
                    self.takeoff_clients[3], 3, '/takeoff', 90):
                raise RuntimeError('UAV3 takeoff failed')
            time.sleep(1.0)
            self.self.barrier_takeoff.wait()

            # Phase 2 — fly to mountain area
            self.get_logger().info('[UAV3] Phase 2 — Flying to mountain area')
            self._fly_to(3, MTN_LAT, MTN_LON, MTN_ALT, 'Mountain Area')

            self.self.barrier_positions.wait()

            # Phase 3 — sweep mountain zone
            # Leg 1: further into mountain
            self.get_logger().info(
                '[UAV3] Phase 3 — Mountain sweep leg 1 (deeper)')
            p1_lat, p1_lon = move_gps(MTN_LAT, MTN_LON, SWEEP_DIST, 60.0)
            self._fly_to(3, p1_lat, p1_lon, MTN_ALT, 'Mountain Deep')

            p2_lat, p2_lon = move_gps(MTN_LAT, MTN_LON, SWEEP_DIST, 120.0)
            self._fly_to(3, p2_lat, p2_lon, MTN_ALT, 'Mountain Ridge')

            self._fly_to(3, MTN_LAT, MTN_LON, MTN_ALT, 'Mountain Entry')

            self.self.barrier_sweep.wait()

            self.get_logger().info(f'[UAV3] Holding {HOLD_TIME}s')
            time.sleep(HOLD_TIME)
            self.self.barrier_hold.wait()

            self.get_logger().info('[UAV3] RTL')
            self._call_service(self.rtl_clients[3], 3, '/rtl')
            self.get_logger().info('[UAV3] ✅ Mission complete')

        except Exception as e:
            self.get_logger().error(f'[UAV3] ERROR: {e}')
            with self.errors_lock:
                self.self.errors.append((3, str(e)))

    # ── launch all threads ────────────────────────────────────────────────────

    def _run_all(self):
        self.get_logger().info('')
        self.get_logger().info(
            '╔══════════════════════════════════════════════════════╗')
        self.get_logger().info(
            '║        City Surveillance Mission — 3 UAVs           ║')
        self.get_logger().info(
            '║                                                      ║')
        self.get_logger().info(
            '║  UAV1 → CLUSTER HEAD                                 ║')
        self.get_logger().info(
            '║          city circle (r=60m) then hover @ 60m       ║')
        self.get_logger().info(
            '║  UAV2 → Member: pond side sweep          @ 40m      ║')
        self.get_logger().info(
            '║  UAV3 → Member: mountain side sweep      @ 50m      ║')
        self.get_logger().info(
            '║                                                      ║')
        self.get_logger().info(
            '║  Monitor cluster:                                    ║')
        self.get_logger().info(
            '║    ros2 topic echo /cluster/status                   ║')
        self.get_logger().info(
            '║  Monitor cameras:                                    ║')
        self.get_logger().info(
            '║    ros2 run image_view image_view                    ║')
        self.get_logger().info(
            '║      --ros-args -r image:=/cluster/cam/uav2         ║')
        self.get_logger().info(
            '╚══════════════════════════════════════════════════════╝')
        self.get_logger().info('')
        self.get_logger().info(
            '[Mission] Waiting 15s for ROS2 subscriptions...')
        time.sleep(15.0)

        threads = [
            threading.Thread(
                target=self._mission_uav1,
                name='UAV1-HEAD', daemon=True),
            threading.Thread(
                target=self._mission_uav2,
                name='UAV2-Pond', daemon=True),
            threading.Thread(
                target=self._mission_uav3,
                name='UAV3-Mountain', daemon=True),
        ]

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.get_logger().info('')
        if self.errors:
            self.get_logger().error(
                '╔══════════════════════════════════════╗')
            self.get_logger().error(
                '║  MISSION FINISHED WITH ERRORS        ║')
            for uid, err in self.errors:
                self.get_logger().error(f'║  UAV{uid}: {err}')
            self.get_logger().error(
                '╚══════════════════════════════════════╝')
        else:
            self.get_logger().info(
                '╔══════════════════════════════════════╗')
            self.get_logger().info(
                '║   CITY MISSION COMPLETE  ✅          ║')
            self.get_logger().info(
                '╚══════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = FleetMission()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
