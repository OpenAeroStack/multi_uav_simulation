"""
multi_mission.py
----------------
Barrier-synchronised 3-drone mission via ROS2.
Mirrors the logic of multi_drone_mission.py but uses ROS2 + drone_bridge.

Mission:
    1. All 3 connect and wait for GPS
    2. All 3 takeoff to 20m              ← barrier sync
    3. UAV1 → Left 50m
       UAV2 → Right 50m
       UAV3 → Forward 50m               ← barrier sync
    4. All 3 hold 5 seconds              ← barrier sync
    5. All 3 RTL simultaneously

No collision — each drone flies a different direction from its own position.

Requires 3 drone_bridge nodes (all at same altitude since directions diverge):
    ros2 run uav_controller drone_bridge --ros-args -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=20.0
    ros2 run uav_controller drone_bridge --ros-args -p uav_id:=2 -p mavlink_port:=5770 -p takeoff_altitude:=20.0
    ros2 run uav_controller drone_bridge --ros-args -p uav_id:=3 -p mavlink_port:=5780 -p takeoff_altitude:=20.0

Usage:
    ros2 run uav_controller multi_mission
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


# ── Mission config ────────────────────────────────────────────────────────────
# Each drone flies a different direction — no collision possible

DRONES = [
    {'uav_id': 1, 'mavlink_port': 5760, 'direction': 'left'},
    {'uav_id': 2, 'mavlink_port': 5770, 'direction': 'right'},
    {'uav_id': 3, 'mavlink_port': 5780, 'direction': 'forward'},
]

TAKEOFF_ALT    = 20.0   # metres — all same, safe since they diverge immediately
MOVE_DISTANCE  = 50.0   # metres
HOLD_TIME      = 5.0    # seconds
WP_RADIUS      = 2.0    # metres arrival threshold

N_DRONES = len(DRONES)

# Barriers — same pattern as original multi_drone_mission.py
barrier_takeoff  = threading.Barrier(N_DRONES)
barrier_waypoint = threading.Barrier(N_DRONES)
barrier_hold     = threading.Barrier(N_DRONES)

errors      = []
errors_lock = threading.Lock()


# ── GPS helpers ───────────────────────────────────────────────────────────────

def offset_gps(lat, lon, distance_m, direction):
    """
    Returns new (lat, lon) offset by distance_m in given direction.
    Uses same formula as original multi_drone_mission.py.
    forward  = North
    backward = South
    right    = East
    left     = West
    """
    R = 6_371_000.0
    if direction == 'forward':
        dlat = (distance_m / R) * (180 / math.pi)
        dlon = 0.0
    elif direction == 'backward':
        dlat = -(distance_m / R) * (180 / math.pi)
        dlon = 0.0
    elif direction == 'right':
        dlat = 0.0
        dlon = (distance_m / R) * (180 / math.pi) / math.cos(math.radians(lat))
    elif direction == 'left':
        dlat = 0.0
        dlon = -(distance_m / R) * (180 / math.pi) / math.cos(math.radians(lat))
    else:
        raise ValueError(f'Unknown direction: {direction}')
    return lat + dlat, lon + dlon


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

class MultiMission(Node):

    def __init__(self):
        super().__init__('multi_mission')

        self.states          = {}
        self.takeoff_clients = {}
        self.rtl_clients     = {}
        self.goto_pubs       = {}

        for cfg in DRONES:
            uid = cfg['uav_id']
            ns  = f'/uav{uid}'
            s   = DroneState()
            self.states[uid] = s

            # Subscribers
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

            # Service clients
            self.takeoff_clients[uid] = self.create_client(
                Trigger, f'{ns}/takeoff')
            self.rtl_clients[uid] = self.create_client(
                Trigger, f'{ns}/rtl')

            # Goto publisher
            self.goto_pubs[uid] = self.create_publisher(
                GeoPoint, f'{ns}/goto', 10)

        threading.Thread(target=self._run_all, daemon=True).start()

    # ── callback factories ────────────────────────────────────────────────────

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
                f'  [UAV{uid}] Service {label} not available '
                f'— is drone_bridge running?')
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            self.get_logger().error(f'  [UAV{uid}] {label} — no response')
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

    def _fly_to(self, uid, lat, lon, alt_m, label, timeout=90):
        self.get_logger().info(
            f'  [UAV{uid}] → Flying to {label} '
            f'({lat:.6f}, {lon:.6f}, alt={alt_m}m)')
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
                if dist < WP_RADIUS and alt_err < 2.0:
                    self.get_logger().info(
                        f'  [UAV{uid}] ✓ Reached {label} '
                        f'(dist={dist:.1f}m  alt={s.rel_alt:.1f}m)')
                    return True
            time.sleep(0.5)
        self.get_logger().warn(
            f'  [UAV{uid}] ⚠ Timeout at {label}')
        return False

    # ── per-drone mission thread ──────────────────────────────────────────────

    def _drone_mission(self, cfg):
        uid       = cfg['uav_id']
        direction = cfg['direction']
        s         = self.states[uid]

        try:
            # ── Phase 0: Wait for GPS ─────────────────────────────────────────
            self.get_logger().info(f'[UAV{uid}] Waiting for GPS...')
            while s.lat is None:
                time.sleep(0.3)
            self.get_logger().info(
                f'[UAV{uid}] GPS ready ({s.lat:.6f}, {s.lon:.6f})')

            self.get_logger().info(f'[UAV{uid}] Waiting 15s for EKF...')
            time.sleep(15.0)

            # ── Phase 1: Takeoff ──────────────────────────────────────────────
            self.get_logger().info(
                f'[UAV{uid}] Phase 1 — Takeoff to {TAKEOFF_ALT}m')
            if not self._call_service(
                    self.takeoff_clients[uid], uid, '/takeoff', timeout=90.0):
                raise RuntimeError(f'UAV{uid} takeoff failed')
            time.sleep(2.0)

            self.get_logger().info(
                f'[UAV{uid}] ⏳ Waiting at takeoff barrier...')
            barrier_takeoff.wait()
            self.get_logger().info(f'[UAV{uid}] ✓ Takeoff barrier — all airborne')

            # ── Phase 2: Move in assigned direction ───────────────────────────
            # Compute target from current position (each drone's own home)
            tgt_lat, tgt_lon = offset_gps(
                s.lat, s.lon, MOVE_DISTANCE, direction)

            self.get_logger().info(
                f'[UAV{uid}] Phase 2 — Move {direction} {MOVE_DISTANCE}m')
            self._fly_to(uid, tgt_lat, tgt_lon, TAKEOFF_ALT,
                         f'WP ({direction})')

            self.get_logger().info(
                f'[UAV{uid}] ⏳ Waiting at waypoint barrier...')
            barrier_waypoint.wait()
            self.get_logger().info(
                f'[UAV{uid}] ✓ Waypoint barrier — all at waypoints')

            # ── Phase 3: Hold ─────────────────────────────────────────────────
            self.get_logger().info(
                f'[UAV{uid}] Phase 3 — Holding {HOLD_TIME}s...')
            time.sleep(HOLD_TIME)

            self.get_logger().info(
                f'[UAV{uid}] ⏳ Waiting at hold barrier...')
            barrier_hold.wait()
            self.get_logger().info(
                f'[UAV{uid}] ✓ Hold barrier — all RTL together')

            # ── Phase 4: RTL ──────────────────────────────────────────────────
            self.get_logger().info(f'[UAV{uid}] Phase 4 — RTL')
            self._call_service(self.rtl_clients[uid], uid, '/rtl')

            self.get_logger().info(f'[UAV{uid}] ✅ Mission complete!')

        except Exception as e:
            self.get_logger().error(f'[UAV{uid}] ERROR: {e}')
            with errors_lock:
                errors.append((uid, str(e)))

    # ── launch all threads ────────────────────────────────────────────────────

    def _run_all(self):
        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════╗')
        self.get_logger().info('║     MULTI-DRONE MISSION START (3 UAVs)  ║')
        self.get_logger().info('║  UAV1: Left   50m                       ║')
        self.get_logger().info('║  UAV2: Right  50m                       ║')
        self.get_logger().info('║  UAV3: Forward 50m                      ║')
        self.get_logger().info('╚══════════════════════════════════════════╝')
        self.get_logger().info('')

        threads = []
        for cfg in DRONES:
            t = threading.Thread(
                target=self._drone_mission,
                args=(cfg,),
                name=f'UAV{cfg["uav_id"]}',
                daemon=True)
            threads.append(t)

        for t in threads:
            t.start()
        for t in threads:
            t.join()

        self.get_logger().info('')
        if errors:
            self.get_logger().error('╔══════════════════════════════════════╗')
            self.get_logger().error('║   MISSION FINISHED WITH ERRORS       ║')
            for uid, err in errors:
                self.get_logger().error(f'║   UAV{uid}: {err}')
            self.get_logger().error('╚══════════════════════════════════════╝')
        else:
            self.get_logger().info('╔══════════════════════════════════════╗')
            self.get_logger().info('║   ALL DRONES COMPLETED MISSION  ✅   ║')
            self.get_logger().info('╚══════════════════════════════════════╝')


def main(args=None):
    rclpy.init(args=args)
    node = MultiMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()