#!/usr/bin/env python3
"""
two_drone_mission.py
────────────────────
Flies 2 UAVs through a shared waypoint route, synchronised at phase barriers.

Talks to drone_bridge over ROS 2, so every command and every position report
crosses the ns-3 simulated radio:

  UAV1  /uav1/{arm,takeoff,rtl}  /uav1/goto  /uav1/gps  /uav1/rel_alt
  UAV2  /uav2/{arm,takeoff,rtl}  /uav2/goto  /uav2/gps  /uav2/rel_alt

Mission flow:
  1. Both wait for a GPS fix, then settle              ← barrier_ready
  2. Both arm and take off to their altitude           ← barrier_takeoff
  3. Both fly their waypoint route                     
  4. Both hold on station                              
  5. Both RTL together                                 ← barrier_landing

Waypoints are GAZEBO world coordinates, read straight from the .world file.
Altitudes differ per aircraft: that vertical separation is the only collision
avoidance, exactly as in city_mission.py.

Run INSIDE the gcsns namespace — drone_bridge lives there and its services are
invisible from the root namespace.

    python3 two_drone_mission.py
"""

import math
import os
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint


# ─── Configuration ──────────────────────────────────────────────────────────
# Waypoints are Gazebo (x, y). road_x_1 runs east-west at y = -45, x from
# -41.3 to 116.3; person_center sits at (40.5, -187.9).
DRONES = [
    {
        "name":     "UAV1",              # this mission detects the stable humans
        "uav_id":   1,
        "spawn":    (-70.0, -22.0),      # iris_1_demo <pose> in the world file
        "altitude": 30.0,
        "waypoints": [
            (-70.0, -120.0),             # south along the approach
            ( 10.0, -160.0),             # turn toward the subjects
            ( 10.0, -160.0),             # hold point, 30 m short of them
        ],
    },
    {
        "name":     "UAV2",              # this mission detects the walking humans
        "uav_id":   2,
        "spawn":    (-70.0, -32.0),      # iris_2_demo <pose>
        "altitude": 20.0,                # 
        "hold_at_waypoint": 15.0,        # hold at the sec15ond waypoint
        "waypoints": [
            (-30.0, -45.0),              # west end of road_x_1
            ( 30.0, -45.0),              # east end
            (-30.0, -45.0),              # back west
        ],
    },
]

SETTLE_TIME     = 15.0    # s — let DDS discovery finish before arming
HOLD_TIME       = 20.0    # s — on station before RTL
WAYPOINT_RADIUS = 5.0     # m — how close counts as "arrived"
CLIMB_TOLERANCE = 0.9     # fraction of target altitude that counts as reached
CLIMB_TOLERANCE_MIN_M = 2.0   # floor on that band, so low targets stay reachable

GPS_TIMEOUT     = 30.0    # s — waiting for the first fix
GPS_STALE       = 20.0    # s — a frozen feed is not a healthy one
SERVICE_TIMEOUT = 120.0   # s — arming legitimately retries this long
CLIMB_TIMEOUT   = 60.0    # s
WAYPOINT_TIMEOUT = 300.0  # s — 240 m at 1 m/s needs ~240 s

METRES_PER_DEG_LAT = 111320.0
EARTH_RADIUS_M     = 6371000.0

# Overrides for automated sweeps, so a run does not need this file edited:
#   DRONES_ONLY=UAV1  UAV1_ALTITUDE=15  python3 two_drone_mission.py
_only = os.environ.get("DRONES_ONLY", "").strip()
if _only:
    _keep = {n.strip() for n in _only.split(",") if n.strip()}
    DRONES = [d for d in DRONES if d["name"] in _keep]
    if not DRONES:
        raise SystemExit(f"DRONES_ONLY={_only!r} matched no aircraft")
for _d in DRONES:
    _alt = os.environ.get(f"{_d['name']}_ALTITUDE", "").strip()
    if _alt:
        _d["altitude"] = float(_alt)

# Barriers: every drone blocks here until all of them arrive.
N_DRONES = len(DRONES)
barrier_ready   = threading.Barrier(N_DRONES)       # both have GPS before either arms
barrier_takeoff = threading.Barrier(N_DRONES)       # boht at altitude before either flies the route
barrier_landing    = threading.Barrier(N_DRONES)    # both drone before RT
 

errors      = []
errors_lock = threading.Lock()


class MissionAborted(RuntimeError):
    """A step did not complete, so anything measured afterwards is worthless."""


# ─── Geometry ───────────────────────────────────────────────────────────────
def move_from(lat, lon, east_m, north_m):
    """Return the point east_m east and north_m north of (lat, lon)."""
    dlat = north_m / METRES_PER_DEG_LAT
    dlon = east_m / (METRES_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two lat/lon points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def gazebo_to_offset(spawn, gx, gy):
    """Gazebo (x, y) -> (east_m, north_m) relative to this drone's spawn.

    NOT plain ENU: the ArduPilot plugin rotates pi about X, so the autopilot
    reports north = gazebo_x and east = -gazebo_y. See docs/COORDINATE_FRAMES.pdf.
    """
    return -(gy - spawn[1]), gx - spawn[0]


# ─── One aircraft ───────────────────────────────────────────────────────────
class Drone:
    """Wraps one drone_bridge instance plus all helper methods."""

    def __init__(self, node: Node, cfg: dict):
        self.node      = node
        self.name      = cfg["name"]
        self.uav_id    = cfg["uav_id"]
        self.spawn     = cfg["spawn"]
        self.altitude  = cfg["altitude"]
        self.waypoints = cfg["waypoints"]
        self.wp_hold   = cfg.get("hold_at_waypoints", 0.0)
        self.ns        = f"/uav{self.uav_id}"

        self.lat = None
        self.lon = None
        self.rel_alt = 0.0
        self.last_gps_t = 0.0

        # Default QoS (RELIABLE) matches drone_bridge's publishers.
        node.create_subscription(NavSatFix, f"{self.ns}/gps", self._on_gps, 10)
        node.create_subscription(Float32, f"{self.ns}/rel_alt", self._on_alt, 10)
        self.pub_goto = node.create_publisher(GeoPoint, f"{self.ns}/goto", 10)

        self.arm_cli     = node.create_client(Trigger, f"{self.ns}/arm")
        self.takeoff_cli = node.create_client(Trigger, f"{self.ns}/takeoff")
        self.rtl_cli     = node.create_client(Trigger, f"{self.ns}/rtl")

    # ── logging ─────────────────────────────────────────────────────────────
    def log(self, msg):
        self.node.get_logger().info(f"[{self.name}] {msg}")

    def warn(self, msg):
        self.node.get_logger().warn(f"[{self.name}] {msg}")

    # ── callbacks ───────────────────────────────────────────────────────────
    def _on_gps(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude
        self.last_gps_t = time.time()

    def _on_alt(self, msg):
        self.rel_alt = msg.data

    # ── health ──────────────────────────────────────────────────────────────
    def wait_for_gps(self):
        """Block until the first fix, bounded so a dead feed fails loudly."""
        deadline = time.time() + GPS_TIMEOUT
        while rclpy.ok() and self.lat is None:
            if time.time() >= deadline:
                raise MissionAborted(
                    f"no GPS on {self.ns}/gps after {GPS_TIMEOUT:.0f} s")
            time.sleep(0.1)

    def check_gps_fresh(self):
        """Raise if the feed froze. A stale fix reads plausible but is not."""
        age = time.time() - self.last_gps_t
        if self.last_gps_t > 0.0 and age > GPS_STALE:
            raise MissionAborted(
                f"position feed stale: no {self.ns}/gps for {age:.0f} s")

    # ── commands ────────────────────────────────────────────────────────────
    def call_service(self, client, name):
        """Call a Trigger service. Raise MissionAborted if it reports failure."""
        self.log(f"calling {name}")
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            raise MissionAborted(f"{name} never appeared")

        future = client.call_async(Trigger.Request())
        # Wait on the future; main() spins the node, so the reply arrives there.
        deadline = time.time() + SERVICE_TIMEOUT
        while future.result() is None and time.time() < deadline:
            time.sleep(0.05)

        result = future.result()
        if result is None:
            raise MissionAborted(f"{name} did not answer in {SERVICE_TIMEOUT:.0f} s")
        self.log(f"  {name} -> {result.message}")
        if not result.success:
            raise MissionAborted(f"{name} failed: {result.message}")

    def goto(self, lat, lon, alt):
        msg = GeoPoint()
        msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
        self.pub_goto.publish(msg)

    # ── phases ──────────────────────────────────────────────────────────────
    def climb_to(self, altitude):
        """Command the current position at a new altitude and wait for it."""
        self.goto(self.lat, self.lon, altitude)
        self.log(f"climbing to {altitude:.0f} m")

        # A band, not a floor: the aircraft takes off to takeoff_altitude
        # (30 m), so any lower target is reached by DESCENDING. A one-sided
        # ">= 0.9 * target" test passes instantly at 30 m and never checks it.
        tol = max(altitude * (1.0 - CLIMB_TOLERANCE), CLIMB_TOLERANCE_MIN_M)
        deadline = time.time() + CLIMB_TIMEOUT
        while rclpy.ok() and time.time() < deadline:
            time.sleep(0.1)
            self.check_gps_fresh()
            if abs(self.rel_alt - altitude) <= tol:
                self.log(f"  reached {self.rel_alt:.1f} m (target {altitude:.0f})")
                return
        raise MissionAborted(
            f"altitude timeout: {self.rel_alt:.1f} m vs target {altitude:.0f} m "
            f"after {CLIMB_TIMEOUT:.0f} s")

    def fly_to(self, lat, lon, label):
        """Fly to a waypoint and wait until it is actually reached."""
        self.goto(lat, lon, self.altitude)
        self.log(f"{label} -> ({lat:.6f}, {lon:.6f})")

        deadline  = time.time() + WAYPOINT_TIMEOUT
        last_send = time.time()
        while rclpy.ok() and time.time() < deadline:
            time.sleep(0.1)
            # Re-send: one dropped goto strands the aircraft for the whole timeout.
            if time.time() - last_send > 2.0:
                self.goto(lat, lon, self.altitude)
                last_send = time.time()

            self.check_gps_fresh()
            if self.lat is None:
                continue
            remaining = distance_m(self.lat, self.lon, lat, lon)
            if remaining <= WAYPOINT_RADIUS:
                self.log(f"  arrived, {remaining:.1f} m off")
                return
        raise MissionAborted(
            f"{label}: not reached within {WAYPOINT_TIMEOUT:.0f} s")

    def hold(self, seconds):
        self.log(f"holding {seconds:.0f} s on station")
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            time.sleep(0.2)
            self.check_gps_fresh()


# ─── Per-drone mission thread ───────────────────────────────────────────────
def run_mission(node, cfg):
    """Full mission for one drone. Errors are recorded for the main thread."""
    drone = Drone(node, cfg)

    try:
        # ── Phase 0: wait for telemetry ─────────────────────────────────────
        drone.log("waiting for the first GPS fix ...")
        drone.wait_for_gps()
        drone.log(f"GPS ready ({drone.lat:.6f}, {drone.lon:.6f}); "
                  f"settling {SETTLE_TIME:.0f} s")
        time.sleep(SETTLE_TIME)
        barrier_ready.wait()

        # Measured from HERE: the position the drone itself reported.
        home_lat, home_lon = drone.lat, drone.lon

        # ── Phase 1: arm and take off ───────────────────────────────────────
        drone.call_service(drone.arm_cli, f"{drone.ns}/arm")
        drone.call_service(drone.takeoff_cli, f"{drone.ns}/takeoff")
        drone.climb_to(drone.altitude)

        drone.log("airborne — waiting at the takeoff barrier")
        barrier_takeoff.wait()

        # ── Phase 2: fly the route ──────────────────────────────────────────
        for n, (gx, gy) in enumerate(drone.waypoints, start=1):
            east, north = gazebo_to_offset(drone.spawn, gx, gy)
            lat, lon = move_from(home_lat, home_lon, east, north)
            drone.log(f"wp{n}: gazebo ({gx:.1f}, {gy:.1f}) "
                      f"= {east:+.0f} m E, {north:+.0f} m N from take-off")
            drone.fly_to(lat, lon, f"wp{n}")
            drone.log(f"wp{n} complete")


        # ── Phase 3: hold together ──────────────────────────────────────────
        drone.hold(HOLD_TIME)
        drone.log("hold complete — waiting for the other aircraft before RTL")
        barrier_landing.wait()


        # ── Phase 4: RTL ────────────────────────────────────────────────────
        drone.call_service(drone.rtl_cli, f"{drone.ns}/rtl")
        drone.log("mission complete")

    except MissionAborted as exc:
        drone.warn(f"MISSION ABORTED: {exc}")
        with errors_lock:
            errors.append((drone.name, str(exc)))
        # Break every barrier: the other aircraft must not wait forever.
        for b in (barrier_ready, barrier_takeoff, barrier_landing):
            b.abort()

        try:
            drone.call_service(drone.rtl_cli, f"{drone.ns}/rtl")
        except Exception as rtl_exc:                       # noqa: BLE001
            drone.warn(f"RTL failed: {rtl_exc}")

    except threading.BrokenBarrierError:
        drone.warn("another aircraft aborted — returning to launch")
        with errors_lock:
            errors.append((drone.name, "aborted with the fleet"))
        try:
            drone.call_service(drone.rtl_cli, f"{drone.ns}/rtl")
        except Exception as rtl_exc:                       # noqa: BLE001
            drone.warn(f"RTL failed: {rtl_exc}")


# ─── Main ───────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = rclpy.create_node("two_drone_mission")

    # One executor, node added ONCE. rclpy.spin_once(node) re-adds and removes
    # the node every call, dropping subscriptions out of the wait set.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info("=" * 58)
    node.get_logger().info(f"  TWO-DRONE MISSION — {N_DRONES} UAVs")
    for cfg in DRONES:
        node.get_logger().info(
            f"    {cfg['name']}: {len(cfg['waypoints'])} waypoints "
            f"@ {cfg['altitude']:.0f} m")
    node.get_logger().info("=" * 58)

    threads = [threading.Thread(target=run_mission, args=(node, cfg),
                                name=cfg["name"], daemon=True)
               for cfg in DRONES]
    for t in threads:
        t.start()

    try:
        while rclpy.ok() and any(t.is_alive() for t in threads):
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        node.get_logger().info("=" * 58)
        if errors:
            node.get_logger().error("  MISSION FINISHED WITH ERRORS")
            for name, err in errors:
                node.get_logger().error(f"    {name}: {err}")
        else:
            node.get_logger().info("  ALL DRONES COMPLETED THE MISSION")
        node.get_logger().info("=" * 58)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
