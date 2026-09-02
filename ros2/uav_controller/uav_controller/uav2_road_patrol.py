#!/usr/bin/env python3
"""Fly one UAV along a list of waypoints given in GAZEBO world coordinates.

    arm -> take off -> climb -> waypoint 1 .. N -> RTL

WHY THIS IS A SEPARATE FILE FROM uav1_patrol_mission.py
    Not because it is a different aircraft -- that one already takes uav_id as
    a parameter and will fly any of them. It is separate because the SEQUENCE
    differs: this walks a LIST of waypoints, while the survey mission holds one
    station at a standoff offset. A list does not fit ROS 2 parameters cleanly,
    so it lives in the file instead.

WHY WAYPOINTS ARE IN GAZEBO COORDINATES
    You read road and building positions out of the .world file, and those are
    Gazebo x/y. Converting them to lat/lon by hand is where mistakes happen.
    Type the Gazebo numbers here and let the script convert.

    The conversion is NOT plain ENU. The ArduPilot plugin applies
    <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED> -- a pi rotation about
    X -- so what the autopilot reports is:

        north =  gazebo_x        east = -gazebo_y

    That mapping is copied from uav1_patrol_mission.py, where it was verified
    against a live vehicle. Do not "fix" it to plain ENU: that sends the drone
    90 degrees off, which is what happened on every flight before 2026-08-30.

    Everything is measured RELATIVE to the position the drone itself reports
    after take-off, so no assumption about the world's GPS origin can be wrong.

Run it INSIDE the gcsns namespace -- drone_bridge lives there, and from the
root namespace the services below do not exist and this script waits forever.

    python3 uav2_road_patrol.py --ros-args -p uav_id:=2
"""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
from geographic_msgs.msg import GeoPoint


# ─── EDIT ME: patrol path in GAZEBO world coordinates ───────────────────────
# road_x_1: y = -45, x from -41.3 to 116.3, 8.4 m wide. Stay inside that range.
WAYPOINTS_GAZEBO = [
    (-30.0, -45.0),      # west end of the patrolled stretch
    ( 30.0, -45.0),      # east end
    (-30.0, -45.0),      # and back
]

# Must match iris_2_demo's <pose> in the world file: waypoints above are
# absolute Gazebo coords and the spawn is what ties them to the drone's GPS.
SPAWN_GAZEBO = (-70.0, -32.0)

DEFAULTS = {
    "uav_id":             2,
    "altitude_m":         30.0,
    "waypoint_timeout_s": 300.0,
    "climb_timeout_s":    60.0,
    "gps_timeout_s":      30.0,
    # 20 s catches a DEAD feed; 5 s tripped on survivable pauses.
    "gps_stale_s":        20.0,         # a FROZEN feed is not a healthy one
    "service_timeout_s":  120.0,        # arm can legitimately retry for ~90 s
    "settle_s":           15.0,         # DDS discovery headroom before arming
    "hold_seconds":       0.0,      # pause at each waypoint; 0 = fly straight through
}

EARTH_RADIUS_M     = 6371000.0
METRES_PER_DEG_LAT = 111320.0
ARRIVAL_RADIUS_M   = 5.0


class MissionAborted(RuntimeError):
    """Raised when a step cannot be completed. main() still flies the RTL."""


# ─── geometry helpers (identical to uav1_patrol_mission.py) ──────────────────
def move_from(lat, lon, east_m, north_m):
    """Return the point east_m east and north_m north of (lat, lon)."""
    dlat = north_m / METRES_PER_DEG_LAT
    dlon = east_m / (METRES_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance in metres between two lat/lon points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def gazebo_to_offset(gx, gy):
    """Gazebo (x, y) -> (east_m, north_m) relative to the spawn point.

    Both terms are DIFFERENCES from spawn, so this stays a pure offset and no
    world GPS origin is assumed. See the module docstring for why gazebo_x is
    north and gazebo_y is negated east.
    """
    north = gx - SPAWN_GAZEBO[0]
    east  = -(gy - SPAWN_GAZEBO[1])
    return east, north


class RoadPatrol(Node):
    """Talks to one drone_bridge instance and flies the waypoint list."""

    def __init__(self):
        super().__init__("road_patrol")
        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)

        self.uav_id        = int(self.get_parameter("uav_id").value)
        self.altitude      = float(self.get_parameter("altitude_m").value)
        self.wp_timeout    = float(self.get_parameter("waypoint_timeout_s").value)
        self.climb_timeout = float(self.get_parameter("climb_timeout_s").value)
        self.gps_timeout   = float(self.get_parameter("gps_timeout_s").value)
        self.gps_stale     = float(self.get_parameter("gps_stale_s").value)
        self.service_timeout = float(self.get_parameter("service_timeout_s").value)
        self.settle = float(self.get_parameter("settle_s").value)
        self.hold_s        = float(self.get_parameter("hold_seconds").value)

        self.ns      = f"/uav{self.uav_id}"
        self.lat     = None
        self.lon     = None
        self.rel_alt = 0.0
        self.last_gps_t = 0.0

        # Default QoS (RELIABLE) matches drone_bridge's publishers.
        # BEST_EFFORT here matched only intermittently and lost whole runs.
        self.create_subscription(NavSatFix, f"{self.ns}/gps", self._cb_gps, 10)
        self.create_subscription(Float32, f"{self.ns}/rel_alt", self._cb_alt, 10)
        self.pub_goto = self.create_publisher(GeoPoint, f"{self.ns}/goto", 10)

        self.arm_cli     = self.create_client(Trigger, f"{self.ns}/arm")
        self.takeoff_cli = self.create_client(Trigger, f"{self.ns}/takeoff")
        self.rtl_cli     = self.create_client(Trigger, f"{self.ns}/rtl")

        self.get_logger().info(
            f"road patrol: {self.ns}, {len(WAYPOINTS_GAZEBO)} waypoints, "
            f"altitude={self.altitude:.0f} m")

    def _cb_gps(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude
        self.last_gps_t = time.time()

    def _cb_alt(self, msg):
        self.rel_alt = msg.data

    # ── primitives ───────────────────────────────────────────────────────────
    def call_service(self, client, name):
        """Call a Trigger service. Raise MissionAborted if it reports failure."""
        self.get_logger().info(f"calling {name}")
        if not client.wait_for_service(timeout_sec=self.service_timeout):
            raise MissionAborted(
                f"{name} never appeared. Is drone_bridge running INSIDE gcsns?")
        future = client.call_async(Trigger.Request())

        # Wait on the future; main() spins this node, so the reply arrives there.
        deadline = time.time() + self.service_timeout
        while future.result() is None and time.time() < deadline:
            time.sleep(0.05)
        result = future.result()
        if result is None:
            raise MissionAborted(
                f"{name} did not answer within {self.service_timeout:.0f} s")
        self.get_logger().info(f"  {name} -> {result.message}")
        if not result.success:
            raise MissionAborted(f"{name} failed: {result.message}")

    def goto(self, lat, lon, alt):
        msg = GeoPoint()
        msg.latitude, msg.longitude, msg.altitude = lat, lon, alt
        self.pub_goto.publish(msg)

    def wait_for_gps(self):
        """Bounded: navsat can stop for good, not merely stall."""
        deadline = time.time() + self.gps_timeout
        while rclpy.ok() and self.lat is None:
            if time.time() >= deadline:
                raise MissionAborted(
                    f"no GPS on {self.ns}/gps after {self.gps_timeout:.0f} s. "
                    "AP_DDS has probably lost its publishers -- check for an "
                    "'establish_session' after 'create_topic' in the "
                    "micro_ros_agent log, and relaunch if so.")
            time.sleep(0.1)


    def check_gps_fresh(self) -> None:
        """Raise if the feed froze. A stale fix reads plausible but is not."""
        age = time.time() - self.last_gps_t
        if self.last_gps_t > 0.0 and age > self.gps_stale:
            raise MissionAborted(
                f"position feed stale: no {self.ns}/gps for {age:.0f} s. "
                "AP_DDS has probably lost its publishers -- look for an "
                "'establish_session' after 'create_topic' in the "
                "micro_ros_agent log.")

    def climb_to(self, altitude):
        """Command the current position at a new altitude and wait for it."""
        self.wait_for_gps()
        self.goto(self.lat, self.lon, altitude)
        self.get_logger().info(f"climbing to {altitude:.0f} m")
        deadline = time.time() + self.climb_timeout
        while rclpy.ok() and time.time() < deadline:
            time.sleep(0.1)
            self.check_gps_fresh()
            if self.rel_alt >= altitude * 0.9:      # 90% is close enough
                self.get_logger().info(f"  reached {self.rel_alt:.1f} m")
                return
        raise MissionAborted(
            f"climb timeout: {self.rel_alt:.1f} m after {self.climb_timeout:.0f} s")

    def fly_to(self, lat, lon, label):
        """Fly to a waypoint and wait until it is actually reached.

        Raises rather than warning. A leg that timed out leaves the aircraft
        somewhere unknown, and everything measured afterwards is worthless.
        """
        self.goto(lat, lon, self.altitude)
        self.get_logger().info(f"{label} -> ({lat:.6f}, {lon:.6f})")
        deadline = time.time() + self.wp_timeout
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
            if remaining <= ARRIVAL_RADIUS_M:
                self.get_logger().info(f"  arrived, {remaining:.1f} m off")
                return
        raise MissionAborted(
            f"{label}: not reached within {self.wp_timeout:.0f} s "
            f"(still {distance_m(self.lat, self.lon, lat, lon):.0f} m away). "
            "Raise waypoint_timeout_s, or raise WPNAV_SPEED in the DDS parm file.")

    def hold(self, seconds):
        if seconds <= 0:
            return
        self.get_logger().info(f"holding {seconds:.0f} s")
        end = time.time() + seconds
        while rclpy.ok() and time.time() < end:
            time.sleep(0.1)


def run_mission(node) -> None:
    try:
        # DDS matching is not instantaneous; arm only once a fix is arriving.
        node.get_logger().info("waiting for the first GPS fix ...")
        node.wait_for_gps()
        node.get_logger().info(
            f"GPS ready ({node.lat:.6f}, {node.lon:.6f}); "
            f"settling {node.settle:.0f} s before arming")
        time.sleep(node.settle)

        node.call_service(node.arm_cli, f"{node.ns}/arm")
        node.call_service(node.takeoff_cli, f"{node.ns}/takeoff")
        node.climb_to(node.altitude)

        # Everything is measured from HERE -- the position the drone reported
        # after climbing, which is its take-off point. No absolute coordinates.
        home_lat, home_lon = node.lat, node.lon
        node.get_logger().info(
            f"take-off point: ({home_lat:.6f}, {home_lon:.6f})")

        for i, (gx, gy) in enumerate(WAYPOINTS_GAZEBO, start=1):
            east, north = gazebo_to_offset(gx, gy)
            lat, lon = move_from(home_lat, home_lon, east, north)
            node.get_logger().info(
                f"wp{i}: gazebo ({gx:.1f}, {gy:.1f}) "
                f"= {east:+.0f} m E, {north:+.0f} m N from take-off")
            node.fly_to(lat, lon, f"wp{i}")
            node.hold(node.hold_s)

    except MissionAborted as exc:
        node.get_logger().error(f"MISSION ABORTED: {exc}")
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        # Always try to bring it home, however the mission ended.
        try:
            node.call_service(node.rtl_cli, f"{node.ns}/rtl")
        except Exception as exc:                    # noqa: BLE001
            node.get_logger().error(f"RTL failed: {exc}")


# ─── running the node ───────────────────────────────────────────────────────
# Mission logic runs in a worker thread; main() spins so telemetry never stalls.

def main() -> None:
    rclpy.init()
    node = RoadPatrol()

    # One executor, node added ONCE. rclpy.spin_once(node) re-adds and removes
    # the node every call, which drops subscriptions out of the wait set.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    worker = threading.Thread(target=run_mission, args=(node,),
                              name="road-patrol", daemon=True)
    worker.start()

    try:
        while rclpy.ok() and worker.is_alive():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
