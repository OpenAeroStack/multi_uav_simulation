#!/usr/bin/env python3
"""Fly one UAV to a set of subjects, hold station while the edge node counts
them, then return to launch.

    arm -> take off -> climb -> approach -> hold -> RTL

WHY IT HOLDS SHORT OF THE SUBJECTS
    The camera is pitched 45 degrees FORWARD-down, not straight down. At 25 m
    altitude it sees a ground band roughly 17-36 m AHEAD of the aircraft, so a
    drone parked directly above the subjects cannot see them: they sit in a
    blind spot under the nose. The mission therefore stops OFFSET_M short of
    them, having approached along the same bearing so it is facing them.

    The 45 degree pitch is deliberate. A standing person's pixel height scales
    as 0.5*sin(2*pitch), which peaks exactly at 45 degrees. See the comment in
    models/iris_N_netns/model.sdf.

Run it INSIDE the gcsns namespace - drone_bridge lives there, and from the root
namespace the services below do not exist and this script waits forever.

    python3 uav1_patrol_mission.py --ros-args -p uav_id:=1

Parameters (all optional; defaults in DEFAULTS below):
    uav_id              which aircraft: 1, 2, ...
    target_east_m       metres EAST of take-off where the subjects are
    target_north_m      metres NORTH of take-off (both read from the .world file)
    altitude_m          hold altitude, metres above launch
    offset_m            how far short of the subjects to hold
    offset_bearing_deg  bearing FROM the subjects TO the hold point.
                        180 = wait to the south, look north.
    hold_seconds        how long to hold station
    waypoint_timeout_s  give up on a leg after this long
    gps_timeout_s       give up waiting for the first GPS fix after this long
    gps_stale_s         abort if the position feed stops updating for this long

Services and topics used (all under /uav<id>/):
    arm, takeoff, rtl   std_srvs/Trigger
    goto                geographic_msgs/GeoPoint   (altitude is RELATIVE)
    gps                 sensor_msgs/NavSatFix      - arrival check
    rel_alt             std_msgs/Float32           - climb check
"""
import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoint


# ─── defaults ───────────────────────────────────────────────────────────────
# Each is also a ROS parameter, overridable per run.
DEFAULTS = {
    "uav_id":             1,
    # Offsets from take-off, in AUTOPILOT axes: north=gazebo_x, east=-gazebo_y.
    # NOT plain ENU -- see docs/COORDINATE_FRAMES.pdf. Do not 'fix' to ENU.
    "target_east_m":      165.9,
    "target_north_m":     110.5,
    "altitude_m":         30.0,
    "offset_m":           30.0,         # hold this far short; 0 = directly overhead
    "offset_bearing_deg": 180.0,        # wait to the south, look north
    "hold_seconds":       30.0,
    "waypoint_timeout_s": 300.0,        # 237 m at 1 m/s needs ~240 s
    "climb_timeout_s":    60.0,
    "gps_timeout_s":      30.0,         # first fix; 0 would mean "wait forever"
    # 20 s catches a DEAD feed; 5 s tripped on survivable pauses.
    "gps_stale_s":        20.0,         # a FROZEN feed is not a healthy one
    "service_timeout_s":  120.0,        # arm can legitimately retry for ~90 s
    "settle_s":           15.0,         # DDS discovery headroom before arming
}

ARRIVAL_RADIUS_M = 2.5      # horizontal distance that counts as "arrived"
EARTH_RADIUS_M   = 6371000.0
METRES_PER_DEG_LAT = 111320.0


class MissionAborted(Exception):
    """A step did not complete, so continuing would measure the wrong thing.

    Raised instead of logging a warning and carrying on. A drone that never
    reached its waypoint still holds, still records, and still reports success
    - the numbers are simply meaningless. Failing loudly is the whole point.
    """


# ─── geometry helpers ────────────────────────────────────────────────────────
def move_from(lat: float, lon: float,
              east_m: float, north_m: float) -> tuple[float, float]:
    """Return the point east_m east and north_m north of (lat, lon).

    Works entirely in metres relative to a position the drone itself reported,
    so no assumption about the world's GPS origin can put it in the wrong place.
    """
    dlat = north_m / METRES_PER_DEG_LAT
    dlon = east_m / (METRES_PER_DEG_LAT * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


# ─── the mission node ────────────────────────────────────────────────────────
class SurveyMission(Node):
    """Talks to one drone_bridge instance and flies the survey."""

    def __init__(self) -> None:
        super().__init__("survey_mission")

        for name, default in DEFAULTS.items():
            self.declare_parameter(name, default)
        self.uav_id    = int(self._param("uav_id"))
        self.target_east  = self._param("target_east_m")
        self.target_north = self._param("target_north_m")
        self.altitude  = self._param("altitude_m")
        self.offset    = self._param("offset_m")
        self.bearing   = self._param("offset_bearing_deg")
        self.hold_s    = self._param("hold_seconds")
        self.wp_timeout    = self._param("waypoint_timeout_s")
        self.climb_timeout = self._param("climb_timeout_s")
        self.gps_timeout   = self._param("gps_timeout_s")
        self.gps_stale     = self._param("gps_stale_s")
        self.service_timeout = self._param("service_timeout_s")
        self.settle = self._param("settle_s")

        self.ns = f"/uav{self.uav_id}"

        # Latest telemetry. None until the first message arrives, which is why
        # every reader below checks for it.
        self.lat: float | None = None
        self.lon: float | None = None
        self.rel_alt: float = 0.0
        self.last_gps_t: float = 0.0

        # Default QoS (RELIABLE) matches drone_bridge's publishers.
        # BEST_EFFORT here matched only intermittently and lost whole runs.
        self.create_subscription(NavSatFix, f"{self.ns}/gps", self._on_gps, 10)
        self.create_subscription(Float32, f"{self.ns}/rel_alt", self._on_alt, 10)

        self.goto_pub    = self.create_publisher(GeoPoint, f"{self.ns}/goto", 10)
        self.arm_cli     = self.create_client(Trigger, f"{self.ns}/arm")
        self.takeoff_cli = self.create_client(Trigger, f"{self.ns}/takeoff")
        self.rtl_cli     = self.create_client(Trigger, f"{self.ns}/rtl")

        self.get_logger().info(
            f"survey: {self.ns} target={self.target_east:+.0f} m E, "
            f"{self.target_north:+.0f} m N from take-off  "
            f"altitude={self.altitude:.0f} m hold={self.hold_s:.0f} s "
            f"stand-off={self.offset:.0f} m")

    def _param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    # ── telemetry callbacks ──────────────────────────────────────────────
    def _on_gps(self, msg: NavSatFix) -> None:
        self.lat, self.lon = msg.latitude, msg.longitude
        self.last_gps_t = time.time()

    def _on_alt(self, msg: Float32) -> None:
        self.rel_alt = msg.data

    # ── one action each ──────────────────────────────────────────────────
    def call_service(self, client, name: str) -> None:
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

    def wait_for_gps(self) -> None:
        """Block until the first GPS fix arrives, so positions are usable.

Bounded: navsat can stop for good, not merely stall."""
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

    def climb_to(self, altitude: float) -> None:
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

    def goto(self, lat: float, lon: float, altitude: float) -> None:
        """Send one waypoint. Altitude is RELATIVE to launch."""
        msg = GeoPoint()
        msg.latitude, msg.longitude, msg.altitude = lat, lon, altitude
        self.goto_pub.publish(msg)

    def fly_to(self, lat: float, lon: float, label: str) -> None:
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
            f"(still {remaining:.0f} m away). Raise waypoint_timeout_s, or "
            f"raise WPNAV_SPEED in the DDS parm file.")

    def hold_station(self, seconds: float) -> None:
        """Stay put while the edge node works. Spinning keeps ROS alive."""
        self.get_logger().info(f"on station - holding {seconds:.0f} s")
        end = time.time() + seconds
        next_report = end - 10.0
        while rclpy.ok() and time.time() < end:
            time.sleep(0.1)
            if time.time() >= next_report:
                self.get_logger().info(f"  {end - time.time():.0f} s left")
                next_report -= 10.0
        self.get_logger().info("hold complete")


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

        # Everything is measured from HERE - the position the drone reported
        # after climbing, which is the take-off point. No absolute coordinates.
        home_lat, home_lon = node.lat, node.lon
        node.get_logger().info(
            f"take-off point: ({home_lat:.6f}, {home_lon:.6f})")

        # Where the people are, relative to take-off.
        east, north = node.target_east, node.target_north

        # Shorten the vector by `offset`, keeping direction: the camera band sits
        # ahead of the nose, and the drone yaws the way it flies.
        span = math.hypot(east, north)
        if span > node.offset:
            scale = (span - node.offset) / span
            east, north = east * scale, north * scale

        target = move_from(home_lat, home_lon, east, north)
        node.get_logger().info(
            f"flying {math.hypot(east, north):.0f} m to station "
            f"({node.offset:.0f} m short of the subjects)")
        node.fly_to(*target, label="station")
        node.hold_station(node.hold_s)

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
    node = SurveyMission()

    # One executor, node added ONCE. rclpy.spin_once(node) re-adds and removes
    # the node every call, which drops subscriptions out of the wait set.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    worker = threading.Thread(target=run_mission, args=(node,),
                              name="survey", daemon=True)
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
