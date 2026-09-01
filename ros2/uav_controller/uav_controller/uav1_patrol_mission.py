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
from rclpy.node import Node

from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoint


# ─── defaults ────────────────────────────────────────────────────────────────
# Grouped here so every tunable number has one obvious home. Each is also a ROS
# parameter, so a run can override it without editing this file.
DEFAULTS = {
    "uav_id":             1,
    # WHERE THE SUBJECTS ARE, as a metre offset FROM THE TAKE-OFF POINT.
    #
    # Not absolute GPS. The drone reports its own position after take-off, and
    # these are added to it, so no assumption about the world's GPS origin can
    # be wrong. Read both numbers straight out of the .world file:
    #
    #     iris_1_demo pose   -70.0  -22.0      <- where the drone starts
    #     person_center pose  40.5 -187.9      <- where the people are
    #     difference        +110.5 -165.9      <- in GAZEBO axes
    #
    # GAZEBO AXES ARE NOT ENU HERE. The ArduPilot plugin applies
    # <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED>, a pi rotation about
    # X, so what the autopilot reports is:
    #
    #     north =  gazebo_x        east = -gazebo_y
    #
    # Verified against the live vehicle: spawned at Gazebo (-70, -22), it
    # reported itself 70 m SOUTH and 22 m EAST of SITL --home. Assuming plain
    # ENU (x=east, y=north) sends it 90 degrees off, which is what happened on
    # every flight before 2026-08-30.
    #
    #     north = 40.5 - (-70.0)  = +110.5
    #     east  = -(-187.9 - -22.0) = +165.9
    "target_east_m":      165.9,
    "target_north_m":     110.5,
    "altitude_m":         30.0,
    "offset_m":           30.0,         # hold this far short; 0 = directly overhead
    "offset_bearing_deg": 180.0,        # wait to the south, look north
    "hold_seconds":       30.0,
    "waypoint_timeout_s": 300.0,        # 237 m at 1 m/s needs ~240 s
    "climb_timeout_s":    60.0,
    "gps_timeout_s":      30.0,         # first fix; 0 would mean "wait forever"
    # 20 s, not 5. The point of this guard is to catch a feed that is DEAD,
    # not one that paused. city_mission.py on the dynamic-data branch has no
    # staleness check at all and works fine, because a gap only makes it wait
    # longer -- whereas 5 s here turned every survivable pause into an abort.
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

        # DEFAULT QoS (depth 10 = RELIABLE), matching city_mission.py on the
        # dynamic-data branch, which works.
        #
        # This was BEST_EFFORT, justified by a comment claiming it "matches
        # drone_bridge's publishers". It does not. AP_DDS_QOS in the bridge is
        # what the bridge uses to SUBSCRIBE to /ap/vN/navsat; the bridge
        # PUBLISHES /uavN/gps with create_publisher(..., 10), i.e. RELIABLE.
        # The mismatch let the subscription match intermittently: one run UAV1
        # received zero GPS messages while its bridge logged the feed flowing.
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

        # Wait on the future rather than spinning it: main() spins this node
        # continuously, so the response arrives on that thread. Spinning here
        # too would mean two executors on one node -- and, worse, it is what
        # used to stall telemetry, because whatever this thread was doing was
        # the ONLY thing servicing the GPS callback.
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

        Bounded, because /uavN/gps can stop forever rather than merely stall:
        AP_DDS sometimes re-establishes its XRCE session without re-creating
        its publishers, and navsat then never returns for the life of the run.
        Waiting unbounded there hung this script -- and with it run_hitl.sh,
        which blocks on `wait` -- until someone noticed and killed it. Failing
        raises MissionAborted like every other wait here, so `finally:` still
        flies the RTL.
        """
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
        """Raise if the position feed has stopped updating.

        A frozen fix is NOT the same failure as a missing one, and it is the
        more dangerous of the two: self.lat still holds a plausible number, so
        every distance computed from it looks reasonable while being measured
        from wherever the aircraft was when the feed died. That is exactly how
        a flight to the waypoint got reported as "never left the pad" -- ns-3's
        ground truth had the aircraft ON the waypoint at the time.

        AP_DDS drops its publishers on some session re-establishes and never
        re-creates them, so this can happen at any point in a run.
        """
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
            # Re-send periodically. One dropped goto used to strand the
            # aircraft on the pad for the whole timeout while the log claimed
            # it was flying; city_mission.py resends every 2 s for this reason.
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
        # Wait for the GPS feed BEFORE arming, then let discovery settle.
        #
        # city_mission.py does exactly this and it is not decoration: the mission
        # node has only just created its subscriptions, and DDS matching is not
        # instantaneous. Arming first and discovering later is how a run reaches
        # "takeoff complete" with self.lat still None -- the aircraft flies, the
        # mission never sees it, and the abort blames a feed that was fine.
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

        # Stand off by `offset` metres so the subjects land in the camera band,
        # which sits AHEAD of the aircraft, not below it. Shorten the vector by
        # that much, keeping its direction: the drone then stops short of them
        # while still pointing at them, because it yaws the way it flies.
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


# ─── running the node ────────────────────────────────────────────────────────
# The mission logic runs in a WORKER thread while main() spins the node, which
# is the arrangement city_mission.py uses and this file previously did not.
#
# It matters more than it looks. Every reader below (arrival checks, climb
# checks, the staleness guard) depends on /uavN/gps callbacks firing. When the
# mission body WAS the main thread, callbacks only ran when it happened to call
# rclpy.spin_once() -- so a blocking service call, or any wait that forgot to
# spin, silently froze the position feed. The mission then measured distance
# from wherever the aircraft was when it stopped listening, decided it had
# never moved, and aborted a flight that was going perfectly.
#
# With the executor spinning continuously in main(), telemetry arrives no
# matter what the mission thread is doing, and the mission thread only sleeps.

def main() -> None:
    rclpy.init()
    node = SurveyMission()

    worker = threading.Thread(target=run_mission, args=(node,),
                              name="survey", daemon=True)
    worker.start()

    try:
        while rclpy.ok() and worker.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info("interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
