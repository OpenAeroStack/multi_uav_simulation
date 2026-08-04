#!/usr/bin/env python3
"""
distance_test_flyer.py
-----------------------
Keyboard-driven flight to fixed, pre-recorded GPS points, for repeatable
distance-sweep testing (B1/B2/B3/etc.) without re-navigating by hand each
time. Reuses exactly the same drone_bridge interfaces already proven in
uav1_patrol_mission.py — /uav1/arm, /uav1/takeoff, /uav1/goto, /uav1/gps,
/uav1/rel_alt, /uav1/rtl.

Keys:
    t       = arm + takeoff to PATROL_ALTITUDE
    1..5    = fly to the corresponding pre-recorded point, wait for arrival
    p       = print current GPS (sanity check against the target)
    r       = switch to RTL
    q       = quit (does NOT auto-RTL — land/RTL manually first if you want)

Run:
    source /opt/ros/humble/setup.bash
    source ~/FYP/multi_uav_simulation/ros2/install/setup.bash
    python3 distance_test_flyer.py
"""

import sys
import termios
import time
import tty
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_srvs.srv import Trigger
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32


# ── Pre-recorded points — edit/add here, no other code changes needed ──────
POINTS = {
    "1": (6.078890, 80.191818, "d01≈58.5m"),
    "2": (6.078890, 80.192001, "d01≈69.5m"),
    "3": (6.078890, 80.192177, "d01≈83.9m"),
    "4": (6.078979, 80.192268, "d01≈97.7m"),
    "5": (6.078979, 80.192451, "d01≈113.8m"),
}

PATROL_ALTITUDE   = 20.0   # matches the altitude these points were recorded at
ARRIVAL_RADIUS_M  = 2.5
WAYPOINT_TIMEOUT  = 40.0
CLIMB_TIMEOUT     = 30.0


def haversine_m(lat1, lon1, lat2, lon2):
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


class DistanceTestFlyer(Node):
    def __init__(self):
        super().__init__('distance_test_flyer')
        self.uav_id = 1
        ns = f'/uav{self.uav_id}'

        self.lat = None
        self.lon = None
        self.rel_alt = 0.0

        qos = QoSProfile(depth=10,
                          reliability=ReliabilityPolicy.BEST_EFFORT,
                          durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(NavSatFix, f'{ns}/gps', self._on_gps, qos)
        self.create_subscription(Float32, f'{ns}/rel_alt', self._on_alt, qos)

        self.goto_pub = self.create_publisher(GeoPoint, f'{ns}/goto', 10)
        self.arm_cli     = self.create_client(Trigger, f'{ns}/arm')
        self.takeoff_cli = self.create_client(Trigger, f'{ns}/takeoff')
        self.rtl_cli     = self.create_client(Trigger, f'{ns}/rtl')

    def _on_gps(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def _on_alt(self, msg):
        self.rel_alt = msg.data

    def call_trigger(self, client, name):
        print(f'  Waiting for service {name}...')
        client.wait_for_service()
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        print(f'  {name} -> success={res.success}  {res.message}')
        return res.success

    def goto(self, lat, lon, alt):
        msg = GeoPoint()
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        self.goto_pub.publish(msg)

    def wait_for_altitude(self, target_alt, timeout=CLIMB_TIMEOUT):
        print(f'  Climbing to {target_alt}m...')
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.rel_alt >= target_alt * 0.9:
                print(f'  -> reached {self.rel_alt:.1f}m')
                return True
        print(f'  -> altitude timeout at {self.rel_alt:.1f}m')
        return False

    def wait_until_arrived(self, lat, lon, timeout=WAYPOINT_TIMEOUT):
        deadline = time.time() + timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.3)
            if self.lat is not None:
                d = haversine_m(self.lat, self.lon, lat, lon)
                if d <= ARRIVAL_RADIUS_M:
                    print(f'  -> arrived (within {d:.1f}m)')
                    return True
        print('  -> waypoint timeout, continuing anyway')
        return False

    def do_takeoff(self):
        if not self.call_trigger(self.arm_cli, f'/uav{self.uav_id}/arm'):
            print('  Arm failed — aborting.')
            return
        if not self.call_trigger(self.takeoff_cli, f'/uav{self.uav_id}/takeoff'):
            print('  Takeoff failed — aborting.')
            return
        while self.lat is None:
            rclpy.spin_once(self, timeout_sec=0.3)
        self.goto(self.lat, self.lon, PATROL_ALTITUDE)
        self.wait_for_altitude(PATROL_ALTITUDE)

    def fly_to_point(self, key):
        lat, lon, label = POINTS[key]
        print(f'  -> flying to point {key} ({label}): ({lat:.6f}, {lon:.6f})')
        self.goto(lat, lon, PATROL_ALTITUDE)
        self.wait_until_arrived(lat, lon)

    def print_gps(self):
        rclpy.spin_once(self, timeout_sec=0.3)
        if self.lat is not None:
            print(f'  Current GPS: lat={self.lat:.6f} lon={self.lon:.6f} '
                  f'alt={self.rel_alt:.1f}m')
        else:
            print('  No GPS yet.')

    def do_rtl(self):
        self.call_trigger(self.rtl_cli, f'/uav{self.uav_id}/rtl')


def getch():
    """Read a single keypress without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_banner():
    print("distance_test_flyer.py")
    print("-----------------------")
    print("Fly to fixed, pre-recorded GPS points for repeatable distance tests.")
    print("Keys:")
    print("    t     = takeoff")
    for k, (lat, lon, label) in POINTS.items():
        print(f"    {k}     = fly to point {k}  ({label})  ({lat:.6f}, {lon:.6f})")
    print("    p     = print current GPS")
    print("    r     = RTL")
    print("    q     = quit (does not auto-RTL)")
    print()


def main():
    rclpy.init()
    node = DistanceTestFlyer()

    print_banner()

    # Spin the node in the background so GPS/altitude subscriptions stay
    # live between keypresses (call_trigger/wait_* still spin inline when
    # they need a fresh result, which is fine — spin_once is reentrant-safe
    # here since we're single-threaded and not spinning concurrently).
    try:
        while True:
            ch = getch()
            if ch == 'q':
                print("Quitting (drone left as-is — RTL/land manually if needed).")
                break
            elif ch == 't':
                node.do_takeoff()
            elif ch in POINTS:
                node.fly_to_point(ch)
            elif ch == 'p':
                node.print_gps()
            elif ch == 'r':
                node.do_rtl()
            else:
                print(f"  (unrecognised key: {ch!r})")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()