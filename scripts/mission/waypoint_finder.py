#!/usr/bin/env python3
"""
waypoint_finder.py
------------------
Manually steer a drone with WASD keys and read its exact GPS.
Use this to find ground-truth waypoint coordinates for missions.

Usage:
    Terminal 1: bash launch/launch_city_dds.sh   (wait for GPS flowing)
    Terminal 2: python3 scripts/waypoint_finder.py --uav 1

Keys:
    w = north 10m    s = south 10m
    a = west  10m    d = east  10m
    W = north 100m   S = south 100m  (capitals = big steps)
    A = west  100m   D = east  100m
    r = rise 5m      f = fall 5m
    R = rise 20m     F = fall 20m
    t = takeoff
    p = print current GPS (copy this into your mission)
    q = quit
"""

import argparse
import sys
import termios
import tty
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoint

STEP_SMALL = 10.0    # metres
STEP_BIG   = 100.0

def move_gps(lat, lon, dist_m, bearing_deg):
    R = 6_371_000.0
    b = math.radians(bearing_deg)
    lat1, lon1 = math.radians(lat), math.radians(lon)
    lat2 = math.asin(math.sin(lat1)*math.cos(dist_m/R) +
                     math.cos(lat1)*math.sin(dist_m/R)*math.cos(b))
    lon2 = lon1 + math.atan2(math.sin(b)*math.sin(dist_m/R)*math.cos(lat1),
                             math.cos(dist_m/R)-math.sin(lat1)*math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)

class WaypointFinder(Node):
    def __init__(self, uav_id, alt):
        super().__init__(f'waypoint_finder_uav{uav_id}')
        self.uav = uav_id
        self.alt = alt
        self.lat = None
        self.lon = None
        self.rel_alt = 0.0
        self.target_lat = None
        self.target_lon = None

        ns = f'/uav{uav_id}'
        self.create_subscription(NavSatFix, f'{ns}/gps', self._gps_cb, 10)
        self.create_subscription(Float32, f'{ns}/rel_alt', self._alt_cb, 10)
        self.takeoff_client = self.create_client(Trigger, f'{ns}/takeoff')
        self.goto_pub = self.create_publisher(GeoPoint, f'{ns}/goto', 10)

        # re-send goto every 2 s so ArduPilot keeps tracking it
        self.create_timer(2.0, self._resend)

    def _gps_cb(self, msg):
        self.lat, self.lon = msg.latitude, msg.longitude

    def _alt_cb(self, msg):
        self.rel_alt = msg.data

    def _resend(self):
        if self.target_lat is not None:
            m = GeoPoint()
            m.latitude, m.longitude = self.target_lat, self.target_lon
            m.altitude = float(self.alt)
            self.goto_pub.publish(m)

    def takeoff(self):
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            print('  takeoff service not available'); return
        fut = self.takeoff_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=90.0)
        r = fut.result()
        print(f'  takeoff: {r.message if r else "no response"}')

    def nudge(self, bearing, dist):
        if self.lat is None:
            print('  no GPS yet'); return
        base_lat = self.target_lat if self.target_lat else self.lat
        base_lon = self.target_lon if self.target_lon else self.lon
        self.target_lat, self.target_lon = move_gps(
            base_lat, base_lon, dist, bearing)
        self._resend()
        d = {0:'N',90:'E',180:'S',270:'W'}[bearing]
        print(f'  → {d} {dist:.0f}m  target=({self.target_lat:.6f}, '
              f'{self.target_lon:.6f})')
    
    def change_alt(self, delta):
        self.alt = max(2.0, self.alt + delta)
        self._resend()
        print(f'  → altitude target = {self.alt:.1f}m '
              f'(current actual = {self.rel_alt:.1f}m)')
        

    def print_gps(self):
        if self.lat is None:
            print('  no GPS yet'); return
        print(f'\n  ═══ CURRENT GPS ═══')
        print(f'  lat = {self.lat:.6f}')
        print(f'  lon = {self.lon:.6f}')
        print(f'  alt = {self.rel_alt:.1f}m')
        print(f'  → paste into mission: ({self.lat:.6f}, {self.lon:.6f})\n')

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uav', type=int, default=1)
    ap.add_argument('--alt', type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = WaypointFinder(args.uav, args.alt)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print(__doc__)
    print(f'Controlling UAV{args.uav} at {args.alt}m\n')

    while True:
        c = getch()
        if   c == 'q': break
        elif c == 't': node.takeoff()
        elif c == 'w': node.nudge(0,   STEP_SMALL)
        elif c == 's': node.nudge(180, STEP_SMALL)
        elif c == 'a': node.nudge(270, STEP_SMALL)
        elif c == 'd': node.nudge(90,  STEP_SMALL)
        elif c == 'W': node.nudge(0,   STEP_BIG)
        elif c == 'S': node.nudge(180, STEP_BIG)
        elif c == 'A': node.nudge(270, STEP_BIG)
        elif c == 'D': node.nudge(90,  STEP_BIG)
        elif c == 'r': node.change_alt(+5.0)   # rise 5m
        elif c == 'f': node.change_alt(-5.0)   # fall 5m
        elif c == 'R': node.change_alt(+20.0)  # rise 20m
        elif c == 'F': node.change_alt(-20.0)  # fall 20m
        elif c == 'p': node.print_gps()

    rclpy.shutdown()

if __name__ == '__main__':
    main()