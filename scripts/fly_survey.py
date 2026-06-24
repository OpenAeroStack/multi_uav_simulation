#!/usr/bin/env python3
"""Take off and fly a north-bound survey path so the camera passes over people.

Run the detector (scripts/yolo_detect_node.py) alongside this to detect humans
from the air. Control is pymavlink over TCP, same as single_drone_takeoff.py.
"""
import argparse
import os
import time

from pymavlink import mavutil

ALTITUDE = 15.0          # metres above home
WAYPOINTS_NED = [        # (north, east) in metres; flies over people in the world
    (10, 0),
    (20, 4),
    (30, -4),
    (40, 0),
]
HOVER_SECONDS = 5        # pause at each waypoint so YOLO can detect


def parse_args():
    p = argparse.ArgumentParser(description="UAV aerial survey flight")
    p.add_argument("--host", default=os.getenv("UAV1_HOST", "127.0.0.1"))
    p.add_argument("--port", type=int, default=int(os.getenv("UAV1_PORT", "5760")))
    return p.parse_args()


ARGS = parse_args()
ADDR = f"tcp:{ARGS.host}:{ARGS.port}"
print(f"Connecting to drone at {ADDR}...")
vehicle = mavutil.mavlink_connection(ADDR, source_system=255)
vehicle.wait_heartbeat()
print(f"Connected! Heartbeat from system {vehicle.target_system}\n")


def request_message_interval(message_id, frequency_hz):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0)


def setup_data_streams():
    print(">>> Requesting data streams...")
    request_message_interval(33, 10)   # GLOBAL_POSITION_INT
    request_message_interval(32, 10)   # LOCAL_POSITION_NED
    request_message_interval(24, 2)    # GPS_RAW_INT
    request_message_interval(193, 2)   # EKF_STATUS_REPORT
    time.sleep(2)


def get_msg():
    return vehicle.recv_match(blocking=True, timeout=2)


def wait_for_ekf():
    print(">>> Waiting for EKF + GPS...")
    ekf_good = gps_good = False
    while not (ekf_good and gps_good):
        msg = get_msg()
        if msg is None:
            continue
        t = msg.get_type()
        if t == 'EKF_STATUS_REPORT' and (msg.flags & 0x1F) == 0x1F:
            ekf_good = True
        if t == 'GPS_RAW_INT' and msg.fix_type >= 3:
            gps_good = True
    print(">>> Ready to fly!\n")


def set_mode(mode):
    print(f">>> Mode -> {mode}")
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
    while True:
        msg = get_msg()
        if msg and msg.get_type() == 'HEARTBEAT' and msg.custom_mode == mode_id:
            return
        time.sleep(0.3)


def arm():
    print(">>> Arming...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    while True:
        msg = get_msg()
        if msg and msg.get_type() == 'HEARTBEAT' \
                and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print(">>> Armed!\n")
            return
        time.sleep(0.3)


def takeoff(alt):
    print(f">>> Taking off to {alt}m...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt)
    while True:
        msg = get_msg()
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            cur = msg.relative_alt / 1000.0
            print(f"    climbing... {cur:.1f}/{alt}m")
            if cur >= alt * 0.95:
                print(">>> Altitude reached!\n")
                return
        time.sleep(0.3)


def goto_ned(north, east, alt):
    """Fly to a local position. NED frame: Down is negative altitude."""
    print(f">>> Going to N={north} E={east} alt={alt}m")
    # type_mask: use position only (ignore vel/accel/yaw) = 0b0000111111111000
    vehicle.mav.set_position_target_local_ned_send(
        0, vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        north, east, -alt,      # x, y, z  (z down -> negative for up)
        0, 0, 0, 0, 0, 0, 0, 0)


# ── Mission ──
setup_data_streams()
wait_for_ekf()
set_mode('GUIDED')
arm()
takeoff(ALTITUDE)

for (n, e) in WAYPOINTS_NED:
    goto_ned(n, e, ALTITUDE)
    time.sleep(HOVER_SECONDS)   # let the detector see what's below/ahead

print(">>> Survey complete. Returning to launch (RTL).")
set_mode('RTL')

try:
    while True:
        get_msg()
except KeyboardInterrupt:
    print("\n>>> Done.")
