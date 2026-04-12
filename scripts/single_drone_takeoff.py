#!/usr/bin/env python3
import argparse
import os
import time
from pymavlink import mavutil

def parse_args():
    parser = argparse.ArgumentParser(description="Single UAV takeoff test")
    parser.add_argument("--host", default=os.getenv("UAV1_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("UAV1_PORT", "5760")))
    return parser.parse_args()


ARGS = parse_args()
ADDR = f"tcp:{ARGS.host}:{ARGS.port}"
print(f"Connecting to drone at {ADDR}...")
vehicle = mavutil.mavlink_connection(ADDR, source_system=255)
vehicle.wait_heartbeat()
print(f"Connected! Heartbeat from system {vehicle.target_system}\n")

def request_message_interval(message_id, frequency_hz):
    """Request a message at a specific frequency using SET_MESSAGE_INTERVAL"""
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,           # param1: message ID
        1e6 / frequency_hz,   # param2: interval in microseconds
        0, 0, 0, 0, 0
    )

def setup_data_streams():
    print(">>> Requesting data streams from ArduPilot...")
    request_message_interval(33,  10)  # GLOBAL_POSITION_INT  at 10hz
    request_message_interval(30,  10)  # ATTITUDE             at 10hz
    request_message_interval(1,    2)  # SYS_STATUS           at 2hz
    request_message_interval(163,  2)  # AHRS                 at 2hz (has EKF info)
    request_message_interval(193,  2)  # EKF_STATUS_REPORT    at 2hz
    request_message_interval(24,   2)  # GPS_RAW_INT          at 2hz
    request_message_interval(74,   5)  # VFR_HUD              at 5hz
    time.sleep(2)
    print(">>> Data streams requested!\n")

def get_msg():
    msg = vehicle.recv_match(blocking=True, timeout=2)
    if msg is None:
        return None
    t = msg.get_type()
    if t == 'GLOBAL_POSITION_INT':
        alt = msg.relative_alt / 1000.0
        print(f"  [GPS]    alt={alt:.2f}m  lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}")
    elif t == 'ATTITUDE':
        print(f"  [IMU]    roll={msg.roll*57.3:.1f}°  pitch={msg.pitch*57.3:.1f}°  yaw={msg.yaw*57.3:.1f}°")
    elif t == 'EKF_STATUS_REPORT':
        print(f"  [EKF]    flags={bin(msg.flags)}  vel_var={msg.velocity_variance:.3f}  pos_var={msg.pos_horiz_variance:.3f}")
    elif t == 'GPS_RAW_INT':
        fix = {0:'No Fix',1:'No Fix',2:'2D Fix',3:'3D Fix',4:'DGPS',5:'RTK Float',6:'RTK Fixed'}.get(msg.fix_type,'Unknown')
        print(f"  [GPS_RAW] fix={fix}  sats={msg.satellites_visible}  hdop={msg.eph/100:.1f}")
    elif t == 'VFR_HUD':
        print(f"  [HUD]    alt={msg.alt:.1f}m  airspeed={msg.airspeed:.1f}m/s  throttle={msg.throttle}%")
    elif t == 'SYS_STATUS':
        print(f"  [BAT]    voltage={msg.voltage_battery/1000.0:.2f}V")
    elif t == 'HEARTBEAT':
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"  [HB]     mode={msg.custom_mode}  armed={armed}")
    elif t == 'STATUSTEXT':
        print(f"  [MSG]    {msg.text}")
    return msg

def wait_for_ekf():
    print(">>> Waiting for EKF and GPS to initialize...")
    ekf_good = False
    gps_good = False
    while not (ekf_good and gps_good):
        msg = get_msg()
        if msg is None:
            continue
        if msg.get_type() == 'EKF_STATUS_REPORT':
            flags = msg.flags
            if (flags & 0x1F) == 0x1F:
                ekf_good = True
                print("  >>> EKF healthy!")
        if msg.get_type() == 'GPS_RAW_INT':
            if msg.fix_type >= 3:
                gps_good = True
                print("  >>> GPS 3D fix acquired!")
    print(">>> Ready to fly!\n")

def set_mode(mode):
    print(f">>> Setting mode to {mode}...")
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        msg = get_msg()
        if msg and msg.get_type() == 'HEARTBEAT':
            if msg.custom_mode == mode_id:
                print(f">>> Mode confirmed: {mode}\n")
                return
        time.sleep(0.5)

def arm():
    print(">>> Arming motors...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = get_msg()
        if msg and msg.get_type() == 'HEARTBEAT':
            if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print(">>> Motors ARMED!\n")
                return
        time.sleep(0.5)

def takeoff(altitude):
    print(f">>> Taking off to {altitude}m...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = get_msg()
        if msg and msg.get_type() == 'GLOBAL_POSITION_INT':
            current_alt = msg.relative_alt / 1000.0
            print(f">>> Climbing... {current_alt:.1f}m / {altitude}m")
            if current_alt >= altitude * 0.95:
                print(f">>> Target altitude {altitude}m reached!\n")
                return
        time.sleep(0.5)

# ── Main ──
setup_data_streams()
wait_for_ekf()
set_mode('GUIDED')
arm()
takeoff(10)

print(">>> Hovering at 10m. Press Ctrl+C to exit.")
try:
    while True:
        get_msg()
except KeyboardInterrupt:
    print("\n>>> Script ended.")