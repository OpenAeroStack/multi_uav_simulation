#!/usr/bin/env python3
import time
import math
from pymavlink import mavutil

print("Connecting to drone...")
vehicle = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
vehicle.wait_heartbeat()
print(f"Connected! Heartbeat from system {vehicle.target_system}\n")

def request_message_interval(message_id, frequency_hz):
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, message_id, 1e6 / frequency_hz, 0, 0, 0, 0, 0
    )

def setup_streams():
    print(">>> Setting up data streams...")
    request_message_interval(33,  10)   # GLOBAL_POSITION_INT
    request_message_interval(30,  10)   # ATTITUDE
    request_message_interval(193,  2)   # EKF_STATUS_REPORT
    request_message_interval(24,   2)   # GPS_RAW_INT
    request_message_interval(74,   5)   # VFR_HUD
    request_message_interval(1,    2)   # SYS_STATUS
    time.sleep(2)
    print(">>> Streams ready!\n")

def get_msg(timeout=2):
    msg = vehicle.recv_match(blocking=True, timeout=timeout)
    if msg is None:
        return None
    t = msg.get_type()
    if t == 'GLOBAL_POSITION_INT':
        alt = msg.relative_alt / 1000.0
        print(f"  [GPS]  alt={alt:.2f}m  lat={msg.lat/1e7:.6f}  lon={msg.lon/1e7:.6f}")
    elif t == 'ATTITUDE':
        print(f"  [IMU]  roll={msg.roll*57.3:.1f}°  pitch={msg.pitch*57.3:.1f}°  yaw={msg.yaw*57.3:.1f}°")
    elif t == 'EKF_STATUS_REPORT':
        print(f"  [EKF]  flags={bin(msg.flags)}")
    elif t == 'GPS_RAW_INT':
        fix = {0:'NoFix',2:'2D',3:'3D',4:'DGPS',5:'RTK'}.get(msg.fix_type,'?')
        print(f"  [GPS_RAW]  fix={fix}  sats={msg.satellites_visible}")
    elif t == 'VFR_HUD':
        print(f"  [HUD]  alt={msg.alt:.1f}m  speed={msg.groundspeed:.1f}m/s  throttle={msg.throttle}%")
    elif t == 'STATUSTEXT':
        print(f"  [MSG]  {msg.text}")
    elif t == 'HEARTBEAT':
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        print(f"  [HB]   armed={armed}  mode={msg.custom_mode}")
    return msg

def get_current_position():
    """Get current GPS position"""
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            return msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0

def wait_for_ekf():
    print(">>> Waiting for EKF and GPS...")
    ekf_ok = False
    gps_ok = False
    while not (ekf_ok and gps_ok):
        msg = get_msg()
        if msg is None:
            continue
        if msg.get_type() == 'EKF_STATUS_REPORT':
            if (msg.flags & 0x1F) == 0x1F:
                ekf_ok = True
                print("  >>> EKF healthy!")
        if msg.get_type() == 'GPS_RAW_INT':
            if msg.fix_type >= 3:
                gps_ok = True
                print("  >>> GPS 3D fix!")
    print(">>> Ready!\n")

def set_mode(mode):
    print(f">>> Setting mode: {mode}")
    mode_id = vehicle.mode_mapping()[mode]
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg and msg.custom_mode == mode_id:
            print(f">>> Mode confirmed: {mode}\n")
            return
        time.sleep(0.5)

def arm():
    print(">>> Arming motors...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    while True:
        msg = vehicle.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print(">>> ARMED!\n")
            return
        time.sleep(0.5)

def takeoff(altitude):
    print(f">>> Taking off to {altitude}m...")
    vehicle.mav.command_long_send(
        vehicle.target_system, vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            alt = msg.relative_alt / 1000.0
            print(f"  Altitude: {alt:.1f}m / {altitude}m")
            if alt >= altitude * 0.95:
                print(f">>> Reached {altitude}m!\n")
                return

def goto(lat, lon, alt):
    """Fly to a specific GPS coordinate"""
    print(f">>> Going to lat={lat:.6f}  lon={lon:.6f}  alt={alt}m")
    vehicle.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(
        0,                                      # time_boot_ms
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,                     # type_mask: only pos
        int(lat * 1e7),                         # lat
        int(lon * 1e7),                         # lon
        alt,                                    # altitude (relative)
        0, 0, 0,                                # vx vy vz
        0, 0, 0,                                # ax ay az
        0, 0                                    # yaw yaw_rate
    ))
def move_distance(distance_m, direction):
    print(f"\n>>> Moving {distance_m}m {direction}...")

    lat, lon, alt = get_current_position()
    print(f"  Current: lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f}m")

    R = 6371000.0

    if direction == 'forward':
        dlat = (distance_m / R) * (180 / math.pi)
        dlon = 0
    elif direction == 'backward':
        dlat = -(distance_m / R) * (180 / math.pi)
        dlon = 0
    elif direction == 'right':
        dlat = 0
        dlon = (distance_m / R) * (180 / math.pi) / math.cos(math.radians(lat))
    elif direction == 'left':
        dlat = 0
        dlon = -(distance_m / R) * (180 / math.pi) / math.cos(math.radians(lat))

    target_lat = lat + dlat
    target_lon = lon + dlon

    print(f"  Target:  lat={target_lat:.6f}  lon={target_lon:.6f}")
    goto(target_lat, target_lon, alt)

    # Wait until reached — no sleep, filter directly for GPS messages
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            continue

        curr_lat = msg.lat / 1e7
        curr_lon = msg.lon / 1e7

        dlat_m = (curr_lat - target_lat) * (math.pi / 180) * R
        dlon_m = (curr_lon - target_lon) * (math.pi / 180) * R * math.cos(math.radians(curr_lat))
        dist_remaining = math.sqrt(dlat_m**2 + dlon_m**2)

        print(f"  Distance remaining: {dist_remaining:.1f}m")

        if dist_remaining < 2.0:
            print(f">>> Reached destination!\n")
            return

def rtl():
    print(">>> Returning to launch (RTL)...")
    set_mode('RTL')
    print(">>> RTL mode engaged. Drone returning home...")

    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg is None:
            continue
        alt = msg.relative_alt / 1000.0
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        print(f"  Returning... alt={alt:.1f}m  lat={lat:.6f}  lon={lon:.6f}")
        if alt < 0.5:
            print(">>> Landed at home!\n")
            return

# ─────────────────────────────
#  MAIN MISSION
# ─────────────────────────────
setup_streams()
wait_for_ekf()

# Arm and takeoff
set_mode('GUIDED')
arm()
takeoff(20)
time.sleep(3)  # Stabilize at altitude

# Save home position
home_lat, home_lon, _ = get_current_position()
print(f"\n>>> Home position saved: lat={home_lat:.6f}  lon={home_lon:.6f}\n")

# Mission
print("=" * 50)
print("  MISSION START")
print("=" * 50)

move_distance(30, 'forward')   # Move 30m forward (North)
time.sleep(3)

move_distance(20, 'left')      # Move 20m left (West)
time.sleep(3)

move_distance(30, 'forward')   # Move 30m forward (North) again
time.sleep(3)

print("=" * 50)
print("  MISSION COMPLETE - RETURNING HOME")
print("=" * 50)

rtl()

print(">>> Mission finished!")