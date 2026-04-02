#!/usr/bin/env python3

import time
from pymavlink import mavutil

def connect_with_retry(max_retries=10):
    """Try to connect to MAVProxy with retries"""
    for attempt in range(max_retries):
        try:
            print(f"Connection attempt {attempt + 1}/{max_retries}...")
            master = mavutil.mavlink_connection('tcp:127.0.0.1:5760', timeout=5)
            
            # Try to get heartbeat with timeout
            master.wait_heartbeat(timeout=5)
            print("Heartbeat received! Connected successfully.")
            return master
        except Exception as e:
            print(f"Attempt {attempt + 1} failed: {e}")
            time.sleep(3)
    
    print("Failed to connect after all attempts")
    return None

# Connect to drone
master = connect_with_retry()
if master is None:
    exit(1)

def set_mode(mode):
    """Change flight mode"""
    print(f"Changing mode to {mode}...")
    mode_id = master.mode_mapping()[mode]
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
        0, mode_id, 0, 0, 0, 0, 0)
    time.sleep(2)
    print(f"Mode changed to {mode}")

def arm_drone():
    """Arm the drone"""
    print("Arming motors...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)
    time.sleep(3)
    print("Motors armed!")

def takeoff(altitude):
    """Takeoff to specified altitude"""
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude)
    
    # Wait for takeoff
    print("Climbing...")
    time.sleep(10)
    print(f"Takeoff complete! Altitude: {altitude}m")

def goto_position(x, y, z):
    """Go to position (x forward, y right, z up)"""
    print(f"Moving to: forward={x}m, right={y}m, altitude={z}m")
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # Position only
        x, y, -z,  # x, y, -z (negative because NED frame)
        0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(8)
    print("Position reached!")

def land():
    """Land the drone"""
    print("Landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)
    time.sleep(12)
    print("Landed!")

# ===== MISSION =====
print("\n=== Starting Mission ===\n")

# 1. Set GUIDED mode
set_mode('GUIDED')

# 2. Arm
arm_drone()

# 3. Takeoff to 10m
takeoff(10)

# 4. Forward 10m
goto_position(10, 0, 10)

# 5. Left 5m
goto_position(10, 5, 10)

# 6. Forward another 10m
goto_position(20, 5, 10)

# 7. Return home
goto_position(0, 0, 10)

# 8. Land
land()

print("\n=== Mission Complete! ===")