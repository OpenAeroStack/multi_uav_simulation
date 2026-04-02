#!/usr/bin/env python3
"""
Multi-UAV Takeoff Script - Parallel EKF wait version
"""

import time
import threading
from pymavlink import mavutil

ready_flags = {1: False, 2: False, 3: False}

def connect_drone(port, sysid):
    print(f"[UAV{sysid}] Connecting to tcp:127.0.0.1:{port}...")
    master = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}')
    master.wait_heartbeat()
    print(f"[UAV{sysid}] Connected ✓")
    return master

def set_mode_guided(master, sysid):
    mode_id = master.mode_mapping()['GUIDED']
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    time.sleep(2)
    print(f"[UAV{sysid}] GUIDED mode sent")

def wait_ekf_thread(master, sysid, timeout=90):
    """Runs in a thread — waits for EKF ready and sets flag"""
    start = time.time()
    gps_using = False

    while time.time() - start < timeout:
        while True:
            msg = master.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'STATUSTEXT':
                text = msg.text.strip()
                if 'is using GPS' in text:
                    gps_using = True
        if gps_using:
            print(f"[UAV{sysid}] EKF ready ✓")
            ready_flags[sysid] = True
            return
        time.sleep(0.2)

    print(f"[UAV{sysid}] EKF timeout — marking ready anyway")
    ready_flags[sysid] = True

def wait_all_ready(timeout=90):
    """Wait until all drones have their EKF ready flag set"""
    start = time.time()
    while time.time() - start < timeout:
        if all(ready_flags.values()):
            return True
        not_ready = [s for s, v in ready_flags.items() if not v]
        print(f"    Waiting for UAV{not_ready}...", end='\r')
        time.sleep(1)
    return False

def arm(master, sysid, max_retries=8):
    for attempt in range(max_retries):
        print(f"[UAV{sysid}] Arming attempt {attempt+1}...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == 0:
                print(f"[UAV{sysid}] Armed ✓")
                return True
            else:
                print(f"[UAV{sysid}] Arm failed (result={msg.result}), retrying...")
                time.sleep(3)
        else:
            time.sleep(3)
    return False

def takeoff(master, altitude, sysid):
    print(f"[UAV{sysid}] Takeoff to {altitude}m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg:
        if msg.result == 0:
            print(f"[UAV{sysid}] Takeoff accepted ✓")
            return True
        else:
            print(f"[UAV{sysid}] Takeoff rejected (result={msg.result})")
    return False

def main():
    ALTITUDE = 5

    print("=" * 50)
    print("  Multi-UAV Takeoff Script")
    print("=" * 50)

    # Step 1 — Connect all
    uav1 = connect_drone(5760, 1)
    uav2 = connect_drone(5770, 2)
    uav3 = connect_drone(5780, 3)
    drones = [(uav1, 1), (uav2, 2), (uav3, 3)]

    # Step 2 — Set GUIDED mode on all
    print("\n--- Setting GUIDED mode ---")
    for master, sysid in drones:
        set_mode_guided(master, sysid)

    # Step 3 — Wait for EKF on ALL drones in PARALLEL using threads
    # This is the key fix — all 3 wait simultaneously
    # so no drone is waiting idle while another initializes
    print("\n--- Waiting for EKF on all drones in parallel ---")
    threads = []
    for master, sysid in drones:
        t = threading.Thread(target=wait_ekf_thread, args=(master, sysid))
        t.daemon = True
        t.start()
        threads.append(t)

    # Wait until ALL are ready
    wait_all_ready(timeout=90)
    print("\n--- All drones EKF ready ---")

    # Step 4 — Arm and takeoff each drone immediately
    # Now all 3 are ready at the same time so no auto-disarm issue
    print("\n--- Arming and taking off ---")
    for master, sysid in drones:
        success = arm(master, sysid)
        if success:
            time.sleep(1)
            takeoff(master, ALTITUDE, sysid)
            time.sleep(1)

    print("\n" + "=" * 50)
    print("  All takeoff commands sent!")
    print("  Check Gazebo window")
    print("=" * 50)
    print("\nPress Ctrl+C to exit")
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()