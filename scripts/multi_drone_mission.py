#!/usr/bin/env python3
"""
Concurrent 3-UAV mission controller with configurable host/port per UAV.

Defaults:
  UAV1 tcp:127.0.0.1:5760
  UAV2 tcp:127.0.0.1:5770
  UAV3 tcp:127.0.0.1:5780

Environment overrides:
  UAV1_HOST, UAV1_PORT, UAV1_DIRECTION
  UAV2_HOST, UAV2_PORT, UAV2_DIRECTION
  UAV3_HOST, UAV3_PORT, UAV3_DIRECTION
  TAKEOFF_ALT, MOVE_DISTANCE, HOLD_TIME, WAYPOINT_RADIUS
"""

import math
import os
import threading
import time

from pymavlink import mavutil


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    return int(value)


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    return float(value)


DRONES = [
    {
        "name": "UAV1",
        "host": "10.42.1.2", # Namespace IP is used not the local host
        "port": 14550,
        "sysid": 1,
        "direction": "left",
    },
    {
        "name": "UAV2",
        "host": "10.42.2.2",
        "port": 14550,
        "sysid": 2,
        "direction": "right",
    },
    {
        "name": "UAV3",
        "host": "10.42.3.2",
        "port": 14550,
        "sysid": 3,
        "direction": "forward",
    },
]

TAKEOFF_ALT = _env_float("TAKEOFF_ALT", 20.0)
MOVE_DISTANCE = _env_float("MOVE_DISTANCE", 50.0)
HOLD_TIME = _env_float("HOLD_TIME", 5.0)
WAYPOINT_RADIUS = _env_float("WAYPOINT_RADIUS", 2.0)

N_DRONES = len(DRONES)
barrier_takeoff = threading.Barrier(N_DRONES)
barrier_waypoint = threading.Barrier(N_DRONES)
barrier_hold = threading.Barrier(N_DRONES)

errors = []
errors_lock = threading.Lock()


class Drone:
    def __init__(self, name: str, host: str, port: int, sysid: int, direction: str):
        self.name = name
        self.host = host
        self.port = port
        self.sysid = sysid
        self.direction = direction
        self.vehicle = None

    def log(self, msg: str):
        print(f"[{self.name}] {msg}", flush=True)

    def connect(self):
        addr = f"tcp:{self.host}:{self.port}"
        self.log(f"Connecting to {addr} ...")
        self.vehicle = mavutil.mavlink_connection(addr, source_system=255)
        self.vehicle.wait_heartbeat()
        self.log(f"Connected - heartbeat from sysid={self.vehicle.target_system}")

    def _request_interval(self, msg_id: int, hz: float):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6 / hz),
            0,
            0,
            0,
            0,
            0,
        )

    def setup_streams(self):
        self.log("Setting up data streams...")
        self._request_interval(33, 10)
        self._request_interval(30, 10)
        self._request_interval(193, 2)
        self._request_interval(24, 2)
        self._request_interval(74, 5)
        self._request_interval(1, 2)
        time.sleep(2)
        self.log("Streams ready")

    def wait_for_ekf_and_gps(self):
        self.log("Waiting for EKF health and GPS 3D fix...")
        ekf_ok = False
        gps_ok = False
        while not (ekf_ok and gps_ok):
            msg = self.vehicle.recv_match(blocking=True, timeout=3)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "EKF_STATUS_REPORT" and not ekf_ok and (msg.flags & 0x1F) == 0x1F:
                ekf_ok = True
                self.log("EKF healthy")
            if t == "GPS_RAW_INT" and not gps_ok and msg.fix_type >= 3:
                gps_ok = True
                self.log(f"GPS 3D fix - {msg.satellites_visible} sats")
        self.log("Pre-flight checks passed")

    def set_mode(self, mode: str):
        self.log(f"Setting mode -> {mode}")
        mode_map = self.vehicle.mode_mapping()
        if mode not in mode_map:
            raise RuntimeError(f"Mode {mode} not supported by target")
        mode_id = mode_map[mode]
        self.vehicle.mav.set_mode_send(
            self.vehicle.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        while True:
            msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if msg and msg.custom_mode == mode_id:
                self.log(f"Mode confirmed: {mode}")
                return
            time.sleep(0.3)

    def arm(self):
        self.log("Arming motors...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        while True:
            msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                self.log("ARMED")
                return
            time.sleep(0.3)

    def takeoff(self, altitude: float):
        self.log(f"Taking off to {altitude}m...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            altitude,
        )
        while True:
            msg = self.vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg is None:
                continue
            alt = msg.relative_alt / 1000.0
            self.log(f"Altitude {alt:.1f}m / {altitude}m")
            if alt >= altitude * 0.95:
                self.log(f"Reached {altitude}m")
                return

    def get_position(self):
        while True:
            msg = self.vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg:
                return (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)

    def goto(self, lat: float, lon: float, alt: float):
        self.vehicle.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        )

    def move_distance(self, distance_m: float, direction: str):
        self.log(f"Moving {distance_m}m {direction}...")
        lat, lon, alt = self.get_position()
        self.log(f"Current pos  lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f}m")

        earth_radius_m = 6_371_000.0
        if direction == "forward":
            dlat = (distance_m / earth_radius_m) * (180 / math.pi)
            dlon = 0.0
        elif direction == "backward":
            dlat = -(distance_m / earth_radius_m) * (180 / math.pi)
            dlon = 0.0
        elif direction == "right":
            dlat = 0.0
            dlon = (distance_m / earth_radius_m) * (180 / math.pi) / math.cos(math.radians(lat))
        elif direction == "left":
            dlat = 0.0
            dlon = -(distance_m / earth_radius_m) * (180 / math.pi) / math.cos(math.radians(lat))
        else:
            raise ValueError(f"Unknown direction: {direction}")

        tgt_lat = lat + dlat
        tgt_lon = lon + dlon
        self.log(f"Target pos   lat={tgt_lat:.6f}  lon={tgt_lon:.6f}")
        self.goto(tgt_lat, tgt_lon, alt)

        while True:
            msg = self.vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg is None:
                continue
            clat = msg.lat / 1e7
            clon = msg.lon / 1e7
            dlat_m = (clat - tgt_lat) * (math.pi / 180) * earth_radius_m
            dlon_m = (clon - tgt_lon) * (math.pi / 180) * earth_radius_m * math.cos(math.radians(clat))
            remaining = math.sqrt(dlat_m ** 2 + dlon_m ** 2)
            self.log(f"Distance remaining: {remaining:.1f}m")
            if remaining < WAYPOINT_RADIUS:
                self.log("Waypoint reached")
                return

    def rtl(self):
        self.log("RTL engaged - returning home...")
        self.set_mode("RTL")
        while True:
            msg = self.vehicle.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
            if msg is None:
                continue
            alt = msg.relative_alt / 1000.0
            self.log(f"Returning... alt={alt:.1f}m")
            if alt < 0.5:
                self.log("Landed at home")
                return


def run_mission(cfg: dict):
    drone = Drone(
        name=cfg["name"],
        host=cfg["host"],
        port=cfg["port"],
        sysid=cfg["sysid"],
        direction=cfg["direction"],
    )

    try:
        drone.connect()
        drone.setup_streams()
        drone.wait_for_ekf_and_gps()
        drone.set_mode("GUIDED")
        drone.arm()

        drone.takeoff(TAKEOFF_ALT)
        time.sleep(2)

        drone.log("Reached altitude - waiting at takeoff barrier...")
        barrier_takeoff.wait()

        drone.move_distance(MOVE_DISTANCE, drone.direction)

        drone.log("Reached waypoint - waiting at waypoint barrier...")
        barrier_waypoint.wait()

        drone.log(f"Holding for {HOLD_TIME}s...")
        time.sleep(HOLD_TIME)

        barrier_hold.wait()

        drone.rtl()
        drone.log("Mission complete!")

    except Exception as exc:
        drone.log(f"ERROR: {exc}")
        with errors_lock:
            errors.append((drone.name, exc))
            
        # These lines are used to prevent the other drones from hanging forever!
        barrier_takeoff.abort()
        barrier_waypoint.abort()
        barrier_hold.abort()


def main():
    print("=" * 60)
    print("  MULTI-DRONE MISSION  -  3 UAVs")
    print("=" * 60)

    print("Connection plan:")
    for cfg in DRONES:
        print(f"  {cfg['name']}: tcp:{cfg['host']}:{cfg['port']} ({cfg['direction']})")
    print()

    threads = []
    for cfg in DRONES:
        t = threading.Thread(target=run_mission, args=(cfg,), name=cfg["name"])
        threads.append(t)

    print("Spawning mission threads...\n")
    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

    print("\n" + "=" * 60)
    if errors:
        print("  MISSION FINISHED WITH ERRORS:")
        for name, exc in errors:
            print(f"  [{name}] {type(exc).__name__}: {exc}")
    else:
        print("  ALL DRONES COMPLETED MISSION SUCCESSFULLY")
    print("=" * 60)


if __name__ == "__main__":
    main()
