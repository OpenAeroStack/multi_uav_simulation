#!/usr/bin/env python3
"""
multi_drone_mission_new.py
──────────────────────────
Updated copy of multi_drone_misison.py.

CHANGE vs the original: arm() previously sent the arm command ONCE and then
waited forever for an armed heartbeat. If ArduPilot rejected the command
(pre-arm checks still settling shortly after SITL boot), the script hung
silently at "Arming motors...". arm() now resends the command every 4s and
prints the flight controller's STATUSTEXT / COMMAND_ACK messages so any
"PreArm: ..." rejection reason is visible. All removed original lines are
kept commented in place.

Controls 3 ArduCopter SITL instances simultaneously using Python threads.

Ports (set by -I flag in launch script):
  UAV1  tcp:127.0.0.1:5760   sysid=1   moves FORWARD  50m
  UAV2  tcp:127.0.0.1:5770   sysid=2   moves BACKWARD 50m
  UAV3  tcp:127.0.0.1:5780   sysid=3   moves LEFT      50m

Mission flow:
  1. All 3 connect and wait for EKF + GPS
  2. All 3 arm and takeoff to 20m          ← barrier sync
  3. All 3 move to their waypoint          ← barrier sync
  4. All 3 hold for 5 seconds              ← barrier sync
  5. All 3 RTL simultaneously
"""

import math
import threading
import time

from pymavlink import mavutil

# ─── Configuration ────────────────────────────────────────────────────────────

DRONES = [
    {"name": "UAV1", "port": 5760, "sysid": 1, "direction": "left"},
    {"name": "UAV2", "port": 5770, "sysid": 2, "direction": "right"},
    {"name": "UAV3", "port": 5780, "sysid": 3, "direction": "forward"},
]

TAKEOFF_ALT   = 20      # metres
MOVE_DISTANCE = 50      # metres
HOLD_TIME     = 5       # seconds
WAYPOINT_RADIUS = 2.0   # metres — how close = "arrived"

# Threading.Barrier: blocks until all 3 threads have reached this point
N_DRONES = len(DRONES)
barrier_takeoff  = threading.Barrier(N_DRONES)
barrier_waypoint = threading.Barrier(N_DRONES)
barrier_hold     = threading.Barrier(N_DRONES)

# Shared list to collect per-thread errors; main thread checks after join
errors = []
errors_lock = threading.Lock()

# ─── Drone class ──────────────────────────────────────────────────────────────

class Drone:
    """Wraps a single pymavlink connection plus all helper methods."""

    def __init__(self, name: str, port: int, sysid: int, direction: str):
        self.name      = name
        self.port      = port
        self.sysid     = sysid
        self.direction = direction
        self.vehicle   = None          # set in connect()

    # ── logging ───────────────────────────────────────────────────────────────

    def log(self, msg: str):
        print(f"[{self.name}] {msg}", flush=True)

    # ── connection ────────────────────────────────────────────────────────────

    def connect(self):
        addr = f"tcp:127.0.0.1:{self.port}"
        self.log(f"Connecting to {addr} ...")
        self.vehicle = mavutil.mavlink_connection(addr, source_system=255)
        self.vehicle.wait_heartbeat()
        self.log(f"Connected — heartbeat from sysid={self.vehicle.target_system}")

    # ── stream setup ──────────────────────────────────────────────────────────

    def _request_interval(self, msg_id: int, hz: float):
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6 / hz),
            0, 0, 0, 0, 0,
        )

    def setup_streams(self):
        self.log("Setting up data streams...")
        self._request_interval(33,  10)   # GLOBAL_POSITION_INT
        self._request_interval(30,  10)   # ATTITUDE
        self._request_interval(193,  2)   # EKF_STATUS_REPORT
        self._request_interval(24,   2)   # GPS_RAW_INT
        self._request_interval(74,   5)   # VFR_HUD
        self._request_interval(1,    2)   # SYS_STATUS
        time.sleep(2)
        self.log("Streams ready")

    # ── EKF / GPS gate ────────────────────────────────────────────────────────

    def wait_for_ekf_and_gps(self):
        self.log("Waiting for EKF health and GPS 3D fix...")
        ekf_ok = gps_ok = False
        while not (ekf_ok and gps_ok):
            msg = self.vehicle.recv_match(blocking=True, timeout=3)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "EKF_STATUS_REPORT" and not ekf_ok:
                if (msg.flags & 0x1F) == 0x1F:
                    ekf_ok = True
                    self.log("EKF healthy")
            if t == "GPS_RAW_INT" and not gps_ok:
                if msg.fix_type >= 3:
                    gps_ok = True
                    self.log(f"GPS 3D fix — {msg.satellites_visible} sats")
        self.log("Pre-flight checks passed")

    # ── mode / arm ────────────────────────────────────────────────────────────

    def set_mode(self, mode: str):
        self.log(f"Setting mode → {mode}")
        mode_id = self.vehicle.mode_mapping()[mode]
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
        # REMOVED: the arm command was sent only once before the wait loop —
        # if ArduPilot rejected it (pre-arm checks still settling after SITL
        # boot), the script hung forever at "Arming motors..." with no
        # indication why:
        # self.vehicle.mav.command_long_send(
        #     self.vehicle.target_system,
        #     self.vehicle.target_component,
        #     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        #     0, 1, 0, 0, 0, 0, 0, 0,
        # )
        # while True:
        #     msg = self.vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        #     if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        #         self.log("ARMED")
        #         return
        #     time.sleep(0.3)
        # ADDED: resend the arm command every 4s until armed, and surface the
        # flight controller's STATUSTEXT messages (e.g. "PreArm: ...") and
        # COMMAND_ACK rejections so the reason is visible instead of silent.
        last_send = 0.0
        while True:
            if time.time() - last_send > 4.0:
                self.vehicle.mav.command_long_send(
                    self.vehicle.target_system,
                    self.vehicle.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0,
                )
                last_send = time.time()
            msg = self.vehicle.recv_match(
                type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"],
                blocking=True, timeout=3,
            )
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT":
                if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                    self.log("ARMED")
                    return
            elif t == "STATUSTEXT":
                self.log(f"FC: {msg.text}")
            elif t == "COMMAND_ACK":
                if (msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                        and msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED):
                    self.log(f"Arm rejected (result={msg.result}) — retrying...")

    # ── takeoff ───────────────────────────────────────────────────────────────

    def takeoff(self, altitude: float):
        self.log(f"Taking off to {altitude}m...")
        self.vehicle.mav.command_long_send(
            self.vehicle.target_system,
            self.vehicle.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude,
        )
        while True:
            msg = self.vehicle.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=5
            )
            if msg is None:
                continue
            alt = msg.relative_alt / 1000.0
            self.log(f"Altitude {alt:.1f}m / {altitude}m")
            if alt >= altitude * 0.95:
                self.log(f"Reached {altitude}m")
                return

    # ── position helpers ──────────────────────────────────────────────────────

    def get_position(self):
        """Block until a fresh GLOBAL_POSITION_INT arrives."""
        while True:
            msg = self.vehicle.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=5
            )
            if msg:
                return (
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.relative_alt / 1000.0,
                )

    def goto(self, lat: float, lon: float, alt: float):
        self.vehicle.mav.send(
            mavutil.mavlink.MAVLink_set_position_target_global_int_message(
                0,
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,   # position only
                int(lat * 1e7),
                int(lon * 1e7),
                alt,
                0, 0, 0,              # vx vy vz (ignored)
                0, 0, 0,              # ax ay az (ignored)
                0, 0,                 # yaw yaw_rate (ignored)
            )
        )

    # ── movement ──────────────────────────────────────────────────────────────

    def move_distance(self, distance_m: float, direction: str):
        self.log(f"Moving {distance_m}m {direction}...")
        lat, lon, alt = self.get_position()
        self.log(f"Current pos  lat={lat:.6f}  lon={lon:.6f}  alt={alt:.1f}m")

        R = 6_371_000.0  # Earth radius in metres

        if   direction == "forward":
            dlat = (distance_m / R) * (180 / math.pi);  dlon = 0.0
        elif direction == "backward":
            dlat = -(distance_m / R) * (180 / math.pi); dlon = 0.0
        elif direction == "right":
            dlat = 0.0;  dlon =  (distance_m / R) * (180 / math.pi) / math.cos(math.radians(lat))
        elif direction == "left":
            dlat = 0.0;  dlon = -(distance_m / R) * (180 / math.pi) / math.cos(math.radians(lat))
        else:
            raise ValueError(f"Unknown direction: {direction}")

        tgt_lat = lat + dlat
        tgt_lon = lon + dlon
        self.log(f"Target pos   lat={tgt_lat:.6f}  lon={tgt_lon:.6f}")
        self.goto(tgt_lat, tgt_lon, alt)

        # Poll until within WAYPOINT_RADIUS
        while True:
            msg = self.vehicle.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=5
            )
            if msg is None:
                continue
            clat = msg.lat / 1e7
            clon = msg.lon / 1e7
            dlat_m = (clat - tgt_lat) * (math.pi / 180) * R
            dlon_m = (clon - tgt_lon) * (math.pi / 180) * R * math.cos(math.radians(clat))
            remaining = math.sqrt(dlat_m ** 2 + dlon_m ** 2)
            self.log(f"Distance remaining: {remaining:.1f}m")
            if remaining < WAYPOINT_RADIUS:
                self.log("Waypoint reached")
                return

    # ── RTL ───────────────────────────────────────────────────────────────────

    def rtl(self):
        self.log("RTL engaged — returning home...")
        self.set_mode("RTL")
        while True:
            msg = self.vehicle.recv_match(
                type="GLOBAL_POSITION_INT", blocking=True, timeout=5
            )
            if msg is None:
                continue
            alt = msg.relative_alt / 1000.0
            self.log(f"Returning... alt={alt:.1f}m")
            if alt < 0.5:
                self.log("Landed at home")
                return


# ─── Per-drone mission thread ─────────────────────────────────────────────────

def run_mission(cfg: dict):
    """
    Full mission for one drone.  Runs in its own thread.
    Errors are caught and recorded so the main thread can report them.
    """
    drone = Drone(
        name      = cfg["name"],
        port      = cfg["port"],
        sysid     = cfg["sysid"],
        direction = cfg["direction"],
    )

    try:
        # ── Phase 0: setup ───────────────────────────────────────────────────
        drone.connect()
        drone.setup_streams()
        drone.wait_for_ekf_and_gps()
        drone.set_mode("GUIDED")
        drone.arm()

        # ── Phase 1: takeoff — sync at barrier_takeoff ────────────────────
        drone.takeoff(TAKEOFF_ALT)
        time.sleep(2)   # brief stabilise hover

        drone.log("Reached altitude — waiting at takeoff barrier...")
        barrier_takeoff.wait()   # all 3 must finish takeoff before anyone moves

        # ── Phase 2: diverge — sync at barrier_waypoint ───────────────────
        drone.move_distance(MOVE_DISTANCE, drone.direction)

        drone.log("Reached waypoint — waiting at waypoint barrier...")
        barrier_waypoint.wait()  # all 3 must arrive before hold starts

        # ── Phase 3: hold — sync at barrier_hold ─────────────────────────
        drone.log(f"Holding for {HOLD_TIME}s...")
        time.sleep(HOLD_TIME)

        barrier_hold.wait()      # all 3 hold, then RTL together

        # ── Phase 4: RTL ─────────────────────────────────────────────────
        drone.rtl()

        drone.log("Mission complete!")

    except Exception as exc:
        drone.log(f"ERROR: {exc}")
        with errors_lock:
            errors.append((drone.name, exc))


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("  MULTI-DRONE MISSION  —  3 UAVs")
    print("=" * 60)

    threads = []
    for cfg in DRONES:
        t = threading.Thread(target=run_mission, args=(cfg,), name=cfg["name"])
        threads.append(t)

    # Start all threads at once
    print("\nSpawning mission threads...\n")
    for t in threads:
        t.start()

    # Wait for all threads to finish
    for t in threads:
        t.join()

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
