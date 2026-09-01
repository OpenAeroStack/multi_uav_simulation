#!/usr/bin/env python3
"""Passive experiment logger for the mech-workshop validation mission.

This is deliberately a standalone ROS 2 script.  It publishes nothing, calls
no services, and opens no MAVLink connection.  Run it inside ``gcsns`` after
``launch_mech_workshop_single_uav.sh`` is ready and before ``test_cruise.py``.

Coordinate convention
---------------------
ArduPilot's DDS ``pose/filtered`` and ``twist/filtered`` topics are converted
by ArduPilot to ROS REP-103 ENU before publication:

    local_x / vx = East
    local_y / vy = North
    local_z / vz = Up

Quaternion and Euler angles are therefore ROS ENU. ``heading_deg`` is derived
as navigation heading (clockwise from North) from the ENU yaw.
"""

from __future__ import annotations

import csv
import json
import math
import os
import platform
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import rclpy
from ardupilot_msgs.msg import Status
from geographic_msgs.msg import GeoPoint, GeoPointStamped
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent.parent
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR / 'experiment_logs'
DEFAULT_SNR_FILE = Path('/tmp/ns3_snr_mw.csv')

# True launch pad and world origin in mech_workshop_validation.world.
DEFAULT_ORIGIN_LAT = 6.0773722
DEFAULT_ORIGIN_LON = 80.1907552

BEST_EFFORT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

GPS_COLUMNS = [
    'elapsed_time_s', 'wall_time', 'wall_time_unix_s',
    'source_timestamp_s', 'source_clock_s',
    'latitude_deg', 'longitude_deg', 'altitude_msl_m',
    'relative_altitude_m',
    'gps_local_east_m', 'gps_local_north_m',
    'local_x_east_m', 'local_y_north_m', 'local_z_up_m',
    'ground_speed_mps', 'vertical_speed_up_mps', 'heading_deg',
    'source_topic',
]

VEHICLE_COLUMNS = [
    'elapsed_time_s', 'wall_time', 'wall_time_unix_s',
    'source_timestamp_s', 'source_clock_s',
    'local_x_east_m', 'local_y_north_m', 'local_z_up_m',
    'roll_deg', 'pitch_deg', 'yaw_enu_deg', 'heading_deg',
    'qx', 'qy', 'qz', 'qw',
    'vx_east_mps', 'vy_north_mps', 'vz_up_mps',
    'ground_speed_mps', 'vertical_speed_up_mps',
    'relative_altitude_m', 'armed', 'flight_mode', 'flying',
]

MISSION_COLUMNS = [
    'elapsed_time_s', 'wall_time', 'wall_time_unix_s',
    'source_timestamp_s', 'source_clock_s',
    'record_reason', 'current_waypoint', 'guided_target_revision',
    'mission_state', 'target_source',
    'target_latitude_deg', 'target_longitude_deg', 'target_altitude_m',
    'target_altitude_reference', 'target_x_east_m', 'target_y_north_m',
    'distance_to_target_m',
    'current_latitude_deg', 'current_longitude_deg',
    'ap_goal_latitude_deg', 'ap_goal_longitude_deg',
    'ap_goal_altitude_msl_m',
]

TELEMETRY_COLUMNS = [
    'elapsed_time_s', 'wall_time', 'wall_time_unix_s',
    'source_timestamp_s', 'source_clock_s',
    'message_type', 'topic', 'sequence_number', 'sysid', 'compid',
    'interarrival_ms',
]

NETWORK_COLUMNS = [
    'elapsed_time_s', 'wall_time', 'wall_time_unix_s',
    'ns3_time_s', 'rx_node', 'freq_mhz', 'pkt_bytes',
    'signal_dbm', 'noise_dbm', 'snr_db',
    'uav_id', 'peer_id',
    'horizontal_distance_to_gcs_m', 'distance_to_gcs_m',
    'position_age_s', 'distance_source',
]

EVENT_COLUMNS = [
    'elapsed_time_s', 'wall_time', 'wall_time_unix_s',
    'event_type', 'description', 'value',
]

SNR_SOURCE_HEADER = [
    'time_s', 'rx_node', 'freq_mhz', 'pkt_bytes',
    'signal_dbm', 'noise_dbm', 'snr_db',
]


class CsvLog:
    """Small CSV writer with a row counter and explicit periodic flushing."""

    def __init__(self, path: Path, columns: list[str]):
        self.path = path
        self.file = path.open('w', newline='', encoding='utf-8')
        self.writer = csv.DictWriter(self.file, fieldnames=columns)
        self.writer.writeheader()
        self.count = 0
        self.lock = threading.Lock()

    def write(self, row: dict[str, Any]) -> None:
        with self.lock:
            self.writer.writerow(row)
            self.count += 1

    def flush(self) -> None:
        with self.lock:
            self.file.flush()

    def close(self) -> None:
        with self.lock:
            if not self.file.closed:
                self.file.flush()
                self.file.close()


def stamp_to_seconds(stamp: Any) -> Optional[float]:
    if stamp is None:
        return None
    sec = int(getattr(stamp, 'sec', 0))
    nanosec = int(getattr(stamp, 'nanosec', 0))
    if sec == 0 and nanosec == 0:
        return None
    return sec + nanosec * 1e-9


def fmt(value: Any, digits: int = 9) -> Any:
    if value is None:
        return ''
    if isinstance(value, float):
        if not math.isfinite(value):
            return ''
        return f'{value:.{digits}f}'
    return value


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float):
    """Return ROS-ENU roll, pitch and yaw in degrees."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def enu_yaw_to_heading(yaw_enu_deg: float) -> float:
    """Convert CCW-from-East ENU yaw to clockwise-from-North heading."""
    return (90.0 - yaw_enu_deg) % 360.0


def geodetic_to_local_enu(
    latitude_deg: float,
    longitude_deg: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
):
    """WGS-84 local tangent approximation returning East and North metres."""
    semi_major = 6_378_137.0
    eccentricity_sq = 6.69437999014e-3
    origin_lat = math.radians(origin_latitude_deg)
    delta_lat = math.radians(latitude_deg - origin_latitude_deg)
    delta_lon = math.radians(longitude_deg - origin_longitude_deg)
    denominator = math.sqrt(1.0 - eccentricity_sq * math.sin(origin_lat) ** 2)
    prime_vertical_radius = semi_major / denominator
    meridional_radius = (
        semi_major * (1.0 - eccentricity_sq) / denominator ** 3)
    east = delta_lon * prime_vertical_radius * math.cos(origin_lat)
    north = delta_lat * meridional_radius
    return east, north


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6_371_000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2)
        * math.sin(delta_lambda / 2.0) ** 2
    )
    return radius * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def unique_run_directory(output_root: Path) -> tuple[str, Path]:
    output_root.mkdir(parents=True, exist_ok=True)
    base = datetime.now().astimezone().strftime('run_%Y-%m-%d_%H-%M-%S')
    candidate = output_root / base
    suffix = 1
    while candidate.exists():
        candidate = output_root / f'{base}_{suffix:02d}'
        suffix += 1
    candidate.mkdir()
    return candidate.name, candidate


def git_revision(path: Path) -> Optional[str]:
    try:
        return subprocess.check_output(
            ['git', '-C', str(path), 'rev-parse', 'HEAD'],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        ).strip()
    except (OSError, subprocess.SubprocessError):
        return None


class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('mech_workshop_experiment_logger')

        self.declare_parameter('vehicle_id', 1)
        self.declare_parameter('output_directory', str(DEFAULT_OUTPUT_ROOT))
        self.declare_parameter('notes', '')
        self.declare_parameter('gps_topic', '/ap/v1/navsat')
        self.declare_parameter('local_position_topic', '/ap/v1/pose/filtered')
        self.declare_parameter('local_velocity_topic', '/ap/v1/twist/filtered')
        self.declare_parameter('status_topic', '/ap/v1/status')
        self.declare_parameter('goal_topic', '/ap/v1/goal_lla')
        self.declare_parameter('source_clock_topic', '/ap/v1/clock')
        self.declare_parameter('relative_altitude_topic', '/uav1/rel_alt')
        self.declare_parameter('mode_topic', '/uav1/mode')
        self.declare_parameter('armed_topic', '/uav1/armed')
        self.declare_parameter('goto_topic', '/uav1/goto')
        self.declare_parameter(
            'global_origin_topic', '/ap/v1/gps_global_origin/filtered')
        self.declare_parameter('network_snr_file', str(DEFAULT_SNR_FILE))
        self.declare_parameter('origin_latitude_deg', DEFAULT_ORIGIN_LAT)
        self.declare_parameter('origin_longitude_deg', DEFAULT_ORIGIN_LON)
        self.declare_parameter('gcs_east_m', -2.0)
        self.declare_parameter('gcs_north_m', 0.0)
        self.declare_parameter('gcs_up_m', 2.9)
        self.declare_parameter('arrival_radius_m', 0.75)
        self.declare_parameter('telemetry_gap_threshold_s', 0.75)

        self.vehicle_id = int(self.get_parameter('vehicle_id').value)
        output_root = Path(
            os.path.expanduser(self.get_parameter('output_directory').value)
        ).resolve()
        self.run_id, self.run_dir = unique_run_directory(output_root)
        self.start_monotonic = time.monotonic()
        self.start_unix = time.time()
        self.start_datetime = datetime.now(timezone.utc)
        self.stop_event = threading.Event()
        self.state_lock = threading.Lock()

        self.origin_lat = float(self.get_parameter('origin_latitude_deg').value)
        self.origin_lon = float(self.get_parameter('origin_longitude_deg').value)
        self.origin_alt_msl = None
        self.origin_source = 'configured mech-workshop launch pad/world origin'
        self.gcs_enu = (
            float(self.get_parameter('gcs_east_m').value),
            float(self.get_parameter('gcs_north_m').value),
            float(self.get_parameter('gcs_up_m').value),
        )
        self.arrival_radius_m = float(
            self.get_parameter('arrival_radius_m').value)
        self.gap_threshold_s = float(
            self.get_parameter('telemetry_gap_threshold_s').value)
        self.snr_path = Path(os.path.expanduser(
            self.get_parameter('network_snr_file').value))

        self.topics = {
            'gps': self.get_parameter('gps_topic').value,
            'local_position': self.get_parameter('local_position_topic').value,
            'local_velocity': self.get_parameter('local_velocity_topic').value,
            'status': self.get_parameter('status_topic').value,
            'goal': self.get_parameter('goal_topic').value,
            'source_clock': self.get_parameter('source_clock_topic').value,
            'relative_altitude': self.get_parameter('relative_altitude_topic').value,
            'mode': self.get_parameter('mode_topic').value,
            'armed': self.get_parameter('armed_topic').value,
            'goto_command': self.get_parameter('goto_topic').value,
            'global_origin': self.get_parameter('global_origin_topic').value,
        }

        self.logs = {
            'gps': CsvLog(self.run_dir / 'gps.csv', GPS_COLUMNS),
            'vehicle_state': CsvLog(
                self.run_dir / 'vehicle_state.csv', VEHICLE_COLUMNS),
            'mission': CsvLog(self.run_dir / 'mission.csv', MISSION_COLUMNS),
            'telemetry_timing': CsvLog(
                self.run_dir / 'telemetry_timing.csv', TELEMETRY_COLUMNS),
            'network': CsvLog(self.run_dir / 'network.csv', NETWORK_COLUMNS),
            'events': CsvLog(self.run_dir / 'events.csv', EVENT_COLUMNS),
        }

        self.source_clock_s = None
        self.latest_gps = None
        self.latest_pose = None
        self.latest_pose_monotonic = None
        self.latest_velocity = None
        self.relative_altitude_m = None
        self.flight_mode = None
        self.armed = None
        self.flying = None
        self.numeric_mode = None
        self.command_target = None
        self.ap_goal = None
        self.target_revision = 0
        self.target_reached = False
        self.airborne = False
        self.telemetry_last_monotonic = None
        self.telemetry_first_monotonic = None
        self.telemetry_arrivals = []
        self.warned_topics = set()
        self._finished = False

        self._subscriptions = []
        self._subscribe_all()
        self.create_timer(1.0, self._flush_logs)
        self.create_timer(5.0, self._check_publishers_once)

        self.network_thread = threading.Thread(
            target=self._tail_snr_file,
            name='ns3-snr-tail',
            daemon=True,
        )
        self.network_thread.start()

        self.metadata = self._build_metadata()
        self._write_metadata()
        self._event(
            'LOGGER_STARTED',
            'Passive experiment logger started',
            self.run_id,
        )
        self.get_logger().info(f'Run directory: {self.run_dir}')
        self.get_logger().info(
            'Passive mode: subscriptions and file reads only; no publishers, '
            'services, or MAVLink connections')

    def _subscribe_all(self) -> None:
        add = self._subscriptions.append
        add(self.create_subscription(
            NavSatFix, self.topics['gps'], self._on_gps, BEST_EFFORT_QOS))
        add(self.create_subscription(
            PoseStamped, self.topics['local_position'],
            self._on_pose, BEST_EFFORT_QOS))
        add(self.create_subscription(
            TwistStamped, self.topics['local_velocity'],
            self._on_velocity, BEST_EFFORT_QOS))
        add(self.create_subscription(
            Status, self.topics['status'], self._on_status, TRANSIENT_QOS))
        add(self.create_subscription(
            GeoPointStamped, self.topics['goal'], self._on_ap_goal,
            TRANSIENT_QOS))
        add(self.create_subscription(
            Clock, self.topics['source_clock'], self._on_source_clock,
            BEST_EFFORT_QOS))
        add(self.create_subscription(
            Float32, self.topics['relative_altitude'],
            self._on_relative_altitude, BEST_EFFORT_QOS))
        add(self.create_subscription(
            String, self.topics['mode'], self._on_mode, BEST_EFFORT_QOS))
        add(self.create_subscription(
            Bool, self.topics['armed'], self._on_armed, BEST_EFFORT_QOS))
        add(self.create_subscription(
            GeoPoint, self.topics['goto_command'], self._on_goto,
            BEST_EFFORT_QOS))
        add(self.create_subscription(
            GeoPointStamped, self.topics['global_origin'], self._on_origin,
            BEST_EFFORT_QOS))

    def _now(self):
        monotonic_now = time.monotonic()
        unix_now = time.time()
        return (
            monotonic_now,
            monotonic_now - self.start_monotonic,
            datetime.fromtimestamp(unix_now, timezone.utc).isoformat(
                timespec='microseconds'),
            unix_now,
        )

    def _common(self, source_timestamp_s=None):
        _, elapsed, wall_iso, wall_unix = self._now()
        return {
            'elapsed_time_s': fmt(elapsed, 6),
            'wall_time': wall_iso,
            'wall_time_unix_s': fmt(wall_unix, 9),
            'source_timestamp_s': fmt(source_timestamp_s, 9),
            'source_clock_s': fmt(self.source_clock_s, 9),
        }

    def _event(self, event_type: str, description: str, value: Any = ''):
        row = self._common()
        row.pop('source_timestamp_s')
        row.pop('source_clock_s')
        row.update({
            'event_type': event_type,
            'description': description,
            'value': value,
        })
        self.logs['events'].write(row)

    def _on_source_clock(self, msg: Clock) -> None:
        self.source_clock_s = stamp_to_seconds(msg.clock)

    def _on_origin(self, msg: GeoPointStamped) -> None:
        position = msg.position
        if not (-90.0 <= position.latitude <= 90.0):
            return
        if not (-180.0 <= position.longitude <= 180.0):
            return
        if abs(position.latitude) < 1e-12 and abs(position.longitude) < 1e-12:
            # An uninitialised origin must not replace the configured launch
            # pad and shift all later GPS-derived local coordinates.
            return
        self.origin_lat = float(position.latitude)
        self.origin_lon = float(position.longitude)
        self.origin_alt_msl = float(position.altitude)
        self.origin_source = self.topics['global_origin']

    def _on_velocity(self, msg: TwistStamped) -> None:
        linear = msg.twist.linear
        with self.state_lock:
            self.latest_velocity = (
                float(linear.x), float(linear.y), float(linear.z),
                stamp_to_seconds(msg.header.stamp),
            )

    def _on_pose(self, msg: PoseStamped) -> None:
        position = msg.pose.position
        orientation = msg.pose.orientation
        source_timestamp = stamp_to_seconds(msg.header.stamp)
        qx = float(orientation.x)
        qy = float(orientation.y)
        qz = float(orientation.z)
        qw = float(orientation.w)
        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        heading = enu_yaw_to_heading(yaw)

        pose = {
            'x': float(position.x),
            'y': float(position.y),
            'z': float(position.z),
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'roll': roll, 'pitch': pitch, 'yaw': yaw, 'heading': heading,
            'source_timestamp': source_timestamp,
        }
        with self.state_lock:
            self.latest_pose = pose
            self.latest_pose_monotonic = time.monotonic()
            velocity = self.latest_velocity

        vx = velocity[0] if velocity else None
        vy = velocity[1] if velocity else None
        vz = velocity[2] if velocity else None
        ground_speed = math.hypot(vx, vy) if vx is not None and vy is not None else None

        row = self._common(source_timestamp)
        row.update({
            'local_x_east_m': fmt(pose['x']),
            'local_y_north_m': fmt(pose['y']),
            'local_z_up_m': fmt(pose['z']),
            'roll_deg': fmt(roll, 6),
            'pitch_deg': fmt(pitch, 6),
            'yaw_enu_deg': fmt(yaw, 6),
            'heading_deg': fmt(heading, 6),
            'qx': fmt(qx), 'qy': fmt(qy), 'qz': fmt(qz), 'qw': fmt(qw),
            'vx_east_mps': fmt(vx),
            'vy_north_mps': fmt(vy),
            'vz_up_mps': fmt(vz),
            'ground_speed_mps': fmt(ground_speed),
            'vertical_speed_up_mps': fmt(vz),
            'relative_altitude_m': fmt(self.relative_altitude_m),
            'armed': '' if self.armed is None else self.armed,
            'flight_mode': self.flight_mode or '',
            'flying': '' if self.flying is None else self.flying,
        })
        self.logs['vehicle_state'].write(row)

    def _on_gps(self, msg: NavSatFix) -> None:
        source_timestamp = stamp_to_seconds(msg.header.stamp)
        latitude = float(msg.latitude)
        longitude = float(msg.longitude)
        altitude = float(msg.altitude)
        gps_east, gps_north = geodetic_to_local_enu(
            latitude, longitude, self.origin_lat, self.origin_lon)

        with self.state_lock:
            pose = dict(self.latest_pose) if self.latest_pose else None
            velocity = self.latest_velocity
            self.latest_gps = (latitude, longitude, altitude, source_timestamp)

        vx = velocity[0] if velocity else None
        vy = velocity[1] if velocity else None
        vz = velocity[2] if velocity else None
        ground_speed = math.hypot(vx, vy) if vx is not None and vy is not None else None

        row = self._common(source_timestamp)
        row.update({
            'latitude_deg': fmt(latitude),
            'longitude_deg': fmt(longitude),
            'altitude_msl_m': fmt(altitude),
            'relative_altitude_m': fmt(self.relative_altitude_m),
            'gps_local_east_m': fmt(gps_east),
            'gps_local_north_m': fmt(gps_north),
            'local_x_east_m': fmt(pose['x'] if pose else None),
            'local_y_north_m': fmt(pose['y'] if pose else None),
            'local_z_up_m': fmt(pose['z'] if pose else None),
            'ground_speed_mps': fmt(ground_speed),
            'vertical_speed_up_mps': fmt(vz),
            'heading_deg': fmt(pose['heading'] if pose else None, 6),
            'source_topic': self.topics['gps'],
        })
        self.logs['gps'].write(row)

        self._record_mission('gps_sample', source_timestamp)
        self._check_waypoint_reached(latitude, longitude)

    def _record_telemetry_arrival(
        self,
        callback_monotonic: float,
        source_timestamp: Optional[float],
        message_type: str,
        topic: str,
    ) -> None:
        interarrival_ms = None
        if self.telemetry_last_monotonic is not None:
            interarrival_ms = (
                callback_monotonic - self.telemetry_last_monotonic) * 1000.0
            self.telemetry_arrivals.append(interarrival_ms)
            if interarrival_ms > self.gap_threshold_s * 1000.0:
                self._event(
                    'TELEMETRY_GAP',
                    'Received GLOBAL_POSITION_INT inter-arrival exceeded '
                    'threshold; this is not labelled packet loss',
                    f'{interarrival_ms:.3f} ms',
                )
        else:
            self.telemetry_first_monotonic = callback_monotonic
        self.telemetry_last_monotonic = callback_monotonic

        row = self._common(source_timestamp)
        row.update({
            'message_type': message_type,
            'topic': topic,
            # ROS 2 does not expose the original MAVLink packet header.
            'sequence_number': '',
            'sysid': self.vehicle_id,
            'compid': '',
            'interarrival_ms': fmt(interarrival_ms, 3),
        })
        self.logs['telemetry_timing'].write(row)

    def _on_relative_altitude(self, msg: Float32) -> None:
        callback_monotonic = time.monotonic()
        altitude = float(msg.data)
        self.relative_altitude_m = altitude
        # drone_bridge publishes exactly one Float32 for every received
        # MAVLink GLOBAL_POSITION_INT, making this the closest non-invasive
        # receiving-side continuity stream available to the logger.
        self._record_telemetry_arrival(
            callback_monotonic,
            None,
            'MAVLINK_GLOBAL_POSITION_INT_BRIDGE_REL_ALT',
            self.topics['relative_altitude'],
        )
        if (
            not self.airborne
            and self.armed is True
            and altitude >= 0.5
        ):
            self.airborne = True
            self._event(
                'TAKEOFF',
                'Relative altitude crossed 0.5 m while armed',
                f'{altitude:.3f} m',
            )
        elif (
            self.airborne
            and self.armed is False
            and altitude <= 0.2
        ):
            self.airborne = False
            self._event(
                'LANDED',
                'Vehicle disarmed below 0.2 m relative altitude',
                f'{altitude:.3f} m',
            )

    def _set_armed(self, value: bool, source: str) -> None:
        previous = self.armed
        self.armed = bool(value)
        if previous is None:
            return
        if previous != self.armed:
            self._event(
                'ARMED' if self.armed else 'DISARMED',
                f'Armed state changed on {source}',
                str(self.armed),
            )

    def _on_armed(self, msg: Bool) -> None:
        self._set_armed(bool(msg.data), self.topics['armed'])

    def _on_status(self, msg: Status) -> None:
        self.flying = bool(msg.flying)
        self.numeric_mode = int(msg.mode)
        self._set_armed(bool(msg.armed), self.topics['status'])

    def _on_mode(self, msg: String) -> None:
        mode = str(msg.data)
        previous = self.flight_mode
        self.flight_mode = mode
        if previous is None:
            self._event('MODE_OBSERVED', 'Initial flight mode observed', mode)
            return
        if mode == previous:
            return
        self._event('MODE_CHANGED', f'{previous} -> {mode}', mode)
        upper = mode.upper()
        if upper == 'LAND':
            self._event('LANDING', 'Vehicle entered LAND mode', mode)
        elif upper == 'RTL':
            self._event('RTL', 'Vehicle entered RTL mode', mode)

    def _on_goto(self, msg: GeoPoint) -> None:
        target = (
            float(msg.latitude), float(msg.longitude), float(msg.altitude),
            'relative_to_home', self.topics['goto_command'],
        )
        changed = (
            self.command_target is None
            or any(
                abs(target[index] - self.command_target[index]) > 1e-8
                for index in range(3)
            )
        )
        self.command_target = target
        if changed:
            self.target_revision += 1
            self.target_reached = False
            self._event(
                'WAYPOINT_CHANGED',
                'New GUIDED geographic target received',
                f'{target[0]:.7f},{target[1]:.7f},{target[2]:.3f}',
            )
        self._record_mission('goto_command')

    def _on_ap_goal(self, msg: GeoPointStamped) -> None:
        p = msg.position
        self.ap_goal = (
            float(p.latitude), float(p.longitude), float(p.altitude),
            stamp_to_seconds(msg.header.stamp),
        )
        self._record_mission('ardupilot_goal', self.ap_goal[3])

    def _mission_state(self) -> str:
        mode = (self.flight_mode or '').upper()
        if mode == 'LAND':
            return 'LANDING'
        if self.armed is False:
            return 'DISARMED'
        if self.command_target is not None and not self.target_reached:
            return 'GUIDED_TO_TARGET'
        if self.command_target is not None and self.target_reached:
            return 'AT_TARGET'
        if self.flying:
            return 'FLYING_NO_GUIDED_TARGET'
        if self.armed:
            return 'ARMED'
        return 'UNKNOWN'

    def _record_mission(
        self, reason: str, source_timestamp: Optional[float] = None
    ) -> None:
        target = self.command_target
        current = self.latest_gps
        target_east = target_north = distance = None
        if target is not None:
            target_east, target_north = geodetic_to_local_enu(
                target[0], target[1], self.origin_lat, self.origin_lon)
        if target is not None and current is not None:
            distance = haversine_m(current[0], current[1], target[0], target[1])

        row = self._common(source_timestamp)
        row.update({
            'record_reason': reason,
            # This mission uses GUIDED targets, not an uploaded waypoint list.
            'current_waypoint': '',
            'guided_target_revision': self.target_revision,
            'mission_state': self._mission_state(),
            'target_source': target[4] if target else '',
            'target_latitude_deg': fmt(target[0] if target else None),
            'target_longitude_deg': fmt(target[1] if target else None),
            'target_altitude_m': fmt(target[2] if target else None),
            'target_altitude_reference': target[3] if target else '',
            'target_x_east_m': fmt(target_east),
            'target_y_north_m': fmt(target_north),
            'distance_to_target_m': fmt(distance),
            'current_latitude_deg': fmt(current[0] if current else None),
            'current_longitude_deg': fmt(current[1] if current else None),
            'ap_goal_latitude_deg': fmt(self.ap_goal[0] if self.ap_goal else None),
            'ap_goal_longitude_deg': fmt(self.ap_goal[1] if self.ap_goal else None),
            'ap_goal_altitude_msl_m': fmt(self.ap_goal[2] if self.ap_goal else None),
        })
        self.logs['mission'].write(row)

    def _check_waypoint_reached(self, latitude: float, longitude: float) -> None:
        if self.command_target is None or self.target_reached:
            return
        distance = haversine_m(
            latitude, longitude,
            self.command_target[0], self.command_target[1],
        )
        if distance <= self.arrival_radius_m:
            self.target_reached = True
            self._event(
                'WAYPOINT_REACHED',
                'Measured GPS entered configured arrival radius',
                f'{distance:.3f} m',
            )
            self._record_mission('waypoint_reached')

    def _network_distance(self):
        with self.state_lock:
            pose = dict(self.latest_pose) if self.latest_pose else None
            pose_time = self.latest_pose_monotonic
        if pose is None or pose_time is None:
            return None, None, None
        dx = pose['x'] - self.gcs_enu[0]
        dy = pose['y'] - self.gcs_enu[1]
        dz = pose['z'] - self.gcs_enu[2]
        return (
            math.hypot(dx, dy),
            math.sqrt(dx * dx + dy * dy + dz * dz),
            time.monotonic() - pose_time,
        )

    def _tail_snr_file(self) -> None:
        source = None
        inode = None
        first_open = True
        existed_at_start = self.snr_path.exists()
        warned_missing = False

        while not self.stop_event.is_set():
            if source is None:
                try:
                    source = self.snr_path.open(
                        'r', newline='', encoding='utf-8')
                    inode = os.fstat(source.fileno()).st_ino
                    if first_open and existed_at_start:
                        source.seek(0, os.SEEK_END)
                    first_open = False
                except FileNotFoundError:
                    if not warned_missing:
                        self.get_logger().warn(
                            f'Network source not present yet: {self.snr_path}; '
                            'network.csv will remain valid and may be empty')
                        warned_missing = True
                    self.stop_event.wait(0.05)
                    continue

            try:
                path_stat = self.snr_path.stat()
            except FileNotFoundError:
                source.close()
                source = None
                inode = None
                self.stop_event.wait(0.05)
                continue

            if path_stat.st_ino != inode:
                source.close()
                source = self.snr_path.open(
                    'r', newline='', encoding='utf-8')
                inode = os.fstat(source.fileno()).st_ino
                continue
            if path_stat.st_size < source.tell():
                source.seek(0)

            line_start = source.tell()
            line = source.readline()
            if not line:
                self.stop_event.wait(0.05)
                continue
            if not line.endswith('\n'):
                source.seek(line_start)
                self.stop_event.wait(0.05)
                continue

            _, elapsed, wall_iso, wall_unix = self._now()
            try:
                fields = [value.strip() for value in next(csv.reader([line]))]
            except csv.Error:
                continue
            if fields == SNR_SOURCE_HEADER:
                continue
            if len(fields) != len(SNR_SOURCE_HEADER):
                continue

            horizontal_distance, distance, position_age = self._network_distance()
            row = {
                'elapsed_time_s': fmt(elapsed, 6),
                'wall_time': wall_iso,
                'wall_time_unix_s': fmt(wall_unix, 9),
                'ns3_time_s': fields[0],
                'rx_node': fields[1],
                'freq_mhz': fields[2],
                'pkt_bytes': fields[3],
                'signal_dbm': fields[4],
                'noise_dbm': fields[5],
                'snr_db': fields[6],
                'uav_id': self.vehicle_id,
                'peer_id': '',
                'horizontal_distance_to_gcs_m': fmt(horizontal_distance),
                'distance_to_gcs_m': fmt(distance),
                'position_age_s': fmt(position_age, 6),
                'distance_source': (
                    f'{self.topics["local_position"]} + configured GCS ENU'
                    if distance is not None else ''
                ),
            }
            self.logs['network'].write(row)

        if source is not None:
            source.close()

    def _flush_logs(self) -> None:
        for log in self.logs.values():
            log.flush()

    def _check_publishers_once(self) -> None:
        for key, topic in self.topics.items():
            if key in self.warned_topics:
                continue
            if self.count_publishers(topic) == 0:
                self.get_logger().warn(
                    f'No publisher currently discovered for optional input '
                    f'{key}: {topic}')
                self.warned_topics.add(key)

    def _available_topics(self):
        try:
            return [
                {'name': name, 'types': types}
                for name, types in sorted(self.get_topic_names_and_types())
            ]
        except Exception:
            return []

    def _build_metadata(self) -> dict[str, Any]:
        ardupilot_root = Path.home() / 'ardu_ws' / 'src' / 'ardupilot'
        return {
            'run_id': self.run_id,
            'start_datetime': self.start_datetime.isoformat(
                timespec='microseconds'),
            'end_datetime': None,
            'start_wall_time_unix_s': self.start_unix,
            'end_wall_time_unix_s': None,
            'duration_s': None,
            'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
            'vehicle_id': self.vehicle_id,
            'sitl_system_id': self.vehicle_id,
            'sitl_system_id_source': (
                'vehicle_id; launcher starts ArduCopter with --sysid 1'),
            'simulation': True,
            'notes': self.get_parameter('notes').value,
            'available_topics': self._available_topics(),
            'selected_topics': self.topics,
            'network_source_file': str(self.snr_path),
            'coordinate_frames': {
                'local_pose_and_velocity': (
                    'ROS REP-103 ENU: X East, Y North, Z Up'),
                'quaternion_and_euler': (
                    'ROS ENU; yaw is counter-clockwise from East'),
                'heading_deg': 'clockwise from North, range [0, 360)',
                'gps_derived_xy': (
                    'WGS-84 local tangent approximation about recorded origin'),
            },
            'gps_local_origin': {
                'latitude_deg': self.origin_lat,
                'longitude_deg': self.origin_lon,
                'altitude_msl_m': self.origin_alt_msl,
                'source': self.origin_source,
            },
            'gcs_position_enu_m': {
                'east': self.gcs_enu[0],
                'north': self.gcs_enu[1],
                'up_antenna': self.gcs_enu[2],
                'source': 'mech_workshop_validation.world defaults/parameters',
            },
            'software': {
                'python': sys.version.split()[0],
                'platform': platform.platform(),
                'logger_file': str(Path(__file__).resolve()),
                'repository_git_commit': git_revision(REPO_ROOT),
                'ardupilot_git_commit': git_revision(ardupilot_root),
            },
            'measurement_notes': {
                'telemetry_timing': (
                    'One row per /uav1/rel_alt publication. drone_bridge emits '
                    'one such message for each received MAVLink '
                    'GLOBAL_POSITION_INT. Inter-arrival uses monotonic callback '
                    'arrival time at the GCS side.'),
                'sequence_numbers': (
                    'Unavailable: drone_bridge does not republish the MAVLink '
                    'sequence, component ID, or time_boot_ms with /uav1/rel_alt. '
                    'No second MAVLink client is opened because it could '
                    'interfere with drone_bridge.'),
                'sim_time': (
                    'No trustworthy Gazebo /clock is visible inside gcsns. '
                    'source_clock_s records /ap/v1/clock separately; ns3_time_s '
                    'records ns-3 internal time and neither is treated as wall time.'),
                'network': (
                    'Raw delivered 802.11 PHY receive rows tailed passively. '
                    'No packet-loss, RTT, throughput, or transmitter identity '
                    'is fabricated. File-writer buffering may delay wall-time '
                    'observation of a group of ns-3 rows.'),
                'mission': (
                    'This is a GUIDED mission, not an uploaded MAVLink waypoint '
                    'mission; current_waypoint is therefore intentionally blank.'),
            },
            'sample_counts': {},
            'telemetry_summary': {},
        }

    def _write_metadata(self) -> None:
        self.metadata['available_topics'] = self._available_topics()
        self.metadata['gps_local_origin'] = {
            'latitude_deg': self.origin_lat,
            'longitude_deg': self.origin_lon,
            'altitude_msl_m': self.origin_alt_msl,
            'source': self.origin_source,
        }
        path = self.run_dir / 'metadata.json'
        path.write_text(
            json.dumps(self.metadata, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )

    def telemetry_summary(self) -> dict[str, Any]:
        count = self.logs['telemetry_timing'].count
        summary: dict[str, Any] = {
            'messages_received': count,
            'sequence_loss_estimate': None,
            'sequence_loss_note': (
                'Unavailable; drone_bridge does not republish the MAVLink '
                'sequence number'),
        }
        if not self.telemetry_arrivals:
            return summary

        gaps = self.telemetry_arrivals
        sorted_gaps = sorted(gaps)
        p95_index = max(0, math.ceil(0.95 * len(sorted_gaps)) - 1)
        duration = (
            self.telemetry_last_monotonic - self.telemetry_first_monotonic
            if self.telemetry_first_monotonic is not None
            and self.telemetry_last_monotonic is not None else 0.0
        )
        summary.update({
            'observed_span_s': duration,
            'mean_rate_hz': ((count - 1) / duration if duration > 0 else None),
            'mean_interarrival_ms': statistics.fmean(gaps),
            'median_interarrival_ms': statistics.median(gaps),
            'p95_interarrival_ms': sorted_gaps[p95_index],
            'maximum_gap_ms': max(gaps),
            'gaps_over_750_ms': sum(gap > 750.0 for gap in gaps),
            'gaps_over_1000_ms': sum(gap > 1000.0 for gap in gaps),
            'gaps_over_1500_ms': sum(gap > 1500.0 for gap in gaps),
        })
        return summary

    def finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        self._event('LOGGER_STOPPED', 'Logger stopped by user or ROS shutdown')
        self.stop_event.set()
        self.network_thread.join(timeout=2.0)
        self._flush_logs()

        end_unix = time.time()
        duration = time.monotonic() - self.start_monotonic
        summary = self.telemetry_summary()
        self.metadata.update({
            'end_datetime': datetime.now(timezone.utc).isoformat(
                timespec='microseconds'),
            'end_wall_time_unix_s': end_unix,
            'duration_s': duration,
            'sample_counts': {
                name: log.count for name, log in self.logs.items()
            },
            'telemetry_summary': summary,
        })
        self._write_metadata()
        for log in self.logs.values():
            log.close()

        print('\n========================================')
        print('Experiment logging completed')
        print('========================================')
        print(f'Run directory: {self.run_dir}')
        print(f'Duration: {duration:.2f} s\n')
        print(f'GPS samples:            {self.logs["gps"].count:7d}')
        print(f'Vehicle-state samples:  {self.logs["vehicle_state"].count:7d}')
        print(f'Mission records:        {self.logs["mission"].count:7d}')
        print(f'Telemetry messages:     {self.logs["telemetry_timing"].count:7d}')
        print(f'Network samples:        {self.logs["network"].count:7d}')
        print(f'Events:                 {self.logs["events"].count:7d}')
        if summary.get('mean_rate_hz') is not None:
            print('\nTelemetry (received GLOBAL_POSITION_INT arrivals):')
            print(f'Mean rate:             {summary["mean_rate_hz"]:8.2f} Hz')
            print(f'Median gap:            {summary["median_interarrival_ms"]:8.1f} ms')
            print(f'P95 gap:               {summary["p95_interarrival_ms"]:8.1f} ms')
            print(f'Maximum gap:           {summary["maximum_gap_ms"]:8.1f} ms')
            print(f'Gaps > 1 s:            {summary["gaps_over_1000_ms"]:8d}')
        print('========================================')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
