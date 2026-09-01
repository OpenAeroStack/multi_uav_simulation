"""
drone_bridge.py
---------------
MAVLink (commands) + AP_DDS (telemetry) bridge for one UAV.

Telemetry (from DDS → republished on /uavN):
    /uav{N}/gps        sensor_msgs/NavSatFix
    /uav{N}/rel_alt    std_msgs/Float32
    /uav{N}/battery    std_msgs/Float32
    /uav{N}/mode       std_msgs/String
    /uav{N}/armed      std_msgs/Bool

Services (MAVLink commands):
    /uav{N}/arm        std_srvs/Trigger
    /uav{N}/disarm     std_srvs/Trigger
    /uav{N}/takeoff    std_srvs/Trigger
    /uav{N}/land       std_srvs/Trigger
    /uav{N}/rtl        std_srvs/Trigger

Topics subscribed (for waypoint commands from mission nodes):
    /uav{N}/goto       geographic_msgs/GeoPoint  (lat, lon, alt)
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import NavSatFix, BatteryState
from geographic_msgs.msg import GeoPoint
from std_srvs.srv import Trigger
from pymavlink import mavutil


AP_DDS_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class DroneBridge(Node):

    def __init__(self):
        super().__init__('drone_bridge')

        # ── parameters ────────────────────────────────────────────────────────
        self.declare_parameter('uav_id',            1)
        self.declare_parameter('mavlink_host',      '127.0.0.1')
        self.declare_parameter('mavlink_port',      5760)
        self.declare_parameter('takeoff_altitude',  10.0)

        self.uav_id = self.get_parameter('uav_id').value
        host        = self.get_parameter('mavlink_host').value
        port        = self.get_parameter('mavlink_port').value
        self.alt    = self.get_parameter('takeoff_altitude').value
        ns          = f'/uav{self.uav_id}'
        ap_ns       = f'/ap/v{self.uav_id}'
        self.ap_navsat_topic = f'{ap_ns}/navsat'
        self.ap_battery_topic = f'{ap_ns}/battery'

        # ── internal state ────────────────────────────────────────────────────
        self.lat     = 0.0
        self.lon     = 0.0
        self.alt_msl = 0.0
        self.rel_alt = 0.0
        self.battery = 0.0
        self.gps_ok  = False
        self.armed   = False
        self.mode    = 'UNKNOWN'

        # ── MAVLink connection ────────────────────────────────────────────────
        self.get_logger().info(
            f'[UAV{self.uav_id}] Connecting MAVLink → tcp:{host}:{port}')
        self.mav = mavutil.mavlink_connection(
            f'tcp:{host}:{port}', source_system=255)
        self.mav.wait_heartbeat()
        self.get_logger().info(f'[UAV{self.uav_id}] MAVLink heartbeat OK')

        # Keep the existing basic-status stream, but configure position output
        # per message so LOCAL_POSITION_NED and other telemetry are unaffected.
        self.mav.mav.request_data_stream_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)

        def request_message_interval(message_id, interval_us, label, rate_hz):
            command = mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
            try:
                self.mav.mav.command_long_send(
                    self.mav.target_system,
                    self.mav.target_component,
                    command,
                    0,
                    message_id,
                    interval_us,
                    0, 0, 0, 0, 0)
                ack = self.mav.recv_match(
                    type='COMMAND_ACK', blocking=True, timeout=2.0,
                    condition=f'COMMAND_ACK.command=={command}')
                if ack is None:
                    self.get_logger().warning(
                        f'[UAV{self.uav_id}] No acknowledgement for '
                        f'{label} {rate_hz:g} Hz request')
                    return
                if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.get_logger().warning(
                        f'[UAV{self.uav_id}] {label} {rate_hz:g} Hz request '
                        f'rejected (MAV_RESULT={ack.result})')
                    return
                if label == 'GLOBAL_POSITION_INT':
                    self.get_logger().info(
                        f'[UAV{self.uav_id}] Requested '
                        'GLOBAL_POSITION_INT at 2 Hz')
            except Exception as exc:
                self.get_logger().warning(
                    f'[UAV{self.uav_id}] Could not request '
                    f'{label} at {rate_hz:g} Hz: {exc}')

        request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            100_000, 'LOCAL_POSITION_NED', 10)
        request_message_interval(
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            500_000, 'GLOBAL_POSITION_INT', 2)

        # ── DDS subscribers (telemetry) ───────────────────────────────────────
        self.create_subscription(
            NavSatFix,    self.ap_navsat_topic,
            self._cb_navsat,  AP_DDS_QOS)
        self.create_subscription(
            BatteryState, self.ap_battery_topic,
            self._cb_battery, AP_DDS_QOS)

        # ── publishers ────────────────────────────────────────────────────────
        self.pub_gps     = self.create_publisher(NavSatFix, f'{ns}/gps',     10)
        self.pub_rel_alt = self.create_publisher(Float32,   f'{ns}/rel_alt', 10)
        self.pub_mode    = self.create_publisher(String,    f'{ns}/mode',    10)
        self.pub_armed   = self.create_publisher(Bool,      f'{ns}/armed',   10)
        self.pub_battery = self.create_publisher(Float32,   f'{ns}/battery', 10)

        # ── goto subscriber ───────────────────────────────────────────────────
        # Mission nodes publish here, bridge forwards via MAVLink
        self.create_subscription(
            GeoPoint, f'{ns}/goto',
            self._cb_goto, 10)

        # ── services ──────────────────────────────────────────────────────────
        self.create_service(Trigger, f'{ns}/arm',     self._srv_arm)
        self.create_service(Trigger, f'{ns}/disarm',  self._srv_disarm)
        self.create_service(Trigger, f'{ns}/takeoff', self._srv_takeoff)
        self.create_service(Trigger, f'{ns}/land',    self._srv_land)
        self.create_service(Trigger, f'{ns}/rtl',     self._srv_rtl)

        # ── MAVLink heartbeat thread ──────────────────────────────────────────
        self._stop = False
        threading.Thread(target=self._mav_loop, daemon=True).start()

        # ── GPS ready notifier ────────────────────────────────────────────────
        self._ready_logged = False
        self.create_timer(1.0, self._check_ready)

        self.get_logger().info(
            f'[UAV{self.uav_id}] Bridge ready'
            f' | waiting for DDS GPS from {self.ap_navsat_topic} ...')
        self.get_logger().info(
            f'[UAV{self.uav_id}] Services: '
            f'{ns}/arm  {ns}/takeoff  {ns}/land  {ns}/rtl')
        self.get_logger().info(
            f'[UAV{self.uav_id}] Goto topic: {ns}/goto  (geographic_msgs/GeoPoint)')

    # ── DDS callbacks ─────────────────────────────────────────────────────────

    def _cb_navsat(self, msg):
        self.lat     = msg.latitude
        self.lon     = msg.longitude
        self.alt_msl = msg.altitude
        self.gps_ok  = (msg.latitude != 0.0 or msg.longitude != 0.0)
        self.pub_gps.publish(msg)

    def _cb_battery(self, msg):
        self.battery = msg.percentage * 100.0
        bat_msg = Float32()
        bat_msg.data = float(self.battery)
        self.pub_battery.publish(bat_msg)

    # ── goto callback ─────────────────────────────────────────────────────────

    def _cb_goto(self, msg: GeoPoint):
        """
        Receives GeoPoint (lat, lon, alt) from mission nodes
        and sends SET_POSITION_TARGET_GLOBAL_INT via MAVLink.
        altitude field of GeoPoint is used as relative altitude.
        """
        self.get_logger().info(
            f'[UAV{self.uav_id}] Goto → '
            f'({msg.latitude:.6f}, {msg.longitude:.6f}, alt={msg.altitude:.1f}m)')
        self.mav.mav.set_position_target_global_int_send(
            0,
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b110111111000,   # position only
            int(msg.latitude  * 1e7),
            int(msg.longitude * 1e7),
            msg.altitude,
            0, 0, 0,
            0, 0, 0,
            0, 0)

    # ── MAVLink heartbeat loop ────────────────────────────────────────────────

    def _mav_loop(self):
        while not self._stop:
            msg = self.mav.recv_match(
                type=['HEARTBEAT', 'GLOBAL_POSITION_INT'],
                blocking=True, timeout=2.0)
            if msg is None:
                continue
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                self.rel_alt = msg.relative_alt / 1000.0  # mm → m
                alt_msg = Float32()
                alt_msg.data = float(self.rel_alt)
                self.pub_rel_alt.publish(alt_msg)
                continue
            # HEARTBEAT
            self.armed = bool(
                msg.base_mode &
                mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            try:
                self.mode = mavutil.mode_string_v10(msg)
            except Exception:
                self.mode = str(msg.custom_mode)

            mode_msg = String(); mode_msg.data = self.mode
            self.pub_mode.publish(mode_msg)
            armed_msg = Bool(); armed_msg.data = self.armed
            self.pub_armed.publish(armed_msg)

    # ── GPS ready notifier ────────────────────────────────────────────────────

    def _check_ready(self):
        if not self._ready_logged and self.gps_ok:
            self.get_logger().info(
                f'[UAV{self.uav_id}] ✓ DDS GPS flowing '
                f'({self.lat:.6f}, {self.lon:.6f})'
                f' — safe to call /uav{self.uav_id}/takeoff now')
            self._ready_logged = True

    # ── MAVLink command helpers ───────────────────────────────────────────────

    def _set_mode(self, mode_name):
        mode_id = self.mav.mode_mapping()[mode_name]
        self.mav.mav.set_mode_send(
            self.mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id)
        self.get_logger().info(f'[UAV{self.uav_id}] Mode → {mode_name}')

    def _send_arm(self, arm: bool):
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1.0 if arm else 0.0, 2989.0, 0, 0, 0, 0, 0)

    def _wait_for(self, condition_fn, timeout=30.0, interval=0.3):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if condition_fn():
                return True
            time.sleep(interval)
        return False

    # ── service handlers ──────────────────────────────────────────────────────

    def _srv_arm(self, req, res):
        self.get_logger().info(
            f'[UAV{self.uav_id}] Waiting for DDS GPS from '
            f'{self.ap_navsat_topic} ...')
        if not self._wait_for(lambda: self.gps_ok, timeout=50.0):
            res.success = False
            res.message = 'GPS not ready — is micro_ros_agent running?'
            self.get_logger().warn(res.message)
            return res
        self._set_mode('GUIDED')
        time.sleep(1.0)

        deadline   = time.time() + 120.0
        last_arm   = 0.0
        attempt    = 0
        while time.time() < deadline:
            if self.armed:
                res.success = True
                res.message = f'UAV{self.uav_id} armed successfully'
                self.get_logger().info(res.message)
                return res
            if time.time() - last_arm > 3.0:
                attempt += 1
                self.get_logger().info(
                    f'[UAV{self.uav_id}] Arm attempt {attempt}...')
                self._send_arm(True)
                last_arm = time.time()
            time.sleep(0.3)

        res.success = False
        res.message = 'Arm timed out — check PreArm messages in SITL console'
        self.get_logger().warn(res.message)
        return res

    def _srv_disarm(self, req, res):
        self._send_arm(False)
        res.success = True
        res.message = f'UAV{self.uav_id} disarmed'
        self.get_logger().info(res.message)
        return res

    def _srv_takeoff(self, req, res):
        if not self.armed:
            arm_res = self._srv_arm(req, Trigger.Response())
            if not arm_res.success:
                res.success = False
                res.message = arm_res.message
                return res
        self.get_logger().info(
            f'[UAV{self.uav_id}] Taking off to {self.alt}m...')
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, self.alt)
        target = self.alt * 0.85
        if self._wait_for(lambda: self.rel_alt >= target, timeout=120.0):
            res.success = True
            res.message = (f'UAV{self.uav_id} reached '
                           f'{self.rel_alt:.1f}m (target {self.alt}m)')
            self.get_logger().info(res.message)
        else:
            res.success = False
            res.message = f'Takeoff timed out — alt {self.rel_alt:.1f}m'
            self.get_logger().warn(res.message)
        return res

    def _srv_land(self, req, res):
        self._set_mode('LAND')
        res.success = True
        res.message = f'UAV{self.uav_id} landing'
        self.get_logger().info(res.message)
        return res

    def _srv_rtl(self, req, res):
        self._set_mode('RTL')
        res.success = True
        res.message = f'UAV{self.uav_id} returning to launch'
        self.get_logger().info(res.message)
        return res

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
