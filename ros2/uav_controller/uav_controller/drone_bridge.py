"""
drone_bridge.py
---------------
MAVLink (commands) + AP_DDS (telemetry) bridge for one UAV.

                    ┌─────────────────┐
                    │  ArduPilot SITL │
                    └────────┬────────┘
                             │
              ┌──────────────┴──────────────┐
              │                             │
        MAVLink TCP                   DDS topics
        (commands only)             (telemetry only)
              │                             │
        arm, mode,               /ap/navsat
        takeoff, RTL             /ap/pose/filtered
                                 /ap/battery
              └──────────────┬──────────────┘
                             │
                      drone_bridge.py
                             │
                    ┌────────┴────────┐
                    │     ROS2        │
                    │  your code      │
                    └─────────────────┘

Telemetry (re-published from DDS onto /uavN namespace):
    /uav{N}/gps        sensor_msgs/NavSatFix   ← from /ap/navsat
    /uav{N}/rel_alt    std_msgs/Float32         ← from /ap/pose/filtered
    /uav{N}/battery    std_msgs/Float32         ← from /ap/battery
    /uav{N}/mode       std_msgs/String          ← from MAVLink heartbeat
    /uav{N}/armed      std_msgs/Bool            ← from MAVLink heartbeat

Commands (services → MAVLink):
    /uav{N}/arm        std_srvs/Trigger
    /uav{N}/disarm     std_srvs/Trigger
    /uav{N}/takeoff    std_srvs/Trigger
    /uav{N}/land       std_srvs/Trigger
    /uav{N}/rtl        std_srvs/Trigger

Parameters:
    uav_id           int    default 1
    mavlink_host     str    default '127.0.0.1'
    mavlink_port     int    default 5760
    takeoff_altitude float  default 10.0
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from pymavlink import mavutil


# QoS that matches what AP_DDS publishes with
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

        # ── internal state ────────────────────────────────────────────────────
        # Telemetry — filled by DDS callbacks
        self.lat     = 0.0
        self.lon     = 0.0
        self.alt_msl = 0.0
        self.rel_alt = 0.0
        self.battery = 0.0
        self.gps_ok  = False  # True once valid navsat arrives

        # Commands state — filled by MAVLink heartbeat
        self.armed = False
        self.mode  = 'UNKNOWN'

        # ── MAVLink connection (commands only) ────────────────────────────────
        self.get_logger().info(
            f'[UAV{self.uav_id}] Connecting MAVLink → tcp:{host}:{port}')
        self.mav = mavutil.mavlink_connection(
            f'tcp:{host}:{port}', source_system=255)
        self.mav.wait_heartbeat()
        self.get_logger().info(f'[UAV{self.uav_id}] MAVLink heartbeat OK')

        # ── DDS subscribers (telemetry) ───────────────────────────────────────
        self.create_subscription(
            NavSatFix, '/ap/navsat',
            self._cb_navsat, AP_DDS_QOS)

        self.create_subscription(
            PoseStamped, '/ap/pose/filtered',
            self._cb_pose, AP_DDS_QOS)

        self.create_subscription(
            BatteryState, '/ap/battery',
            self._cb_battery, AP_DDS_QOS)

        # ── publishers (re-publish on /uavN for mission code) ─────────────────
        self.pub_gps     = self.create_publisher(NavSatFix, f'{ns}/gps',     10)
        self.pub_rel_alt = self.create_publisher(Float32,   f'{ns}/rel_alt', 10)
        self.pub_mode    = self.create_publisher(String,    f'{ns}/mode',    10)
        self.pub_armed   = self.create_publisher(Bool,      f'{ns}/armed',   10)
        self.pub_battery = self.create_publisher(Float32,   f'{ns}/battery', 10)

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
            f' | waiting for DDS GPS from /ap/navsat ...')
        self.get_logger().info(
            f'[UAV{self.uav_id}] Services available: '
            f'{ns}/arm  {ns}/takeoff  {ns}/land  {ns}/rtl')

    # ── DDS callbacks ─────────────────────────────────────────────────────────

    def _cb_navsat(self, msg: NavSatFix):
        """GPS from /ap/navsat (DDS)."""
        self.lat     = msg.latitude
        self.lon     = msg.longitude
        self.alt_msl = msg.altitude
        self.gps_ok  = (msg.latitude != 0.0 or msg.longitude != 0.0)
        self.pub_gps.publish(msg)

    def _cb_pose(self, msg: PoseStamped):
        """Pose from /ap/pose/filtered (DDS). Z = altitude above home in m."""
        self.rel_alt = msg.pose.position.z
        alt_msg = Float32()
        alt_msg.data = float(self.rel_alt)
        self.pub_rel_alt.publish(alt_msg)

    def _cb_battery(self, msg: BatteryState):
        """Battery from /ap/battery (DDS). percentage is 0.0-1.0."""
        self.battery = msg.percentage * 100.0
        bat_msg = Float32()
        bat_msg.data = float(self.battery)
        self.pub_battery.publish(bat_msg)

    # ── MAVLink heartbeat loop (armed + mode only) ────────────────────────────

    def _mav_loop(self):
        """Reads only HEARTBEAT — everything else comes from DDS now."""
        while not self._stop:
            msg = self.mav.recv_match(
                type=['HEARTBEAT'], blocking=True, timeout=2.0)
            if msg is None:
                continue
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

    def _set_mode(self, mode_name: str):
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
            0, 1.0 if arm else 0.0, 0, 0, 0, 0, 0, 0)

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
            f'[UAV{self.uav_id}] Waiting for DDS GPS...')
        if not self._wait_for(lambda: self.gps_ok, timeout=30.0):
            res.success = False
            res.message = ('GPS not ready via DDS — '
                           'is micro_ros_agent running?')
            self.get_logger().warn(res.message)
            return res

        self._set_mode('GUIDED')
        time.sleep(0.5)
        self._send_arm(True)

        if self._wait_for(lambda: self.armed, timeout=20.0):
            res.success = True
            res.message = f'UAV{self.uav_id} armed successfully'
            self.get_logger().info(res.message)
        else:
            res.success = False
            res.message = 'Arm timed out — check PreArm messages'
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
        if self._wait_for(lambda: self.rel_alt >= target, timeout=30.0):
            res.success = True
            res.message = (f'UAV{self.uav_id} reached '
                           f'{self.rel_alt:.1f}m (target {self.alt}m)')
            self.get_logger().info(res.message)
        else:
            res.success = False
            res.message = (f'Takeoff timed out — '
                           f'current alt {self.rel_alt:.1f}m')
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