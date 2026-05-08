"""
drone_bridge.py
---------------
Connects to ArduPilot SITL via MAVLink TCP and exposes:

  Publishers  (10 Hz):
    /uav{N}/gps        sensor_msgs/NavSatFix
    /uav{N}/rel_alt    std_msgs/Float32
    /uav{N}/mode       std_msgs/String
    /uav{N}/armed      std_msgs/Bool
    /uav{N}/battery    std_msgs/Float32   (percent 0-100)

  Services:
    /uav{N}/arm        std_srvs/Trigger
    /uav{N}/disarm     std_srvs/Trigger
    /uav{N}/takeoff    std_srvs/Trigger   (arms first if needed)
    /uav{N}/land       std_srvs/Trigger
    /uav{N}/rtl        std_srvs/Trigger

Parameters:
    uav_id           (int,   default 1)
    mavlink_host     (str,   default '127.0.0.1')
    mavlink_port     (int,   default 5760)
    takeoff_altitude (float, default 10.0)

Usage:
    ros2 run uav_controller drone_bridge --ros-args \
        -p uav_id:=1 -p mavlink_port:=5760 -p takeoff_altitude:=10.0
"""

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
from pymavlink import mavutil


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

        # ── MAVLink connection ────────────────────────────────────────────────
        self.get_logger().info(
            f'[UAV{self.uav_id}] Connecting MAVLink → tcp:{host}:{port}')
        self.mav = mavutil.mavlink_connection(
            f'tcp:{host}:{port}', source_system=255)
        self.mav.wait_heartbeat()
        self.get_logger().info(f'[UAV{self.uav_id}] Heartbeat OK')
        self._request_streams()

        # ── internal state (updated by MAVLink thread) ────────────────────────
        self.lat      = 0.0
        self.lon      = 0.0
        self.alt_msl  = 0.0
        self.rel_alt  = 0.0
        self.mode     = 'UNKNOWN'
        self.armed    = False
        self.battery  = 0.0
        self.ekf_ok   = False
        self.gps_fix  = 0

        # ── publishers ────────────────────────────────────────────────────────
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

        # ── timers ────────────────────────────────────────────────────────────
        self.create_timer(0.1, self._publish_state)  # 10 Hz

        # ── MAVLink receive thread ────────────────────────────────────────────
        self._stop = False
        threading.Thread(target=self._mav_loop, daemon=True).start()

        self.get_logger().info(
            f'[UAV{self.uav_id}] Bridge ready  '
            f'| services: {ns}/arm  {ns}/takeoff  {ns}/land  {ns}/rtl')
        
        self.create_timer(1.0, self._check_ready)
        self._ready_logged = False
    
    


    # ── MAVLink helpers ───────────────────────────────────────────────────────
    
    def _check_ready(self):
        if not self._ready_logged and self.gps_fix >= 3:
            self.get_logger().info(
                f'[UAV{self.uav_id}] GPS ready (fix={self.gps_fix})'
                f' — safe to call /uav1/takeoff now')
            self._ready_logged = True
    
    def _request_streams(self):
        for stream_id, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_POSITION,       5),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,         5),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,         2),
            (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,2),
        ]:
            self.mav.mav.request_data_stream_send(
                self.mav.target_system, self.mav.target_component,
                stream_id, rate, 1)

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
        """Poll condition_fn() until True or timeout. Returns bool."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if condition_fn():
                return True
            time.sleep(interval)
        return False

    # ── MAVLink receive loop (runs in background thread) ──────────────────────

    def _mav_loop(self):
        while not self._stop:
            msg = self.mav.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            t = msg.get_type()

            if t == 'GLOBAL_POSITION_INT':
                self.lat     = msg.lat / 1e7
                self.lon     = msg.lon / 1e7
                self.alt_msl = msg.alt / 1000.0
                self.rel_alt = msg.relative_alt / 1000.0

            elif t == 'HEARTBEAT':
                self.armed = bool(
                    msg.base_mode &
                    mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    self.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    self.mode = str(msg.custom_mode)

            elif t == 'SYS_STATUS':
                self.battery = float(msg.battery_remaining)  # 0-100

            elif t == 'EKF_STATUS_REPORT':
                self.ekf_ok = (msg.flags & 0x1F) == 0x1F

            elif t == 'GPS_RAW_INT':
                self.gps_fix = msg.fix_type

    # ── state publisher (10 Hz timer) ─────────────────────────────────────────

    def _publish_state(self):
        now = self.get_clock().now().to_msg()

        gps = NavSatFix()
        gps.header.stamp = now
        gps.latitude     = self.lat
        gps.longitude    = self.lon
        gps.altitude     = self.alt_msl
        self.pub_gps.publish(gps)

        alt_msg = Float32(); alt_msg.data = float(self.rel_alt)
        self.pub_rel_alt.publish(alt_msg)

        mode_msg = String(); mode_msg.data = self.mode
        self.pub_mode.publish(mode_msg)

        armed_msg = Bool(); armed_msg.data = self.armed
        self.pub_armed.publish(armed_msg)

        bat_msg = Float32(); bat_msg.data = float(self.battery)
        self.pub_battery.publish(bat_msg)

    # ── service handlers ──────────────────────────────────────────────────────

    def _srv_arm(self, req, res):
        # Wait for EKF + GPS before arming
        self.get_logger().info(f'[UAV{self.uav_id}] Waiting for EKF + GPS...')
        if not self._wait_for(
                lambda: self.gps_fix >= 3, timeout=30.0):
            res.success = False
            res.message = 'EKF or GPS not ready — cannot arm'
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
            res.message = 'Arm timed out — check PreArm messages in MAVProxy'
            self.get_logger().warn(res.message)
        return res

    def _srv_disarm(self, req, res):
        self._send_arm(False)
        res.success = True
        res.message = f'UAV{self.uav_id} disarmed'
        self.get_logger().info(res.message)
        return res

    def _srv_takeoff(self, req, res):
        # Auto-arm if not already armed
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

        # Wait until 85% of target altitude reached
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
