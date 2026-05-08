"""
takeoff_mission.py
------------------
Calls the drone_bridge services to arm and takeoff automatically.
drone_bridge MUST be running before this script.

Usage:
    ros2 run uav_controller takeoff_mission
    ros2 run uav_controller takeoff_mission --ros-args -p uav_id:=1
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float32, String, Bool


class TakeoffMission(Node):

    def __init__(self):
        super().__init__('takeoff_mission')

        self.declare_parameter('uav_id', 1)
        self.uav_id = self.get_parameter('uav_id').value
        ns = f'/uav{self.uav_id}'

        # Subscribe to bridge state topics so we can log live feedback
        self.rel_alt = 0.0
        self.mode    = '---'
        self.armed   = False

        self.create_subscription(Float32, f'{ns}/rel_alt', self._cb_alt,   10)
        self.create_subscription(String,  f'{ns}/mode',    self._cb_mode,  10)
        self.create_subscription(Bool,    f'{ns}/armed',   self._cb_armed, 10)

        # Service clients
        self._cli_takeoff = self.create_client(Trigger, f'{ns}/takeoff')
        self._cli_rtl     = self.create_client(Trigger, f'{ns}/rtl')
        self._cli_land    = self.create_client(Trigger, f'{ns}/land')

        # Start mission after a short delay (let subscriptions settle)
        self.create_timer(2.0, self._start_mission)
        self._mission_started = False

        self.get_logger().info(
            f'TakeoffMission ready for UAV{self.uav_id}. '
            f'Starting in 2 seconds...')

    # ── state callbacks ───────────────────────────────────────────────────────

    def _cb_alt(self, msg):
        self.rel_alt = msg.data

    def _cb_mode(self, msg):
        self.mode = msg.data

    def _cb_armed(self, msg):
        self.armed = msg.data

    # ── service call helper ───────────────────────────────────────────────────

    def _call(self, client, label):
        self.get_logger().info(f'→ Calling: {label}')

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                f'Service {label} not available — is drone_bridge running?')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is None:
            self.get_logger().error(f'{label} — no response received')
            return False

        result = future.result()
        if result.success:
            self.get_logger().info(f'✓ {result.message}')
        else:
            self.get_logger().error(f'✗ {result.message}')
        return result.success

    # ── mission ───────────────────────────────────────────────────────────────

    def _start_mission(self):
        # Timer fires repeatedly — only run once
        if self._mission_started:
            return
        self._mission_started = True

        self.get_logger().info('=== Starting arm + takeoff mission ===')
        self.get_logger().info(
            f'    Current mode: {self.mode}  |  armed: {self.armed}  '
            f'|  alt: {self.rel_alt:.1f}m')

        # Step 1: Takeoff (drone_bridge auto-arms first)
        success = self._call(self._cli_takeoff, '/takeoff')

        if success:
            self.get_logger().info(
                f'✓ Airborne at {self.rel_alt:.1f}m — hovering for 10 seconds')

            # Hover for 10 seconds using a one-shot timer
            self.create_timer(10.0, self._do_rtl)
        else:
            self.get_logger().error(
                'Takeoff failed. Check drone_bridge logs for PreArm errors.')

    def _do_rtl(self):
        self.get_logger().info('=== Hover complete — RTL ===')
        self._call(self._cli_rtl, '/rtl')
        self.get_logger().info(
            'Mission done. Drone returning to launch. '
            'You can Ctrl-C this node now.')


def main(args=None):
    rclpy.init(args=args)
    node = TakeoffMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
