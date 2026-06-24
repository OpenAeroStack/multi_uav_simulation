#!/usr/bin/env python3
"""DDS-native aerial survey: arm, take off, fly forward over the people, RTL.

Pure ROS2/DDS control via ArduPilot's AP_DDS interface -- NO pymavlink / MAVLink.
  Services : /ap/mode_switch, /ap/arm_motors, /ap/experimental/takeoff
  Movement : /ap/cmd_vel  (TwistStamped, frame_id="base_link" = body frame)
  Feedback : /ap/pose/filtered (altitude)

Run the detector (scripts/yolo_detect_node.py) alongside to detect humans.
Requires:  source /opt/ros/humble/setup.bash && source ~/ardu_ws/install/setup.bash
"""
import time

import rclpy
from rclpy.node import Node

from ardupilot_msgs.srv import ModeSwitch, ArmMotors, Takeoff
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped

GUIDED = 4
RTL = 6
ALTITUDE = 15.0          # metres
FORWARD_SPEED = 4.0      # m/s (body x = forward = north at yaw 0)
SURVEY_DISTANCE = 45.0   # metres to cover the line of people
BASE_LINK = "base_link"


class SurveyDDS(Node):
    def __init__(self):
        super().__init__("survey_dds")
        self.mode_cli = self.create_client(ModeSwitch, "/ap/mode_switch")
        self.arm_cli = self.create_client(ArmMotors, "/ap/arm_motors")
        self.takeoff_cli = self.create_client(Takeoff, "/ap/experimental/takeoff")
        self.cmd_pub = self.create_publisher(TwistStamped, "/ap/cmd_vel", 10)
        self.create_subscription(PoseStamped, "/ap/pose/filtered",
                                self.on_pose, 10)
        self.altitude = 0.0

    def on_pose(self, msg):
        # /ap/pose/filtered is ENU -> z is altitude (up)
        self.altitude = msg.pose.position.z

    # ---- helpers ----
    def call(self, client, request, name):
        self.get_logger().info(f">>> waiting for service {name}...")
        client.wait_for_service()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def set_mode(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        res = self.call(self.mode_cli, req, "/ap/mode_switch")
        self.get_logger().info(f">>> mode set, current mode = {res.curr_mode}")

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        res = self.call(self.arm_cli, req, "/ap/arm_motors")
        self.get_logger().info(f">>> arm result = {res.result}")
        return res.result

    def takeoff(self, alt):
        req = Takeoff.Request()
        req.alt = float(alt)
        res = self.call(self.takeoff_cli, req, "/ap/experimental/takeoff")
        self.get_logger().info(f">>> takeoff accepted = {res.status}")

    def publish_velocity(self, vx):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_LINK        # body frame: x forward, y left, z up
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        self.cmd_pub.publish(msg)

    def wait_for_altitude(self, target, timeout=30.0):
        self.get_logger().info(f">>> climbing to {target}m...")
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.altitude >= target * 0.95:
                self.get_logger().info(f">>> reached {self.altitude:.1f}m")
                return
        self.get_logger().warn(f">>> altitude timeout (at {self.altitude:.1f}m)")

    def fly_forward(self, distance, speed):
        """Stream velocity commands so the drone flies forward over the people."""
        duration = distance / speed
        self.get_logger().info(f">>> flying forward {distance}m at {speed} m/s "
                              f"({duration:.0f}s)")
        rate_hz = 10.0
        end = time.time() + duration
        while rclpy.ok() and time.time() < end:
            self.publish_velocity(speed)          # must stream or GUIDED times out
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)
        # stop
        for _ in range(5):
            self.publish_velocity(0.0)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = SurveyDDS()
    try:
        node.set_mode(GUIDED)
        if not node.arm():
            node.get_logger().error("Arm failed -- check EKF/GPS ready. Aborting.")
            return
        node.takeoff(ALTITUDE)
        node.wait_for_altitude(ALTITUDE)
        node.fly_forward(SURVEY_DISTANCE, FORWARD_SPEED)
        node.get_logger().info(">>> survey done -- switching to RTL")
        node.set_mode(RTL)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
