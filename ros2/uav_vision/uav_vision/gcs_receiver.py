"""
gcs_receiver.py
───────────────
Runs inside gcsns. The ground station endpoint for both pipeline modes:

  ground mode: subscribes to /relay/uavN/compressed
               (compressed frames that crossed the wireless link)

  edge mode:   subscribes to /detections/uavN
               (tiny detection results that crossed the wireless link)

In both modes: logs receipt time, message size, and sequence number.
This data feeds directly into metrics_logger.py for the paper comparison.

Parameters:
  uav_id          (int, default 1)    — which UAV to receive from
  processing_mode (str, default edge) — 'edge' or 'ground'

Run inside gcsns:
  ros2 run uav_vision gcs_receiver --ros-args -p uav_id:=1 -p processing_mode:=ground
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class GcsReceiver(Node):

    MODES = ('edge', 'ground')

    def __init__(self):
        super().__init__('gcs_receiver')

        self.declare_parameter('uav_id', 1)
        self.declare_parameter('processing_mode', 'edge')

        self._uav_id = self.get_parameter('uav_id').value
        self._mode   = self.get_parameter('processing_mode').value

        if self._mode not in self.MODES:
            self.get_logger().error(
                f"processing_mode must be one of {self.MODES}, "
                f"got '{self._mode}'. Defaulting to 'edge'.")
            self._mode = 'edge'

        self._frame_count = 0
        self._last_seq    = -1

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        if self._mode == 'ground':
            topic = f'/relay/uav{self._uav_id}/compressed'
            self._sub = self.create_subscription(
                CompressedImage, topic, self._on_compressed, qos)
            self.get_logger().info(
                f'[GCS] GROUND mode: receiving from {topic}')

        else:
            # Edge mode: receive detection results (small String messages)
            topic = f'/detections/uav{self._uav_id}'
            self._sub = self.create_subscription(
                String, topic, self._on_detection, qos)
            self.get_logger().info(
                f'[GCS] EDGE mode: receiving detections from {topic}')

        self._topic = topic

    def _on_compressed(self, msg: CompressedImage) -> None:
        """Called each time a compressed frame arrives from the wireless link."""
        self._frame_count += 1
        receipt_time = time.monotonic()
        size_kb = len(msg.data) / 1024

        # Capture time from message header (set by relay node at send time)
        capture_sec  = msg.header.stamp.sec
        capture_nsec = msg.header.stamp.nanosec
        capture_time = capture_sec + capture_nsec * 1e-9

        self.get_logger().info(
            f'[GCS] frame #{self._frame_count:04d} | '
            f'size={size_kb:.1f} KB | '
            f'receipt={receipt_time:.3f}s')

    def _on_detection(self, msg: String) -> None:
        """Called each time a detection result arrives from the wireless link."""
        self._frame_count += 1
        receipt_time = time.monotonic()
        size_b = len(msg.data)

        self.get_logger().info(
            f'[GCS] detection #{self._frame_count:04d} | '
            f'size={size_b}B | '
            f'data={msg.data[:80]} | '
            f'receipt={receipt_time:.3f}s')


def main(args=None):
    rclpy.init(args=args)
    node = GcsReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
