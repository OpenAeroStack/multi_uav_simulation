"""
camera_relay.py
───────────────
Runs inside a UAV namespace (e.g. uav1ns). Subscribes to the raw camera
topic (visible because Gazebo runs in the root namespace and DDS reaches
into the UAV namespace), then routes frames based on processing_mode:

  edge  : publish raw Image locally on /cluster/cam/uavN
           → detector.py (running in same namespace) picks it up
           → only tiny detection results cross the wireless link

  ground: compress to JPEG, publish CompressedImage on /relay/uavN/compressed
           → crosses the wireless link to gcsns
           → detector.py running in gcsns picks it up

Parameters (ros2 --ros-args -p name:=val):
  uav_id          (int,   default 1)    — which UAV (1/2/3)
  processing_mode (str,   default edge) — 'edge' or 'ground'
  jpeg_quality    (int,   default 50)   — JPEG quality for ground mode
  frame_rate_hz   (float, default 1.0)  — output rate (throttle from source)

NOTE: deliberately avoids cv_bridge due to NumPy 2.x incompatibility.
Uses numpy directly for image buffer access instead.
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage


class CameraRelay(Node):

    MODES = ('edge', 'ground')

    def __init__(self):
        super().__init__('camera_relay')

        self.declare_parameter('uav_id', 1)
        self.declare_parameter('processing_mode', 'edge')
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('frame_rate_hz', 1.0)

        self._uav_id  = self.get_parameter('uav_id').value
        self._mode    = self.get_parameter('processing_mode').value
        self._quality = int(self.get_parameter('jpeg_quality').value)
        self._rate_hz = float(self.get_parameter('frame_rate_hz').value)

        if self._mode not in self.MODES:
            self.get_logger().error(
                f"processing_mode must be one of {self.MODES}, "
                f"got '{self._mode}'. Defaulting to 'edge'.")
            self._mode = 'edge'

        self._latest_msg = None
        self._min_interval = 1.0 / self._rate_hz

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        in_topic = f'/uav{self._uav_id}/camera/image_raw'
        self._sub = self.create_subscription(Image, in_topic, self._on_frame, qos)

        if self._mode == 'edge':
            out_topic = f'/cluster/cam/uav{self._uav_id}'
            self._pub = self.create_publisher(Image, out_topic, 1)
            self.get_logger().info(
                f'[UAV{self._uav_id}] EDGE mode: {in_topic} -> {out_topic} '
                f'at {self._rate_hz:.1f} Hz (raw, local)')
        else:
            out_topic = f'/relay/uav{self._uav_id}/compressed'
            self._pub = self.create_publisher(CompressedImage, out_topic, 1)
            self.get_logger().info(
                f'[UAV{self._uav_id}] GROUND mode: {in_topic} -> {out_topic} '
                f'at {self._rate_hz:.1f} Hz, JPEG quality={self._quality}')

        self._out_topic = out_topic
        self.create_timer(self._min_interval, self._publish)

    def _on_frame(self, msg: Image) -> None:
        """Buffer latest frame — never block, just overwrite."""
        self._latest_msg = msg

    def _img_msg_to_numpy(self, msg: Image) -> np.ndarray:
        """Convert ROS Image to numpy array without cv_bridge."""
        dtype = np.uint8
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4,
                    'bgra8': 4, 'mono8': 1, 'R8G8B8': 3}
        n_ch = channels.get(msg.encoding, 3)
        arr = np.frombuffer(msg.data, dtype=dtype)
        arr = arr.reshape((msg.height, msg.width, n_ch))
        # Gazebo publishes RGB8 — convert to BGR for OpenCV
        if msg.encoding in ('rgb8', 'R8G8B8'):
            arr = arr[:, :, ::-1].copy()
        return arr

    def _publish(self) -> None:
        """Throttled publish: send buffered frame if one exists."""
        if self._latest_msg is None:
            return

        msg = self._latest_msg
        self._latest_msg = None

        if self._mode == 'edge':
            self._pub.publish(msg)

        else:
            try:
                cv_img = self._img_msg_to_numpy(msg)
                ok, buf = cv2.imencode(
                    '.jpg', cv_img,
                    [cv2.IMWRITE_JPEG_QUALITY, self._quality])
                if not ok:
                    self.get_logger().warning('JPEG encode failed — frame dropped')
                    return
                out = CompressedImage()
                out.header = msg.header
                out.format = 'jpeg'
                out.data = buf.tobytes()
                self._pub.publish(out)
                kb = len(out.data) / 1024
                self.get_logger().info(
                    f'[UAV{self._uav_id}] ground frame sent: {kb:.1f} KB')
            except Exception as e:
                self.get_logger().error(f'Compression error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
