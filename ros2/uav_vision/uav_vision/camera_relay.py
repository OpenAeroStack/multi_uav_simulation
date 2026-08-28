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

import json
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
        self.declare_parameter('experiment_sequence_ids', False)
        self.declare_parameter('transport_trace_path', '')

        self._uav_id  = self.get_parameter('uav_id').value
        self._mode    = self.get_parameter('processing_mode').value
        self._quality = int(self.get_parameter('jpeg_quality').value)
        self._rate_hz = float(self.get_parameter('frame_rate_hz').value)
        self._experiment_sequence_ids = bool(
            self.get_parameter('experiment_sequence_ids').value)
        trace_path = str(self.get_parameter('transport_trace_path').value)
        self._transport_trace = (
            open(trace_path, 'a', encoding='utf-8') if trace_path else None)

        if self._mode not in self.MODES:
            self.get_logger().error(
                f"processing_mode must be one of {self.MODES}, "
                f"got '{self._mode}'. Defaulting to 'edge'.")
            self._mode = 'edge'

        self._latest_msg = None
        self._sent_count = 0
        self._min_interval = 1.0 / self._rate_hz

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        edge_output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        ground_output_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        in_topic = f'/uav{self._uav_id}/camera/image_raw'
        self._sub = self.create_subscription(
            Image, in_topic, self._on_frame, camera_qos)

        if self._mode == 'edge':
            out_topic = f'/cluster/cam/uav{self._uav_id}'
            self._pub = self.create_publisher(Image, out_topic, edge_output_qos)
            self.get_logger().info(
                f'[UAV{self._uav_id}] EDGE mode: {in_topic} -> {out_topic} '
                f'at {self._rate_hz:.1f} Hz (raw, local)')
            self.get_logger().info('EDGE output QoS: RELIABLE')
        else:
            out_topic = f'/relay/uav{self._uav_id}/compressed'
            self._pub = self.create_publisher(
                CompressedImage, out_topic, ground_output_qos)
            self.get_logger().info(
                f'[UAV{self._uav_id}] GROUND mode: {in_topic} -> {out_topic} '
                f'at {self._rate_hz:.1f} Hz, JPEG quality={self._quality}')
            self.get_logger().info('GROUND output QoS: BEST_EFFORT')

        self._out_topic = out_topic
        self.create_timer(self._min_interval, self._publish)

    def _trace_event(self, payload: dict) -> None:
        if self._transport_trace is None:
            return
        self._transport_trace.write(json.dumps(payload) + '\n')
        self._transport_trace.flush()

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
        sequence_id = self._sent_count + 1

        if self._mode == 'edge':
            frame_admission_time = time.time()
            t_publish = frame_admission_time
            msg.header.stamp.sec = int(t_publish)
            msg.header.stamp.nanosec = int((t_publish % 1) * 1e9)
            msg.header.frame_id = (
                f'{t_publish:.9f}|{sequence_id}'
                if self._experiment_sequence_ids else str(t_publish))
            self._pub.publish(msg)
            self._sent_count += 1
            if self._experiment_sequence_ids:
                self._trace_event({
                    'event': 'relay_publish',
                    'sequence_id': sequence_id,
                    'frame_admission_time': frame_admission_time,
                    'relay_publish_time': t_publish,
                    'frame_size_bytes': len(msg.data),
                    'jpeg_encode_ms': None,
                })
            self.get_logger().info(
                f'[UAV{self._uav_id}] edge frame sent '
                f'#{self._sent_count:04d}')

        else:
            try:
                frame_admission_time = time.time()
                cv_img = self._img_msg_to_numpy(msg)
                t_encode_start = time.time()
                ok, buf = cv2.imencode(
                    '.jpg', cv_img,
                    [cv2.IMWRITE_JPEG_QUALITY, self._quality])
                t_encode_end = time.time()
                if not ok:
                    self.get_logger().warning('JPEG encode failed — frame dropped')
                    return
                out = CompressedImage()
                out.header = msg.header
                out.format = 'jpeg'
                out.data = buf.tobytes()
                # Preserve the historical encode-start stamp and, outside
                # experiments, the historical encode-end frame_id.
                out.header.stamp.sec = int(t_encode_start)
                out.header.stamp.nanosec = int((t_encode_start % 1) * 1e9)
                t_publish = time.time()
                out.header.frame_id = (
                    f'{t_publish:.9f}|{sequence_id}'
                    if self._experiment_sequence_ids else str(t_encode_end))
                self._pub.publish(out)
                self._sent_count += 1
                if self._experiment_sequence_ids:
                    self._trace_event({
                        'event': 'relay_publish',
                        'sequence_id': sequence_id,
                        'frame_admission_time': frame_admission_time,
                        'relay_publish_time': t_publish,
                        'jpeg_encode_start_time': t_encode_start,
                        'jpeg_encode_end_time': t_encode_end,
                        'jpeg_encode_ms': (
                            (t_encode_end - t_encode_start) * 1000.0),
                        'frame_size_bytes': len(out.data),
                        'jpeg_size_bytes': len(out.data),
                    })
                kb = len(out.data) / 1024
                self.get_logger().info(
                    f'[UAV{self._uav_id}] ground frame sent '
                    f'#{self._sent_count:04d}: {kb:.1f} KB')
            except Exception as e:
                self.get_logger().error(f'Compression error: {e}')

    def destroy_node(self):
        if self._transport_trace is not None:
            self._transport_trace.close()
        super().destroy_node()


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
