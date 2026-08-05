"""
detector.py
───────────
Location-agnostic YOLO human detector. Same node runs in two places
depending on processing mode:

  edge  : runs inside uav1ns (same namespace as camera_relay)
           subscribes to /cluster/cam/uav1 (raw, local — no wireless crossing)
           publishes /detections/uav1 (tiny JSON — crosses wireless to gcsns)

  ground: runs inside gcsns
           subscribes to /relay/uav1/compressed (already crossed wireless)
           publishes /detections/uav1 (stays in gcsns)

The mode is determined purely by WHERE you launch this node, not by any
parameter. Both modes use identical inference code.

Publishes two topics:
  /detections/uavN     — std_msgs/String, JSON detection results
                         (this is what crosses the wireless link in edge mode)
  /debug/uavN/annotated — sensor_msgs/Image, bounding-box overlay
                          (local only — for rqt visualization, never crosses link)

JSON format:
  {
    "frame_id": "1785045147.471",   # relay send timestamp (for latency calc)
    "inference_ms": 45.2,           # YOLO inference time
    "detections": [
      {"class": "person", "confidence": 0.87, "bbox": [x1,y1,x2,y2]}
    ]
  }

Parameters:
  uav_id          (int,   default 1)
  processing_mode (str,   default 'edge') — determines input topic
  model_path      (str,   default ~/yolo_env/yolov8n.pt)
  conf_threshold  (float, default 0.25)   — minimum detection confidence
  publish_debug   (bool,  default True)   — whether to publish annotated image

NOTE: must be launched with ~/yolo_env active since ultralytics lives there.
"""

import json
import os
import time


import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from datetime import datetime


class Detector(Node):

    def __init__(self):
        super().__init__('detector')

        self.declare_parameter('uav_id', 1)
        self.declare_parameter('processing_mode', 'edge')
        self.declare_parameter(
            'model_path',
            os.path.expanduser('~/yolo_env/yolov8n.pt'))
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('target_classes', '0')

        self._uav_id    = self.get_parameter('uav_id').value
        self._mode      = self.get_parameter('processing_mode').value
        model_path      = self.get_parameter('model_path').value
        self._conf      = float(self.get_parameter('conf_threshold').value)
        self._debug     = bool(self.get_parameter('publish_debug').value)
        self._target_classes = [
            int(c) for c in
            str(self.get_parameter('target_classes').value).split(',')]

        # Load YOLO model
        self.get_logger().info(
            f'[Detector] Loading model: {model_path}, '
            f'confidence threshold={self._conf}')
        try:
            from ultralytics import YOLO
            self._model = YOLO(model_path)
            self.get_logger().info('[Detector] Model loaded OK')
        except Exception as e:
            self.get_logger().error(f'[Detector] Model load failed: {e}')
            raise

        self._frame_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Input topic depends on mode
        if self._mode == 'edge':
            in_topic = f'/cluster/cam/uav{self._uav_id}'
            self._sub = self.create_subscription(
                Image, in_topic, self._on_raw, qos)
            self.get_logger().info(
                f'[Detector] EDGE mode: {in_topic} -> inference locally '
                f'(throttled via camera_relay, rate-matched to ground)')
        else:
            in_topic = f'/relay/uav{self._uav_id}/compressed'
            self._sub = self.create_subscription(
                CompressedImage, in_topic, self._on_compressed, qos)
            self.get_logger().info(
                f'[Detector] GROUND mode: {in_topic} -> inference at GCS')

        # Output topics — same regardless of mode
        det_topic   = f'/detections/uav{self._uav_id}'
        debug_topic = f'/debug/uav{self._uav_id}/annotated'

        self._det_pub = self.create_publisher(String, det_topic, 10)
        if self._debug:
            self._debug_pub = self.create_publisher(Image, debug_topic, 1)
        else:
            self._debug_pub = None

        self.get_logger().info(
            f'[Detector] Publishing detections -> {det_topic}')
        if self._debug:
            self.get_logger().info(
                f'[Detector] Publishing debug -> {debug_topic} '
                f'(view with rqt_image_view)')

    def _img_msg_to_numpy(self, msg: Image) -> np.ndarray:
        """Convert raw ROS Image to numpy BGR array without cv_bridge."""
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4,
                    'bgra8': 4, 'mono8': 1, 'R8G8B8': 3}
        n_ch = channels.get(msg.encoding, 3)
        arr  = np.frombuffer(msg.data, dtype=np.uint8)
        arr  = arr.reshape((msg.height, msg.width, n_ch))
        if msg.encoding in ('rgb8', 'R8G8B8'):
            arr = arr[:, :, ::-1].copy()
        return arr

    def _compressed_to_numpy(self, msg: CompressedImage) -> np.ndarray:
        """Decode JPEG CompressedImage to numpy BGR array."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)

    def _run_inference(self, img_bgr: np.ndarray, frame_id: str,
                        compression_ms: float = -1.0, decode_ms: float = -1.0) -> None:
        """Run YOLO, publish detection JSON and optional annotated image."""
        self._frame_count += 1
        t0 = time.time()

        results = self._model(
        img_bgr,
        imgsz=960,
        conf=self._conf,
        classes=self._target_classes,
        verbose=False
        )

        inference_ms = (time.time() - t0) * 1000.0

        # Build detection list
        detections = []
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append({
                'class':      results[0].names[int(box.cls)],
                'confidence': round(float(box.conf), 3),
                'bbox':       [round(x1), round(y1),
                               round(x2), round(y2)]})

        payload = json.dumps({
            'frame_id':       frame_id,
            'inference_ms':   round(inference_ms, 1),
            'compression_ms': round(compression_ms, 1),
            'decode_ms':      round(decode_ms, 1),
            'detections':     detections})

        msg_out      = String()
        msg_out.data = payload
        self._det_pub.publish(msg_out)

        self.get_logger().info(
            f'[Detector] #{self._frame_count:04d} | '
            f'{len(detections)} detections(s) | '
            f'inference={inference_ms:.0f}ms')

        # Annotated debug image
        if self._debug_pub is not None:
            annotated = results[0].plot()

            # Always create directory
            save_dir = "/tmp/yolo_frames"
            os.makedirs(save_dir, exist_ok=True)

            # Save only frames with detections
            if len(detections) > 0:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

                filename = os.path.join(
                    save_dir,
                    f"det_{timestamp}.jpg"
                )

                cv2.imwrite(filename, annotated)

            debug_msg = Image()
            debug_msg.height = annotated.shape[0]
            debug_msg.width = annotated.shape[1]
            debug_msg.encoding = 'bgr8'
            debug_msg.step = annotated.shape[1] * 3
            debug_msg.data = annotated.tobytes()

            self._debug_pub.publish(debug_msg)

    def _on_raw(self, msg: Image) -> None:
        
        """Edge mode: throttled frame from camera_relay (rate-matched to
        ground mode). frame_id/stamp were set by camera_relay at publish
        time, so we reuse that rather than re-stamping here."""
        frame_id = msg.header.frame_id if msg.header.frame_id else str(time.time())
        img = self._img_msg_to_numpy(msg)
        self._run_inference(img, frame_id, compression_ms=-1.0, decode_ms=-1.0)

    def _on_compressed(self, msg: CompressedImage) -> None:
        """Ground mode: compressed frame that already crossed wireless link."""
        t_decode_start = time.time()
        img = self._compressed_to_numpy(msg)
        t_decode_end = time.time()
        if img is None:
            self.get_logger().warning('JPEG decode failed — frame dropped')
            return
        decode_ms = (t_decode_end - t_decode_start) * 1000.0

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            send_t = float(msg.header.frame_id)
            compression_ms = (send_t - stamp_s) * 1000.0 if stamp_s > 0 else -1.0
        except ValueError:
            compression_ms = -1.0

        self._run_inference(img, msg.header.frame_id,
                             compression_ms=compression_ms, decode_ms=decode_ms)


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
