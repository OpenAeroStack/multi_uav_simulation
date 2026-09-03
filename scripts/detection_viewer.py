#!/usr/bin/env python3
"""Show the Pi's detections drawn on the camera image. Runs on the HOST.

The edge node does not publish an annotated image: sending a 2.76 MB picture
back on every detection loaded the host and contradicted the architecture, in
which only detection results leave the aircraft. It sends ~118 bytes of JSON
instead, and that JSON already contains every bounding box.

This node puts the picture back together where it is free to do so:

    /uav1/camera/image_raw   Gazebo, already on this machine
  + /detections/uav1         118 B, arrived through ns-3
  -> a window with boxes drawn

Nothing extra crosses the cable and the Pi does no extra work.

    export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/config/fastdds_hitl_eth.xml
    source /opt/ros/humble/setup.bash
    python3 scripts/detection_viewer.py

Keys:  s = save the current frame to /tmp/detection_NNN.png
       q = quit

NOTE: while this runs, Gazebo serialises each image for one more subscriber.
That costs host CPU but no cable bandwidth. Close it before taking timing
measurements.
"""
import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

BOX = (0, 220, 0)          # green, BGR
STALE_BOX = (0, 170, 220)  # amber once the detection is old
TEXT = (255, 255, 255)

# The detector runs at about 1 Hz while the camera runs at 5 Hz, so the same
# boxes get drawn on several frames. Past this age they are shown amber, to
# make clear they describe an earlier frame rather than this one.
STALE_AFTER_S = 1.5


class DetectionViewer(Node):
    def __init__(self):
        super().__init__("detection_viewer")

        self.declare_parameter("image_topic", "/uav1/camera/image_raw")
        self.declare_parameter("detection_topic", "/detections/uav1")
        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detection_topic").value

        # Title from the topic, not hardcoded: two viewers both labelled UAV1
        # is indistinguishable on screen. "/uav2/camera/image_raw" -> "UAV2".
        parts = [p for p in image_topic.split("/") if p]
        self.window = f"{parts[0].upper() if parts else 'UAV'} camera + edge detections"

        self.bridge = CvBridge()
        self.frame = None
        self.boxes = []
        self.inference_ms = None
        self.det_time = 0.0
        self.saved = 0

        # Same policy as the detector: newest frame only, never block the
        # publisher. A slow viewer must not throttle the simulation.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, image_topic, self._on_image, qos)
        self.create_subscription(String, det_topic, self._on_detections, 10)

        self.get_logger().info(f"image      : {image_topic}")
        self.get_logger().info(f"detections : {det_topic}")
        self.get_logger().info("s = save frame,  q = quit")

    def _on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.boxes = payload.get("detections", [])
        self.inference_ms = payload.get("inference_ms")
        self.det_time = time.time()

    def _on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def draw(self):
        if self.frame is None:
            return
        img = self.frame.copy()
        age = time.time() - self.det_time if self.det_time else 999
        colour = BOX if age < STALE_AFTER_S else STALE_BOX

        for det in self.boxes:
            box = det.get("bbox")
            if not box or len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(v) for v in box)
            cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
            label = f"{det.get('class', 'person')} {det.get('confidence', 0):.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour, -1)
            cv2.putText(img, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        banner = f"detections: {len(self.boxes)}"
        if self.inference_ms is not None:
            banner += f"   inference: {self.inference_ms:.0f} ms on the Pi"
        if age < 900:
            banner += f"   age: {age:.1f} s"
        cv2.putText(img, banner, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, TEXT, 2, cv2.LINE_AA)

        cv2.imshow(self.window, img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            path = f"/tmp/detection_{self.saved:03d}.png"
            cv2.imwrite(path, img)
            self.get_logger().info(f"saved {path}")
            self.saved += 1
        elif key == ord("q"):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = DetectionViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.draw()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
