#!/usr/bin/env python3
"""Subscribe to the UAV camera, run YOLOv8, and report human detections.

Topic in : /uav1/camera/image_raw          (sensor_msgs/Image)
Topics out: /uav1/camera/annotated         (sensor_msgs/Image, boxes drawn)
            /uav1/detections/humans         (std_msgs/String, JSON list)

Brought over unmodified from the anton-ap_dds branch, where it detected humans
correctly. Kept as a KNOWN-GOOD REFERENCE for the current world and mission:
if this reports humans and uav_vision/detector.py does not, the difference is
in the detector settings; if neither reports any, the drone is not flying over
the people and no detector change will help.

Runs on the HOST only. It needs cv_bridge (present in the host venv via
system-site-packages, absent on the Pi, where uav_vision deliberately avoids it
because of the NumPy 2.x ABI clash) and cv2.imshow needs a display.

Run with:
    export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/config/fastdds_hitl_eth.xml
    source /opt/ros/humble/setup.bash
    venv/bin/python scripts/yolo_detect_node.py \
        --ros-args -p model_path:=<repo>/yolov8n.pt
"""
import json
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

PERSON_CLASS_ID = 0      # COCO: 0 == "person"
CONF_THRESHOLD = 0.4


class YoloHumanDetector(Node):
    def __init__(self):
        super().__init__("yolo_human_detector")

        self.declare_parameter("image_topic", "/uav1/camera/image_raw")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("show_window", True)

        image_topic = self.get_parameter("image_topic").value
        model_path = self.get_parameter("model_path").value
        self.show_window = self.get_parameter("show_window").value

        self.bridge = CvBridge()
        self.model = YOLO(model_path)

        self.sub = self.create_subscription(Image, image_topic, self.on_image, 10)
        self.img_pub = self.create_publisher(Image, "/uav1/camera/annotated", 10)
        self.det_pub = self.create_publisher(String, "/uav1/detections/humans", 10)

        self.frame_count = 0
        self.get_logger().info(f"Listening on {image_topic}, detecting humans...")
        self.get_logger().info(
            f"model={model_path} conf={CONF_THRESHOLD} "
            f"classes=[{PERSON_CLASS_ID}] imgsz=default(640)")

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.frame_count += 1

        # classes=[0] -> only run/report the "person" class
        # Timed around the model call ONLY — not the image conversion or the
        # annotate/publish that follow — so the number is comparable with
        # uav_vision/detector.py's inference_ms and with a standalone run.
        t0 = time.time()
        result = self.model(frame, classes=[PERSON_CLASS_ID],
                            conf=CONF_THRESHOLD, verbose=False)[0]
        inference_ms = (time.time() - t0) * 1000.0

        humans = [
            {"conf": float(b.conf), "box": [float(x) for x in b.xyxy[0]]}
            for b in result.boxes
        ]
        self.det_pub.publish(String(data=json.dumps(humans)))

        # Log EVERY frame, not just the ones with hits: a run of zero-detection
        # frames still carries the timing data the edge-vs-ground comparison
        # needs, and silence makes a stalled node look identical to an empty
        # scene.
        self.get_logger().info(
            f"#{self.frame_count:04d} | {len(humans)} human(s) | "
            f"inference={inference_ms:.0f}ms")

        annotated = result.plot()
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(annotated, "bgr8"))
        if self.show_window:
            cv2.imshow("UAV1 human detection", annotated)
            cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloHumanDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
