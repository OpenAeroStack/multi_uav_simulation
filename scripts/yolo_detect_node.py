#!/usr/bin/env python3
"""Subscribe to the UAV camera, run YOLOv8, and report human detections.

Topic in : /uav1/camera/image_raw          (sensor_msgs/Image)
Topics out: /uav1/detections/humans         (std_msgs/String, JSON list)
            /detections/uav1                (std_msgs/String, GCS schema)

This node NEVER publishes an annotated image. Sending a 2.76 MB frame back to
the host on every detection saturated the same cable the camera arrives on and
starved the lockstep simulation of CPU, so simulated time fell behind and the
mission's waypoint timeouts fired even though the aircraft was flying correctly.
It also contradicts the architecture: the edge node exists to emit detection
results only. Use show_window for a local preview instead — it never touches
the network.

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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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

        # BEST_EFFORT, depth 1 — NOT the default RELIABLE/depth-10.
        #
        # Gazebo's camera publisher is RELIABLE, so a RELIABLE subscriber makes
        # it wait for this node to acknowledge every frame. The Pi needs ~1 s per
        # frame, so the publisher was throttled to the Pi's speed and the whole
        # simulation's camera fell to ~0.27 Hz. Worse, when the Pi rebooted
        # mid-run the unacknowledged samples kept the publisher stalled at its
        # retransmission heartbeat.
        #
        # BEST_EFFORT with depth 1 means the publisher never blocks and this node
        # always works on the newest frame instead of a queue of stale ones —
        # which is what you want for real-time detection anyway, since a frame
        # that arrives a second late is worthless.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,
                         depth=1)
        self.sub = self.create_subscription(Image, image_topic, self.on_image, qos)
        self.det_pub = self.create_publisher(String, "/uav1/detections/humans", 10)

        # Second publisher on uav_vision's topic, so the results actually reach
        # the ground station. gcs_receiver and metrics_logger subscribe to
        # /detections/uavN; without this the Pi publishes to nobody and the GCS
        # listens to nothing — two nodes talking past each other.
        # The payload carries frame_id and inference_ms as well as the boxes,
        # matching uav_vision/detector.py, so the downstream latency and
        # message-size measurements stay comparable between the two detectors.
        self.gcs_pub = self.create_publisher(String, "/detections/uav1", 10)

        self.frame_count = 0
        self.get_logger().info(f"Listening on {image_topic}, detecting humans...")
        self.get_logger().info(
            f"model={model_path} conf={CONF_THRESHOLD} "
            f"classes=[{PERSON_CLASS_ID}] imgsz=default(640) "
            f"show_window={self.show_window}")

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

        # Same detections in uav_vision's schema, for the ground station.
        # frame_id is stamped here rather than at capture because this node has
        # no access to the camera's send time — it is a receipt reference, and
        # only meaningful for cross-machine latency once the two clocks are
        # synchronised (chrony), which they are not yet.
        self.gcs_pub.publish(String(data=json.dumps({
            "frame_id": str(time.time()),
            "inference_ms": round(inference_ms, 1),
            "detections": [
                {"class": "person",
                 "confidence": round(h["conf"], 3),
                 "bbox": [round(v) for v in h["box"]]}
                for h in humans],
        })))

        # Log EVERY frame, not just the ones with hits: a run of zero-detection
        # frames still carries the timing data the edge-vs-ground comparison
        # needs, and silence makes a stalled node look identical to an empty
        # scene.
        self.get_logger().info(
            f"#{self.frame_count:04d} | {len(humans)} human(s) | "
            f"inference={inference_ms:.0f}ms")

        # Local preview only, and only if asked for. result.plot() draws into a
        # full-resolution copy of the frame, so it is skipped entirely when the
        # window is off — which is always on the headless Pi. Nothing here
        # crosses the network.
        if self.show_window:
            cv2.imshow("UAV1 human detection", result.plot())
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
