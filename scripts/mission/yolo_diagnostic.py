#!/usr/bin/env python3
import sys, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from ultralytics import YOLO

class Grab(Node):
    def __init__(self, topic):
        super().__init__('yolo_diag')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Image, topic, self.cb, qos)
        self.got = False

    def cb(self, msg):
        if self.got:
            return
        self.got = True
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1, 'R8G8B8': 3}
        n = channels.get(msg.encoding, 3)
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, n))
        if msg.encoding in ('rgb8', 'R8G8B8'):
            arr = arr[:, :, ::-1].copy()
        print(f"Frame: {msg.width}x{msg.height}, encoding={msg.encoding}")

        model = YOLO('~/yolo_env/yolov8n.pt'.replace('~', '/home/randilsk'))
        results = model(arr, verbose=False, conf=0.02)  # NO class filter at all

        boxes = results[0].boxes
        print(f"\nTotal raw detections (any class): {len(boxes)}")
        for box in boxes:
            cls_id = int(box.cls)
            name = results[0].names[cls_id]
            conf = float(box.conf)
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            w, h = x2 - x1, y2 - y1
            print(f"  class={name:15s} conf={conf:.3f}  bbox_size={w:.0f}x{h:.0f}px")

        annotated = results[0].plot()
        cv2.imwrite('/tmp/yolo_diag_unfiltered.png', annotated)
        print("\nSaved /tmp/yolo_diag_unfiltered.png (all classes, conf>=0.02)")
        rclpy.shutdown()

def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/uav1/camera/image_raw'
    rclpy.init()
    node = Grab(topic)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
