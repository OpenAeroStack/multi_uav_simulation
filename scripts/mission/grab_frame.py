#!/usr/bin/env python3
import sys, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class Grab(Node):
    def __init__(self, topic, outfile):
        super().__init__('frame_grab')
        self.outfile = outfile
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
        cv2.imwrite(self.outfile, arr)
        print(f"Saved {self.outfile} ({msg.width}x{msg.height}, encoding={msg.encoding})")
        rclpy.shutdown()

def main():
    topic = sys.argv[1] if len(sys.argv) > 1 else '/debug/uav1/annotated'
    outfile = sys.argv[2] if len(sys.argv) > 2 else '/tmp/frame.png'
    rclpy.init()
    node = Grab(topic, outfile)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
