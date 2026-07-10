"""
snr_monitor.py
---------------
Tails NS-3's snr_log.csv (per-packet SNR, written by MonitorSnifferCallback
in the NS-3 scenario) and republishes a smoothed per-UAV SNR value on ROS 2.

NS-3 node index → UAV mapping (per architecture doc §5.1):
    node 0 = GCS     (not published)
    node 1 = UAV1    -> /uav1/snr
    node 2 = UAV2    -> /uav2/snr
    node 3 = UAV3    -> /uav3/snr

CSV columns: time_s, rx_node, signal_dbm, noise_dbm, snr_db
"""

import os
import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

NODE_TO_UAV = {1: 1, 2: 2, 3: 3}   # NS-3 node index -> UAV id (node 0 = GCS, skipped)
WINDOW_SIZE = 20                    # rolling average over last N samples


class SnrMonitor(Node):
    def __init__(self):
        super().__init__('snr_monitor')
        self.declare_parameter('snr_log_path', '/tmp/snr_log.csv')
        self.path = self.get_parameter('snr_log_path').value

        self.pubs = {
            uid: self.create_publisher(Float32, f'/uav{uid}/snr', 10)
            for uid in NODE_TO_UAV.values()
        }
        self.windows = {uid: deque(maxlen=WINDOW_SIZE) for uid in NODE_TO_UAV.values()}

        self._stop = False
        threading.Thread(target=self._tail_loop, daemon=True).start()
        self.get_logger().info(f'[snr_monitor] Tailing {self.path}')

    def _tail_loop(self):
        # Wait for NS-3 to create the file
        while not os.path.exists(self.path) and not self._stop:
            time.sleep(1.0)

        with open(self.path, 'r') as f:
            f.readline()  # skip header
            f.seek(0, os.SEEK_END)  # start at end, only read new lines
            while not self._stop:
                line = f.readline()
                if not line:
                    time.sleep(0.2)
                    continue
                self._handle_line(line.strip())

    def _handle_line(self, line):
        parts = line.split(',')
        if len(parts) != 5:
            return
        try:
            _, rx_node, _, _, snr_db = parts
            rx_node = int(rx_node)
            snr_db = float(snr_db)
        except ValueError:
            return

        uid = NODE_TO_UAV.get(rx_node)
        if uid is None:
            return  # GCS or unknown node, skip

        self.windows[uid].append(snr_db)
        avg = sum(self.windows[uid]) / len(self.windows[uid])
        msg = Float32()
        msg.data = avg
        self.pubs[uid].publish(msg)

    def destroy_node(self):
        self._stop = True
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SnrMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()