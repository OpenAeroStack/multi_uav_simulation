# Used for verification of obstacle loss values in the simulation.
# UAV1 is kept stationary at (-10,0,1) while UAV2 stays at x=3 and sweeps
# along the y-axis. The ray crosses the wall plane (x~0) at ~77% of the link,
# so the link is blocked roughly while y is in [-2.6, +2.6] (loss > 0) and
# clear outside that band (loss = 0).
# Prints the loss reported back on /link_obstacle_loss for each position.
#
# CHANGED (GCS added): ids are NS-3 node ids -- 0 = GCS, 1..3 = UAV1..UAV3.
# This script used to publish ids 0 and 1 for its two UAVs and read loss(0,1);
# under the current numbering that would move the GROUND STATION and measure
# the GCS<->UAV1 link instead of the pair being swept. It now uses ids 1 and 2
# and reads loss(1,2).
#
# NOTE: only the two swept nodes are published here, so nodes 0 and 3 never
# receive a position. That is expected for this focused test, but it means
# NS-3's integration check will report them as missing -- ignore that when
# running this script.


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time

class SweepTest(Node):
    def __init__(self):
        super().__init__('sweep_test')
        self.pub = self.create_publisher(Float32MultiArray, '/uav_world_positions', 10)
        self.sub = self.create_subscription(
            Float32MultiArray, '/link_obstacle_loss', self._loss_cb, 10)
        self.y = -10.0
        self.loss12 = None

    def _loss_cb(self, msg):
        # payload: [i, j, loss_dB, i, j, loss_dB, ...]
        for k in range(0, len(msg.data) - 2, 3):
            if int(msg.data[k]) == 1 and int(msg.data[k + 1]) == 2:
                self.loss12 = msg.data[k + 2]

    def run(self):
        while rclpy.ok() and self.y <= 10.0:
            msg = Float32MultiArray()
            # UAV1 (node 1) fixed at (-10,0,1); UAV2 (node 2) at x=3 sweeping in y
            msg.data = [1.0, -10.0, 0.0, 1.0,
                        2.0, 3.0, self.y, 1.0]
            self.pub.publish(msg)

            # spin for 0.5s so the plugin's loss reply gets processed
            deadline = time.time() + 0.5
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            loss = 'n/a' if self.loss12 is None else f'{self.loss12:.1f} dB'
            self.get_logger().info(f'UAV2 y = {self.y:+5.1f}  ->  loss(1,2) = {loss}')
            self.y += 0.5

def main():
    rclpy.init()
    node = SweepTest()
    node.run()

if __name__ == '__main__':
    main()
