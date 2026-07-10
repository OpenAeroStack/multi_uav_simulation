# Used for verification of obstacle loss values in the simulation.
# UAV0 is kept stationary at (-10,0,1) while UAV1 stays at x=3 and sweeps
# along the y-axis. The ray crosses the concrete wall plane (x~0) at ~77%
# of the link, so the link is blocked roughly while y is in [-2.6, +2.6]
# (loss ~15+ dB) and clear outside that band (loss = 0).
# Prints the loss reported back on /link_obstacle_loss for each position.


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
        self.loss01 = None

    def _loss_cb(self, msg):
        # payload: [i, j, loss_dB, i, j, loss_dB, ...]
        for k in range(0, len(msg.data) - 2, 3):
            if int(msg.data[k]) == 0 and int(msg.data[k + 1]) == 1:
                self.loss01 = msg.data[k + 2]

    def run(self):
        while rclpy.ok() and self.y <= 10.0:
            msg = Float32MultiArray()
            # UAV0 fixed at (-10,0,1), UAV1 at x=3 sweeping in y, z=1
            msg.data = [0.0, -10.0, 0.0, 1.0,
                        1.0, 3.0, self.y, 1.0]
            self.pub.publish(msg)

            # spin for 0.5s so the plugin's loss reply gets processed
            deadline = time.time() + 0.5
            while time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

            loss = 'n/a' if self.loss01 is None else f'{self.loss01:.1f} dB'
            self.get_logger().info(f'UAV1 y = {self.y:+5.1f}  ->  loss(0,1) = {loss}')
            self.y += 0.5

def main():
    rclpy.init()
    node = SweepTest()
    node.run()

if __name__ == '__main__':
    main()
