import rclpy
from rclpy.node import Node
import socket

# Update this import to match your custom ROS 2 message package
# from uav_interfaces.msg import LinkPenalty

class Ns3UdpBridge(Node):
    def __init__(self):
        super().__init__('ns3_udp_bridge')
        
        # Open a local UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ns3_address = ('127.0.0.1', 5555) # Localhost, Port 5555
        
        # Subscribe to Gazebo's line-of-sight penalties
        '''
        self.subscription = self.create_subscription(
            LinkPenalty,
            '/uav_network/los_penalties',
            self.listener_callback,
            10
        )
        '''
        self.get_logger().info('ROS 2 to NS-3 UDP Bridge Started.')

    # Position sending
    def uav_pose_callback(self, uav_id, msg):
        # Extract coordinates from the ROS 2 Pose message
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # Format a position update string: "POSE 1 15.5 20.0 5.0"
        payload = f"POSE {uav_id} {x} {y} {z}"
        self.sock.sendto(payload.encode('utf-8'), self.ns3_address)


    # Penalty sending using gazebo to ns3
    def listener_callback(self, msg):
        # Format the data into a simple space-separated string
        # e.g., "1 2 15.5" (TxID RxID Penalty_dB)
        payload = f"{msg.tx_id} {msg.rx_id} {msg.penalty_db}"
        
        # Send it over UDP to NS-3
        self.sock.sendto(payload.encode('utf-8'), self.ns3_address)

def main(args=None):
    rclpy.init(args=args)
    bridge_node = Ns3UdpBridge()
    rclpy.spin(bridge_node)
    
    bridge_node.sock.close()
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
