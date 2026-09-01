# Used to brdge between NS3 and ROS2 for UAV simulation (ROS2 End)
# (ROS2 → NS3: UAV positions + obstacle loss updates, NS3 → ROS2: per-node RSSI, PDR, throughput metrics)


# This Python script does the glue: it subscribes to /link_obstacle_loss (and /uav_world_positions) using normal ROS2 APIs,
# then re-sends that same data down a Unix domain socket — a simple, low-level inter-process communication channel that any program 
# (including NS3's C++ code) can open and read from, regardless of whether it understands ROS2 at all.

# ros2_ws/src/uav_cluster/uav_cluster/ns3_ros2_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import socket, json, threading, os, time

SOCK_PATH = '/tmp/ns3_uav_bridge.sock'

class NS3ROS2Bridge(Node):
    """
    Bidirectional bridge:
    ROS2 → NS3: UAV positions + obstacle loss updates
    NS3 → ROS2: per-node RSSI, PDR, throughput metrics
    """
    def __init__(self):
        super().__init__('ns3_ros2_bridge')
        self.n_uavs = int(os.getenv('N_UAVS', '3'))

        # Receive positions from micro-ROS (world frame, aggregated)
        self.create_subscription(Float32MultiArray,
            '/uav_world_positions', self._pos_cb, 10)
        # Receive obstacle loss from Gazebo plugin
        self.create_subscription(Float32MultiArray,
            '/link_obstacle_loss', self._obs_cb, 10)

        # Publish NS3 metrics back to ROS2
        self.metrics_pub = self.create_publisher(
            Float32MultiArray, '/ns3_link_metrics', 10)

        # Set up Unix domain socket server (NS3 connects as client)
        if os.path.exists(SOCK_PATH):
            os.remove(SOCK_PATH)
        self.server_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server_sock.bind(SOCK_PATH)
        self.server_sock.listen(1)
        self.client_sock = None
        self.send_lock = threading.Lock()

        self.get_logger().info('Waiting for NS3 to connect on socket...')
        threading.Thread(target=self._accept_loop, daemon=True).start()

    def _accept_loop(self):
        while rclpy.ok():
            self.client_sock, _ = self.server_sock.accept()
            self.get_logger().info('NS3 connected to bridge socket')
            threading.Thread(
                target=self._recv_from_ns3,
                args=(self.client_sock,), daemon=True).start()

    def _send_to_ns3(self, payload: dict):
        if self.client_sock is None:
            return
        try:
            data = (json.dumps(payload) + '\n').encode()
            with self.send_lock:
                self.client_sock.sendall(data)
        except (BrokenPipeError, OSError):
            self.client_sock = None

    def _recv_from_ns3(self, sock):
        buf = b""
        while True:
            try:
                chunk = sock.recv(8192)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    self._handle_ns3_msg(json.loads(line.decode()))
            except (OSError, json.JSONDecodeError):
                break

    def _handle_ns3_msg(self, msg: dict):
        if msg.get('type') == 'link_metrics':
            # metrics: [[uav_id, rssi_dBm, pdr], ...]
            flat = []
            for entry in msg['metrics']:
                flat.extend(entry)  # [uid, rssi, pdr]
            pub_msg = Float32MultiArray()
            pub_msg.data = [float(x) for x in flat]
            self.metrics_pub.publish(pub_msg)

    # ── Forward positions & obstacle losses to NS3 ──
    def _pos_cb(self, msg: Float32MultiArray):
        self._send_to_ns3({'type': 'positions', 'data': list(msg.data)})

    def _obs_cb(self, msg: Float32MultiArray):
        self._send_to_ns3({'type': 'obstacle_loss', 'data': list(msg.data)})

def main():
    rclpy.init()
    rclpy.spin(NS3ROS2Bridge())