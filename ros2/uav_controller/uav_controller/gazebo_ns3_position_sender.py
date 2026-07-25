"""Send Gazebo ENU model positions to ns-3 over a localhost UDP socket.

Wire format (one complete ASCII snapshot per datagram):
  NS3POS1,sequence,monotonic_time_ms,
  0,gcs_x,gcs_y,gcs_z,
  1,uav1_x,uav1_y,uav1_z,
  2,uav2_x,uav2_y,uav2_z,
  3,uav3_x,uav3_y,uav3_z\n
Coordinates are Gazebo world-frame Cartesian metres (x=east, y=north, z=up).
No GPS latitude or longitude is transmitted.
"""

import argparse
import socket
import sys
import time
from typing import Dict, Optional, Tuple

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


Position = Tuple[float, float, float]


def encode_snapshot(sequence: int, timestamp_ms: int,
                    positions: Dict[int, Position]) -> bytes:
    """Encode one complete, explicitly UAV-ID-keyed position snapshot."""
    fields = ['NS3POS1', str(sequence), str(timestamp_ms)]
    for entity_id in (0, 1, 2, 3):
        x, y, z = positions[entity_id]
        fields.extend((str(entity_id), f'{x:.3f}', f'{y:.3f}', f'{z:.3f}'))
    return (','.join(fields) + '\n').encode('ascii')


class GazeboNs3PositionSender(Node):
    """Publish complete Gazebo position snapshots to an ns-3 host socket."""

    def __init__(self) -> None:
        super().__init__('gazebo_ns3_position_sender')

        self.declare_parameter('destination_host', '127.0.0.1')
        self.declare_parameter('destination_port', 5555)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('model_states_topic', '/gazebo/model_states')
        self.declare_parameter('uav1_model_name', 'iris_1_demo')
        self.declare_parameter('uav2_model_name', 'iris_2_demo')
        self.declare_parameter('uav3_model_name', 'iris_3_demo')
        self.declare_parameter('gcs_x', 0.0)
        self.declare_parameter('gcs_y', 0.0)
        self.declare_parameter('gcs_z', 0.0)

        host = str(self.get_parameter('destination_host').value)
        port = int(self.get_parameter('destination_port').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        topic = str(self.get_parameter('model_states_topic').value)
        if not 0 < rate_hz <= 100:
            raise ValueError('publish_rate_hz must be greater than 0 and at most 100')
        if not 1 <= port <= 65535:
            raise ValueError('destination_port must be in the range 1..65535')

        self._destination = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sequence = 0
        self._positions: Dict[int, Position] = {
            0: (
                float(self.get_parameter('gcs_x').value),
                float(self.get_parameter('gcs_y').value),
                float(self.get_parameter('gcs_z').value),
            )
        }
        self._model_names = {
            1: str(self.get_parameter('uav1_model_name').value),
            2: str(self.get_parameter('uav2_model_name').value),
            3: str(self.get_parameter('uav3_model_name').value),
        }
        self._last_missing_log_ns = 0
        self._missing_log_period_ns = 5_000_000_000

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(ModelStates, topic, self._model_states_cb, qos)
        self.create_timer(1.0 / rate_hz, self._send_snapshot)

        self.get_logger().info(
            f'Gazebo → ns-3 position sender: {topic} → udp://{host}:{port} '
            f'at {rate_hz:g} Hz')
        self.get_logger().info(
            'Model mapping: ' + ', '.join(
                f'UAV{uav_id}={name}' for uav_id, name in self._model_names.items()))

    def _model_states_cb(self, message: ModelStates) -> None:
        indices = {name: index for index, name in enumerate(message.name)}
        missing = []

        for uav_id, model_name in self._model_names.items():
            index = indices.get(model_name)
            if index is None or index >= len(message.pose):
                self._positions.pop(uav_id, None)
                missing.append(model_name)
                continue
            position = message.pose[index].position
            self._positions[uav_id] = (
                float(position.x), float(position.y), float(position.z))

        if missing:
            now_ns = time.monotonic_ns()
            if now_ns - self._last_missing_log_ns >= self._missing_log_period_ns:
                available = ', '.join(message.name) if message.name else '(none)'
                self.get_logger().warning(
                    f'Missing Gazebo models: {", ".join(missing)}; '
                    f'available models: {available}')
                self._last_missing_log_ns = now_ns

    def _send_snapshot(self) -> None:
        missing_ids = [entity_id for entity_id in (1, 2, 3)
                       if entity_id not in self._positions]
        if missing_ids:
            return

        timestamp_ms = time.monotonic_ns() // 1_000_000
        packet = encode_snapshot(self._sequence, timestamp_ms, self._positions)
        try:
            self._socket.sendto(packet, self._destination)
        except OSError as error:
            self.get_logger().error(f'UDP position send failed: {error}')
            return
        self._sequence = (self._sequence + 1) & 0xFFFFFFFFFFFFFFFF

    def destroy_node(self) -> None:
        self._socket.close()
        super().destroy_node()


def run_test_mode(host: str, port: int, count: int, rate_hz: float) -> int:
    """Send deterministic synthetic snapshots without connecting to Gazebo."""
    positions = {
        0: (0.0, 0.0, 0.0),
        1: (0.0, 0.0, 60.0),
        2: (50.0, 0.0, 40.0),
        3: (-50.0, 0.0, 50.0),
    }
    destination = (host, port)
    interval = 1.0 / rate_hz
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        for sequence in range(count):
            packet = encode_snapshot(
                sequence, time.monotonic_ns() // 1_000_000, positions)
            udp_socket.sendto(packet, destination)
            print(packet.decode('ascii').rstrip())
            if sequence + 1 < count:
                time.sleep(interval)
    return 0


def main(args: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--test', action='store_true')
    parser.add_argument('--test-host', default='127.0.0.1')
    parser.add_argument('--test-port', type=int, default=5555)
    parser.add_argument('--test-count', type=int, default=3)
    parser.add_argument('--test-rate-hz', type=float, default=10.0)
    parsed, ros_args = parser.parse_known_args(args=args)

    if parsed.test:
        if parsed.test_count < 1 or parsed.test_rate_hz <= 0:
            parser.error('--test-count and --test-rate-hz must be positive')
        raise SystemExit(run_test_mode(
            parsed.test_host, parsed.test_port,
            parsed.test_count, parsed.test_rate_hz))

    rclpy.init(args=ros_args)
    node = GazeboNs3PositionSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
