#!/usr/bin/env python3
"""Passive GCS observer for sequence-aware rate-sweep detection results."""

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class GroundResultObserver(Node):

    def __init__(self) -> None:
        super().__init__('rate_sweep_ground_result_observer')
        self.declare_parameter('uav_id', 1)
        self.declare_parameter('trace_path', '')
        uav_id = int(self.get_parameter('uav_id').value)
        trace_path = str(self.get_parameter('trace_path').value)
        if not trace_path:
            raise ValueError('trace_path must be provided')
        self._trace = open(trace_path, 'a', encoding='utf-8')
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        topic = f'/detections/uav{uav_id}'
        self._subscription = self.create_subscription(
            String, topic, self._on_result, detection_qos)
        self.get_logger().info(f'[GroundResultObserver] ready on {topic}')

    def _on_result(self, msg: String) -> None:
        ground_result_receipt_time = time.time()
        try:
            payload = json.loads(msg.data)
            sequence_id = int(payload['sequence_id'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        self._trace.write(json.dumps({
            'event': 'ground_result_receipt',
            'sequence_id': sequence_id,
            'ground_result_receipt_time': ground_result_receipt_time,
        }) + '\n')
        self._trace.flush()

    def destroy_node(self) -> None:
        self._trace.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundResultObserver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
