"""
metrics_logger.py
─────────────────
Passive subscriber — publishes nothing, changes nothing in the pipeline.
Records per-frame metrics to a timestamped CSV for the edge vs ground
comparison. One file per run so experiments are cleanly separated.

CSV columns:
  run_id           — label for this experimental run (e.g. ground_run1)
  timestamp_s      — wall-clock time of receipt (time.time(), seconds since epoch)
  mode             — 'ground' or 'edge'
  uav_id           — which UAV
  frame_num        — sequential counter
  size_bytes       — message payload size in bytes
  capture_sec      — ROS header stamp seconds (set at relay/Gazebo send time)
  capture_nsec     — ROS header stamp nanoseconds
  latency_ms       — (receipt wall time) - (capture wall time) in milliseconds
                     Valid because relay and gcsns are on the same machine —
                     no clock sync issues. Measures: Gazebo publish → relay
                     compress → publish → wireless channel → gcsns receive.
  navsat_age_ms    — ms since last /ap/v1/navsat message arrived
                     Use this to detect DDS degradation under video load.
                     Compare: navsat_age_ms with video running vs without.

Parameters:
  uav_id           (int, default 1)
  processing_mode  (str, default 'ground') — labels the CSV filename
  run_id           (str, default '')       — labels rows and filename
  csv_dir          (str, default '/tmp')   — where to save CSV files
"""

import csv
import os
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage, NavSatFix
from std_msgs.msg import String


class MetricsLogger(Node):

    def __init__(self):
        super().__init__('metrics_logger')

        self.declare_parameter('uav_id', 1)
        self.declare_parameter('processing_mode', 'ground')
        self.declare_parameter('run_id', '')
        self.declare_parameter('csv_dir', '/tmp')

        self._uav_id = self.get_parameter('uav_id').value
        self._mode   = self.get_parameter('processing_mode').value
        run_id       = self.get_parameter('run_id').value
        csv_dir      = self.get_parameter('csv_dir').value

        self._run_id = run_id if run_id else f'{self._mode}_run'
        self._ground_count  = 0
        self._edge_count    = 0
        self._last_navsat_t = None  # wall time of last navsat message

        # Timestamped filename — one file per run
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = (
            f'metrics_uav{self._uav_id}_'
            f'{self._mode}_'
            f'{self._run_id}_'
            f'{ts}.csv')
        self._csv_path = os.path.join(csv_dir, filename)

        self._csv_file = open(self._csv_path, 'w', newline='')
        self._writer   = csv.writer(self._csv_file)
        self._writer.writerow([
            'run_id', 'timestamp_s', 'mode', 'uav_id', 'frame_num',
            'size_bytes', 'capture_sec', 'capture_nsec',
            'latency_ms', 'navsat_age_ms'])
        self._csv_file.flush()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Ground mode — compressed frames
        self._ground_sub = self.create_subscription(
            CompressedImage,
            f'/relay/uav{self._uav_id}/compressed',
            self._on_ground, qos)

        # Edge mode — detection results
        self._edge_sub = self.create_subscription(
            String,
            f'/detections/uav{self._uav_id}',
            self._on_edge, qos)

        # DDS health monitor — track navsat arrival rate
        # This subscription goes to gcsns's micro_ros_agent output
        self._navsat_sub = self.create_subscription(
            NavSatFix,
            f'/ap/v{self._uav_id}/navsat',
            self._on_navsat, qos)

        self.get_logger().info(
            f'[MetricsLogger] UAV{self._uav_id} | '
            f'mode={self._mode} | run_id={self._run_id}')
        self.get_logger().info(f'  CSV: {self._csv_path}')

    def _on_navsat(self, msg: NavSatFix) -> None:
        """Track DDS health — just record arrival time, no CSV row."""
        self._last_navsat_t = time.time()

    def _navsat_age_ms(self) -> float:
        """ms since last navsat — high value means DDS is degrading."""
        if self._last_navsat_t is None:
            return -1.0
        return (time.time() - self._last_navsat_t) * 1000.0

    def _on_ground(self, msg: CompressedImage) -> None:
        self._ground_count += 1
        receipt_t  = time.time()   # wall clock — same reference as header stamp
        size_b     = len(msg.data)

        capture_sec  = msg.header.stamp.sec
        capture_nsec = msg.header.stamp.nanosec
        capture_t    = capture_sec + capture_nsec * 1e-9

        try:
            send_t     = float(msg.header.frame_id)
            latency_ms = (receipt_t - send_t) * 1000.0
        except (ValueError, AttributeError):
            latency_ms = -1.0
        navsat_age   = self._navsat_age_ms()

        self._writer.writerow([
            self._run_id, f'{receipt_t:.6f}', 'ground',
            self._uav_id, self._ground_count, size_b,
            capture_sec, capture_nsec,
            f'{latency_ms:.2f}', f'{navsat_age:.1f}'])
        self._csv_file.flush()

        self.get_logger().info(
            f'[ground] #{self._ground_count:04d} | '
            f'{size_b/1024:.1f} KB | '
            f'latency={latency_ms:.0f} ms | '
            f'navsat_age={navsat_age:.0f} ms')

    def _on_edge(self, msg: String) -> None:
        self._edge_count += 1
        receipt_t  = time.time()
        size_b     = len(msg.data.encode())
        navsat_age = self._navsat_age_ms()

        self._writer.writerow([
            self._run_id, f'{receipt_t:.6f}', 'edge',
            self._uav_id, self._edge_count, size_b,
            0, 0, 0.0, f'{navsat_age:.1f}'])
        self._csv_file.flush()

        self.get_logger().info(
            f'[edge] #{self._edge_count:04d} | '
            f'{size_b}B | '
            f'navsat_age={navsat_age:.0f} ms | '
            f'{msg.data[:60]}')

    def destroy_node(self):
        self._csv_file.close()
        self.get_logger().info(
            f'[MetricsLogger] CSV saved: {self._csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
