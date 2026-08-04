#!/usr/bin/env python3
"""
telemetry_health.py — Phase C: DDS telemetry health under load conditions.

Subscribes to /ap/v{uav_id}/navsat, records wall-clock arrival time for
every message, and reports inter-arrival gap statistics (rate, mean,
median, p95, p99, max gap) — used to check whether concurrent video/
detection traffic on the same simulated wireless channel degrades
flight-critical GPS telemetry.

Run this INSIDE gcsns (same namespace as the DDS agent), for the SAME
fixed duration under each of four conditions:

  C0 — full mission running, no vision nodes at all
  C1 — stationary hold, no vision nodes (static baseline)
  C2 — stationary hold, detector only (edge mode)
  C3 — stationary hold, relay + detector (ground mode)

Usage:
    python3 telemetry_health.py --uav_id 1 --duration 120 --label c1_baseline
"""
import argparse
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix


def compute_gap_stats(arrivals):
    """Pure function, testable without rclpy — takes a list of wall-clock
    arrival timestamps (seconds), returns gap statistics in ms."""
    n = len(arrivals)
    if n < 2:
        return None
    gaps_ms = [(arrivals[i] - arrivals[i - 1]) * 1000.0 for i in range(1, n)]
    gaps_sorted = sorted(gaps_ms)

    def pct(p):
        idx = min(int(round(p / 100 * (len(gaps_sorted) - 1))),
                   len(gaps_sorted) - 1)
        return gaps_sorted[idx]

    return {
        "n_messages": n,
        "mean_gap_ms": sum(gaps_ms) / len(gaps_ms),
        "median_gap_ms": gaps_sorted[len(gaps_sorted) // 2],
        "p95_gap_ms": pct(95),
        "p99_gap_ms": pct(99),
        "max_gap_ms": max(gaps_ms),
        "min_gap_ms": min(gaps_ms),
    }


class TelemetryHealthLogger(Node):
    def __init__(self, uav_id, duration, label, results_root):
        super().__init__('telemetry_health_logger')
        self.uav_id = uav_id
        self.duration = duration
        self.label = label
        self.results_root = results_root
        self.arrivals = []
        self.start_time = time.time()
        self._finished = False

        self.raw_dir = os.path.join(results_root, "phase_c_middleware", "raw")
        self.processed_dir = os.path.join(results_root, "phase_c_middleware", "processed")
        os.makedirs(self.raw_dir, exist_ok=True)
        os.makedirs(self.processed_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.raw_dir, f"telemetry_{label}_{ts}.csv")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            NavSatFix, f'/ap/v{uav_id}/navsat', self._on_navsat, qos)

        self.get_logger().info(
            f'[TelemetryHealth] label={label} duration={duration}s '
            f'topic=/ap/v{uav_id}/navsat')
        self.get_logger().info(f'  Raw CSV: {self.csv_path}')

        self.create_timer(0.5, self._check_done)

    def _on_navsat(self, msg):
        self.arrivals.append(time.time())

    def _check_done(self):
        if not self._finished and time.time() - self.start_time >= self.duration:
            self._finished = True
            self.finish()

    def finish(self):
        n = len(self.arrivals)
        self.get_logger().info(f'[TelemetryHealth] Done. {n} messages received.')

        with open(self.csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['label', 'arrival_time_s', 'gap_from_prev_ms'])
            prev = None
            for t in self.arrivals:
                gap = f'{(t - prev) * 1000.0:.2f}' if prev is not None else ''
                w.writerow([self.label, f'{t:.6f}', gap])
                prev = t

        stats = compute_gap_stats(self.arrivals)
        if stats is None:
            print(f"\n=== Telemetry health: {self.label} ===")
            print(f"  ** Fewer than 2 messages received in {self.duration}s — "
                  f"cannot compute gap statistics. **")
            print(f"  Is the DDS agent / SITL actually running and reachable?")
            print(f"  Raw CSV saved: {self.csv_path}")
            return

        rate_hz = stats["n_messages"] / self.duration
        print(f"\n=== Telemetry health: {self.label} ===")
        print(f"  messages received : {stats['n_messages']}")
        print(f"  duration          : {self.duration}s")
        print(f"  mean rate         : {rate_hz:.2f} Hz")
        print(f"  mean gap          : {stats['mean_gap_ms']:.1f} ms")
        print(f"  median gap        : {stats['median_gap_ms']:.1f} ms")
        print(f"  p95 gap           : {stats['p95_gap_ms']:.1f} ms")
        print(f"  p99 gap           : {stats['p99_gap_ms']:.1f} ms")
        print(f"  max gap           : {stats['max_gap_ms']:.1f} ms")
        print(f"\n  Raw CSV saved: {self.csv_path}")

        summary_path = os.path.join(self.processed_dir, "c_telemetry_summary.csv")
        exists = os.path.isfile(summary_path)
        with open(summary_path, 'a', newline='') as f:
            w = csv.writer(f)
            if not exists:
                w.writerow(['label', 'duration_s', 'n_messages', 'rate_hz',
                            'mean_gap_ms', 'median_gap_ms', 'p95_gap_ms',
                            'p99_gap_ms', 'max_gap_ms'])
            w.writerow([self.label, self.duration, stats['n_messages'],
                        f'{rate_hz:.2f}', f'{stats["mean_gap_ms"]:.1f}',
                        f'{stats["median_gap_ms"]:.1f}', f'{stats["p95_gap_ms"]:.1f}',
                        f'{stats["p99_gap_ms"]:.1f}', f'{stats["max_gap_ms"]:.1f}'])
        print(f"  Appended to: {summary_path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--uav_id', type=int, default=1)
    ap.add_argument('--duration', type=float, default=120.0)
    ap.add_argument('--label', default='run')
    ap.add_argument('--results_root', default=os.path.expanduser(
        "~/FYP/multi_uav_simulation/results"))
    args = ap.parse_args()

    rclpy.init()
    node = TelemetryHealthLogger(args.uav_id, args.duration, args.label,
                                  args.results_root)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stopped early — still compute/save stats for whatever was captured,
        # rather than losing the run entirely.
        if not node._finished:
            node._finished = True
            node.finish()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()