#!/usr/bin/env python3
"""
Save lossless PNG frames from /uav1/camera/image_raw.

Run directly as a Python file while the simulation pipeline is active.
Captures one frame per second and stops automatically.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FrameCapture(Node):
    def __init__(
        self,
        topic: str,
        output_dir: Path,
        max_frames: int,
        rate_hz: float,
    ) -> None:
        super().__init__("d4_frame_capture")

        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.max_frames = max_frames
        self.saved_count = 0
        self.latest_msg: Image | None = None
        self.latest_sequence = 0
        self.last_saved_sequence = -1

        self.csv_file = (self.output_dir / "frames.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(
            [
                "filename",
                "capture_time_s",
                "width",
                "height",
                "encoding",
            ]
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            Image,
            topic,
            self.on_frame,
            qos,
        )

        self.timer = self.create_timer(1.0 / rate_hz, self.save_latest)

        self.get_logger().info(
            f"Saving {max_frames} frames from {topic} at {rate_hz} Hz"
        )
        self.get_logger().info(f"Output: {output_dir}")

    def on_frame(self, msg: Image) -> None:
        self.latest_sequence += 1
        self.latest_msg = msg

    @staticmethod
    def to_bgr(msg: Image) -> np.ndarray:
        channels = {
            "rgb8": 3,
            "R8G8B8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }

        if msg.encoding not in channels:
            raise ValueError(f"Unsupported encoding: {msg.encoding}")

        channel_count = channels[msg.encoding]
        expected_row_bytes = msg.width * channel_count

        raw = np.frombuffer(msg.data, dtype=np.uint8)

        if msg.step * msg.height > raw.size:
            raise ValueError(
                f"Image buffer too small: {raw.size} bytes"
            )

        rows = raw[: msg.step * msg.height].reshape(
            msg.height,
            msg.step,
        )

        pixels = rows[:, :expected_row_bytes]

        if channel_count == 1:
            image = pixels.reshape(msg.height, msg.width)
        else:
            image = pixels.reshape(
                msg.height,
                msg.width,
                channel_count,
            )

        if msg.encoding in ("rgb8", "R8G8B8"):
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "rgba8":
            image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
        elif msg.encoding == "bgra8":
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        return image.copy()

    def save_latest(self) -> None:
        if self.latest_msg is None:
            return

        if self.latest_sequence == self.last_saved_sequence:
            return

        try:
            image = self.to_bgr(self.latest_msg)

            filename = f"frame_{self.saved_count + 1:04d}.png"
            path = self.output_dir / filename

            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"Could not save {path}")

            now = time.time()

            self.writer.writerow(
                [
                    filename,
                    f"{now:.6f}",
                    self.latest_msg.width,
                    self.latest_msg.height,
                    self.latest_msg.encoding,
                ]
            )
            self.csv_file.flush()

            self.saved_count += 1
            self.last_saved_sequence = self.latest_sequence

            self.get_logger().info(
                f"Saved {filename} "
                f"({self.saved_count}/{self.max_frames})"
            )

            if self.saved_count >= self.max_frames:
                self.get_logger().info("Capture completed")
                rclpy.shutdown()

        except Exception as exc:
            self.get_logger().error(str(exc))

    def close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--topic",
        default="/uav1/camera/image_raw",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(
            "results/phase_d_application/raw/d4_reference_01"
        ),
    )
    parser.add_argument(
        "--frames",
        type=int,
        default=60,
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
    )

    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    node = FrameCapture(
        topic=args.topic,
        output_dir=args.output,
        max_frames=args.frames,
        rate_hz=args.rate,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()