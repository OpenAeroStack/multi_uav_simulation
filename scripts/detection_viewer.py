#!/usr/bin/env python3
"""Show the Pi's detections drawn on the camera image. Runs on the HOST.

The edge node does not publish an annotated image: sending a 2.76 MB picture
back on every detection loaded the host and contradicted the architecture, in
which only detection results leave the aircraft. It sends ~118 bytes of JSON
instead, and that JSON already contains every bounding box.

This node puts the picture back together where it is free to do so:

    /uav1/camera/image_raw   Gazebo, already on this machine
  + /detections/uav1         118 B, arrived through ns-3
  -> a window with boxes drawn

Nothing extra crosses the cable and the Pi does no extra work.

    export FASTRTPS_DEFAULT_PROFILES_FILE=<repo>/config/fastdds_hitl_eth.xml
    source /opt/ros/humble/setup.bash
    python3 scripts/detection_viewer.py

Keys:  s = save the current frame to /tmp/detection_NNN.png
       q = quit

NOTE: while this runs, Gazebo serialises each image for one more subscriber.
That costs host CPU but no cable bandwidth. Close it before taking timing
measurements.
"""
import glob
import json
import os
import re
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

BOX = (0, 220, 0)          # green, BGR
STALE_BOX = (0, 170, 220)  # amber once the detection is old
TEXT = (255, 255, 255)

# The detector runs at about 1 Hz while the camera runs at 5 Hz, so the same
# boxes get drawn on several frames. Past this age they are shown amber, to
# make clear they describe an earlier frame rather than this one.
STALE_AFTER_S = 1.5

HUD_H = 78                      # telemetry band drawn ABOVE the image
HUD_BG = (38, 28, 20)           # BGR, matches the deck's ink
HUD_KEY = (161, 148, 138)       # muted label
HUD_VAL = (255, 255, 255)
HUD_WARN = (11, 67, 176)        # BGR of the deck's caution orange
TELEMETRY_PERIOD_S = 2.0        # sysfs and log reads are throttled to this


def _host_cpu_sensor():
    """Path to a real CPU temperature, or None.

    thermal_zone0 on this laptop is an INT3400 control device reading 20 C —
    not a core temperature — so the sensor is discovered by name.
    """
    for h in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
        try:
            with open(os.path.join(h, "name")) as f:
                if f.read().strip() in ("coretemp", "k10temp", "cpu_thermal"):
                    return os.path.join(h, "temp1_input")
        except OSError:
            continue
    for z in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        try:
            with open(os.path.join(z, "type")) as f:
                if f.read().strip() == "x86_pkg_temp":
                    return os.path.join(z, "temp")
        except OSError:
            continue
    return None


class DetectionViewer(Node):
    def __init__(self):
        super().__init__("detection_viewer")

        self.declare_parameter("image_topic", "/uav1/camera/image_raw")
        self.declare_parameter("detection_topic", "/detections/uav1")
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("uav_model", "")          # default: iris_<N>_demo
        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detection_topic").value

        # Title from the topic, not hardcoded: two viewers both labelled UAV1
        # is indistinguishable on screen. "/uav2/camera/image_raw" -> "UAV2".
        parts = [p for p in image_topic.split("/") if p]
        self.uav = parts[0].upper() if parts else "UAV"
        self.window = f"{self.uav} camera + edge detections"

        m = re.search(r"(\d+)", self.uav)
        self.uav_id = int(m.group(1)) if m else 1
        self.model = (self.get_parameter("uav_model").value
                      or f"iris_{self.uav_id}_demo")

        self.bridge = CvBridge()
        self.frame = None
        self.boxes = []
        self.inference_ms = None
        self.det_time = 0.0
        self.saved = 0

        # Telemetry shown in the HUD band.
        self.pos = None            # (x, y, z) Gazebo world, from model_states
        self.speed = None          # horizontal ground speed, m/s
        self.pi_temp = None
        self.pi_clock = None
        self.pi_load = None
        self.pi_throttled = None
        self.host_temp = None
        self._host_sensor = _host_cpu_sensor()
        self._pi_log = f"/tmp/thermal_uav{self.uav_id}.log"
        self._telem_at = 0.0

        # Same policy as the detector: newest frame only, never block the
        # publisher. A slow viewer must not throttle the simulation.
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, image_topic, self._on_image, qos)
        self.create_subscription(String, det_topic, self._on_detections, 10)

        # Ground-truth pose and velocity. drone_bridge publishes /uavN/gps but
        # it lives in gcsns, which this window cannot see from the root
        # namespace; Gazebo's model_states is published right here.
        try:
            from gazebo_msgs.msg import ModelStates
            self.create_subscription(
                ModelStates, self.get_parameter("model_states_topic").value,
                self._on_states, qos)
        except ImportError:
            self.get_logger().warning(
                "gazebo_msgs not available — position and speed stay blank")

        self.get_logger().info(f"image      : {image_topic}")
        self.get_logger().info(f"detections : {det_topic}")
        self.get_logger().info("s = save frame,  q = quit")

    def _on_detections(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.boxes = payload.get("detections", [])
        self.inference_ms = payload.get("inference_ms")
        self.det_time = time.time()

    def _on_image(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _on_states(self, msg):
        """Ground-truth pose and velocity for this aircraft."""
        try:
            i = msg.name.index(self.model)
        except ValueError:
            return
        p = msg.pose[i].position
        v = msg.twist[i].linear
        self.pos = (p.x, p.y, p.z)
        self.speed = (v.x ** 2 + v.y ** 2) ** 0.5      # horizontal ground speed

    def _read_telemetry(self):
        """Board and host temperatures. Throttled: these are file reads, and
        the draw loop runs at camera rate."""
        now = time.time()
        if now - self._telem_at < TELEMETRY_PERIOD_S:
            return
        self._telem_at = now

        if self._host_sensor:
            try:
                with open(self._host_sensor) as f:
                    self.host_temp = int(f.read().strip()) / 1000.0
            except (OSError, ValueError):
                self.host_temp = None

        # Written by run_missions.sh during a flight; absent before one starts.
        try:
            with open(self._pi_log, "rb") as f:
                f.seek(0, os.SEEK_END)
                back = min(400, f.tell())
                f.seek(-back, os.SEEK_END)
                last = f.read().decode(errors="ignore").strip().splitlines()[-1]
        except (OSError, IndexError):
            return

        def grab(pat, cast=float):
            m = re.search(pat, last)
            return cast(m.group(1)) if m else None

        self.pi_temp = grab(r"temp=([\d.]+)")
        clock = grab(r"frequency\(\d+\)=(\d+)", int)
        self.pi_clock = clock / 1_000_000 if clock else None
        self.pi_load = grab(r"load=([\d.]+)")
        self.pi_throttled = grab(r"throttled=(0x[0-9a-fA-F]+)", str)

    def _with_hud(self, img, age):
        """Return the frame with a telemetry band added ABOVE it.

        A band rather than an overlay: at 640x384 an overlay this size covers
        the part of the road the detections are in.
        """
        import numpy as np
        self._read_telemetry()

        h, w = img.shape[:2]
        out = np.zeros((h + HUD_H, w, 3), dtype=img.dtype)
        out[:HUD_H, :] = HUD_BG
        out[HUD_H:, :] = img

        f, sc, th = cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1

        def put(x, y, key, val, colour=HUD_VAL):
            cv2.putText(out, key, (x, y), f, sc, HUD_KEY, th, cv2.LINE_AA)
            cv2.putText(out, val, (x + 52, y), f, sc, colour, th, cv2.LINE_AA)

        cv2.putText(out, self.uav, (10, 22), f, 0.62, HUD_VAL, 2, cv2.LINE_AA)

        pos = (f"{self.pos[0]:+.0f} {self.pos[1]:+.0f} {self.pos[2]:.0f} m"
               if self.pos else "--")
        put(10, 44, "pos", pos)
        put(10, 62, "spd", f"{self.speed:.1f} m/s" if self.speed is not None else "--")

        x2 = w // 2 - 40
        put(x2, 22, "det", str(len(self.boxes)),
            HUD_VAL if age < STALE_AFTER_S else HUD_WARN)
        put(x2, 44, "inf",
            f"{self.inference_ms:.0f} ms" if self.inference_ms is not None else "--")
        put(x2, 62, "age",
            f"{age:.1f} s" if age < 900 else "--",
            HUD_VAL if age < STALE_AFTER_S else HUD_WARN)

        x3 = w - 210
        hot = self.pi_temp is not None and self.pi_temp >= 70
        put(x3, 22, "Pi",
            f"{self.pi_temp:.1f}C" if self.pi_temp is not None else "--",
            HUD_WARN if hot else HUD_VAL)
        put(x3, 44, "clk",
            f"{self.pi_clock:.0f} MHz  ld {self.pi_load:.1f}"
            if self.pi_clock and self.pi_load is not None else "--")
        put(x3, 62, "host",
            f"{self.host_temp:.1f}C" if self.host_temp is not None else "--")

        return out

    def draw(self):
        if self.frame is None:
            return
        img = self.frame.copy()
        age = time.time() - self.det_time if self.det_time else 999
        colour = BOX if age < STALE_AFTER_S else STALE_BOX

        for det in self.boxes:
            box = det.get("bbox")
            if not box or len(box) != 4:
                continue
            x1, y1, x2, y2 = (int(v) for v in box)
            cv2.rectangle(img, (x1, y1), (x2, y2), colour, 2)
            label = f"{det.get('class', 'person')} {det.get('confidence', 0):.2f}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 6), (x1 + tw + 4, y1), colour, -1)
            cv2.putText(img, label, (x1 + 2, y1 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)

        img = self._with_hud(img, age)
        cv2.imshow(self.window, img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            path = f"/tmp/detection_{self.saved:03d}.png"
            cv2.imwrite(path, img)
            self.get_logger().info(f"saved {path}")
            self.saved += 1
        elif key == ord("q"):
            raise KeyboardInterrupt


def main():
    rclpy.init()
    node = DetectionViewer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.draw()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
