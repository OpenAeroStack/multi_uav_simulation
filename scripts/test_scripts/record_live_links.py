#!/usr/bin/env python3
"""
record_live_links.py
────────────────────
PASSIVE recorder for a live Gazebo + NS-3 run. Subscribes to the four link
topics and writes them to CSV in test_logs/ .

  IT PUBLISHES NOTHING AND LAUNCHES NOTHING.

That is the whole point, and it is the difference between this script and
run_channel_validation.py. The validation harness is a DRIVER: it starts its
own NS-3 and publishes synthetic positions and obstacle loss, so running it
against a live setup makes every node's position alternate between the real
drone and the harness's fixed coordinates at 10 Hz. This script only listens,
so it is safe to start and stop at any time during a flight.

There is deliberately no create_publisher() call anywhere below.

What it records that NS-3's own --csvPath does NOT
──────────────────────────────────────────────────
  * the RAW obstacle loss straight off /link_obstacle_loss. NS-3's validation
    CSV logs the EMA-SMOOTHED value, so recording both lets you see the
    smoothing (and check EmaAlpha) rather than having to trust it.
  * Gazebo-side node positions with wall-clock timestamps, so the Gazebo and
    NS-3 sides can be correlated after the fact.
  * link-change EVENTS at the full 10 Hz feed rate, not just at the sample
    rate -- a transition lasting one ray-cast tick still gets recorded.
  * staleness. Every row carries the age of each input, so a dead publisher
    shows up as a growing age instead of silently repeated values.

Topics (all SUBSCRIBE):
  /uav_world_positions   [id, x, y, z, ...]        from world_pos_publisher.py
  /link_obstacle_loss    [i, j, loss_dB, ...]      from obstacle_raycast_plugin
  /ns3_link_rssi         [a, b, rssi_dBm, ...]     from NS-3
  /ns3_link_snr          [a, b, snr_dB, ...]       from NS-3

Outputs (in test_logs/, timestamped so runs never overwrite each other):
  live_<tag>_samples.csv   one row per link per sample tick
  live_<tag>_events.csv    one row per link state change
  live_<tag>_summary.csv   per-link totals written at exit

Usage
  source /opt/ros/humble/setup.bash
  python3 scripts/record_live_links.py                 # until Ctrl-C
  python3 scripts/record_live_links.py --duration 300  # 5 minutes
  python3 scripts/record_live_links.py --tag mission1 --rate 5
"""

import argparse
import csv
import math
import os
import signal
import sys
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUTDIR_DEFAULT = os.path.join(REPO, "test_logs")

NODE_NAMES = {0: "GCS", 1: "UAV1", 2: "UAV2", 3: "UAV3"}


def link_label(a, b):
    return f"{NODE_NAMES.get(a, f'N{a}')}-{NODE_NAMES.get(b, f'N{b}')}"


class LinkRecorder(Node):
    def __init__(self, args):
        super().__init__("link_recorder")
        self.args = args
        self.t0 = time.time()

        # Latest state, each with the wall time it arrived, so staleness is
        # always recoverable rather than inferred.
        self.pos = {}            # node id -> (x, y, z)
        self.pos_t = 0.0
        self.loss = {}           # (a,b) -> raw loss dB
        self.loss_t = 0.0
        self.rssi = {}           # (a,b) -> dBm
        self.rssi_t = 0.0
        self.snr = {}            # (a,b) -> dB
        self.snr_t = 0.0

        self.blocked = {}        # (a,b) -> bool, for edge detection
        self.n_events = 0
        self.n_samples = 0
        self.stats = {}          # (a,b) -> accumulator

        os.makedirs(args.outdir, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        tag = f"{args.tag}_{stamp}" if args.tag else stamp
        self.tag = tag

        self.samples_path = os.path.join(args.outdir, f"live_{tag}_samples.csv")
        self.events_path  = os.path.join(args.outdir, f"live_{tag}_events.csv")
        self.summary_path = os.path.join(args.outdir, f"live_{tag}_summary.csv")

        self._sf = open(self.samples_path, "w", newline="")
        self._sw = csv.writer(self._sf)
        self._sw.writerow([
            "wall_time_iso", "t_rel_s", "link", "node_a", "node_b",
            "distance_m", "obstacle_loss_db_raw", "rssi_dbm", "snr_db",
            "pos_age_s", "obs_age_s", "ns3_age_s"])

        self._ef = open(self.events_path, "w", newline="")
        self._ew = csv.writer(self._ef)
        self._ew.writerow([
            "wall_time_iso", "t_rel_s", "link", "node_a", "node_b",
            "event", "prev_loss_db", "new_loss_db",
            "distance_m", "rssi_dbm", "snr_db"])

        # SUBSCRIPTIONS ONLY. Adding a publisher here would break the one
        # guarantee this script makes.
        self.create_subscription(Float32MultiArray, "/uav_world_positions",
                                 self._on_pos, 10)
        self.create_subscription(Float32MultiArray, "/link_obstacle_loss",
                                 self._on_loss, 10)
        self.create_subscription(Float32MultiArray, "/ns3_link_rssi",
                                 self._on_rssi, 10)
        self.create_subscription(Float32MultiArray, "/ns3_link_snr",
                                 self._on_snr, 10)

        self.create_timer(1.0 / args.rate, self._sample)
        self.create_timer(5.0, self._heartbeat)

        self.get_logger().info(f"recording (passive) -> {self.samples_path}")
        self.get_logger().info(
            f"sample rate {args.rate} Hz, block threshold "
            f"{args.block_threshold} dB, duration "
            f"{'unlimited' if args.duration <= 0 else str(args.duration)+'s'}")

    # ── callbacks ────────────────────────────────────────────────────────────
    def _on_pos(self, msg):
        d = msg.data
        for i in range(0, len(d) - 3, 4):
            self.pos[int(d[i])] = (float(d[i+1]), float(d[i+2]), float(d[i+3]))
        self.pos_t = time.time()

    def _on_loss(self, msg):
        """Raw ray-caster output. Edge detection happens HERE, at the full feed
        rate, so a transition shorter than one sample tick is still captured."""
        d = msg.data
        now = time.time()
        for i in range(0, len(d) - 2, 3):
            a, b, v = int(d[i]), int(d[i+1]), float(d[i+2])
            if a > b:
                a, b = b, a
            key = (a, b)
            prev = self.loss.get(key)
            self.loss[key] = v

            was = self.blocked.get(key, False)
            now_blocked = v > self.args.block_threshold
            if prev is None:
                self.blocked[key] = now_blocked
                continue
            if now_blocked != was:
                self.blocked[key] = now_blocked
                self._event(key, "BLOCKED" if now_blocked else "CLEARED",
                            prev, v, now)
            elif abs(v - prev) >= self.args.change_threshold:
                self._event(key, "LOSS_CHANGE", prev, v, now)
        self.loss_t = now

    def _on_rssi(self, msg):
        d = msg.data
        for i in range(0, len(d) - 2, 3):
            a, b = int(d[i]), int(d[i+1])
            self.rssi[(min(a, b), max(a, b))] = float(d[i+2])
        self.rssi_t = time.time()

    def _on_snr(self, msg):
        d = msg.data
        for i in range(0, len(d) - 2, 3):
            a, b = int(d[i]), int(d[i+1])
            self.snr[(min(a, b), max(a, b))] = float(d[i+2])
        self.snr_t = time.time()

    # ── helpers ──────────────────────────────────────────────────────────────
    def _distance(self, a, b):
        if a not in self.pos or b not in self.pos:
            return None
        (x1, y1, z1), (x2, y2, z2) = self.pos[a], self.pos[b]
        return math.sqrt((x1-x2)**2 + (y1-y2)**2 + (z1-z2)**2)

    def _event(self, key, kind, prev, new, now):
        a, b = key
        d = self._distance(a, b)
        self._ew.writerow([
            datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            f"{now - self.t0:.3f}", link_label(a, b), a, b, kind,
            f"{prev:.3f}", f"{new:.3f}",
            "" if d is None else f"{d:.3f}",
            self._fmt(self.rssi.get(key)), self._fmt(self.snr.get(key))])
        self._ef.flush()
        self.n_events += 1
        self.get_logger().info(
            f"{link_label(a,b)}: {kind}  {prev:.1f} -> {new:.1f} dB"
            + (f"  (d={d:.1f} m)" if d is not None else ""))

    @staticmethod
    def _fmt(v):
        return "" if v is None else f"{v:.3f}"

    # ── periodic sampling ────────────────────────────────────────────────────
    def _sample(self):
        now = time.time()
        if self.args.duration > 0 and (now - self.t0) >= self.args.duration:
            raise KeyboardInterrupt

        # Union of every link seen on any topic, so a link missing from one
        # feed still produces a row (with blanks) rather than disappearing.
        keys = set(self.loss) | set(self.rssi) | set(self.snr)
        if not keys and len(self.pos) >= 2:
            ids = sorted(self.pos)
            keys = {(a, b) for i, a in enumerate(ids) for b in ids[i+1:]}
        if not keys:
            return

        iso = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        ns3_t = max(self.rssi_t, self.snr_t)
        for key in sorted(keys):
            a, b = key
            d = self._distance(a, b)
            self._sw.writerow([
                iso, f"{now - self.t0:.3f}", link_label(a, b), a, b,
                "" if d is None else f"{d:.3f}",
                self._fmt(self.loss.get(key)),
                self._fmt(self.rssi.get(key)),
                self._fmt(self.snr.get(key)),
                f"{now - self.pos_t:.3f}" if self.pos_t else "",
                f"{now - self.loss_t:.3f}" if self.loss_t else "",
                f"{now - ns3_t:.3f}" if ns3_t else ""])

            st = self.stats.setdefault(key, dict(
                n=0, blocked=0, snr_sum=0.0, snr_n=0,
                snr_min=None, snr_max=None, d_min=None, d_max=None,
                loss_max=0.0))
            st["n"] += 1
            if self.blocked.get(key):
                st["blocked"] += 1
            s = self.snr.get(key)
            if s is not None:
                st["snr_sum"] += s
                st["snr_n"] += 1
                st["snr_min"] = s if st["snr_min"] is None else min(st["snr_min"], s)
                st["snr_max"] = s if st["snr_max"] is None else max(st["snr_max"], s)
            if d is not None:
                st["d_min"] = d if st["d_min"] is None else min(st["d_min"], d)
                st["d_max"] = d if st["d_max"] is None else max(st["d_max"], d)
            lv = self.loss.get(key)
            if lv is not None:
                st["loss_max"] = max(st["loss_max"], lv)

        self.n_samples += 1
        self._sf.flush()

    def _heartbeat(self):
        """A recorder that quietly writes blank columns for ten minutes is
        worse than one that says nothing is arriving."""
        now = time.time()
        missing = []
        if not self.pos_t:                 missing.append("/uav_world_positions")
        if not self.loss_t:                missing.append("/link_obstacle_loss")
        if not (self.rssi_t or self.snr_t):missing.append("/ns3_link_* (NS-3)")
        if missing:
            self.get_logger().warn("no data yet on: " + ", ".join(missing))
            return
        stale = []
        for name, t in (("positions", self.pos_t), ("obstacle", self.loss_t),
                        ("ns3", max(self.rssi_t, self.snr_t))):
            if now - t > 3.0:
                stale.append(f"{name} {now-t:.0f}s")
        prefix = f"{self.n_samples} samples, {self.n_events} events"
        if stale:
            self.get_logger().warn(f"{prefix} -- STALE: {', '.join(stale)}")
        else:
            self.get_logger().info(f"{prefix}, all feeds live")

    # ── shutdown ─────────────────────────────────────────────────────────────
    def finish(self):
        with open(self.summary_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["link", "node_a", "node_b", "samples",
                        "pct_blocked", "distance_min_m", "distance_max_m",
                        "snr_min_db", "snr_mean_db", "snr_max_db",
                        "max_obstacle_loss_db"])
            for key in sorted(self.stats):
                a, b = key
                st = self.stats[key]
                mean = st["snr_sum"] / st["snr_n"] if st["snr_n"] else None
                w.writerow([
                    link_label(a, b), a, b, st["n"],
                    f"{100.0*st['blocked']/st['n']:.1f}" if st["n"] else "",
                    self._fmt(st["d_min"]), self._fmt(st["d_max"]),
                    self._fmt(st["snr_min"]), self._fmt(mean),
                    self._fmt(st["snr_max"]), f"{st['loss_max']:.3f}"])
        self._sf.close()
        self._ef.close()
        print(f"\nrecorded {self.n_samples} sample ticks, {self.n_events} link events")
        print(f"  samples -> {self.samples_path}")
        print(f"  events  -> {self.events_path}")
        print(f"  summary -> {self.summary_path}")


def main():
    ap = argparse.ArgumentParser(
        description="Passive CSV recorder for a live Gazebo + NS-3 run. "
                    "Publishes nothing; safe to run alongside a flight.")
    ap.add_argument("--outdir", default=OUTDIR_DEFAULT)
    ap.add_argument("--tag", default="", help="prefix for the output filenames")
    ap.add_argument("--rate", type=float, default=2.0,
                    help="sample rows per second (default 2). Link EVENTS are "
                         "always caught at the full feed rate regardless.")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="seconds to record; 0 = until Ctrl-C")
    ap.add_argument("--block-threshold", type=float, default=3.0,
                    help="dB above which a link counts as blocked; match "
                         "BlockThresholdDb in NS-3 (default 3.0)")
    ap.add_argument("--change-threshold", type=float, default=1.0,
                    help="dB change that logs a LOSS_CHANGE event (default 1.0)")
    args = ap.parse_args()

    rclpy.init()
    node = LinkRecorder(args)
    signal.signal(signal.SIGINT, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
