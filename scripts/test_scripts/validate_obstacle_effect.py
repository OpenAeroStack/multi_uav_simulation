#!/usr/bin/env python3
"""Validate that Gazebo obstacles affect the NS-3 namespace data path.

This script starts no simulator and publishes no synthetic channel data. Run it
while scripts/launch_city_dds.sh is active, then move a UAV behind a tagged
obstacle and back into clear line of sight.

Modes:
  --observe          Record until Ctrl-C (or --duration), then require real
                     clear/blocked/channel/traffic evidence.
  --negative-control SIGSTOP the running NS-3 process, prove all GCS wireless
                     pings fail, then terminate NS-3 so the launcher can clean
                     up. This mode intentionally ends the running simulation.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import re
import shutil
import signal
import statistics
import subprocess
import sys
import threading
import time
from typing import Dict, Iterable, List, Optional, Tuple

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
except ImportError as exc:
    raise SystemExit(
        "ROS 2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
        "and the project workspace before running this script."
    ) from exc


REPO = Path(__file__).resolve().parents[2]
RESULT_ROOT = REPO / "test_logs"
NS3_LOG = Path("/tmp/ns3_stdout.log")
NS3_CHANNEL_CSV = Path("/tmp/ns3_link_validation.csv")
NAMESPACES = ("gcsns", "uav1", "uav2", "uav3")
TAPS = ("tap-gcs", "tap-uav1", "tap-uav2", "tap-uav3")
UAV_IPS = ((1, "10.42.0.11"), (2, "10.42.0.12"), (3, "10.42.0.13"))
LINKS = tuple((a, b) for a in range(4) for b in range(a + 1, 4))
TOPICS = (
    "/uav_world_positions",
    "/link_obstacle_loss",
    "/ns3_link_rssi",
    "/ns3_link_snr",
)
CLEAR_EPS_DB = 0.25
MAX_TEST_RATE_BIT_S = 500_000


def run(
    command: List[str], timeout: float = 10.0, check: bool = False
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
        check=check,
    )


def sudo(command: List[str], timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return run(["sudo", *command], timeout=timeout)


def median(values: Iterable[float]) -> Optional[float]:
    finite = [value for value in values if math.isfinite(value)]
    return statistics.median(finite) if finite else None


class Recorder(Node):
    def __init__(self, output: Path):
        super().__init__("obstacle_effect_validator")
        self.output = output
        self.lock = threading.Lock()
        self.latest_loss: Dict[Tuple[int, int], float] = {}
        self.latest_rssi: Dict[Tuple[int, int], float] = {}
        self.latest_snr: Dict[Tuple[int, int], float] = {}
        self.seen_position_ids: set[int] = set()
        self.sample_counts = {"loss": 0, "rssi": 0, "snr": 0}
        self.files = {}
        self.writers = {}
        for key, filename in (
            ("loss", "obstacle_loss_samples.csv"),
            ("rssi", "rssi_samples.csv"),
            ("snr", "snr_samples.csv"),
        ):
            handle = (output / filename).open("w", newline="", encoding="utf-8")
            writer = csv.writer(handle)
            writer.writerow(
                ["wall_time", "node_a", "node_b", f"{key}_value", "raw_obstacle_loss_db"]
            )
            self.files[key] = handle
            self.writers[key] = writer

        self.create_subscription(
            Float32MultiArray, "/uav_world_positions", self.on_positions, 10
        )
        self.create_subscription(
            Float32MultiArray, "/link_obstacle_loss", self.on_loss, 10
        )
        self.create_subscription(Float32MultiArray, "/ns3_link_rssi", self.on_rssi, 10)
        self.create_subscription(Float32MultiArray, "/ns3_link_snr", self.on_snr, 10)

    def on_positions(self, message: Float32MultiArray) -> None:
        with self.lock:
            for offset in range(0, len(message.data) - 3, 4):
                self.seen_position_ids.add(int(message.data[offset]))

    @staticmethod
    def triples(data: Iterable[float]) -> Iterable[Tuple[int, int, float]]:
        values = list(data)
        for offset in range(0, len(values) - 2, 3):
            a, b = sorted((int(values[offset]), int(values[offset + 1])))
            yield a, b, float(values[offset + 2])

    def on_loss(self, message: Float32MultiArray) -> None:
        now = time.time()
        with self.lock:
            for a, b, value in self.triples(message.data):
                self.latest_loss[(a, b)] = value
                self.writers["loss"].writerow([now, a, b, value, value])
                self.sample_counts["loss"] += 1
            self.files["loss"].flush()

    def record_metric(self, key: str, message: Float32MultiArray) -> None:
        now = time.time()
        with self.lock:
            for a, b, value in self.triples(message.data):
                loss = self.latest_loss.get((a, b), math.nan)
                if key == "rssi":
                    self.latest_rssi[(a, b)] = value
                elif key == "snr":
                    self.latest_snr[(a, b)] = value
                self.writers[key].writerow([now, a, b, value, loss])
                self.sample_counts[key] += 1
            self.files[key].flush()

    def on_rssi(self, message: Float32MultiArray) -> None:
        self.record_metric("rssi", message)

    def on_snr(self, message: Float32MultiArray) -> None:
        self.record_metric("snr", message)

    def loss_for(self, link: Tuple[int, int]) -> float:
        with self.lock:
            return self.latest_loss.get(link, math.nan)

    def close(self) -> None:
        for handle in self.files.values():
            handle.close()


class Validation:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        stamp = time.strftime("%Y%m%dT%H%M%S", time.gmtime())
        self.output = RESULT_ROOT / f"obstacle_effect_{stamp}_{os.getpid()}"
        self.output.mkdir(parents=True, exist_ok=False)
        self.results: List[Tuple[str, bool, str]] = []
        self.recorder: Optional[Recorder] = None
        self.executor: Optional[SingleThreadedExecutor] = None
        self.spin_thread: Optional[threading.Thread] = None
        self.ping_file = (self.output / "ping_results.csv").open(
            "w", newline="", encoding="utf-8"
        )
        self.ping_writer = csv.writer(self.ping_file)
        self.ping_writer.writerow(
            [
                "wall_time",
                "phase",
                "node_a",
                "node_b",
                "destination",
                "raw_obstacle_loss_db",
                "sent",
                "received",
                "loss_pct",
                "rtt_avg_ms",
            ]
        )

    def result(self, name: str, passed: bool, detail: str) -> None:
        self.results.append((name, passed, detail))
        print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}")

    def snapshot_network(self) -> None:
        destination = self.output / "namespace_interface_state.txt"
        commands = [
            ["sudo", "ip", "netns", "list"],
            ["ip", "-brief", "address"],
            ["ip", "route"],
            ["bridge", "link"],
        ]
        for namespace in NAMESPACES:
            commands.extend(
                [
                    ["sudo", "ip", "netns", "exec", namespace, "ip", "-brief", "address"],
                    ["sudo", "ip", "netns", "exec", namespace, "ip", "route"],
                    ["sudo", "ip", "netns", "exec", namespace, "ss", "-H", "-lntup"],
                ]
            )
        for tap in TAPS:
            commands.append(["ip", "-s", "link", "show", tap])

        with destination.open("w", encoding="utf-8") as handle:
            for command in commands:
                handle.write(f"$ {' '.join(command)}\n")
                try:
                    completed = run(command, timeout=10)
                    handle.write(completed.stdout)
                except (OSError, subprocess.TimeoutExpired) as exc:
                    handle.write(f"ERROR: {exc}\n")
                handle.write("\n")

    def check_namespaces(self) -> bool:
        completed = sudo(["ip", "netns", "list"])
        existing = {line.split()[0] for line in completed.stdout.splitlines() if line}
        missing = sorted(set(NAMESPACES) - existing)
        passed = completed.returncode == 0 and not missing
        self.result(
            "wireless namespaces",
            passed,
            "all four exist" if passed else f"missing: {', '.join(missing)}",
        )
        return passed

    def check_taps(self) -> bool:
        failures = []
        for tap in TAPS:
            base = Path("/sys/class/net") / tap
            if not base.exists():
                failures.append(f"{tap}: absent")
                continue
            try:
                carrier = (base / "carrier").read_text(encoding="utf-8").strip()
            except OSError as exc:
                failures.append(f"{tap}: cannot read carrier ({exc})")
                continue
            bridge_path = base / "master"
            if carrier != "1" or not bridge_path.exists():
                failures.append(
                    f"{tap}: carrier={carrier}, bridge={'yes' if bridge_path.exists() else 'no'}"
                )
        passed = not failures
        self.result(
            "TAP attachment",
            passed,
            "all TAPs have carrier and bridge masters"
            if passed
            else "; ".join(failures),
        )
        return passed

    def wait_for_topics(self) -> bool:
        assert self.recorder is not None
        deadline = time.monotonic() + self.args.timeout
        missing = list(TOPICS)
        while time.monotonic() < deadline:
            missing = [
                topic for topic in TOPICS if self.recorder.count_publishers(topic) < 1
            ]
            if not missing:
                break
            time.sleep(0.2)
        passed = not missing
        self.result(
            "root ROS publishers",
            passed,
            "all required topics active" if passed else f"missing: {', '.join(missing)}",
        )
        if not passed:
            return False

        payload_deadline = time.monotonic() + self.args.timeout
        while time.monotonic() < payload_deadline:
            with self.recorder.lock:
                ready = (
                    self.recorder.seen_position_ids.issuperset({0, 1, 2, 3})
                    and all(self.recorder.sample_counts[key] > 0 for key in ("loss", "rssi", "snr"))
                )
            if ready:
                break
            time.sleep(0.2)
        self.result(
            "root ROS samples",
            ready,
            "positions include IDs 0-3 and all link streams produced data"
            if ready
            else "publishers exist but complete samples did not arrive",
        )
        return ready

    def check_integration_log(self) -> bool:
        deadline = time.monotonic() + self.args.timeout
        text = ""
        while time.monotonic() < deadline:
            if NS3_LOG.exists():
                text = NS3_LOG.read_text(encoding="utf-8", errors="replace")
                if "***** INCOMPLETE FEED *****" in text:
                    break
                if (
                    "[integration check] OK: positions for all 4 nodes "
                    "and obstacle reports for all 6 links have been received."
                    in text
                ):
                    break
            time.sleep(0.2)
        excerpt_lines = [
            line
            for line in text.splitlines()
            if "integration check" in line
            or "INCOMPLETE FEED" in line
            or "No position ever" in line
            or "No obstacle report" in line
        ]
        (self.output / "ns3_log_excerpt.txt").write_text(
            "\n".join(excerpt_lines[-80:]) + ("\n" if excerpt_lines else ""),
            encoding="utf-8",
        )
        passed = (
            "positions for all 4 nodes and obstacle reports for all 6 links"
            in text
            and "***** INCOMPLETE FEED *****" not in text
        )
        self.result(
            "NS-3 integration check",
            passed,
            "all 4 nodes and 6 links received"
            if passed
            else "missing, timed out, or reported INCOMPLETE FEED",
        )
        return passed

    @staticmethod
    def parse_ping(output: str, sent: int) -> Tuple[int, float, float]:
        packet_match = re.search(
            r"(\d+) packets transmitted, (\d+) received.*?(\d+(?:\.\d+)?)% packet loss",
            output,
        )
        received = int(packet_match.group(2)) if packet_match else 0
        loss_pct = float(packet_match.group(3)) if packet_match else 100.0
        rtt_match = re.search(
            r"(?:rtt|round-trip) min/avg/max/(?:mdev|stddev) = "
            r"[0-9.]+/([0-9.]+)/",
            output,
        )
        rtt_avg = float(rtt_match.group(1)) if rtt_match else math.nan
        return received, loss_pct, rtt_avg

    def ping(self, node: int, ip: str, phase: str, count: int = 3) -> Tuple[int, float, float]:
        assert self.recorder is not None
        link = (0, node)
        raw_loss = self.recorder.loss_for(link)
        completed = sudo(
            [
                "ip",
                "netns",
                "exec",
                "gcsns",
                "ping",
                "-c",
                str(count),
                "-i",
                "0.25",
                "-W",
                "1",
                ip,
            ],
            timeout=count * 2 + 5,
        )
        received, loss_pct, rtt_avg = self.parse_ping(completed.stdout, count)
        self.ping_writer.writerow(
            [
                time.time(),
                phase,
                0,
                node,
                ip,
                raw_loss,
                count,
                received,
                loss_pct,
                rtt_avg,
            ]
        )
        self.ping_file.flush()
        return received, loss_pct, rtt_avg

    def initial_pings(self) -> bool:
        failures = []
        for node, ip in UAV_IPS:
            received, loss_pct, _ = self.ping(node, ip, "initial", count=3)
            if received == 0:
                failures.append(f"{ip}: {loss_pct:.0f}% loss")
        passed = not failures
        self.result(
            "GCS wireless pings with NS-3 running",
            passed,
            "all three UAVs replied" if passed else "; ".join(failures),
        )
        return passed

    def start_ros(self) -> None:
        rclpy.init()
        self.recorder = Recorder(self.output)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.recorder)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=False)
        self.spin_thread.start()

    def stop_ros(self) -> None:
        if self.executor is not None:
            self.executor.shutdown()
        if self.spin_thread is not None:
            self.spin_thread.join(timeout=5)
        if self.recorder is not None:
            self.recorder.destroy_node()
            self.recorder.close()
        if rclpy.ok():
            rclpy.shutdown()

    def prerequisites(self) -> bool:
        checks = [
            self.check_namespaces(),
            self.check_taps(),
            self.wait_for_topics(),
            self.check_integration_log(),
            self.initial_pings(),
        ]
        return all(checks)

    def observe(self) -> None:
        print(
            "Observe mode: move a UAV behind a tagged obstacle and back to clear "
            "line of sight. Press Ctrl-C to finish and validate."
        )
        started = time.monotonic()
        next_print = 0.0
        try:
            while self.args.duration <= 0 or time.monotonic() - started < self.args.duration:
                now = time.monotonic()
                if now >= next_print:
                    assert self.recorder is not None
                    with self.recorder.lock:
                        losses = dict(self.recorder.latest_loss)
                        rssi = dict(self.recorder.latest_rssi)
                        snr = dict(self.recorder.latest_snr)
                    formatted = " | ".join(
                        f"{a}-{b} loss={losses.get((a, b), math.nan):.2f}dB "
                        f"RSSI={rssi.get((a, b), math.nan):.2f}dBm "
                        f"SNR={snr.get((a, b), math.nan):.2f}dB"
                        for a, b in LINKS
                    )
                    print(formatted, flush=True)
                    next_print = now + 1.0
                for node, ip in UAV_IPS:
                    raw_loss = self.recorder.loss_for((0, node))  # type: ignore[union-attr]
                    phase = "blocked" if math.isfinite(raw_loss) and raw_loss > CLEAR_EPS_DB else "clear"
                    self.ping(node, ip, phase, count=2)
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("Observation stopped; evaluating captured evidence.")

    def read_ns3_rows(self) -> List[dict]:
        if not NS3_CHANNEL_CSV.exists():
            return []
        destination = self.output / "ns3_channel_samples.csv"
        shutil.copy2(NS3_CHANNEL_CSV, destination)
        with destination.open(newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))

    def evaluate_observation(self) -> None:
        rows = self.read_ns3_rows()
        grouped: Dict[Tuple[int, int], Dict[int, List[dict]]] = {}
        for row in rows:
            try:
                link = tuple(sorted((int(row["node_a"]), int(row["node_b"]))))
                state = int(row["blocked"])
                if link in LINKS and state in (0, 1):
                    grouped.setdefault(link, {0: [], 1: []})[state].append(row)
            except (KeyError, TypeError, ValueError):
                continue

        clear_exists = any(
            math.isfinite(float(row.get("obstacle_loss_db", "nan")))
            and abs(float(row["obstacle_loss_db"])) <= CLEAR_EPS_DB
            for row in rows
        )
        self.result(
            "clear-link evidence",
            clear_exists,
            f"at least one smoothed loss <= {CLEAR_EPS_DB} dB"
            if clear_exists
            else "no approximately-zero obstacle loss captured",
        )

        blocked_loss_exists = any(
            math.isfinite(float(row.get("obstacle_loss_db", "nan")))
            and float(row["obstacle_loss_db"]) > CLEAR_EPS_DB
            for row in rows
        )
        self.result(
            "positive obstacle-loss evidence",
            blocked_loss_exists,
            "at least one smoothed obstacle loss is greater than zero"
            if blocked_loss_exists
            else "no positive obstacle loss captured; put a UAV behind a tagged obstacle",
        )

        transitioned = {
            link: states
            for link, states in grouped.items()
            if states[0]
            and states[1]
            and any(abs(float(row["fading_m"]) - 3.0) < 1e-6 for row in states[0])
            and any(abs(float(row["fading_m"]) - 1.0) < 1e-6 for row in states[1])
        }
        state_detail = ", ".join(f"{a}-{b}" for a, b in transitioned) or "none"
        self.result(
            "LoS/NLoS transition",
            bool(transitioned),
            f"both blocked=0/m=3 and blocked=1/m=1 captured on: {state_detail}",
        )

        signal_links = []
        for link, states in transitioned.items():
            clear_rssi = median(float(row["faded_rx_dbm"]) for row in states[0])
            blocked_rssi = median(float(row["faded_rx_dbm"]) for row in states[1])
            clear_snr = median(float(row["snr_db"]) for row in states[0])
            blocked_snr = median(float(row["snr_db"]) for row in states[1])
            if (
                clear_rssi is not None
                and blocked_rssi is not None
                and clear_snr is not None
                and blocked_snr is not None
                and (clear_rssi - blocked_rssi > 0.5 or clear_snr - blocked_snr > 0.5)
            ):
                signal_links.append(
                    f"{link[0]}-{link[1]} "
                    f"RSSI {clear_rssi:.2f}->{blocked_rssi:.2f}, "
                    f"SNR {clear_snr:.2f}->{blocked_snr:.2f} dB"
                )
        self.result(
            "RSSI/SNR obstacle effect",
            bool(signal_links),
            "; ".join(signal_links) if signal_links else "no median decrease >0.5 dB",
        )

        ping_rows: Dict[Tuple[int, int], Dict[str, List[dict]]] = {}
        with (self.output / "ping_results.csv").open(
            newline="", encoding="utf-8"
        ) as handle:
            for row in csv.DictReader(handle):
                if row["phase"] not in ("clear", "blocked"):
                    continue
                link = (int(row["node_a"]), int(row["node_b"]))
                ping_rows.setdefault(link, {"clear": [], "blocked": []})[
                    row["phase"]
                ].append(row)

        traffic_effects = []
        for link, states in ping_rows.items():
            if not states["clear"] or not states["blocked"]:
                continue
            clear_success = statistics.mean(
                100.0 - float(row["loss_pct"]) for row in states["clear"]
            )
            blocked_success = statistics.mean(
                100.0 - float(row["loss_pct"]) for row in states["blocked"]
            )
            clear_rtt = median(float(row["rtt_avg_ms"]) for row in states["clear"])
            blocked_rtt = median(float(row["rtt_avg_ms"]) for row in states["blocked"])
            loss_effect = clear_success - blocked_success >= 10.0
            latency_effect = (
                clear_rtt is not None
                and blocked_rtt is not None
                and blocked_rtt > clear_rtt * 1.2
                and blocked_rtt - clear_rtt > 0.1
            )
            if loss_effect or latency_effect:
                traffic_effects.append(
                    f"{link[0]}-{link[1]} success {clear_success:.1f}%"
                    f"->{blocked_success:.1f}%, RTT {clear_rtt}->{blocked_rtt} ms"
                )
        self.result(
            "real namespace traffic effect",
            bool(traffic_effects),
            "; ".join(traffic_effects)
            if traffic_effects
            else "no GCS link had both states plus >=10-point loss or measurable RTT increase",
        )

    def find_ns3_pids(self) -> List[int]:
        completed = run(
            ["pgrep", "-u", str(os.getuid()), "-f", "[t]hree_uav_tapbridge_integrated"],
            timeout=5,
        )
        return [
            int(value)
            for value in completed.stdout.split()
            if value.isdigit() and int(value) != os.getpid()
        ]

    def negative_control(self) -> None:
        pids = self.find_ns3_pids()
        if not pids:
            self.result("negative-control NS-3 process", False, "no integrated NS-3 PID found")
            return
        print(f"Stopping NS-3 scheduling with SIGSTOP: {pids}")
        for pid in pids:
            os.kill(pid, signal.SIGSTOP)
        time.sleep(1.0)

        failures = []
        for node, ip in UAV_IPS:
            received, loss_pct, _ = self.ping(node, ip, "negative-control", count=3)
            if received != 0 or loss_pct < 100.0:
                failures.append(f"{ip}: received={received}, loss={loss_pct}%")
        self.result(
            "negative-control wireless isolation",
            not failures,
            "all three pings failed with NS-3 stopped"
            if not failures
            else "; ".join(failures),
        )

        # End the intentionally destructive mode cleanly. SIGCONT is required
        # before SIGTERM can be acted upon by a stopped process.
        for pid in pids:
            try:
                os.kill(pid, signal.SIGCONT)
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def write_summary(self) -> int:
        failures = [result for result in self.results if not result[1]]
        lines = [
            "# Obstacle-effect validation summary",
            "",
            f"- Mode: `{self.args.mode}`",
            f"- Result: **{'FAIL' if failures else 'PASS'}**",
            f"- Nominal test-traffic ceiling: `{MAX_TEST_RATE_BIT_S} bit/s`",
            "- Actual generated traffic: ICMP only, two or three small packets per link per cycle",
            "",
            "| Check | Result | Detail |",
            "|---|---|---|",
        ]
        for name, passed, detail in self.results:
            safe = detail.replace("|", "\\|").replace("\n", " ")
            lines.append(f"| {name} | {'PASS' if passed else 'FAIL'} | {safe} |")
        lines.extend(
            [
                "",
                "No success row is generated unless its corresponding live check passed.",
            ]
        )
        (self.output / "summary_report.md").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
        return 1 if failures else 0

    def execute(self) -> int:
        print(f"Results: {self.output}")
        try:
            sudo_auth = subprocess.run(["sudo", "-v"], timeout=60).returncode
        except (OSError, subprocess.TimeoutExpired):
            sudo_auth = 1
        if sudo_auth != 0:
            self.result("sudo authorization", False, "sudo -v failed")
            return self.write_summary()

        self.snapshot_network()
        self.start_ros()
        try:
            if not self.prerequisites():
                return self.write_summary()
            if self.args.mode == "observe":
                self.observe()
                self.evaluate_observation()
            else:
                self.negative_control()
            return self.write_summary()
        finally:
            self.stop_ros()
            self.ping_file.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--observe", dest="mode", action="store_const", const="observe")
    modes.add_argument(
        "--negative-control",
        dest="mode",
        action="store_const",
        const="negative-control",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="observe for this many seconds; 0 means until Ctrl-C",
    )
    parser.add_argument(
        "--timeout", type=float, default=30.0, help="startup check timeout in seconds"
    )
    args = parser.parse_args()
    if args.duration < 0 or args.timeout <= 0:
        parser.error("--duration must be >= 0 and --timeout must be > 0")
    return args


def main() -> int:
    os.environ["ROS_DOMAIN_ID"] = "0"
    os.environ["ROS2CLI_NO_DAEMON"] = "1"
    validation = Validation(parse_args())
    return validation.execute()


if __name__ == "__main__":
    sys.exit(main())
