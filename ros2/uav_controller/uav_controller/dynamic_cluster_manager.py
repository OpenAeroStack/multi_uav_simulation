#!/usr/bin/env python3

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Int32, String


LinkKey = Tuple[int, int]
Vector3 = Tuple[float, float, float]


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def make_link_key(node_a: int, node_b: int) -> LinkKey:
    return min(node_a, node_b), max(node_a, node_b)


class DynamicClusterManager(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_cluster_manager")

        # Fleet configuration.
        self.declare_parameter("num_uavs", 3)

        # Election configuration.
        self.declare_parameter("election_period_sec", 2.0)
        self.declare_parameter("metric_timeout_sec", 5.0)
        self.declare_parameter("minimum_ch_hold_sec", 10.0)
        self.declare_parameter("switch_margin", 0.12)
        self.declare_parameter("consecutive_wins", 3)

        # Link thresholds.
        self.declare_parameter("candidate_min_gcs_snr_db", 3.0)
        self.declare_parameter("ch_failure_gcs_snr_db", -2.0)
        self.declare_parameter("member_min_snr_db", 5.0)

        # Normalization configuration.
        self.declare_parameter("snr_min_db", -10.0)
        self.declare_parameter("snr_max_db", 30.0)
        self.declare_parameter("max_obstacle_loss_db", 30.0)
        self.declare_parameter("relative_speed_reference_mps", 10.0)

        # Weighted clustering score.
        self.declare_parameter("weight_gcs", 0.40)
        self.declare_parameter("weight_neighbors", 0.30)
        self.declare_parameter("weight_mobility", 0.20)
        self.declare_parameter("weight_obstacle", 0.10)

        self.num_uavs = int(self.get_parameter("num_uavs").value)

        if self.num_uavs < 1:
            raise ValueError(
                f"num_uavs must be >= 1; received {self.num_uavs}"
            )

        self.election_period = float(
            self.get_parameter("election_period_sec").value
        )
        self.metric_timeout = float(
            self.get_parameter("metric_timeout_sec").value
        )
        self.minimum_ch_hold = float(
            self.get_parameter("minimum_ch_hold_sec").value
        )
        self.switch_margin = float(
            self.get_parameter("switch_margin").value
        )
        self.consecutive_wins = int(
            self.get_parameter("consecutive_wins").value
        )

        self.candidate_min_gcs_snr = float(
            self.get_parameter("candidate_min_gcs_snr_db").value
        )
        self.ch_failure_gcs_snr = float(
            self.get_parameter("ch_failure_gcs_snr_db").value
        )
        self.member_min_snr = float(
            self.get_parameter("member_min_snr_db").value
        )

        self.snr_min = float(self.get_parameter("snr_min_db").value)
        self.snr_max = float(self.get_parameter("snr_max_db").value)
        self.max_obstacle_loss = float(
            self.get_parameter("max_obstacle_loss_db").value
        )
        self.relative_speed_reference = float(
            self.get_parameter("relative_speed_reference_mps").value
        )

        raw_weights = {
            "gcs": float(self.get_parameter("weight_gcs").value),
            "neighbors": float(
                self.get_parameter("weight_neighbors").value
            ),
            "mobility": float(
                self.get_parameter("weight_mobility").value
            ),
            "obstacle": float(
                self.get_parameter("weight_obstacle").value
            ),
        }

        total_weight = sum(raw_weights.values())

        if total_weight <= 0.0:
            raise ValueError("Cluster score weights must be positive.")

        self.weights = {
            name: value / total_weight
            for name, value in raw_weights.items()
        }

        self.snr: Dict[LinkKey, float] = {}
        self.rssi: Dict[LinkKey, float] = {}
        self.obstacle_loss: Dict[LinkKey, float] = {}

        self.positions: Dict[int, Vector3] = {}
        self.velocities: Dict[int, Vector3] = {}
        self.position_times: Dict[int, float] = {}

        self.last_update: Dict[str, float] = {}

        self.primary_ch = 0
        self.backup_ch = 0
        self.cluster_epoch = 0
        self.primary_since = time.monotonic()

        self.current_challenger = 0
        self.challenger_wins = 0

        # Backup-CH hysteresis (mirrors the primary's challenger streak). Without
        # this the backup is just backup_candidates[0] every tick, so ns-3 fading
        # jitter flips it between near-equal candidates on every election.
        self.backup_challenger = 0
        self.backup_challenger_wins = 0

        self.last_waiting_log = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(
            Float32MultiArray,
            "/ns3_link_snr",
            self.snr_callback,
            sensor_qos,
        )

        self.create_subscription(
            Float32MultiArray,
            "/ns3_link_rssi",
            self.rssi_callback,
            sensor_qos,
        )

        self.create_subscription(
            Float32MultiArray,
            "/link_obstacle_loss",
            self.obstacle_callback,
            sensor_qos,
        )

        self.create_subscription(
            Float32MultiArray,
            "/uav_world_positions",
            self.position_callback,
            sensor_qos,
        )

        self.assignment_publisher = self.create_publisher(
            String,
            "/cluster/assignment",
            state_qos,
        )

        self.event_publisher = self.create_publisher(
            String,
            "/cluster/event",
            state_qos,
        )

        self.score_publisher = self.create_publisher(
            Float32MultiArray,
            "/cluster/scores",
            state_qos,
        )

        self.primary_publisher = self.create_publisher(
            Int32,
            "/cluster/primary_ch",
            state_qos,
        )

        self.backup_publisher = self.create_publisher(
            Int32,
            "/cluster/backup_ch",
            state_qos,
        )

        self.create_timer(
            self.election_period,
            self.election_callback,
        )

        self.get_logger().info(
            f"Dynamic cluster manager started for {self.num_uavs} UAVs"
        )

    def parse_link_array(
        self,
        data: List[float],
        destination: Dict[LinkKey, float],
    ) -> None:
        for index in range(0, len(data) - 2, 3):
            node_a = int(round(data[index]))
            node_b = int(round(data[index + 1]))
            value = float(data[index + 2])

            if node_a == node_b:
                continue

            if not (
                0 <= node_a <= self.num_uavs
                and 0 <= node_b <= self.num_uavs
            ):
                continue

            destination[make_link_key(node_a, node_b)] = value

    def snr_callback(self, msg: Float32MultiArray) -> None:
        self.parse_link_array(msg.data, self.snr)
        self.last_update["snr"] = time.monotonic()

    def rssi_callback(self, msg: Float32MultiArray) -> None:
        self.parse_link_array(msg.data, self.rssi)
        self.last_update["rssi"] = time.monotonic()

    def obstacle_callback(self, msg: Float32MultiArray) -> None:
        self.parse_link_array(msg.data, self.obstacle_loss)
        self.last_update["obstacle"] = time.monotonic()

    def position_callback(self, msg: Float32MultiArray) -> None:
        now = time.monotonic()
        data = msg.data

        for index in range(0, len(data) - 3, 4):
            node_id = int(round(data[index]))

            if not 0 <= node_id <= self.num_uavs:
                continue

            new_position = (
                float(data[index + 1]),
                float(data[index + 2]),
                float(data[index + 3]),
            )

            previous_position = self.positions.get(node_id)
            previous_time = self.position_times.get(node_id)

            if previous_position is not None and previous_time is not None:
                delta_time = now - previous_time

                if 0.02 < delta_time < 5.0:
                    self.velocities[node_id] = (
                        (new_position[0] - previous_position[0]) / delta_time,
                        (new_position[1] - previous_position[1]) / delta_time,
                        (new_position[2] - previous_position[2]) / delta_time,
                    )
            else:
                self.velocities[node_id] = (0.0, 0.0, 0.0)

            self.positions[node_id] = new_position
            self.position_times[node_id] = now

        self.last_update["positions"] = now

    def get_link(
        self,
        values: Dict[LinkKey, float],
        node_a: int,
        node_b: int,
        default: float,
    ) -> float:
        return values.get(
            make_link_key(node_a, node_b),
            default,
        )

    def normalize_snr(self, snr_db: float) -> float:
        denominator = self.snr_max - self.snr_min

        if denominator <= 0:
            return 0.0

        return clamp(
            (snr_db - self.snr_min) / denominator,
            0.0,
            1.0,
        )

    def required_links(self) -> List[LinkKey]:
        links: List[LinkKey] = []

        for node_a in range(self.num_uavs + 1):
            for node_b in range(node_a + 1, self.num_uavs + 1):
                links.append((node_a, node_b))

        return links

    def metrics_ready(self) -> Tuple[bool, str]:
        now = time.monotonic()

        for metric in ("snr", "obstacle", "positions"):
            update_time = self.last_update.get(metric)

            if update_time is None:
                return False, f"waiting_for_{metric}"

            if now - update_time > self.metric_timeout:
                return False, f"stale_{metric}"

        missing_snr = [
            link
            for link in self.required_links()
            if link not in self.snr
        ]

        if missing_snr:
            return False, f"missing_snr_links={missing_snr}"

        missing_obstacle = [
            link
            for link in self.required_links()
            if link not in self.obstacle_loss
        ]

        if missing_obstacle:
            return False, f"missing_obstacle_links={missing_obstacle}"

        return True, "ready"

    def mobility_stability(self, uav_id: int) -> float:
        own_velocity = self.velocities.get(
            uav_id,
            (0.0, 0.0, 0.0),
        )

        relative_speeds: List[float] = []

        for neighbor in range(1, self.num_uavs + 1):
            if neighbor == uav_id:
                continue

            neighbor_velocity = self.velocities.get(
                neighbor,
                (0.0, 0.0, 0.0),
            )

            relative_speed = math.sqrt(
                (own_velocity[0] - neighbor_velocity[0]) ** 2
                + (own_velocity[1] - neighbor_velocity[1]) ** 2
                + (own_velocity[2] - neighbor_velocity[2]) ** 2
            )

            relative_speeds.append(relative_speed)

        if not relative_speeds:
            return 1.0

        average_relative_speed = (
            sum(relative_speeds) / len(relative_speeds)
        )

        reference = max(self.relative_speed_reference, 0.1)

        return math.exp(-average_relative_speed / reference)

    def calculate_score(self, uav_id: int) -> Dict[str, float]:
        gcs_snr = self.get_link(
            self.snr,
            0,
            uav_id,
            -100.0,
        )

        gcs_rssi = self.get_link(
            self.rssi,
            0,
            uav_id,
            -200.0,
        )

        neighbor_snrs: List[float] = []

        for neighbor in range(1, self.num_uavs + 1):
            if neighbor == uav_id:
                continue

            neighbor_snrs.append(
                self.get_link(
                    self.snr,
                    uav_id,
                    neighbor,
                    -100.0,
                )
            )

        reachable_neighbors = sum(
            1
            for value in neighbor_snrs
            if value >= self.member_min_snr
        )

        if neighbor_snrs:

            average_neighbor_quality = (
                sum(
                    self.normalize_snr(value)
                    for value in neighbor_snrs
                )
                / len(neighbor_snrs)
            )

            neighbor_coverage = (
                reachable_neighbors
                / len(neighbor_snrs)
            )

        else:

            # N=1:
            # there are no UAV-to-UAV neighbors to evaluate.
            #
            # Treat this criterion as satisfied rather than
            # dividing by zero or permanently disqualifying UAV1.
            average_neighbor_quality = 1.0
            neighbor_coverage = 1.0

        neighbor_score = (
            0.70 * average_neighbor_quality
            + 0.30 * neighbor_coverage
        )

        relevant_losses = [
            self.get_link(
                self.obstacle_loss,
                0,
                uav_id,
                self.max_obstacle_loss,
            )
        ]

        for neighbor in range(1, self.num_uavs + 1):
            if neighbor == uav_id:
                continue

            relevant_losses.append(
                self.get_link(
                    self.obstacle_loss,
                    uav_id,
                    neighbor,
                    self.max_obstacle_loss,
                )
            )

        average_loss = sum(relevant_losses) / len(relevant_losses)

        obstacle_robustness = 1.0 - clamp(
            average_loss / max(self.max_obstacle_loss, 0.1),
            0.0,
            1.0,
        )

        mobility_score = self.mobility_stability(uav_id)
        gcs_score = self.normalize_snr(gcs_snr)

        total_score = (
            self.weights["gcs"] * gcs_score
            + self.weights["neighbors"] * neighbor_score
            + self.weights["mobility"] * mobility_score
            + self.weights["obstacle"] * obstacle_robustness
        )

        has_required_neighbor = (
            self.num_uavs == 1
            or reachable_neighbors >= 1
        )

        is_candidate = (
            gcs_snr >= self.candidate_min_gcs_snr
            and has_required_neighbor
        )

        return {
            "score": total_score,
            "candidate": 1.0 if is_candidate else 0.0,
            "gcs_snr_db": gcs_snr,
            "gcs_rssi_dbm": gcs_rssi,
            "neighbor_score": neighbor_score,
            "reachable_neighbors": float(reachable_neighbors),
            "mobility_stability": mobility_score,
            "average_obstacle_loss_db": average_loss,
            "obstacle_robustness": obstacle_robustness,
        }

    def create_assignments(
        self,
        score_details: Dict[int, Dict[str, float]],
    ) -> List[Dict[str, object]]:
        assignments: List[Dict[str, object]] = []

        for uav_id in range(1, self.num_uavs + 1):
            if uav_id == self.primary_ch:
                role = "PRIMARY_CH"
                parent = 0
                route = [0, uav_id]

            elif uav_id == self.backup_ch:
                role = "BACKUP_CH"
                parent = 0
                route = [0, uav_id]

            elif self.primary_ch == 0:
                role = "DISCONNECTED"
                parent = 0
                route = []

            else:
                primary_snr = self.get_link(
                    self.snr,
                    uav_id,
                    self.primary_ch,
                    -100.0,
                )

                backup_snr = (
                    self.get_link(
                        self.snr,
                        uav_id,
                        self.backup_ch,
                        -100.0,
                    )
                    if self.backup_ch != 0
                    else -100.0
                )

                if primary_snr >= self.member_min_snr:
                    role = "MEMBER"
                    parent = self.primary_ch
                    route = [0, self.primary_ch, uav_id]

                elif backup_snr >= self.member_min_snr:
                    role = "MEMBER"
                    parent = self.backup_ch
                    route = [0, self.backup_ch, uav_id]

                elif (
                    score_details[uav_id]["gcs_snr_db"]
                    >= self.candidate_min_gcs_snr
                ):
                    role = "DIRECT_MEMBER"
                    parent = 0
                    route = [0, uav_id]

                else:
                    role = "DISCONNECTED"
                    parent = 0
                    route = []

            assignments.append(
                {
                    "uav_id": uav_id,
                    "role": role,
                    "parent": parent,
                    "route": route,
                    "score": round(
                        score_details[uav_id]["score"],
                        4,
                    ),
                    "gcs_snr_db": round(
                        score_details[uav_id]["gcs_snr_db"],
                        2,
                    ),
                }
            )

        return assignments

    def publish_state(
        self,
        score_details: Dict[int, Dict[str, float]],
        reason: str,
        changed: bool,
        old_primary: int,
        old_backup: int,
    ) -> None:
        assignments = self.create_assignments(score_details)

        state = {
            "status": "ACTIVE" if self.primary_ch else "NO_GATEWAY",
            "num_uavs": self.num_uavs,
            "epoch": self.cluster_epoch,
            "primary_ch": self.primary_ch,
            "backup_ch": self.backup_ch,
            "assignments": assignments,
        }

        assignment_message = String()
        assignment_message.data = json.dumps(
            state,
            sort_keys=True,
        )
        self.assignment_publisher.publish(assignment_message)

        score_message = Float32MultiArray()

        # Layout per UAV:
        # [id, score, candidate, GCS SNR, GCS RSSI,
        #  neighbor score, mobility stability, obstacle robustness]
        for uav_id in range(1, self.num_uavs + 1):
            details = score_details[uav_id]

            score_message.data.extend(
                [
                    float(uav_id),
                    float(details["score"]),
                    float(details["candidate"]),
                    float(details["gcs_snr_db"]),
                    float(details["gcs_rssi_dbm"]),
                    float(details["neighbor_score"]),
                    float(details["mobility_stability"]),
                    float(details["obstacle_robustness"]),
                ]
            )

        self.score_publisher.publish(score_message)

        primary_message = Int32()
        primary_message.data = self.primary_ch
        self.primary_publisher.publish(primary_message)

        backup_message = Int32()
        backup_message.data = self.backup_ch
        self.backup_publisher.publish(backup_message)

        if changed:
            event = {
                "epoch": self.cluster_epoch,
                "reason": reason,
                "old_primary": old_primary,
                "new_primary": self.primary_ch,
                "old_backup": old_backup,
                "new_backup": self.backup_ch,
            }

            event_message = String()
            event_message.data = json.dumps(
                event,
                sort_keys=True,
            )
            self.event_publisher.publish(event_message)

            self.get_logger().info(
                f"Epoch {self.cluster_epoch}: "
                f"primary=UAV{self.primary_ch}, "
                f"backup=UAV{self.backup_ch}, "
                f"reason={reason}"
            )

    def election_callback(self) -> None:
        ready, reason = self.metrics_ready()

        if not ready:
            now = time.monotonic()

            if now - self.last_waiting_log >= 5.0:
                self.get_logger().warning(
                    f"Waiting for clustering metrics: {reason}"
                )
                self.last_waiting_log = now

            return

        score_details = {
            uav_id: self.calculate_score(uav_id)
            for uav_id in range(1, self.num_uavs + 1)
        }

        candidates = sorted(
            [
                uav_id
                for uav_id, details in score_details.items()
                if bool(details["candidate"])
            ],
            key=lambda uav_id: (
                -score_details[uav_id]["score"],
                uav_id,
            ),
        )

        proposed_primary = candidates[0] if candidates else 0

        old_primary = self.primary_ch
        old_backup = self.backup_ch
        reason = "stable"

        current_failed = False

        if self.primary_ch != 0:
            current_failed = (
                score_details[self.primary_ch]["gcs_snr_db"]
                < self.ch_failure_gcs_snr
            )

        if self.primary_ch == 0:
            self.primary_ch = proposed_primary
            reason = "initial_election"

        elif current_failed:
            self.primary_ch = proposed_primary
            reason = "primary_link_failure"
            self.current_challenger = 0
            self.challenger_wins = 0

        elif (
            proposed_primary != 0
            and proposed_primary != self.primary_ch
        ):
            challenger_score = score_details[
                proposed_primary
            ]["score"]

            current_score = score_details[
                self.primary_ch
            ]["score"]

            held_long_enough = (
                time.monotonic() - self.primary_since
                >= self.minimum_ch_hold
            )

            clearly_better = (
                challenger_score
                > current_score + self.switch_margin
            )

            if held_long_enough and clearly_better:
                if self.current_challenger == proposed_primary:
                    self.challenger_wins += 1
                else:
                    self.current_challenger = proposed_primary
                    self.challenger_wins = 1

                if self.challenger_wins >= self.consecutive_wins:
                    self.primary_ch = proposed_primary
                    reason = "better_candidate_stable"
                    self.current_challenger = 0
                    self.challenger_wins = 0
            else:
                self.current_challenger = 0
                self.challenger_wins = 0

        else:
            self.current_challenger = 0
            self.challenger_wins = 0

        if self.primary_ch != old_primary:
            self.primary_since = time.monotonic()

        backup_candidates = [
            candidate
            for candidate in candidates
            if candidate != self.primary_ch
        ]

        proposed_backup = backup_candidates[0] if backup_candidates else 0

        if old_backup == self.primary_ch or old_backup not in backup_candidates:
            # Current backup is gone (promoted, failed, or no longer a
            # candidate) — take the best available immediately.
            self.backup_ch = proposed_backup
            self.backup_challenger = 0
            self.backup_challenger_wins = 0
        elif proposed_backup and proposed_backup != old_backup:
            # A different candidate now leads: only switch if it stays clearly
            # ahead of the incumbent for consecutive_wins ticks.
            clearly_better = (
                score_details[proposed_backup]["score"]
                > score_details[old_backup]["score"] + self.switch_margin
            )
            if clearly_better:
                if self.backup_challenger == proposed_backup:
                    self.backup_challenger_wins += 1
                else:
                    self.backup_challenger = proposed_backup
                    self.backup_challenger_wins = 1
            else:
                self.backup_challenger = 0
                self.backup_challenger_wins = 0

            if self.backup_challenger_wins >= self.consecutive_wins:
                self.backup_ch = proposed_backup
                self.backup_challenger = 0
                self.backup_challenger_wins = 0
            else:
                self.backup_ch = old_backup
        else:
            # Incumbent still leads (or ties) — keep it.
            self.backup_ch = old_backup
            self.backup_challenger = 0
            self.backup_challenger_wins = 0

        changed = (
            self.primary_ch != old_primary
            or self.backup_ch != old_backup
        )

        if changed:
            self.cluster_epoch += 1

            if (
                self.primary_ch == old_primary
                and self.backup_ch != old_backup
            ):
                reason = "backup_reselected"

        self.publish_state(
            score_details,
            reason,
            changed,
            old_primary,
            old_backup,
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = DynamicClusterManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
