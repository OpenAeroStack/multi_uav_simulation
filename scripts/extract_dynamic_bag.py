#!/usr/bin/env python3

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message


REQUIRED_TOPICS = {
    "/uav_world_positions",
    "/link_obstacle_loss",
    "/ns3_link_rssi",
    "/ns3_link_snr",
    "/cluster/primary_ch",
    "/cluster/backup_ch",
    "/cluster/scores",
    "/cluster/assignment",
    "/cluster/event",
}


def count_changes(samples):
    """Count actual value transitions, ignoring repeated publications."""
    changes = 0
    previous = None

    for _, value in samples:
        if previous is not None and value != previous:
            changes += 1
        previous = value

    return changes


def role_percentages(samples, experiment_end):
    """Estimate percentage of observed time assigned to each UAV."""
    if not samples:
        return {}

    durations = Counter()

    for index, (time_s, uav_id) in enumerate(samples):
        if index + 1 < len(samples):
            next_time = samples[index + 1][0]
        else:
            next_time = experiment_end

        duration = max(0.0, next_time - time_s)
        durations[uav_id] += duration

    total = sum(durations.values())

    if total <= 0:
        return {}

    return {
        uav_id: 100.0 * duration / total
        for uav_id, duration in durations.items()
    }


def main():
    parser = argparse.ArgumentParser(
        description="Extract multi-UAV dynamic clustering ROS bag data."
    )
    parser.add_argument("bag_path", help="Path to the ROS 2 bag directory")
    parser.add_argument("output_directory", help="CSV output directory")
    args = parser.parse_args()

    bag_path = Path(args.bag_path).expanduser().resolve()
    output_directory = Path(args.output_directory).expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)

    if not bag_path.exists():
        raise FileNotFoundError(f"Bag directory not found: {bag_path}")

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        ConverterOptions("", ""),
    )

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    missing_topics = REQUIRED_TOPICS.difference(topic_types)

    if missing_topics:
        print("Warning: the following topics are not present:")
        for topic in sorted(missing_topics):
            print(f"  {topic}")

    positions_file = open(
        output_directory / "positions.csv",
        "w",
        newline="",
        encoding="utf-8",
    )
    links_file = open(
        output_directory / "network_links.csv",
        "w",
        newline="",
        encoding="utf-8",
    )
    roles_file = open(
        output_directory / "cluster_roles.csv",
        "w",
        newline="",
        encoding="utf-8",
    )
    events_file = open(
        output_directory / "cluster_events.csv",
        "w",
        newline="",
        encoding="utf-8",
    )
    scores_file = open(
        output_directory / "cluster_scores_raw.csv",
        "w",
        newline="",
        encoding="utf-8",
    )
    assignments_file = open(
        output_directory / "cluster_assignments.csv",
        "w",
        newline="",
        encoding="utf-8",
    )

    positions_writer = csv.writer(positions_file)
    links_writer = csv.writer(links_file)
    roles_writer = csv.writer(roles_file)
    events_writer = csv.writer(events_file)
    scores_writer = csv.writer(scores_file)
    assignments_writer = csv.writer(assignments_file)

    positions_writer.writerow(
        ["time_s", "node_id", "x_m", "y_m", "z_m"]
    )
    links_writer.writerow(
        ["time_s", "metric", "source", "destination", "value"]
    )
    roles_writer.writerow(
        ["time_s", "role", "uav_id"]
    )
    events_writer.writerow(
        ["time_s", "event"]
    )
    scores_writer.writerow(
        ["time_s", "raw_score_array"]
    )
    assignments_writer.writerow(
        ["time_s", "assignment"]
    )

    first_timestamp = None
    last_timestamp = None

    primary_samples = []
    backup_samples = []
    event_count = 0
    score_count = 0

    while reader.has_next():
        topic, serialized_data, timestamp = reader.read_next()

        if first_timestamp is None:
            first_timestamp = timestamp

        last_timestamp = timestamp
        time_s = (timestamp - first_timestamp) / 1_000_000_000.0

        if topic not in REQUIRED_TOPICS:
            continue

        message_type = get_message(topic_types[topic])
        message = deserialize_message(serialized_data, message_type)

        if topic == "/uav_world_positions":
            values = list(message.data)

            for index in range(0, len(values), 4):
                if index + 3 >= len(values):
                    break

                positions_writer.writerow(
                    [
                        f"{time_s:.6f}",
                        int(values[index]),
                        values[index + 1],
                        values[index + 2],
                        values[index + 3],
                    ]
                )

        elif topic in {
            "/link_obstacle_loss",
            "/ns3_link_rssi",
            "/ns3_link_snr",
        }:
            metric_names = {
                "/link_obstacle_loss": "obstacle_loss_db",
                "/ns3_link_rssi": "rssi_dbm",
                "/ns3_link_snr": "snr_db",
            }

            values = list(message.data)

            for index in range(0, len(values), 3):
                if index + 2 >= len(values):
                    break

                links_writer.writerow(
                    [
                        f"{time_s:.6f}",
                        metric_names[topic],
                        int(values[index]),
                        int(values[index + 1]),
                        values[index + 2],
                    ]
                )

        elif topic == "/cluster/primary_ch":
            uav_id = int(message.data)
            primary_samples.append((time_s, uav_id))
            roles_writer.writerow(
                [f"{time_s:.6f}", "primary", uav_id]
            )

        elif topic == "/cluster/backup_ch":
            uav_id = int(message.data)
            backup_samples.append((time_s, uav_id))
            roles_writer.writerow(
                [f"{time_s:.6f}", "backup", uav_id]
            )

        elif topic == "/cluster/event":
            event_count += 1
            events_writer.writerow(
                [f"{time_s:.6f}", message.data]
            )

        elif topic == "/cluster/scores":
            score_count += 1
            scores_writer.writerow(
                [
                    f"{time_s:.6f}",
                    json.dumps(list(message.data)),
                ]
            )

        elif topic == "/cluster/assignment":
            assignments_writer.writerow(
                [f"{time_s:.6f}", message.data]
            )

    positions_file.close()
    links_file.close()
    roles_file.close()
    events_file.close()
    scores_file.close()
    assignments_file.close()

    if first_timestamp is None or last_timestamp is None:
        raise RuntimeError("No messages were found in the ROS bag.")

    duration_s = (
        last_timestamp - first_timestamp
    ) / 1_000_000_000.0

    primary_changes = count_changes(primary_samples)
    backup_changes = count_changes(backup_samples)

    primary_percentages = role_percentages(
        primary_samples,
        duration_s,
    )
    backup_percentages = role_percentages(
        backup_samples,
        duration_s,
    )

    trial_name = bag_path.parent.parent.name

    summary_path = output_directory / "summary.csv"

    with open(
        summary_path,
        "w",
        newline="",
        encoding="utf-8",
    ) as summary_file:
        writer = csv.writer(summary_file)

        writer.writerow(
            [
                "trial",
                "duration_s",
                "primary_samples",
                "backup_samples",
                "score_samples",
                "event_messages",
                "primary_changes",
                "backup_changes",
                "initial_primary",
                "final_primary",
                "initial_backup",
                "final_backup",
                "uav1_primary_time_percent",
                "uav2_primary_time_percent",
                "uav3_primary_time_percent",
            ]
        )

        writer.writerow(
            [
                trial_name,
                f"{duration_s:.6f}",
                len(primary_samples),
                len(backup_samples),
                score_count,
                event_count,
                primary_changes,
                backup_changes,
                primary_samples[0][1] if primary_samples else "",
                primary_samples[-1][1] if primary_samples else "",
                backup_samples[0][1] if backup_samples else "",
                backup_samples[-1][1] if backup_samples else "",
                f"{primary_percentages.get(1, 0.0):.3f}",
                f"{primary_percentages.get(2, 0.0):.3f}",
                f"{primary_percentages.get(3, 0.0):.3f}",
            ]
        )

    print(f"Extraction completed: {trial_name}")
    print(f"Duration: {duration_s:.3f} s")
    print(f"Primary-head changes: {primary_changes}")
    print(f"Backup-head changes: {backup_changes}")

    if primary_samples:
        print(
            f"Primary head: UAV{primary_samples[0][1]} "
            f"-> UAV{primary_samples[-1][1]}"
        )

    if backup_samples:
        print(
            f"Backup head: UAV{backup_samples[0][1]} "
            f"-> UAV{backup_samples[-1][1]}"
        )

    print(f"Results saved to: {output_directory}")


if __name__ == "__main__":
    main()
