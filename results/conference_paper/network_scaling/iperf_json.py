#!/usr/bin/env python3
"""Version-tolerant iperf3 UDP JSON parsing for network scaling."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


class IperfSchemaError(ValueError):
    """Raised when required UDP metrics cannot be identified safely."""


def _required(summary: dict[str, Any], fields: tuple[str, ...], label: str,
              end_keys: list[str]) -> None:
    missing = [field for field in fields if field not in summary]
    if missing:
        raise IperfSchemaError(
            f"{label} missing required fields {missing}; available end keys: {end_keys}")


def _udp_streams(end: dict[str, Any], sender: bool) -> list[dict[str, Any]]:
    output = []
    for stream in end.get("streams", []):
        udp = stream.get("udp") if isinstance(stream, dict) else None
        if isinstance(udp, dict) and bool(udp.get("sender")) is sender:
            output.append(udp)
    return output


def _sum_streams(streams: list[dict[str, Any]], end_keys: list[str],
                 label: str) -> dict[str, float | int]:
    if not streams:
        raise IperfSchemaError(
            f"no {label} UDP stream summary; available end keys: {end_keys}")
    for stream in streams:
        _required(stream, ("bytes", "bits_per_second", "packets", "seconds"),
                  f"{label} UDP stream", end_keys)
    return {
        "bytes": sum(int(stream["bytes"]) for stream in streams),
        "bits_per_second": sum(float(stream["bits_per_second"]) for stream in streams),
        "packets": sum(int(stream["packets"]) for stream in streams),
        "seconds": max(float(stream["seconds"]) for stream in streams),
    }


def parse_udp_json(path: Path) -> dict[str, float | int | str]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if "error" in data:
        raise IperfSchemaError(f"iperf3 error in {path}: {data['error']}")
    end = data.get("end")
    if not isinstance(end, dict):
        raise IperfSchemaError(f"missing end object in {path}; top-level keys: {list(data)}")
    end_keys = list(end)

    # Newer iperf3 releases expose an explicit sender summary. Iperf 3.9 on
    # this host instead exposes sender=true UDP streams plus one combined sum.
    explicit_sent = end.get("sum_sent")
    if isinstance(explicit_sent, dict):
        _required(explicit_sent, ("bytes", "bits_per_second", "packets"),
                  "end.sum_sent", end_keys)
        sender = explicit_sent
        sender_layout = "end.sum_sent"
    else:
        streams = _udp_streams(end, True)
        if streams:
            sender = _sum_streams(streams, end_keys, "sender")
            sender_layout = "end.streams[].udp[sender=true]"
        elif isinstance(end.get("sum"), dict) and end["sum"].get("sender") is True:
            sender = end["sum"]
            _required(sender, ("bytes", "bits_per_second", "packets"),
                      "end.sum sender summary", end_keys)
            sender_layout = "end.sum[sender=true]"
        else:
            raise IperfSchemaError(
                f"cannot identify UDP sender summary; available end keys: {end_keys}")

    sender_bytes = int(sender["bytes"])
    sender_packets = int(sender["packets"])
    if sender_bytes <= 0 or sender_packets <= 0:
        raise IperfSchemaError(
            f"invalid sender bytes/packets ({sender_bytes}/{sender_packets}); "
            f"available end keys: {end_keys}")

    # Receiver preference order: explicit client summary (new releases), an
    # embedded server summary, then iperf 3.9's combined end.sum. In 3.9 the
    # combined sum retains sender bytes/rate but carries receiver duration,
    # lost_packets, lost_percent and jitter. With iperf's fixed UDP datagram
    # size, receiver bytes and goodput are deterministically recoverable.
    receiver: dict[str, Any] | None = None
    receiver_layout = ""
    combined_39 = False
    if isinstance(end.get("sum_received"), dict):
        receiver = end["sum_received"]
        receiver_layout = "end.sum_received"
    else:
        embedded = data.get("server_output_json")
        embedded_end = embedded.get("end", {}) if isinstance(embedded, dict) else {}
        if isinstance(embedded_end.get("sum_received"), dict):
            receiver = embedded_end["sum_received"]
            receiver_layout = "server_output_json.end.sum_received"
        elif isinstance(embedded_end.get("sum"), dict):
            receiver = embedded_end["sum"]
            receiver_layout = "server_output_json.end.sum"
        elif isinstance(end.get("sum"), dict):
            receiver = end["sum"]
            receiver_layout = "end.sum combined UDP summary"
            combined_39 = True

    if receiver is None:
        raise IperfSchemaError(
            f"cannot identify UDP receiver summary; available end keys: {end_keys}")
    _required(receiver, ("seconds", "packets", "lost_packets", "jitter_ms"),
              receiver_layout, end_keys)
    total_packets = int(receiver["packets"])
    lost_packets = int(receiver["lost_packets"])
    receiver_seconds = float(receiver["seconds"])
    if total_packets <= 0 or lost_packets < 0 or lost_packets > total_packets or receiver_seconds <= 0:
        raise IperfSchemaError(
            f"invalid receiver packet/duration fields in {receiver_layout}; "
            f"available end keys: {end_keys}")
    receiver_packets = total_packets - lost_packets

    if combined_39 or int(receiver.get("bytes", 0)) <= 0:
        bytes_per_datagram = sender_bytes / sender_packets
        receiver_bytes = bytes_per_datagram * receiver_packets
        receiver_bps = receiver_bytes * 8.0 / receiver_seconds
        receiver_layout += "+fixed-datagram derivation"
    else:
        _required(receiver, ("bytes", "bits_per_second"), receiver_layout, end_keys)
        receiver_bytes = float(receiver["bytes"])
        receiver_bps = float(receiver["bits_per_second"])

    return {
        "actual_sender_mbps": float(sender["bits_per_second"]) / 1e6,
        "sender_bytes": sender_bytes,
        "sender_datagrams": sender_packets,
        "receiver_bytes": int(round(receiver_bytes)),
        "receiver_datagrams": receiver_packets,
        "received_goodput_mbps": receiver_bps / 1e6,
        "lost_datagrams": lost_packets,
        "packet_loss_ratio": lost_packets / total_packets,
        "jitter_ms": float(receiver["jitter_ms"]),
        "sender_schema": sender_layout,
        "receiver_schema": receiver_layout,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Parse iperf3 UDP JSON robustly.")
    parser.add_argument("json_path", type=Path)
    parser.add_argument("--smoke-min-goodput-mbps", type=float)
    args = parser.parse_args()
    metrics = parse_udp_json(args.json_path)
    print(json.dumps(metrics, sort_keys=True))
    if (args.smoke_min_goodput_mbps is not None and
            float(metrics["received_goodput_mbps"]) < args.smoke_min_goodput_mbps):
        raise SystemExit(
            f"FAIL: received goodput {metrics['received_goodput_mbps']:.3f} Mbit/s "
            f"below {args.smoke_min_goodput_mbps:.3f} Mbit/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
