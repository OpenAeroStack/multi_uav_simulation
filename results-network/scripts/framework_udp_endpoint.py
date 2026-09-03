#!/usr/bin/env python3
"""Controlled UDP sender/receiver used across the simulated Wi-Fi path."""

from __future__ import annotations

import argparse
import csv
import signal
import socket
import struct
import time
from pathlib import Path


HEADER = struct.Struct("!Qd")  # sequence number, sender wall-clock timestamp
RUNNING = True


def stop(_signum, _frame) -> None:
    global RUNNING
    RUNNING = False


def sender(args: argparse.Namespace) -> None:
    if args.packet_size < HEADER.size:
        raise SystemExit(f"packet size must be at least {HEADER.size} bytes")
    interval = args.packet_size * 8.0 / (args.rate_mbps * 1e6)
    payload = bytes(args.packet_size - HEADER.size)
    destination = (args.host, args.port)
    deadline = time.monotonic() + args.duration
    sequence = 0
    next_send = time.monotonic()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, \
            args.output.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["sequence", "send_time_s", "packet_size"])
        last_flush = time.monotonic()
        while RUNNING and time.monotonic() < deadline:
            now = time.monotonic()
            if now < next_send:
                time.sleep(next_send - now)
            send_time = time.time()
            packet = HEADER.pack(sequence, send_time) + payload
            sock.sendto(packet, destination)
            writer.writerow([sequence, f"{send_time:.9f}", len(packet)])
            sequence += 1
            next_send += interval
            if time.monotonic() - last_flush >= 1.0:
                output.flush()
                last_flush = time.monotonic()
    print(f"sent_packets={sequence} rate_mbps={args.rate_mbps:g} "
          f"duration_s={args.duration:g}")


def receiver(args: argparse.Namespace) -> None:
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    last_packet = None
    received = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, \
            args.output.open("w", newline="") as output:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.bind((args.bind, args.port))
        sock.settimeout(0.2)
        writer = csv.writer(output)
        writer.writerow([
            "sequence", "send_time_s", "receive_time_s", "packet_size"])
        output.flush()
        while RUNNING and time.monotonic() - started < args.max_runtime:
            try:
                packet, _peer = sock.recvfrom(65535)
            except socket.timeout:
                if (last_packet is not None
                        and time.monotonic() - last_packet >= args.idle_timeout):
                    break
                continue
            receive_time = time.time()
            if len(packet) < HEADER.size:
                continue
            sequence, send_time = HEADER.unpack_from(packet)
            writer.writerow([
                sequence, f"{send_time:.9f}", f"{receive_time:.9f}", len(packet)])
            received += 1
            last_packet = time.monotonic()
            if received % 100 == 0:
                output.flush()
    print(f"received_packets={received}")


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    send = sub.add_parser("send")
    send.add_argument("--host", required=True)
    send.add_argument("--port", type=int, default=5202)
    send.add_argument("--duration", type=float, default=120.0)
    send.add_argument("--rate-mbps", type=float, default=1.0)
    send.add_argument("--packet-size", type=int, default=1200)
    send.add_argument("--output", type=Path, required=True)
    receive = sub.add_parser("receive")
    receive.add_argument("--bind", default="10.42.0.11")
    receive.add_argument("--port", type=int, default=5202)
    receive.add_argument("--max-runtime", type=float, default=135.0)
    receive.add_argument("--idle-timeout", type=float, default=3.0)
    receive.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if getattr(args, "duration", 1.0) <= 0 or getattr(args, "rate_mbps", 1.0) <= 0:
        parser.error("duration and rate must be greater than zero")
    return args


def main() -> None:
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    args = arguments()
    sender(args) if args.mode == "send" else receiver(args)


if __name__ == "__main__":
    main()
