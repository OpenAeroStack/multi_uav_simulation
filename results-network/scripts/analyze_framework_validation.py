#!/usr/bin/env python3
"""Analyze integrated Gazebo/ROS 2/ns-3/TapBridge framework behavior."""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


RESULTS_ROOT = Path(__file__).resolve().parents[1]
NUMERIC_COLUMNS = [
    "wall_time_s", "timestamp_s", "tx_node", "rx_node", "distance_m",
    "obstacle_loss_db", "path_loss_only_rssi_dbm",
    "after_obstacle_rssi_dbm", "faded_rssi_dbm", "fading_db", "snr_db",
    "nakagami_m",
    "obstacle_report_received", "position_report_received",
    "phy_tx_packets", "phy_rx_packets", "phy_drop_packets",
    "phy_tx_bytes", "phy_rx_bytes", "interval_phy_tx_packets",
    "interval_phy_rx_packets", "interval_phy_drop_packets",
    "interval_phy_tx_bytes", "interval_phy_rx_bytes",
    "interval_phy_drop_rate", "interval_phy_rx_throughput_mbps",
]


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Framework-level obstruction and network response analysis")
    parser.add_argument("framework_csv", type=Path)
    parser.add_argument("--sent-csv", type=Path)
    parser.add_argument("--received-csv", type=Path)
    parser.add_argument("--ping-log", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--settling-seconds", type=float, default=2.0)
    parser.add_argument("--event-window-seconds", type=float, default=5.0)
    parser.add_argument("--sustained-seconds", type=float, default=1.0)
    return parser.parse_args()


def output_path(path: Path) -> Path:
    resolved = path.expanduser().resolve()
    try:
        resolved.relative_to(RESULTS_ROOT)
    except ValueError:
        raise SystemExit(f"Output must be inside {RESULTS_ROOT}: {resolved}")
    resolved.mkdir(parents=True, exist_ok=True)
    return resolved


def load_framework(path: Path) -> pd.DataFrame:
    path = path.expanduser().resolve()
    if not path.is_file():
        raise SystemExit(f"Framework CSV not found: {path}")
    frame = pd.read_csv(path)
    missing = [column for column in NUMERIC_COLUMNS if column not in frame]
    if missing:
        raise SystemExit(
            "Framework CSV lacks the new synchronized fields: " + ", ".join(missing))
    for column in NUMERIC_COLUMNS:
        frame[column] = pd.to_numeric(frame[column], errors="coerce")
    frame = frame.replace([np.inf, -np.inf], np.nan)
    frame = frame.dropna(subset=NUMERIC_COLUMNS + ["los_state"])
    frame = frame[(frame["obstacle_report_received"] == 1)
                  & (frame["position_report_received"] == 1)]
    frame["los_state"] = frame["los_state"].astype(str).str.strip()
    frame = frame[frame["los_state"].isin(["LoS", "NLoS"])]
    frame = frame.sort_values("timestamp_s").drop_duplicates("timestamp_s")
    if frame.empty:
        raise SystemExit("No rows remain after removing invalid/startup samples")
    return frame.reset_index(drop=True)


def load_traffic(sent_path: Path | None, received_path: Path | None,
                 framework: pd.DataFrame) -> tuple[pd.DataFrame, pd.DataFrame]:
    if sent_path is None and received_path is None:
        return pd.DataFrame(), pd.DataFrame()
    if sent_path is None or received_path is None:
        raise SystemExit("Provide both --sent-csv and --received-csv")
    sent = pd.read_csv(sent_path.expanduser()).drop_duplicates("sequence")
    received = pd.read_csv(received_path.expanduser()).drop_duplicates("sequence")
    for column in ["sequence", "send_time_s", "packet_size"]:
        sent[column] = pd.to_numeric(sent[column], errors="coerce")
    for column in ["sequence", "send_time_s", "receive_time_s", "packet_size"]:
        received[column] = pd.to_numeric(received[column], errors="coerce")
    sent = sent.dropna().sort_values("send_time_s")
    received = received.dropna().sort_values("receive_time_s")
    received_sequences = set(received["sequence"].astype(np.int64))
    sent["received"] = sent["sequence"].astype(np.int64).isin(received_sequences)
    sent["wall_bin"] = np.floor(sent["send_time_s"])
    received["wall_bin"] = np.floor(received["receive_time_s"])

    delivery = sent.groupby("wall_bin").agg(
        sent_packets=("sequence", "count"),
        received_packets=("received", "sum"),
        sent_bytes=("packet_size", "sum"),
    )
    delivery["lost_packets"] = delivery["sent_packets"] - delivery["received_packets"]
    delivery["pdr"] = delivery["received_packets"] / delivery["sent_packets"]
    rx = received.groupby("wall_bin").agg(
        rx_packets=("sequence", "count"), rx_bytes=("packet_size", "sum"))
    traffic = delivery.join(rx, how="outer").fillna(0).reset_index()
    traffic["throughput_mbps"] = traffic["rx_bytes"] * 8.0 / 1e6
    traffic["packet_loss_percent"] = 100.0 * (1.0 - traffic["pdr"])
    traffic["wall_time_s"] = traffic["wall_bin"] + 0.5
    traffic["timestamp_s"] = np.interp(
        traffic["wall_time_s"], framework["wall_time_s"], framework["timestamp_s"])
    state_lookup = pd.merge_asof(
        traffic.sort_values("timestamp_s"),
        framework[["timestamp_s", "los_state"]].sort_values("timestamp_s"),
        on="timestamp_s", direction="nearest")
    traffic["los_state"] = state_lookup["los_state"].to_numpy()
    return traffic, received


def load_ping(path: Path | None, framework: pd.DataFrame) -> pd.DataFrame:
    if path is None or not path.expanduser().is_file():
        return pd.DataFrame(columns=["wall_time_s", "timestamp_s", "rtt_ms"])
    pattern = re.compile(r"^\[(\d+(?:\.\d+)?)\].*time[=<]([0-9.]+)\s*ms")
    rows = []
    for line in path.expanduser().read_text(errors="replace").splitlines():
        match = pattern.search(line)
        if match:
            rows.append((float(match.group(1)), float(match.group(2))))
    ping = pd.DataFrame(rows, columns=["wall_time_s", "rtt_ms"])
    if not ping.empty:
        ping["timestamp_s"] = np.interp(
            ping["wall_time_s"], framework["wall_time_s"], framework["timestamp_s"])
        states = pd.merge_asof(
            ping.sort_values("timestamp_s"),
            framework[["timestamp_s", "los_state"]].sort_values("timestamp_s"),
            on="timestamp_s", direction="nearest")
        ping["los_state"] = states["los_state"].to_numpy()
    return ping


def sustained_transitions(frame: pd.DataFrame, duration: float) -> list[tuple[float, str, str]]:
    times = frame["timestamp_s"].to_numpy()
    states = frame["los_state"].to_numpy()
    transitions: list[tuple[float, str, str]] = []
    for index in range(1, len(frame)):
        if states[index] == states[index - 1]:
            continue
        end = np.searchsorted(times, times[index] + duration, side="left")
        if end < len(states) and np.all(states[index:end + 1] == states[index]):
            transitions.append((float(times[index]), str(states[index - 1]), str(states[index])))
    return transitions


def stable_mask(frame: pd.DataFrame, transitions: list[tuple[float, str, str]],
                settling: float) -> np.ndarray:
    keep = np.ones(len(frame), dtype=bool)
    time_values = frame["timestamp_s"].to_numpy()
    for transition_time, _old, _new in transitions:
        keep &= np.abs(time_values - transition_time) > settling
    return keep


def regime_metrics(frame: pd.DataFrame, traffic: pd.DataFrame,
                   ping: pd.DataFrame) -> dict[str, dict[str, float]]:
    result: dict[str, dict[str, float]] = {}
    for state in ["LoS", "NLoS"]:
        group = frame[frame["los_state"] == state]
        tg = traffic[traffic["los_state"] == state] if not traffic.empty else traffic
        if group.empty:
            continue
        drop_events = float(group["interval_phy_drop_packets"].sum())
        rx_events = float(group["interval_phy_rx_packets"].sum())
        pdr = (float(tg["received_packets"].sum()) / float(tg["sent_packets"].sum())
               if not tg.empty and tg["sent_packets"].sum() > 0 else math.nan)
        if not ping.empty:
            pg = ping[ping["los_state"] == state]
            rtt = float(pg["rtt_ms"].mean()) if not pg.empty else math.nan
        else:
            rtt = math.nan
        result[state] = {
            "samples": float(len(group)),
            "distance_m": float(group["distance_m"].mean()),
            "obstacle_loss_db": float(group["obstacle_loss_db"].mean()),
            "nakagami_m": float(group["nakagami_m"].median()),
            "rssi_dbm": float(group["faded_rssi_dbm"].mean()),
            "snr_db": float(group["snr_db"].mean()),
            "phy_drop_rate_percent": (100.0 * drop_events / (drop_events + rx_events)
                                      if drop_events + rx_events > 0 else math.nan),
            "pdr_percent": 100.0 * pdr if math.isfinite(pdr) else math.nan,
            "throughput_mbps": (float(tg["rx_bytes"].sum()) * 8.0
                                / max(len(tg), 1) / 1e6 if not tg.empty else math.nan),
            "rtt_ms": rtt,
        }
    return result


def window_metrics(frame: pd.DataFrame, traffic: pd.DataFrame,
                   start: float, end: float) -> dict[str, float]:
    group = frame[(frame["timestamp_s"] >= start) & (frame["timestamp_s"] < end)]
    tg = traffic[(traffic["timestamp_s"] >= start)
                 & (traffic["timestamp_s"] < end)] if not traffic.empty else traffic
    drops = float(group["interval_phy_drop_packets"].sum())
    rx = float(group["interval_phy_rx_packets"].sum())
    sent = float(tg["sent_packets"].sum()) if not tg.empty else 0.0
    received = float(tg["received_packets"].sum()) if not tg.empty else 0.0
    return {
        "obstacle_loss_db": float(group["obstacle_loss_db"].mean()),
        "rssi_dbm": float(group["faded_rssi_dbm"].mean()),
        "snr_db": float(group["snr_db"].mean()),
        "phy_drops": drops,
        "pdr_percent": 100.0 * received / sent if sent > 0 else math.nan,
        "throughput_mbps": (float(tg["rx_bytes"].sum()) * 8.0
                            / max(end - start, 1e-9) / 1e6 if not tg.empty else math.nan),
        "phy_drop_rate_percent": 100.0 * drops / (drops + rx) if drops + rx > 0 else math.nan,
    }


def plot_timeline(frame: pd.DataFrame, traffic: pd.DataFrame,
                  transitions: list[tuple[float, str, str]], output: Path) -> None:
    time_values = frame["timestamp_s"].to_numpy()
    fig, axes = plt.subplots(8, 1, figsize=(13, 18), sharex=True)
    axes[0].plot(time_values, frame["distance_m"], color="tab:blue")
    axes[0].set_ylabel("Distance\n(m)")
    axes[1].plot(time_values, frame["obstacle_loss_db"], color="tab:brown")
    axes[1].set_ylabel("Obstacle\nloss (dB)")
    axes[2].step(time_values, (frame["los_state"] == "NLoS").astype(int), where="post")
    axes[2].set_yticks([0, 1], ["LoS", "NLoS"])
    axes[2].set_ylabel("State")
    axes[3].step(time_values, frame["nakagami_m"], where="post", color="tab:purple")
    axes[3].set_ylabel("Nakagami m")
    axes[4].plot(time_values, frame["faded_rssi_dbm"], lw=0.7, label="RSSI")
    axes[4].plot(time_values, frame["after_obstacle_rssi_dbm"], lw=1.4,
                 label="deterministic baseline")
    axes[4].set_ylabel("RSSI (dBm)")
    axes[4].legend(loc="best", fontsize=8)
    axes[5].plot(time_values, frame["interval_phy_drop_packets"], color="tab:red")
    axes[5].set_ylabel("PHY drops\nper interval")
    if not traffic.empty:
        axes[6].plot(traffic["timestamp_s"], traffic["pdr"] * 100.0,
                     marker=".", color="tab:green")
        axes[7].plot(traffic["timestamp_s"], traffic["throughput_mbps"],
                     marker=".", color="tab:cyan")
    axes[6].set_ylabel("App PDR (%)")
    axes[6].set_ylim(-5, 105)
    axes[7].set_ylabel("App Rx\n(Mbps)")
    axes[7].set_xlabel("Simulation time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
        for transition_time, old, new in transitions:
            axis.axvline(transition_time, color="black", ls="--", lw=1)
            if axis is axes[0]:
                axis.text(transition_time, axis.get_ylim()[1], f" {old}→{new}",
                          va="top", fontsize=8)
    fig.suptitle("Integrated framework response to simulated obstruction")
    fig.tight_layout()
    fig.savefig(output / "framework_validation_timeline.png", dpi=160)
    plt.close(fig)


def plot_comparison(metrics: dict[str, dict[str, float]], output: Path) -> None:
    definitions = [
        ("distance_m", "Mean distance (m)"),
        ("obstacle_loss_db", "Obstacle loss (dB)"),
        ("rssi_dbm", "Mean RSSI (dBm)"),
        ("snr_db", "Mean SNR (dB)"),
        ("phy_drop_rate_percent", "PHY drop rate (%)"),
        ("pdr_percent", "Application PDR (%)"),
        ("throughput_mbps", "Rx throughput (Mbps)"),
        ("rtt_ms", "RTT (ms)"),
    ]
    fig, axes = plt.subplots(2, 4, figsize=(14, 7))
    for axis, (key, title) in zip(axes.flat, definitions):
        states = [state for state in ["LoS", "NLoS"] if state in metrics]
        values = [metrics[state][key] for state in states]
        axis.bar(states, values, color=["tab:blue" if s == "LoS" else "tab:orange"
                                        for s in states])
        axis.set_title(title)
        axis.grid(axis="y", alpha=0.25)
        if not values or all(not math.isfinite(value) for value in values):
            axis.text(0.5, 0.5, "not recorded", transform=axis.transAxes,
                      ha="center", va="center")
    fig.suptitle("LoS versus NLoS integrated-framework metrics")
    fig.tight_layout()
    fig.savefig(output / "los_vs_nlos_framework_metrics.png", dpi=160)
    plt.close(fig)


def plot_nakagami(frame: pd.DataFrame, output: Path) -> None:
    fig, axis = plt.subplots(figsize=(8, 5))
    for state, colour in [("LoS", "tab:blue"), ("NLoS", "tab:orange")]:
        group = frame[frame["los_state"] == state]
        if group.empty:
            continue
        power = np.power(10.0, group["fading_db"].to_numpy() / 10.0)
        power /= np.mean(power)
        mean, variance = float(np.mean(power)), float(np.var(power, ddof=1))
        estimate = mean * mean / variance if math.isfinite(variance) and variance > 0.0 else math.nan
        estimate_label = f"{estimate:.2f}" if math.isfinite(estimate) else "undefined (zero variance)"
        axis.hist(power, bins=60, range=(0, 5), density=True, alpha=0.35,
                  color=colour, label=f"{state}: configured m={group['nakagami_m'].median():g}, "
                                      f"moments m={estimate_label}")
    axis.set_xlabel("Normalized received power P / E[P]")
    axis.set_ylabel("Probability density")
    axis.set_title("Secondary Nakagami verification (periodic model samples)")
    axis.grid(alpha=0.25)
    if axis.has_data():
        axis.legend()
    fig.tight_layout()
    fig.savefig(output / "nakagami_los_vs_nlos.png", dpi=160)
    plt.close(fig)


def value(number: float, decimals: int = 2) -> str:
    return f"{number:.{decimals}f}" if math.isfinite(number) else "N/A"


def summary_text(metrics: dict[str, dict[str, float]],
                 transitions: list[tuple[float, str, str]],
                 before: dict[str, float] | None,
                 after: dict[str, float] | None) -> str:
    lines = [
        "FRAMEWORK VALIDATION SUMMARY",
        "----------------------------",
        "",
        "Metric                         LoS             NLoS",
        "-----------------------------------------------------",
    ]
    definitions = [
        ("samples", "Samples", 0), ("distance_m", "Mean distance (m)", 2),
        ("obstacle_loss_db", "Obstacle loss (dB)", 2),
        ("nakagami_m", "Nakagami m", 2), ("rssi_dbm", "Mean RSSI (dBm)", 2),
        ("snr_db", "Mean SNR (dB)", 2),
        ("phy_drop_rate_percent", "PHY drop rate (%)", 2),
        ("pdr_percent", "Application PDR (%)", 2),
        ("throughput_mbps", "Throughput (Mbps)", 3), ("rtt_ms", "RTT (ms)", 2),
    ]
    for key, label, decimals in definitions:
        los = metrics.get("LoS", {}).get(key, math.nan)
        nlos = metrics.get("NLoS", {}).get(key, math.nan)
        lines.append(f"{label:<30} {value(los, decimals):>10}      {value(nlos, decimals):>10}")

    if "LoS" in metrics and "NLoS" in metrics:
        los, nlos = metrics["LoS"], metrics["NLoS"]
        distance_difference = nlos["distance_m"] - los["distance_m"]
        lines += ["", "RELATIVE CHANGES (NLoS minus LoS)",
                  f"Distance difference:       {distance_difference:+.2f} m",
                  f"RSSI change:               {nlos['rssi_dbm'] - los['rssi_dbm']:+.2f} dB",
                  f"SNR change:                {nlos['snr_db'] - los['snr_db']:+.2f} dB"]
        if math.isfinite(los["pdr_percent"]) and math.isfinite(nlos["pdr_percent"]):
            lines.append(f"PDR change:                {nlos['pdr_percent'] - los['pdr_percent']:+.2f} points")
        if (math.isfinite(los["phy_drop_rate_percent"])
                and math.isfinite(nlos["phy_drop_rate_percent"])):
            lines.append(
                f"PHY drop-rate change:      "
                f"{nlos['phy_drop_rate_percent'] - los['phy_drop_rate_percent']:+.2f} points")
        if (math.isfinite(los["throughput_mbps"])
                and math.isfinite(nlos["throughput_mbps"])):
            lines.append(
                f"Throughput change:         "
                f"{nlos['throughput_mbps'] - los['throughput_mbps']:+.3f} Mbps")
        if abs(distance_difference) > max(5.0, 0.1 * los["distance_m"]):
            lines += ["", f"WARNING: mean LoS distance = {los['distance_m']:.1f} m and mean "
                      f"NLoS distance = {nlos['distance_m']:.1f} m.",
                      "Observed degradation includes both distance and obstacle effects."]

    if transitions:
        lines += ["", "TRANSITIONS"]
        lines.extend(f"{old} -> {new} at t={time_value:.2f} s"
                     for time_value, old, new in transitions)
    if before is not None and after is not None:
        lines += ["", "FIRST SUSTAINED LoS -> NLoS EVENT",
                  "Metric                         Before          After",
                  "-----------------------------------------------------"]
        for key, label in [
            ("obstacle_loss_db", "Obstacle loss (dB)"), ("rssi_dbm", "RSSI (dBm)"),
            ("snr_db", "SNR (dB)"), ("phy_drops", "PHY drops"),
            ("phy_drop_rate_percent", "PHY drop rate (%)"),
            ("pdr_percent", "Application PDR (%)"),
            ("throughput_mbps", "Throughput (Mbps)"),
        ]:
            lines.append(f"{label:<30} {value(before[key]):>10}      {value(after[key]):>10}")
        lines += ["", "The observed network changes are temporally aligned with the "
                  "Gazebo-derived blockage event; this is not a claim of strict "
                  "physical causality."]
    else:
        lines += ["", "No sustained LoS -> NLoS transition was detected."]
    lines += ["", "Channel/RSSI values are periodic simulated link evaluations.",
              "PHY counters come from actual WifiPhy trace events.",
              "PDR and throughput come from actual sequenced UDP traffic through TapBridge."]
    return "\n".join(lines) + "\n"


def main() -> int:
    args = arguments()
    output = output_path(args.output_dir)
    framework = load_framework(args.framework_csv)
    traffic, received = load_traffic(args.sent_csv, args.received_csv, framework)
    ping = load_ping(args.ping_log, framework)
    transitions = sustained_transitions(framework, args.sustained_seconds)
    keep = stable_mask(framework, transitions, args.settling_seconds)
    stable = framework[keep].copy()
    stable_traffic = traffic.copy()
    stable_ping = ping.copy()
    if not stable_traffic.empty:
        for transition_time, _old, _new in transitions:
            stable_traffic = stable_traffic[
                np.abs(stable_traffic["timestamp_s"] - transition_time)
                > args.settling_seconds]
    if not stable_ping.empty:
        for transition_time, _old, _new in transitions:
            stable_ping = stable_ping[
                np.abs(stable_ping["timestamp_s"] - transition_time)
                > args.settling_seconds]
    metrics = regime_metrics(stable, stable_traffic, stable_ping)

    first_blockage = next((event for event in transitions
                           if event[1] == "LoS" and event[2] == "NLoS"), None)
    before = after = None
    if first_blockage:
        transition_time = first_blockage[0]
        window = args.event_window_seconds
        before = window_metrics(framework, traffic, transition_time - window,
                                transition_time)
        after_start = transition_time + args.settling_seconds
        after = window_metrics(framework, traffic, after_start, after_start + window)

    text = summary_text(metrics, transitions, before, after)
    print(text)
    (output / "summary.txt").write_text(text, encoding="utf-8")
    pd.DataFrame(metrics).T.to_csv(output / "los_vs_nlos_metrics.csv")
    if not traffic.empty:
        traffic.to_csv(output / "aligned_packet_metrics.csv", index=False)
    plot_timeline(framework, traffic, transitions, output)
    plot_comparison(metrics, output)
    plot_nakagami(stable, output)
    print(f"Outputs written to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
