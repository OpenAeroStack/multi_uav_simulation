#!/usr/bin/env python3
"""
Real-time networking dashboard for the multi-UAV city simulation.

Shows, for a fleet of any size from 1 to 5 UAVs plus the ground station:

  * a 2D map of the fleet with the live link topology drawn on it
  * every link's distance, SNR, RSSI, obstacle loss and up/down state
  * the dynamic clustering state: epoch, primary/backup cluster head, and the
    full per-UAV score breakdown that drove the election
  * which members are currently relaying through the cluster head
  * SNR and obstacle-loss history

Fleet size is discovered at runtime. Nothing here is hardcoded to three UAVs:
the node set comes from whatever appears on /uav_world_positions and
/cluster/assignment, so starting the simulation with 4 or 5 UAVs just makes
more nodes, links, table rows and traces appear.

This subscribes with rclpy rather than shelling out to `ros2 topic echo`, so it
must run where the topics are visible — that is, inside the gcsns namespace:

    sudo ip netns exec gcsns sudo -H -u $USER bash -c '
        source /opt/ros/humble/setup.bash
        source ~/FYP/multi_uav_simulation/ros2/install/setup.bash
        python3 scripts/viz_dashboard.py'

IMPORTANT — you cannot open http://localhost:8050 straight away. A network
namespace has its own complete network stack, so the port above is bound to
*gcsns's* loopback, not the one your browser uses, and the root namespace
deliberately holds no address on 10.42.0.0/24. The browser gets
ERR_CONNECTION_REFUSED. In a second terminal, run:

    scripts/dashboard_bridge.sh

which relays the port across the namespace boundary through a UNIX socket.
THEN open http://localhost:8050.

Requirements: pip install dash plotly
"""

import argparse
import json
import math
import threading
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String


MAX_UAVS = 5
HISTORY = 240          # samples kept per timeline (~4 min at 1 Hz)
STALE_SEC = 5.0        # a feed older than this is shown as stale

# Matches ns-3: below this an incoming frame cannot be decoded.
RX_SENSITIVITY_DBM = -82.0
# Mirrors dynamic_cluster_manager's member_min_snr_db default.
USABLE_SNR_DB = 5.0

# GitHub-dark palette, shared by every panel so colour means one thing.
BG = "#0d1117"
PANEL = "#161b22"
BORDER = "#30363d"
TEXT = "#e6edf3"
MUTED = "#8b949e"
GOOD = "#3fb950"
WARN = "#e3b341"
BAD = "#f85149"
ACCENT = "#58a6ff"
RELAY = "#bc8cff"

# One stable colour per node so a UAV keeps its identity across every panel.
NODE_COLORS = {
    0: "#f0883e",   # GCS
    1: "#58a6ff",
    2: "#3fb950",
    3: "#e3b341",
    4: "#ff7b72",
    5: "#bc8cff",
}


def node_label(node_id: int) -> str:
    return "GCS" if node_id == 0 else f"UAV{node_id}"


def link_key(a: int, b: int) -> Tuple[int, int]:
    return (min(a, b), max(a, b))


# ── ROS side ──────────────────────────────────────────────────────────────────

class NetworkMonitor(Node):
    """Collects every networking signal the simulation publishes."""

    def __init__(self, forced_num_uavs: int = 0) -> None:
        super().__init__("viz_dashboard")

        self.lock = threading.Lock()
        self.forced_num_uavs = forced_num_uavs

        # node id -> (x, y, z)
        self.positions: Dict[int, Tuple[float, float, float]] = {}
        # link -> value
        self.snr: Dict[Tuple[int, int], float] = {}
        self.rssi: Dict[Tuple[int, int], float] = {}
        self.obstacle: Dict[Tuple[int, int], float] = {}
        # uav -> score breakdown
        self.scores: Dict[int, Dict[str, float]] = {}

        self.primary_ch = 0
        self.backup_ch = 0
        self.epoch = 0
        self.cluster_status = "WAITING"
        self.assignments: Dict[str, object] = {}
        self.relayed: Dict[int, int] = {}
        self.events: deque = deque(maxlen=12)

        # uav -> per-drone telemetry straight off the bridges
        self.rel_alt: Dict[int, float] = {}
        self.mode: Dict[int, str] = {}
        self.armed: Dict[int, bool] = {}

        # timeline history
        self.times: deque = deque(maxlen=HISTORY)
        self.snr_history: Dict[Tuple[int, int], deque] = {}
        self.obstacle_history: Dict[Tuple[int, int], deque] = {}

        self.last_seen: Dict[str, float] = {}
        # Running maximum of node ids actually seen. Grows only, so a UAV that
        # boots late does not make the fleet size flicker mid-run.
        self.discovered_uavs = 0
        # Fleet size as declared by the cluster manager, which is authoritative
        # and can shrink. Preferred over discovered_uavs while it is fresh.
        self.reported_uavs = 0
        self.reported_uavs_t = 0.0

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
            Float32MultiArray, "/uav_world_positions",
            self._on_positions, sensor_qos)
        self.create_subscription(
            Float32MultiArray, "/ns3_link_snr",
            self._make_link_cb("snr", self.snr), sensor_qos)
        self.create_subscription(
            Float32MultiArray, "/ns3_link_rssi",
            self._make_link_cb("rssi", self.rssi), sensor_qos)
        self.create_subscription(
            Float32MultiArray, "/link_obstacle_loss",
            self._make_link_cb("obstacle", self.obstacle), sensor_qos)

        self.create_subscription(
            Float32MultiArray, "/cluster/scores",
            self._on_scores, state_qos)
        self.create_subscription(
            Int32, "/cluster/primary_ch", self._on_primary, state_qos)
        self.create_subscription(
            Int32, "/cluster/backup_ch", self._on_backup, state_qos)
        self.create_subscription(
            String, "/cluster/assignment", self._on_assignment, state_qos)
        self.create_subscription(
            String, "/cluster/event", self._on_event, state_qos)
        self.create_subscription(
            String, "/cluster/relay", self._on_relay, state_qos)

        # Per-UAV telemetry. Subscribing for the maximum fleet costs nothing
        # when the topics do not exist, and means a UAV that appears later is
        # picked up without a restart.
        for uav_id in range(1, MAX_UAVS + 1):
            self.create_subscription(
                Float32, f"/uav{uav_id}/rel_alt",
                self._make_alt_cb(uav_id), 10)
            self.create_subscription(
                String, f"/uav{uav_id}/mode",
                self._make_mode_cb(uav_id), 10)
            self.create_subscription(
                Bool, f"/uav{uav_id}/armed",
                self._make_armed_cb(uav_id), 10)

        self.create_timer(1.0, self._sample_history)

        self.get_logger().info(
            "Dashboard monitor started "
            f"({'fleet forced to ' + str(forced_num_uavs) if forced_num_uavs else 'fleet auto-detected'})"
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_positions(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        with self.lock:
            for index in range(0, len(data) - 3, 4):
                node_id = int(round(data[index]))
                self.positions[node_id] = (
                    float(data[index + 1]),
                    float(data[index + 2]),
                    float(data[index + 3]),
                )
            uavs = [n for n in self.positions if n > 0]
            if uavs:
                self.discovered_uavs = max(self.discovered_uavs, max(uavs))
            self.last_seen["positions"] = time.time()

    def _make_link_cb(self, name: str, target: Dict[Tuple[int, int], float]):
        def callback(msg: Float32MultiArray) -> None:
            data = list(msg.data)
            with self.lock:
                for index in range(0, len(data) - 2, 3):
                    a = int(round(data[index]))
                    b = int(round(data[index + 1]))
                    if a == b:
                        continue
                    target[link_key(a, b)] = float(data[index + 2])
                    if max(a, b) > self.discovered_uavs:
                        self.discovered_uavs = max(a, b)
                self.last_seen[name] = time.time()
        return callback

    def _on_scores(self, msg: Float32MultiArray) -> None:
        # [id, score, candidate, gcs_snr, gcs_rssi, neighbor, mobility, obstacle]
        data = list(msg.data)
        with self.lock:
            self.scores.clear()
            for index in range(0, len(data) - 7, 8):
                uav_id = int(round(data[index]))
                self.scores[uav_id] = {
                    "score": data[index + 1],
                    "candidate": data[index + 2],
                    "gcs_snr_db": data[index + 3],
                    "gcs_rssi_dbm": data[index + 4],
                    "neighbor_score": data[index + 5],
                    "mobility_stability": data[index + 6],
                    "obstacle_robustness": data[index + 7],
                }
            if self.scores:
                self.discovered_uavs = max(
                    self.discovered_uavs, max(self.scores))
            self.last_seen["scores"] = time.time()

    def _on_primary(self, msg: Int32) -> None:
        with self.lock:
            self.primary_ch = int(msg.data)
            self.last_seen["cluster"] = time.time()

    def _on_backup(self, msg: Int32) -> None:
        with self.lock:
            self.backup_ch = int(msg.data)

    def _on_assignment(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.epoch = int(state.get("epoch", 0))
            self.cluster_status = str(state.get("status", "UNKNOWN"))
            self.assignments = state.get("assignments", {})
            reported = int(state.get("num_uavs", 0))
            if reported:
                # Authoritative, unlike the running maximum below: if the
                # simulation is restarted with a smaller fleet while this
                # dashboard keeps running, this is what shrinks it back.
                self.reported_uavs = reported
                self.reported_uavs_t = time.time()
            self.last_seen["cluster"] = time.time()

    def _on_event(self, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.events.appendleft((time.time(), event))

    def _on_relay(self, msg: String) -> None:
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.relayed = {
                int(member): int(head)
                for member, head in state.get("relayed", {}).items()
            }
            for change in state.get("changes", []):
                self.events.appendleft(
                    (time.time(), {"relay": change}))
            self.last_seen["relay"] = time.time()

    def _make_alt_cb(self, uav_id: int):
        def callback(msg: Float32) -> None:
            with self.lock:
                self.rel_alt[uav_id] = float(msg.data)
                self.last_seen[f"uav{uav_id}"] = time.time()
        return callback

    def _make_mode_cb(self, uav_id: int):
        def callback(msg: String) -> None:
            with self.lock:
                self.mode[uav_id] = msg.data
        return callback

    def _make_armed_cb(self, uav_id: int):
        def callback(msg: Bool) -> None:
            with self.lock:
                self.armed[uav_id] = bool(msg.data)
        return callback

    def _sample_history(self) -> None:
        """One timeline sample per second, for every link currently known."""
        with self.lock:
            self.times.append(time.time())
            for key in self.snr:
                self.snr_history.setdefault(
                    key, deque(maxlen=HISTORY)).append(self.snr[key])
            for key in self.obstacle:
                self.obstacle_history.setdefault(
                    key, deque(maxlen=HISTORY)).append(self.obstacle[key])

    # ── snapshot for the UI thread ────────────────────────────────────────────

    def snapshot(self) -> dict:
        with self.lock:
            if self.forced_num_uavs:
                num_uavs = self.forced_num_uavs
            elif time.time() - self.reported_uavs_t < STALE_SEC * 3:
                num_uavs = self.reported_uavs
            else:
                num_uavs = self.discovered_uavs
            num_uavs = max(0, min(MAX_UAVS, num_uavs))
            return {
                "num_uavs": num_uavs,
                "positions": dict(self.positions),
                "snr": dict(self.snr),
                "rssi": dict(self.rssi),
                "obstacle": dict(self.obstacle),
                "scores": {k: dict(v) for k, v in self.scores.items()},
                "primary_ch": self.primary_ch,
                "backup_ch": self.backup_ch,
                "epoch": self.epoch,
                "cluster_status": self.cluster_status,
                "relayed": dict(self.relayed),
                "events": list(self.events),
                "rel_alt": dict(self.rel_alt),
                "mode": dict(self.mode),
                "armed": dict(self.armed),
                "last_seen": dict(self.last_seen),
                "snr_history": {
                    k: list(v) for k, v in self.snr_history.items()},
                "obstacle_history": {
                    k: list(v) for k, v in self.obstacle_history.items()},
            }


# ── helpers ───────────────────────────────────────────────────────────────────

def distance_m(positions: dict, a: int, b: int) -> Optional[float]:
    if a not in positions or b not in positions:
        return None
    ax, ay, az = positions[a]
    bx, by, bz = positions[b]
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2)


def link_state(snr_db: Optional[float], rssi_dbm: Optional[float]) -> str:
    """UP / MARGINAL / DOWN from whichever metric is available."""
    if rssi_dbm is not None and rssi_dbm < RX_SENSITIVITY_DBM:
        return "DOWN"
    if snr_db is None:
        return "UNKNOWN"
    if snr_db < 0.0:
        return "DOWN"
    if snr_db < USABLE_SNR_DB:
        return "MARGINAL"
    return "UP"


STATE_COLOR = {
    "UP": GOOD,
    "MARGINAL": WARN,
    "DOWN": BAD,
    "UNKNOWN": MUTED,
}


def all_links(num_uavs: int) -> List[Tuple[int, int]]:
    """Every GCS<->UAV and UAV<->UAV pair for the current fleet."""
    return [
        (a, b)
        for a in range(0, num_uavs + 1)
        for b in range(a + 1, num_uavs + 1)
    ]


def panel(title: str, *children, span: int = 1):
    return html.Div(
        [html.Div(title, style={
            "color": MUTED, "fontSize": "12px", "letterSpacing": "0.08em",
            "textTransform": "uppercase", "marginBottom": "10px"})]
        + list(children),
        style={
            "backgroundColor": PANEL,
            "border": f"1px solid {BORDER}",
            "borderRadius": "8px",
            "padding": "14px 16px",
            "gridColumn": f"span {span}",
            "minWidth": 0,
        },
    )


def table(headers: List[str], rows: List, empty: str = "waiting for data…"):
    if not rows:
        return html.Div(empty, style={"color": MUTED, "padding": "12px 0"})
    return html.Table(
        [html.Thead(html.Tr([
            html.Th(h, style={
                "textAlign": "left", "color": MUTED, "fontWeight": "500",
                "padding": "6px 10px 6px 0", "fontSize": "12px",
                "borderBottom": f"1px solid {BORDER}", "whiteSpace": "nowrap"})
            for h in headers]))]
        + [html.Tbody(rows)],
        style={"width": "100%", "borderCollapse": "collapse",
               "fontSize": "13px", "fontVariantNumeric": "tabular-nums"},
    )


def cell(value, color: str = TEXT, bold: bool = False):
    return html.Td(value, style={
        "padding": "5px 10px 5px 0", "color": color,
        "fontWeight": "600" if bold else "400",
        "borderBottom": f"1px solid {BORDER}", "whiteSpace": "nowrap"})


def dark_layout(fig, **kwargs):
    # Callers override margin/legend, so merge rather than pass both through
    # to update_layout — duplicating a keyword there is a TypeError.
    layout = dict(
        paper_bgcolor=PANEL, plot_bgcolor=PANEL, font_color=TEXT,
        font_family="ui-monospace, SFMono-Regular, Menlo, monospace",
        font_size=11, margin=dict(l=45, r=14, t=10, b=36),
        legend=dict(bgcolor="rgba(0,0,0,0)", font_size=10),
    )
    layout.update(kwargs)
    fig.update_layout(**layout)
    fig.update_xaxes(gridcolor=BORDER, zerolinecolor=BORDER, color=MUTED)
    fig.update_yaxes(gridcolor=BORDER, zerolinecolor=BORDER, color=MUTED)
    return fig


# ── Dash app ──────────────────────────────────────────────────────────────────

app = dash.Dash(__name__, title="Multi-UAV Network Monitor")
monitor: Optional[NetworkMonitor] = None

app.layout = html.Div(
    style={"backgroundColor": BG, "color": TEXT, "minHeight": "100vh",
           "fontFamily": "ui-monospace, SFMono-Regular, Menlo, monospace",
           "padding": "18px 22px"},
    children=[
        html.Div(style={"display": "flex", "alignItems": "baseline",
                        "gap": "18px", "marginBottom": "16px"},
                 children=[
            html.H1("Multi-UAV Network Monitor",
                    style={"fontSize": "19px", "margin": 0, "color": TEXT}),
            html.Div(id="header-status", style={"color": MUTED,
                                                "fontSize": "13px"}),
        ]),

        html.Div(
            style={"display": "grid",
                   "gridTemplateColumns": "repeat(3, minmax(0, 1fr))",
                   "gap": "14px"},
            children=[
                panel("Fleet position & link topology (top-down)",
                      dcc.Graph(id="map-plot", style={"height": "430px"},
                                config={"displayModeBar": False}),
                      span=2),
                panel("Cluster state", html.Div(id="cluster-panel")),

                panel("Link matrix — every node pair", html.Div(id="link-table"),
                      span=2),
                panel("Relay routing", html.Div(id="relay-panel")),

                panel("SNR history (dB)",
                      dcc.Graph(id="snr-plot", style={"height": "260px"},
                                config={"displayModeBar": False}),
                      span=2),
                panel("Obstacle loss by link (dB)",
                      dcc.Graph(id="obstacle-plot", style={"height": "260px"},
                                config={"displayModeBar": False})),

                panel("UAV telemetry & cluster-head scores",
                      html.Div(id="uav-table"), span=2),
                panel("Recent cluster events", html.Div(id="event-log")),
            ]),

        dcc.Interval(id="ticker", interval=1000, n_intervals=0),
    ])


@app.callback(
    Output("header-status", "children"),
    Output("map-plot", "figure"),
    Output("cluster-panel", "children"),
    Output("link-table", "children"),
    Output("relay-panel", "children"),
    Output("snr-plot", "figure"),
    Output("obstacle-plot", "figure"),
    Output("uav-table", "children"),
    Output("event-log", "children"),
    Input("ticker", "n_intervals"),
)
def refresh(_):
    state = monitor.snapshot()
    num_uavs = state["num_uavs"]
    positions = state["positions"]
    links = all_links(num_uavs)

    return (
        build_header(state),
        build_map(state, links),
        build_cluster_panel(state),
        build_link_table(state, links),
        build_relay_panel(state),
        build_snr_plot(state, links),
        build_obstacle_plot(state, links),
        build_uav_table(state),
        build_event_log(state),
    )


def build_header(state: dict):
    now = time.time()
    parts = []

    num_uavs = state["num_uavs"]
    parts.append(html.Span(
        f"{num_uavs} UAV{'s' if num_uavs != 1 else ''} + GCS"
        if num_uavs else "no fleet detected yet",
        style={"color": TEXT if num_uavs else WARN}))

    for name, label in (("positions", "positions"), ("snr", "ns-3"),
                        ("obstacle", "raycast"), ("cluster", "clustering")):
        seen = state["last_seen"].get(name)
        live = seen is not None and (now - seen) < STALE_SEC
        parts.append(html.Span(
            f"● {label}",
            style={"color": GOOD if live else BAD, "marginLeft": "14px"},
            title=("live" if live else "no data / stale")))

    return parts


def build_map(state: dict, links):
    """Top-down 2D map. Altitude is a label, not an axis, so the city layout
    stays readable — a 3D scene makes overlapping links impossible to judge."""
    positions = state["positions"]
    relayed = state["relayed"]
    traces = []

    # Links first so node markers draw on top of them.
    for a, b in links:
        if a not in positions or b not in positions:
            continue
        snr = state["snr"].get(link_key(a, b))
        rssi = state["rssi"].get(link_key(a, b))
        obstacle = state["obstacle"].get(link_key(a, b), 0.0)
        status = link_state(snr, rssi)
        distance = distance_m(positions, a, b)

        # Same visual language as the Gazebo ray markers: solid to the ground
        # station, dashed between UAVs.
        is_relay_hop = (
            (a == 0 and b in relayed) or (b == 0 and a in relayed) or
            (relayed.get(a) == b or relayed.get(b) == a)
        )
        traces.append(go.Scatter(
            x=[positions[a][0], positions[b][0]],
            y=[positions[a][1], positions[b][1]],
            mode="lines",
            line=dict(
                color=RELAY if is_relay_hop else STATE_COLOR[status],
                width=3.5 if is_relay_hop else 1.8,
                dash="solid" if a == 0 else "dash"),
            opacity=1.0 if status != "DOWN" else 0.35,
            hoverinfo="text",
            hovertext=(
                f"{node_label(a)} ↔ {node_label(b)}<br>"
                f"{status}"
                f"{' · RELAY HOP' if is_relay_hop else ''}<br>"
                f"distance {distance:.0f} m<br>"
                f"SNR {snr:.1f} dB<br>" if snr is not None else ""
            ) + (f"obstacle {obstacle:.0f} dB" if obstacle else "clear path"),
            showlegend=False,
        ))

    # Node markers.
    for node_id in sorted(positions):
        if node_id > state["num_uavs"]:
            continue
        x, y, z = positions[node_id]
        role = ""
        if node_id == state["primary_ch"]:
            role = " ★CH"
        elif node_id == state["backup_ch"] and node_id != 0:
            role = " ☆bk"

        traces.append(go.Scatter(
            x=[x], y=[y], mode="markers+text",
            marker=dict(
                size=20 if node_id == 0 else 15,
                color=NODE_COLORS.get(node_id, ACCENT),
                symbol="square" if node_id == 0 else "circle",
                line=dict(
                    width=3 if node_id == state["primary_ch"] else 0,
                    color=WARN)),
            text=[f"{node_label(node_id)}{role}<br>{z:.0f} m"],
            textposition="top center",
            textfont=dict(size=10, color=TEXT),
            hovertext=f"{node_label(node_id)}<br>x {x:.0f} y {y:.0f} alt {z:.0f} m",
            hoverinfo="text",
            showlegend=False,
        ))

    fig = go.Figure(traces)
    dark_layout(fig, margin=dict(l=45, r=14, t=14, b=40))
    fig.update_xaxes(title="X (m)", scaleanchor="y", scaleratio=1)
    fig.update_yaxes(title="Y (m)")
    if not positions:
        fig.add_annotation(
            text="waiting for /uav_world_positions", showarrow=False,
            font=dict(color=MUTED, size=13))
    return fig


def build_cluster_panel(state: dict):
    primary = state["primary_ch"]
    backup = state["backup_ch"]
    status = state["cluster_status"]

    def badge(label, value, color):
        return html.Div([
            html.Div(label, style={"color": MUTED, "fontSize": "11px"}),
            html.Div(value, style={"color": color, "fontSize": "17px",
                                   "fontWeight": "600"}),
        ], style={"marginBottom": "12px"})

    return html.Div([
        badge("Status", status,
              GOOD if status == "ACTIVE" else (WARN if status == "WAITING" else BAD)),
        badge("Epoch", str(state["epoch"]), TEXT),
        badge("Primary cluster head",
              node_label(primary) if primary else "none",
              NODE_COLORS.get(primary, MUTED) if primary else BAD),
        badge("Backup cluster head",
              node_label(backup) if backup else "none",
              NODE_COLORS.get(backup, MUTED) if backup else MUTED),
    ])


def build_link_table(state: dict, links):
    positions = state["positions"]
    relayed = state["relayed"]
    rows = []

    for a, b in links:
        snr = state["snr"].get(link_key(a, b))
        rssi = state["rssi"].get(link_key(a, b))
        obstacle = state["obstacle"].get(link_key(a, b))
        distance = distance_m(positions, a, b)
        status = link_state(snr, rssi)

        is_relay_hop = (relayed.get(a) == b or relayed.get(b) == a or
                        (a == 0 and b in relayed) or (b == 0 and a in relayed))

        rows.append(html.Tr([
            cell(f"{node_label(a)} ↔ {node_label(b)}", bold=True),
            cell(f"{distance:.0f} m" if distance is not None else "—"),
            cell(f"{snr:.1f}" if snr is not None else "—",
                 GOOD if (snr or -99) >= USABLE_SNR_DB else
                 (WARN if (snr or -99) >= 0 else BAD)),
            cell(f"{rssi:.1f}" if rssi is not None else "—",
                 GOOD if (rssi or -999) > RX_SENSITIVITY_DBM else BAD),
            cell(f"{obstacle:.0f}" if obstacle else "clear",
                 BAD if obstacle else GOOD),
            cell(status, STATE_COLOR[status], bold=True),
            cell("relay hop" if is_relay_hop else "", RELAY),
        ]))

    return table(
        ["Link", "Distance", "SNR dB", "RSSI dBm", "Obstacle dB",
         "State", "Role"],
        rows,
        empty="waiting for ns-3 link metrics…")


def build_relay_panel(state: dict):
    relayed = state["relayed"]
    if not state["last_seen"].get("relay"):
        return html.Div(
            "no relay state published yet",
            style={"color": MUTED})
    if not relayed:
        return html.Div([
            html.Div("All members direct to GCS",
                     style={"color": GOOD, "fontSize": "14px"}),
            html.Div("No member needs the cluster head to reach the ground "
                     "station right now.",
                     style={"color": MUTED, "fontSize": "12px",
                            "marginTop": "6px"}),
        ])

    rows = []
    for member, head in sorted(relayed.items()):
        rows.append(html.Tr([
            cell(node_label(member), NODE_COLORS.get(member, TEXT), bold=True),
            cell("→"),
            cell(node_label(head), NODE_COLORS.get(head, TEXT), bold=True),
            cell("→ GCS", MUTED),
        ]))
    return html.Div([
        html.Div(f"{len(relayed)} member(s) relaying",
                 style={"color": RELAY, "fontSize": "14px",
                        "marginBottom": "8px"}),
        table(["Member", "", "Via", ""], rows),
    ])


def build_snr_plot(state: dict, links):
    fig = go.Figure()
    history = state["snr_history"]

    for a, b in links:
        series = history.get(link_key(a, b))
        if not series:
            continue
        fig.add_trace(go.Scatter(
            y=series, mode="lines", name=f"{node_label(a)}↔{node_label(b)}",
            line=dict(
                color=NODE_COLORS.get(b if a == 0 else a, ACCENT),
                width=2 if a == 0 else 1.2,
                dash="solid" if a == 0 else "dot"),
        ))

    fig.add_hline(y=USABLE_SNR_DB, line_dash="dash", line_color=WARN,
                  annotation_text="usable SNR",
                  annotation_font=dict(color=WARN, size=10))
    dark_layout(fig, legend=dict(orientation="h", y=-0.18, font_size=9,
                                 bgcolor="rgba(0,0,0,0)"))
    fig.update_yaxes(title="SNR (dB)")
    fig.update_xaxes(title="samples (1 Hz)")
    if not fig.data:
        fig.add_annotation(text="waiting for /ns3_link_snr", showarrow=False,
                           font=dict(color=MUTED, size=13))
    return fig


def build_obstacle_plot(state: dict, links):
    """Current obstacle loss per link — a bar reads faster than a timeline
    when the question is 'which links are blocked right now'."""
    labels, values, colors = [], [], []
    for a, b in links:
        loss = state["obstacle"].get(link_key(a, b))
        if loss is None:
            continue
        labels.append(f"{node_label(a)}↔{node_label(b)}")
        values.append(loss)
        colors.append(BAD if loss > 0 else GOOD)

    fig = go.Figure(go.Bar(
        x=values, y=labels, orientation="h",
        marker_color=colors,
        hovertemplate="%{y}: %{x:.0f} dB<extra></extra>"))
    dark_layout(fig, margin=dict(l=95, r=14, t=10, b=36))
    fig.update_xaxes(title="obstacle loss (dB)")
    fig.update_yaxes(autorange="reversed")
    if not values:
        fig.add_annotation(text="waiting for /link_obstacle_loss",
                           showarrow=False, font=dict(color=MUTED, size=13))
    return fig


def build_uav_table(state: dict):
    rows = []
    for uav_id in range(1, state["num_uavs"] + 1):
        score = state["scores"].get(uav_id, {})
        role = "member"
        role_color = MUTED
        if uav_id == state["primary_ch"]:
            role, role_color = "CLUSTER HEAD", WARN
        elif uav_id == state["backup_ch"]:
            role, role_color = "backup", ACCENT

        armed = state["armed"].get(uav_id)
        alt = state["rel_alt"].get(uav_id)
        candidate = score.get("candidate", 0.0) >= 0.5
        gcs_snr = score.get("gcs_snr_db")

        rows.append(html.Tr([
            cell(node_label(uav_id), NODE_COLORS.get(uav_id, TEXT), bold=True),
            cell(role, role_color, bold=(role == "CLUSTER HEAD")),
            cell(f"{alt:.1f} m" if alt is not None else "—"),
            cell(state["mode"].get(uav_id, "—")),
            cell("armed" if armed else ("disarmed" if armed is not None else "—"),
                 GOOD if armed else MUTED),
            cell(f"{gcs_snr:.1f}" if gcs_snr is not None else "—",
                 GOOD if (gcs_snr or -99) >= USABLE_SNR_DB else BAD),
            cell(f"{score['score']:.3f}" if "score" in score else "—"),
            cell("yes" if candidate else "no", GOOD if candidate else MUTED),
            cell(f"{score['neighbor_score']:.2f}"
                 if "neighbor_score" in score else "—"),
            cell(f"{score['mobility_stability']:.2f}"
                 if "mobility_stability" in score else "—"),
            cell(f"{score['obstacle_robustness']:.2f}"
                 if "obstacle_robustness" in score else "—"),
        ]))

    return table(
        ["UAV", "Role", "Alt", "Mode", "State", "GCS SNR", "CH score",
         "Eligible", "Neighbour", "Mobility", "Obst. robust"],
        rows,
        empty="waiting for fleet telemetry…")


def build_event_log(state: dict):
    if not state["events"]:
        return html.Div("no cluster events yet", style={"color": MUTED})

    items = []
    for stamp, event in state["events"]:
        clock = time.strftime("%H:%M:%S", time.localtime(stamp))

        if "relay" in event:
            text, color = event["relay"], RELAY
        else:
            text = (f"epoch {event.get('epoch', '?')}: "
                    f"CH {node_label(event.get('old_primary', 0))} → "
                    f"{node_label(event.get('new_primary', 0))} "
                    f"({event.get('reason', '')})")
            color = ACCENT

        items.append(html.Div([
            html.Span(clock, style={"color": MUTED, "marginRight": "8px"}),
            html.Span(text, style={"color": color}),
        ], style={"fontSize": "12px", "marginBottom": "5px",
                  "lineHeight": "1.4"}))
    return html.Div(items)


# ── entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    global monitor

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--num-uavs", type=int, default=0,
        help=f"pin the fleet size (1-{MAX_UAVS}); default 0 = auto-detect")
    parser.add_argument("--port", type=int, default=8050)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    if args.num_uavs and not 1 <= args.num_uavs <= MAX_UAVS:
        parser.error(f"--num-uavs must be between 1 and {MAX_UAVS}")

    rclpy.init()
    monitor = NetworkMonitor(forced_num_uavs=args.num_uavs)

    spinner = threading.Thread(
        target=rclpy.spin, args=(monitor,), daemon=True)
    spinner.start()

    print(f"Dashboard → http://localhost:{args.port}")
    try:
        app.run(debug=False, host=args.host, port=args.port)
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
