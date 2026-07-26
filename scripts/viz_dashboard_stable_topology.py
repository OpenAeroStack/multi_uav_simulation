#!/usr/bin/env python3
"""
Real-time UAV, NS-3, and dynamic-clustering dashboard.

The dashboard subscribes directly to ROS 2 topics:

Position and wireless inputs
----------------------------
/uav_world_positions
/ns3_link_rssi
/ns3_link_snr
/link_obstacle_loss

Dynamic-clustering outputs
--------------------------
/cluster/assignment
/cluster/scores
/cluster/primary_ch
/cluster/backup_ch
/cluster/event

Requirements
------------
sudo apt install python3-pip
pip3 install dash plotly

Run after the integrated launcher is active:

source /opt/ros/humble/setup.bash
source ~/simulation/multi_uav_simulation/ros2/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS2CLI_NO_DAEMON=1

python3 scripts/viz_dashboard_dynamic_clustering.py

Open:
http://localhost:8050
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from typing import Any, Dict, List, Tuple

import dash
from dash import Input, Output, dcc, html
import plotly.graph_objects as go
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float32MultiArray, Int32, String


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

UAV_IDS = [1, 2, 3]
NODE_IDS = [0] + UAV_IDS
TRACE_FILE = "/tmp/ns3_wifi_trace.tr"

# These must match dynamic_cluster_manager.py.
WEIGHTS = {
    "gcs": 0.40,
    "neighbors": 0.30,
    "mobility": 0.20,
    "obstacle": 0.10,
}

SNR_MIN_DB = -10.0
SNR_MAX_DB = 30.0
CANDIDATE_MIN_GCS_SNR_DB = 3.0
MEMBER_MIN_SNR_DB = 5.0

# Used only as a fallback before /ns3_link_rssi starts publishing.
PATH_LOSS_EXP = 2.0
REF_DISTANCE = 1.0
REF_LOSS_DB = 46.67
TX_POWER_DBM = 20.0
RX_SENSITIVITY_DBM = -82.0

DASH_HOST = "0.0.0.0"
DASH_PORT = 8050

# Stable top-down topology view.
#
# The map starts with a useful fixed-size window and only expands when a UAV
# travels outside it. It never contracts during a run, so the axes do not jump.
TOPOLOGY_MIN_SPAN_M = 420.0
TOPOLOGY_PADDING_M = 40.0
TOPOLOGY_INITIAL_CENTER_X_M = 100.0
TOPOLOGY_INITIAL_CENTER_Y_M = -40.0


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

def default_position(node_id: int) -> Dict[str, float]:
    if node_id == 0:
        return {"x": 0.0, "y": 0.0, "z": 2.9}

    return {"x": 0.0, "y": 0.0, "z": 10.0}


positions: Dict[int, Dict[str, float]] = {
    node_id: default_position(node_id)
    for node_id in NODE_IDS
}

link_rssi: Dict[Tuple[int, int], float] = {}
link_snr: Dict[Tuple[int, int], float] = {}
link_obstacle_loss: Dict[Tuple[int, int], float] = {}

cluster_scores: Dict[int, Dict[str, float]] = {
    uav_id: {
        "score": 0.0,
        "candidate": 0.0,
        "gcs_snr_db": -100.0,
        "gcs_rssi_dbm": -200.0,
        "neighbor_score": 0.0,
        "mobility_stability": 0.0,
        "obstacle_robustness": 0.0,
    }
    for uav_id in UAV_IDS
}

cluster_state: Dict[str, Any] = {
    "status": "WAITING",
    "epoch": 0,
    "primary_ch": 0,
    "backup_ch": 0,
    "assignments": [],
    "last_update": 0.0,
}

cluster_events: deque = deque(maxlen=30)

score_history: Dict[int, deque] = {
    uav_id: deque(maxlen=240)
    for uav_id in UAV_IDS
}
score_time_history: Dict[int, deque] = {
    uav_id: deque(maxlen=240)
    for uav_id in UAV_IDS
}

trace_stats = {
    "tx": deque(maxlen=200),
    "rx": deque(maxlen=200),
    "loss_pct": deque(maxlen=200),
    "times": deque(maxlen=200),
}

last_topic_update: Dict[str, float] = {}
state_lock = threading.Lock()

# These bounds persist for the lifetime of the dashboard. They may expand, but
# they never shrink. This prevents the Plotly map from zooming in and out every
# time the coordinates update.
topology_bounds = {
    "xmin": TOPOLOGY_INITIAL_CENTER_X_M - TOPOLOGY_MIN_SPAN_M / 2.0,
    "xmax": TOPOLOGY_INITIAL_CENTER_X_M + TOPOLOGY_MIN_SPAN_M / 2.0,
    "ymin": TOPOLOGY_INITIAL_CENTER_Y_M - TOPOLOGY_MIN_SPAN_M / 2.0,
    "ymax": TOPOLOGY_INITIAL_CENTER_Y_M + TOPOLOGY_MIN_SPAN_M / 2.0,
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def link_key(node_a: int, node_b: int) -> Tuple[int, int]:
    return min(node_a, node_b), max(node_a, node_b)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def normalize_gcs_snr(snr_db: float) -> float:
    span = SNR_MAX_DB - SNR_MIN_DB

    if span <= 0.0:
        return 0.0

    return clamp(
        (snr_db - SNR_MIN_DB) / span,
        0.0,
        1.0,
    )


def distance_3d(
    position_a: Dict[str, float],
    position_b: Dict[str, float],
) -> float:
    return math.sqrt(
        (position_a["x"] - position_b["x"]) ** 2
        + (position_a["y"] - position_b["y"]) ** 2
        + (position_a["z"] - position_b["z"]) ** 2
    )


def calculated_rssi_fallback(
    position_a: Dict[str, float],
    position_b: Dict[str, float],
) -> float:
    distance = distance_3d(position_a, position_b)

    if distance < 0.1:
        return TX_POWER_DBM

    distance = max(distance, REF_DISTANCE)

    path_loss = (
        REF_LOSS_DB
        + 10.0
        * PATH_LOSS_EXP
        * math.log10(distance / REF_DISTANCE)
    )

    return TX_POWER_DBM - path_loss


def quality_label(rssi_dbm: float) -> str:
    if rssi_dbm > -60.0:
        return "Strong"

    if rssi_dbm > -72.0:
        return "Medium"

    return "Weak"


def link_color(rssi_dbm: float) -> str:
    if rssi_dbm > -60.0:
        return "#3fb950"

    if rssi_dbm > -72.0:
        return "#e3b341"

    return "#f85149"


def role_color(role: str) -> str:
    return {
        "PRIMARY_CH": "#58a6ff",
        "BACKUP_CH": "#d2a8ff",
        "MEMBER": "#3fb950",
        "DIRECT_MEMBER": "#e3b341",
        "DISCONNECTED": "#f85149",
    }.get(role, "#8b949e")


def display_uav(uav_id: int) -> str:
    return "None" if uav_id == 0 else f"UAV{uav_id}"


def parse_link_array(
    values: List[float],
) -> Dict[Tuple[int, int], float]:
    parsed: Dict[Tuple[int, int], float] = {}

    for index in range(0, len(values) - 2, 3):
        node_a = int(round(values[index]))
        node_b = int(round(values[index + 1]))
        value = float(values[index + 2])

        if node_a == node_b:
            continue

        parsed[link_key(node_a, node_b)] = value

    return parsed


def get_assignment_map(
    state: Dict[str, Any],
) -> Dict[int, Dict[str, Any]]:
    assignment_map: Dict[int, Dict[str, Any]] = {}

    for assignment in state.get("assignments", []):
        try:
            assignment_map[int(assignment["uav_id"])] = assignment
        except (KeyError, TypeError, ValueError):
            continue

    return assignment_map


def update_stable_topology_bounds(
    current_positions: Dict[int, Dict[str, float]],
) -> Tuple[List[float], List[float]]:
    """Return equal-scale X/Y bounds that expand but never contract."""

    valid_points = [
        position
        for position in current_positions.values()
        if all(
            math.isfinite(float(position.get(axis, 0.0)))
            for axis in ("x", "y")
        )
    ]

    if not valid_points:
        return (
            [topology_bounds["xmin"], topology_bounds["xmax"]],
            [topology_bounds["ymin"], topology_bounds["ymax"]],
        )

    requested_xmin = min(point["x"] for point in valid_points) - TOPOLOGY_PADDING_M
    requested_xmax = max(point["x"] for point in valid_points) + TOPOLOGY_PADDING_M
    requested_ymin = min(point["y"] for point in valid_points) - TOPOLOGY_PADDING_M
    requested_ymax = max(point["y"] for point in valid_points) + TOPOLOGY_PADDING_M

    topology_bounds["xmin"] = min(topology_bounds["xmin"], requested_xmin)
    topology_bounds["xmax"] = max(topology_bounds["xmax"], requested_xmax)
    topology_bounds["ymin"] = min(topology_bounds["ymin"], requested_ymin)
    topology_bounds["ymax"] = max(topology_bounds["ymax"], requested_ymax)

    x_center = (topology_bounds["xmin"] + topology_bounds["xmax"]) / 2.0
    y_center = (topology_bounds["ymin"] + topology_bounds["ymax"]) / 2.0
    common_span = max(
        topology_bounds["xmax"] - topology_bounds["xmin"],
        topology_bounds["ymax"] - topology_bounds["ymin"],
        TOPOLOGY_MIN_SPAN_M,
    )

    topology_bounds["xmin"] = x_center - common_span / 2.0
    topology_bounds["xmax"] = x_center + common_span / 2.0
    topology_bounds["ymin"] = y_center - common_span / 2.0
    topology_bounds["ymax"] = y_center + common_span / 2.0

    return (
        [topology_bounds["xmin"], topology_bounds["xmax"]],
        [topology_bounds["ymin"], topology_bounds["ymax"]],
    )


def blank_figure(message: str, height: int = 300) -> go.Figure:
    figure = go.Figure()

    figure.add_annotation(
        text=message,
        x=0.5,
        y=0.5,
        xref="paper",
        yref="paper",
        showarrow=False,
        font={"color": "#8b949e", "size": 15},
    )

    figure.update_layout(
        height=height,
        paper_bgcolor="#161b22",
        plot_bgcolor="#161b22",
        margin={"l": 20, "r": 20, "t": 20, "b": 20},
        xaxis={"visible": False},
        yaxis={"visible": False},
    )

    return figure


# ---------------------------------------------------------------------------
# ROS 2 subscriber node
# ---------------------------------------------------------------------------

class DashboardRosNode(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_cluster_dashboard")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.create_subscription(
            Float32MultiArray,
            "/uav_world_positions",
            self.position_callback,
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
            "/ns3_link_snr",
            self.snr_callback,
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
            "/cluster/scores",
            self.score_callback,
            state_qos,
        )

        self.create_subscription(
            String,
            "/cluster/assignment",
            self.assignment_callback,
            state_qos,
        )

        self.create_subscription(
            Int32,
            "/cluster/primary_ch",
            self.primary_callback,
            state_qos,
        )

        self.create_subscription(
            Int32,
            "/cluster/backup_ch",
            self.backup_callback,
            state_qos,
        )

        self.create_subscription(
            String,
            "/cluster/event",
            self.event_callback,
            state_qos,
        )

        self.get_logger().info(
            "Dashboard ROS subscriptions are active."
        )

    def position_callback(self, msg: Float32MultiArray) -> None:
        now = time.time()

        with state_lock:
            for index in range(0, len(msg.data) - 3, 4):
                node_id = int(round(msg.data[index]))

                if node_id not in NODE_IDS:
                    continue

                positions[node_id] = {
                    "x": float(msg.data[index + 1]),
                    "y": float(msg.data[index + 2]),
                    "z": float(msg.data[index + 3]),
                }

            last_topic_update["positions"] = now

    def rssi_callback(self, msg: Float32MultiArray) -> None:
        with state_lock:
            link_rssi.clear()
            link_rssi.update(parse_link_array(list(msg.data)))
            last_topic_update["rssi"] = time.time()

    def snr_callback(self, msg: Float32MultiArray) -> None:
        with state_lock:
            link_snr.clear()
            link_snr.update(parse_link_array(list(msg.data)))
            last_topic_update["snr"] = time.time()

    def obstacle_callback(
        self,
        msg: Float32MultiArray,
    ) -> None:
        with state_lock:
            link_obstacle_loss.clear()
            link_obstacle_loss.update(
                parse_link_array(list(msg.data))
            )
            last_topic_update["obstacle"] = time.time()

    def score_callback(
        self,
        msg: Float32MultiArray,
    ) -> None:
        now = time.time()
        values = list(msg.data)

        with state_lock:
            for index in range(0, len(values) - 7, 8):
                uav_id = int(round(values[index]))

                if uav_id not in UAV_IDS:
                    continue

                details = {
                    "score": float(values[index + 1]),
                    "candidate": float(values[index + 2]),
                    "gcs_snr_db": float(values[index + 3]),
                    "gcs_rssi_dbm": float(values[index + 4]),
                    "neighbor_score": float(values[index + 5]),
                    "mobility_stability": float(
                        values[index + 6]
                    ),
                    "obstacle_robustness": float(
                        values[index + 7]
                    ),
                }

                cluster_scores[uav_id] = details
                score_history[uav_id].append(details["score"])
                score_time_history[uav_id].append(now)

            last_topic_update["scores"] = now

    def assignment_callback(self, msg: String) -> None:
        try:
            decoded = json.loads(msg.data)
        except json.JSONDecodeError:
            decoded = {
                "status": "INVALID_JSON",
                "raw": msg.data,
            }

        with state_lock:
            cluster_state.update(decoded)
            cluster_state["last_update"] = time.time()
            last_topic_update["assignment"] = time.time()

    def primary_callback(self, msg: Int32) -> None:
        with state_lock:
            cluster_state["primary_ch"] = int(msg.data)
            last_topic_update["primary"] = time.time()

    def backup_callback(self, msg: Int32) -> None:
        with state_lock:
            cluster_state["backup_ch"] = int(msg.data)
            last_topic_update["backup"] = time.time()

    def event_callback(self, msg: String) -> None:
        try:
            decoded = json.loads(msg.data)
        except json.JSONDecodeError:
            decoded = {"raw": msg.data}

        decoded["_received_at"] = time.time()

        with state_lock:
            cluster_events.appendleft(decoded)
            last_topic_update["event"] = time.time()


def ros_spin_thread() -> None:
    rclpy.init()
    node = DashboardRosNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"ROS dashboard thread failed: {error}")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


# ---------------------------------------------------------------------------
# NS-3 trace reader
# ---------------------------------------------------------------------------

def ns3_trace_reader() -> None:
    while True:
        try:
            with open(TRACE_FILE, "r", encoding="utf-8") as trace_file:
                trace_file.seek(0, 2)

                tx_count = 0
                rx_count = 0
                last_report = time.time()

                while True:
                    line = trace_file.readline()

                    if not line:
                        time.sleep(0.05)
                    elif line.startswith("t "):
                        tx_count += 1
                    elif line.startswith("r "):
                        rx_count += 1

                    now = time.time()

                    if now - last_report < 1.0:
                        continue

                    loss_percentage = 0.0

                    if tx_count > 0:
                        loss_percentage = max(
                            0.0,
                            (tx_count - rx_count)
                            / tx_count
                            * 100.0,
                        )

                    with state_lock:
                        trace_stats["tx"].append(tx_count)
                        trace_stats["rx"].append(rx_count)
                        trace_stats["loss_pct"].append(
                            loss_percentage
                        )
                        trace_stats["times"].append(now)

                    tx_count = 0
                    rx_count = 0
                    last_report = now

        except FileNotFoundError:
            time.sleep(1.0)
        except Exception as error:
            print(f"NS-3 trace reader error: {error}")
            time.sleep(1.0)


# ---------------------------------------------------------------------------
# Dash layout helpers
# ---------------------------------------------------------------------------

CARD_STYLE = {
    "backgroundColor": "#161b22",
    "border": "1px solid #30363d",
    "borderRadius": "10px",
    "padding": "16px",
}

TABLE_STYLE = {
    "width": "100%",
    "borderCollapse": "collapse",
    "fontSize": "13px",
}

HEADER_CELL_STYLE = {
    "textAlign": "left",
    "padding": "9px",
    "borderBottom": "1px solid #30363d",
    "color": "#8b949e",
    "whiteSpace": "nowrap",
}

CELL_STYLE = {
    "padding": "9px",
    "borderBottom": "1px solid #21262d",
    "whiteSpace": "nowrap",
}


def metric_card(
    label: str,
    value: str,
    detail: str,
    color: str = "#e6edf3",
) -> html.Div:
    return html.Div(
        style={
            **CARD_STYLE,
            "padding": "14px",
            "minHeight": "88px",
        },
        children=[
            html.Div(
                label,
                style={
                    "color": "#8b949e",
                    "fontSize": "12px",
                    "marginBottom": "7px",
                },
            ),
            html.Div(
                value,
                style={
                    "fontSize": "24px",
                    "fontWeight": "700",
                    "color": color,
                },
            ),
            html.Div(
                detail,
                style={
                    "fontSize": "12px",
                    "color": "#8b949e",
                    "marginTop": "5px",
                },
            ),
        ],
    )


# ---------------------------------------------------------------------------
# Dash application
# ---------------------------------------------------------------------------

app = dash.Dash(
    __name__,
    title="UAV Dynamic Clustering Monitor",
)

app.layout = html.Div(
    style={
        "backgroundColor": "#0d1117",
        "color": "#e6edf3",
        "fontFamily": "monospace",
        "padding": "20px",
        "minHeight": "100vh",
    },
    children=[
        html.H2(
            "UAV Swarm — Dynamic Clustering and NS-3 Monitor",
            style={
                "textAlign": "center",
                "color": "#58a6ff",
                "marginBottom": "6px",
            },
        ),
        html.Div(
            "Live weighted score, cluster-head election, backup selection, "
            "assignment routes, positions, and wireless conditions",
            style={
                "textAlign": "center",
                "color": "#8b949e",
                "marginBottom": "18px",
            },
        ),

        html.Div(
            id="cluster-summary",
            style={
                "display": "grid",
                "gridTemplateColumns": (
                    "repeat(auto-fit, minmax(170px, 1fr))"
                ),
                "gap": "12px",
                "marginBottom": "20px",
            },
        ),

        html.Div(
            style={
                "display": "grid",
                "gridTemplateColumns": (
                    "repeat(auto-fit, minmax(420px, 1fr))"
                ),
                "gap": "20px",
            },
            children=[
                html.Div(
                    style=CARD_STYLE,
                    children=[
                        html.H4(
                            "Live Cluster Topology — Stable Top-Down XY",
                            style={"color": "#8b949e"},
                        ),
                        dcc.Graph(
                            id="position-plot",
                            style={"height": "470px"},
                            config={"displaylogo": False},
                        ),
                    ],
                ),

                html.Div(
                    style=CARD_STYLE,
                    children=[
                        html.H4(
                            "Current Cluster Assignments",
                            style={"color": "#8b949e"},
                        ),
                        html.Div(id="assignment-table"),
                    ],
                ),
            ],
        ),

        html.Div(
            style={
                **CARD_STYLE,
                "marginTop": "20px",
            },
            children=[
                html.H4(
                    "Dynamic Cluster-Head Score Calculation",
                    style={"color": "#8b949e"},
                ),
                html.Div(
                    [
                        "Score = ",
                        html.Span(
                            "0.40 × GCS",
                            style={"color": "#58a6ff"},
                        ),
                        " + ",
                        html.Span(
                            "0.30 × Neighbor",
                            style={"color": "#3fb950"},
                        ),
                        " + ",
                        html.Span(
                            "0.20 × Mobility",
                            style={"color": "#e3b341"},
                        ),
                        " + ",
                        html.Span(
                            "0.10 × Obstacle",
                            style={"color": "#d2a8ff"},
                        ),
                    ],
                    style={
                        "marginBottom": "12px",
                        "fontSize": "14px",
                    },
                ),
                html.Div(
                    id="score-table",
                    style={"overflowX": "auto"},
                ),
            ],
        ),

        html.Div(
            style={
                "display": "grid",
                "gridTemplateColumns": (
                    "repeat(auto-fit, minmax(420px, 1fr))"
                ),
                "gap": "20px",
                "marginTop": "20px",
            },
            children=[
                html.Div(
                    style=CARD_STYLE,
                    children=[
                        html.H4(
                            "Cluster Score History",
                            style={"color": "#8b949e"},
                        ),
                        dcc.Graph(
                            id="score-history-plot",
                            style={"height": "320px"},
                            config={"displaylogo": False},
                        ),
                    ],
                ),

                html.Div(
                    style=CARD_STYLE,
                    children=[
                        html.H4(
                            "Wireless Link Metrics",
                            style={"color": "#8b949e"},
                        ),
                        html.Div(
                            id="link-table",
                            style={"overflowX": "auto"},
                        ),
                    ],
                ),

                html.Div(
                    style=CARD_STYLE,
                    children=[
                        html.H4(
                            "NS-3 Packet Loss",
                            style={"color": "#8b949e"},
                        ),
                        dcc.Graph(
                            id="loss-plot",
                            style={"height": "300px"},
                            config={"displaylogo": False},
                        ),
                    ],
                ),

                html.Div(
                    style=CARD_STYLE,
                    children=[
                        html.H4(
                            "Cluster Election Events",
                            style={"color": "#8b949e"},
                        ),
                        html.Div(id="event-log"),
                    ],
                ),
            ],
        ),

        dcc.Interval(
            id="dashboard-ticker",
            interval=500,
            n_intervals=0,
        ),
    ],
)


@app.callback(
    Output("cluster-summary", "children"),
    Output("position-plot", "figure"),
    Output("assignment-table", "children"),
    Output("score-table", "children"),
    Output("score-history-plot", "figure"),
    Output("link-table", "children"),
    Output("loss-plot", "figure"),
    Output("event-log", "children"),
    Input("dashboard-ticker", "n_intervals"),
)
def update_dashboard(_: int):
    with state_lock:
        current_positions = {
            node_id: dict(position)
            for node_id, position in positions.items()
        }
        current_rssi = dict(link_rssi)
        current_snr = dict(link_snr)
        current_obstacle = dict(link_obstacle_loss)
        current_scores = {
            uav_id: dict(details)
            for uav_id, details in cluster_scores.items()
        }
        current_cluster_state = dict(cluster_state)
        current_cluster_state["assignments"] = [
            dict(assignment)
            for assignment in cluster_state.get(
                "assignments",
                [],
            )
        ]
        current_events = [
            dict(event)
            for event in cluster_events
        ]
        current_trace_stats = {
            key: list(values)
            for key, values in trace_stats.items()
        }
        current_score_history = {
            uav_id: list(values)
            for uav_id, values in score_history.items()
        }
        current_score_times = {
            uav_id: list(values)
            for uav_id, values in score_time_history.items()
        }
        current_topic_update = dict(last_topic_update)

    primary_ch = int(
        current_cluster_state.get("primary_ch", 0) or 0
    )
    backup_ch = int(
        current_cluster_state.get("backup_ch", 0) or 0
    )
    epoch = int(current_cluster_state.get("epoch", 0) or 0)
    status = str(
        current_cluster_state.get("status", "WAITING")
    )

    assignment_map = get_assignment_map(
        current_cluster_state
    )

    # ------------------------------------------------------------------
    # Summary cards
    # ------------------------------------------------------------------

    assignment_age = (
        time.time()
        - current_topic_update.get("assignment", 0.0)
    )

    if "assignment" not in current_topic_update:
        age_text = "No assignment received"
    else:
        age_text = f"Updated {assignment_age:.1f}s ago"

    last_event_reason = "No event received"

    if current_events:
        last_event_reason = str(
            current_events[0].get("reason", "event")
        )

    summary_cards = [
        metric_card(
            "Cluster status",
            status,
            age_text,
            "#3fb950" if status == "ACTIVE" else "#e3b341",
        ),
        metric_card(
            "Primary cluster head",
            display_uav(primary_ch),
            f"Election epoch {epoch}",
            role_color("PRIMARY_CH"),
        ),
        metric_card(
            "Backup cluster head",
            display_uav(backup_ch),
            (
                "No eligible backup"
                if backup_ch == 0
                else "Standby gateway"
            ),
            (
                "#8b949e"
                if backup_ch == 0
                else role_color("BACKUP_CH")
            ),
        ),
        metric_card(
            "Latest election event",
            last_event_reason.replace("_", " ").title(),
            (
                time.strftime(
                    "%H:%M:%S",
                    time.localtime(
                        current_events[0].get(
                            "_received_at",
                            time.time(),
                        )
                    ),
                )
                if current_events
                else "Waiting"
            ),
            "#d2a8ff",
        ),
    ]

    # ------------------------------------------------------------------
    # Stable top-down topology
    # ------------------------------------------------------------------

    node_colors = []
    node_sizes = []
    node_symbols = []
    node_labels = []
    hover_text = []

    for node_id in NODE_IDS:
        if node_id == 0:
            role = "GCS"
            color = "#f0f6fc"
            size = 18
            symbol = "square"
            label = "GCS"
        else:
            assignment = assignment_map.get(node_id, {})
            role = str(assignment.get("role", "UNASSIGNED"))
            color = role_color(role)
            size = (
                24
                if role == "PRIMARY_CH"
                else 21
                if role == "BACKUP_CH"
                else 17
            )
            symbol = (
                "diamond"
                if role == "PRIMARY_CH"
                else "square"
                if role == "BACKUP_CH"
                else "circle"
            )
            label = f"UAV{node_id}"

        position = current_positions[node_id]

        node_colors.append(color)
        node_sizes.append(size)
        node_symbols.append(symbol)
        node_labels.append(
            f"{label}<br>{position['z']:.0f} m"
        )
        hover_text.append(
            f"{label}<br>"
            f"Role: {role}<br>"
            f"X: {position['x']:.2f} m<br>"
            f"Y: {position['y']:.2f} m<br>"
            f"Altitude: {position['z']:.2f} m"
        )

    selected_edges = set()

    for assignment in assignment_map.values():
        route = assignment.get("route", [])

        if not isinstance(route, list):
            continue

        for index in range(len(route) - 1):
            selected_edges.add(
                link_key(
                    int(route[index]),
                    int(route[index + 1]),
                )
            )

    link_traces = []

    for node_a in NODE_IDS:
        for node_b in NODE_IDS:
            if node_b <= node_a:
                continue

            key = link_key(node_a, node_b)
            rssi_dbm = current_rssi.get(key)

            if rssi_dbm is None:
                rssi_dbm = calculated_rssi_fallback(
                    current_positions[node_a],
                    current_positions[node_b],
                )

            snr_db = current_snr.get(key, float("nan"))
            obstacle_db = current_obstacle.get(
                key,
                float("nan"),
            )

            is_cluster_edge = key in selected_edges
            label_a = "GCS" if node_a == 0 else f"UAV{node_a}"
            label_b = f"UAV{node_b}"

            link_traces.append(
                go.Scatter(
                    x=[
                        current_positions[node_a]["x"],
                        current_positions[node_b]["x"],
                    ],
                    y=[
                        current_positions[node_a]["y"],
                        current_positions[node_b]["y"],
                    ],
                    mode="lines",
                    line={
                        "color": link_color(rssi_dbm),
                        "width": 7 if is_cluster_edge else 2,
                        "dash": "solid" if is_cluster_edge else "dot",
                    },
                    opacity=1.0 if is_cluster_edge else 0.30,
                    hovertext=(
                        f"{label_a} ↔ {label_b}<br>"
                        f"RSSI: {rssi_dbm:.1f} dBm<br>"
                        f"SNR: {snr_db:.1f} dB<br>"
                        f"Obstacle loss: {obstacle_db:.1f} dB<br>"
                        f"Selected cluster route: "
                        f"{'YES' if is_cluster_edge else 'NO'}"
                    ),
                    hoverinfo="text",
                    showlegend=False,
                )
            )

    node_trace = go.Scatter(
        x=[
            current_positions[node_id]["x"]
            for node_id in NODE_IDS
        ],
        y=[
            current_positions[node_id]["y"]
            for node_id in NODE_IDS
        ],
        mode="markers+text",
        text=node_labels,
        textposition="top center",
        hovertext=hover_text,
        hoverinfo="text",
        marker={
            "size": node_sizes,
            "color": node_colors,
            "symbol": node_symbols,
            "line": {
                "color": "#f0f6fc",
                "width": 1.5,
            },
        },
        name="Nodes",
        showlegend=False,
    )

    x_range, y_range = update_stable_topology_bounds(
        current_positions
    )

    position_figure = go.Figure(
        data=link_traces + [node_trace]
    )

    position_figure.update_layout(
        paper_bgcolor="#161b22",
        plot_bgcolor="#161b22",
        font={"color": "#e6edf3"},
        showlegend=False,
        margin={"l": 55, "r": 20, "t": 25, "b": 50},
        uirevision="stable-cluster-topology-v1",
        xaxis={
            "title": "World X (m)",
            "color": "#8b949e",
            "range": x_range,
            "showgrid": True,
            "gridcolor": "#21262d",
            "zeroline": True,
            "zerolinecolor": "#484f58",
            "fixedrange": False,
            "constrain": "domain",
        },
        yaxis={
            "title": "World Y (m)",
            "color": "#8b949e",
            "range": y_range,
            "showgrid": True,
            "gridcolor": "#21262d",
            "zeroline": True,
            "zerolinecolor": "#484f58",
            "fixedrange": False,
            "scaleanchor": "x",
            "scaleratio": 1,
        },
        hovermode="closest",
        annotations=[
            {
                "x": 0.01,
                "y": 0.99,
                "xref": "paper",
                "yref": "paper",
                "xanchor": "left",
                "yanchor": "top",
                "showarrow": False,
                "text": (
                    "Thick solid = selected cluster route"
                    "<br>Thin dotted = available wireless link"
                    "<br>Node label second line = altitude"
                ),
                "font": {
                    "color": "#8b949e",
                    "size": 11,
                },
                "bgcolor": "rgba(13,17,23,0.75)",
                "bordercolor": "#30363d",
                "borderwidth": 1,
                "borderpad": 5,
            }
        ],
    )

    # ------------------------------------------------------------------
    # Assignment table
    # ------------------------------------------------------------------

    assignment_rows = []

    for uav_id in UAV_IDS:
        assignment = assignment_map.get(
            uav_id,
            {
                "uav_id": uav_id,
                "role": "WAITING",
                "parent": 0,
                "route": [],
                "score": current_scores[uav_id]["score"],
                "gcs_snr_db": current_scores[uav_id][
                    "gcs_snr_db"
                ],
            },
        )

        role = str(assignment.get("role", "WAITING"))
        parent = int(assignment.get("parent", 0) or 0)
        route = assignment.get("route", [])

        route_text = (
            " → ".join(
                "GCS" if int(node) == 0 else f"UAV{int(node)}"
                for node in route
            )
            if isinstance(route, list) and route
            else "No route"
        )

        assignment_rows.append(
            html.Tr(
                [
                    html.Td(
                        f"UAV{uav_id}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        role,
                        style={
                            **CELL_STYLE,
                            "color": role_color(role),
                            "fontWeight": "700",
                        },
                    ),
                    html.Td(
                        display_uav(parent)
                        if parent != 0
                        else "GCS",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        route_text,
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{float(assignment.get('score', 0.0)):.4f}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{float(assignment.get('gcs_snr_db', -100.0)):.2f}",
                        style=CELL_STYLE,
                    ),
                ]
            )
        )

    assignment_table = html.Table(
        [
            html.Thead(
                html.Tr(
                    [
                        html.Th(
                            heading,
                            style=HEADER_CELL_STYLE,
                        )
                        for heading in [
                            "UAV",
                            "Role",
                            "Parent",
                            "Selected route",
                            "Score",
                            "GCS SNR",
                        ]
                    ]
                )
            ),
            html.Tbody(assignment_rows),
        ],
        style=TABLE_STYLE,
    )

    # ------------------------------------------------------------------
    # Full score-calculation table
    # ------------------------------------------------------------------

    ranked_uavs = sorted(
        UAV_IDS,
        key=lambda uav_id: (
            -current_scores[uav_id]["score"],
            uav_id,
        ),
    )
    rank_map = {
        uav_id: rank
        for rank, uav_id in enumerate(
            ranked_uavs,
            start=1,
        )
    }

    score_rows = []

    for uav_id in UAV_IDS:
        details = current_scores[uav_id]

        gcs_normalized = normalize_gcs_snr(
            details["gcs_snr_db"]
        )

        gcs_contribution = (
            WEIGHTS["gcs"] * gcs_normalized
        )
        neighbor_contribution = (
            WEIGHTS["neighbors"]
            * details["neighbor_score"]
        )
        mobility_contribution = (
            WEIGHTS["mobility"]
            * details["mobility_stability"]
        )
        obstacle_contribution = (
            WEIGHTS["obstacle"]
            * details["obstacle_robustness"]
        )

        reconstructed_total = (
            gcs_contribution
            + neighbor_contribution
            + mobility_contribution
            + obstacle_contribution
        )

        selected_role = str(
            assignment_map.get(uav_id, {}).get(
                "role",
                "WAITING",
            )
        )

        score_rows.append(
            html.Tr(
                [
                    html.Td(
                        f"#{rank_map[uav_id]} UAV{uav_id}",
                        style={
                            **CELL_STYLE,
                            "fontWeight": "700",
                        },
                    ),
                    html.Td(
                        "YES"
                        if details["candidate"] > 0.5
                        else "NO",
                        style={
                            **CELL_STYLE,
                            "color": (
                                "#3fb950"
                                if details["candidate"] > 0.5
                                else "#f85149"
                            ),
                        },
                    ),
                    html.Td(
                        selected_role,
                        style={
                            **CELL_STYLE,
                            "color": role_color(selected_role),
                        },
                    ),
                    html.Td(
                        f"{details['gcs_snr_db']:.2f}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{gcs_normalized:.4f}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{gcs_contribution:.4f}",
                        style={
                            **CELL_STYLE,
                            "color": "#58a6ff",
                        },
                    ),
                    html.Td(
                        f"{details['neighbor_score']:.4f}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{neighbor_contribution:.4f}",
                        style={
                            **CELL_STYLE,
                            "color": "#3fb950",
                        },
                    ),
                    html.Td(
                        f"{details['mobility_stability']:.4f}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{mobility_contribution:.4f}",
                        style={
                            **CELL_STYLE,
                            "color": "#e3b341",
                        },
                    ),
                    html.Td(
                        f"{details['obstacle_robustness']:.4f}",
                        style=CELL_STYLE,
                    ),
                    html.Td(
                        f"{obstacle_contribution:.4f}",
                        style={
                            **CELL_STYLE,
                            "color": "#d2a8ff",
                        },
                    ),
                    html.Td(
                        f"{details['score']:.4f}",
                        style={
                            **CELL_STYLE,
                            "fontWeight": "700",
                        },
                    ),
                    html.Td(
                        f"{reconstructed_total:.4f}",
                        style=CELL_STYLE,
                    ),
                ]
            )
        )

    score_table = html.Table(
        [
            html.Thead(
                html.Tr(
                    [
                        html.Th(
                            heading,
                            style=HEADER_CELL_STYLE,
                        )
                        for heading in [
                            "Rank/UAV",
                            "Candidate",
                            "Role",
                            "GCS SNR",
                            "GCS norm",
                            "0.40×GCS",
                            "Neighbor",
                            "0.30×Neighbor",
                            "Mobility",
                            "0.20×Mobility",
                            "Obstacle",
                            "0.10×Obstacle",
                            "Published total",
                            "Recalculated",
                        ]
                    ]
                )
            ),
            html.Tbody(score_rows),
        ],
        style=TABLE_STYLE,
    )

    # ------------------------------------------------------------------
    # Score history
    # ------------------------------------------------------------------

    score_history_figure = go.Figure()

    score_colors = {
        1: "#58a6ff",
        2: "#3fb950",
        3: "#f0883e",
    }

    has_score_history = False

    for uav_id in UAV_IDS:
        values = current_score_history[uav_id]
        timestamps = current_score_times[uav_id]

        if values:
            has_score_history = True

        readable_times = [
            time.strftime(
                "%H:%M:%S",
                time.localtime(timestamp),
            )
            for timestamp in timestamps
        ]

        score_history_figure.add_trace(
            go.Scatter(
                x=readable_times,
                y=values,
                mode="lines",
                name=f"UAV{uav_id}",
                line={"color": score_colors[uav_id]},
            )
        )

    if not has_score_history:
        score_history_figure = blank_figure(
            "Waiting for /cluster/scores",
            height=320,
        )
    else:
        score_history_figure.update_layout(
            paper_bgcolor="#161b22",
            plot_bgcolor="#161b22",
            font={"color": "#e6edf3"},
            margin={"l": 45, "r": 10, "t": 10, "b": 40},
            yaxis={
                "title": "Weighted score",
                "range": [0.0, 1.0],
                "color": "#8b949e",
            },
            xaxis={
                "title": "Time",
                "color": "#8b949e",
                "tickangle": -35,
            },
            legend={"bgcolor": "#0d1117"},
        )

    # ------------------------------------------------------------------
    # Link table
    # ------------------------------------------------------------------

    link_rows = []

    for node_a in NODE_IDS:
        for node_b in NODE_IDS:
            if node_b <= node_a:
                continue

            key = link_key(node_a, node_b)
            distance = distance_3d(
                current_positions[node_a],
                current_positions[node_b],
            )

            rssi_dbm = current_rssi.get(key)

            if rssi_dbm is None:
                rssi_dbm = calculated_rssi_fallback(
                    current_positions[node_a],
                    current_positions[node_b],
                )

            snr_db = current_snr.get(
                key,
                float("nan"),
            )
            obstacle_db = current_obstacle.get(
                key,
                float("nan"),
            )

            if node_a == 0:
                threshold = CANDIDATE_MIN_GCS_SNR_DB
            else:
                threshold = MEMBER_MIN_SNR_DB

            snr_up = (
                not math.isnan(snr_db)
                and snr_db >= threshold
            )

            label_a = "GCS" if node_a == 0 else f"UAV{node_a}"
            label_b = f"UAV{node_b}"

            link_rows.append(
                html.Tr(
                    [
                        html.Td(
                            f"{label_a} ↔ {label_b}",
                            style=CELL_STYLE,
                        ),
                        html.Td(
                            f"{distance:.1f} m",
                            style=CELL_STYLE,
                        ),
                        html.Td(
                            f"{rssi_dbm:.2f} dBm",
                            style=CELL_STYLE,
                        ),
                        html.Td(
                            f"{snr_db:.2f} dB",
                            style=CELL_STYLE,
                        ),
                        html.Td(
                            f"{obstacle_db:.2f} dB",
                            style=CELL_STYLE,
                        ),
                        html.Td(
                            quality_label(rssi_dbm),
                            style={
                                **CELL_STYLE,
                                "color": link_color(rssi_dbm),
                            },
                        ),
                        html.Td(
                            "UP" if snr_up else "DOWN",
                            style={
                                **CELL_STYLE,
                                "color": (
                                    "#3fb950"
                                    if snr_up
                                    else "#f85149"
                                ),
                                "fontWeight": "700",
                            },
                        ),
                        html.Td(
                            "SELECTED"
                            if key in selected_edges
                            else "",
                            style={
                                **CELL_STYLE,
                                "color": "#58a6ff",
                            },
                        ),
                    ]
                )
            )

    link_table = html.Table(
        [
            html.Thead(
                html.Tr(
                    [
                        html.Th(
                            heading,
                            style=HEADER_CELL_STYLE,
                        )
                        for heading in [
                            "Link",
                            "Distance",
                            "RSSI",
                            "SNR",
                            "Obstacle loss",
                            "Quality",
                            "Eligibility",
                            "Cluster route",
                        ]
                    ]
                )
            ),
            html.Tbody(link_rows),
        ],
        style=TABLE_STYLE,
    )

    # ------------------------------------------------------------------
    # Packet loss
    # ------------------------------------------------------------------

    if current_trace_stats["loss_pct"]:
        loss_times = [
            time.strftime(
                "%H:%M:%S",
                time.localtime(timestamp),
            )
            for timestamp in current_trace_stats["times"]
        ]

        loss_figure = go.Figure(
            go.Scatter(
                x=loss_times,
                y=current_trace_stats["loss_pct"],
                mode="lines+markers",
                line={"color": "#f78166"},
                fill="tozeroy",
                fillcolor="rgba(247,129,102,0.15)",
                name="Packet loss",
            )
        )

        loss_figure.update_layout(
            paper_bgcolor="#161b22",
            plot_bgcolor="#161b22",
            font={"color": "#e6edf3"},
            margin={"l": 45, "r": 10, "t": 10, "b": 40},
            yaxis={
                "title": "Loss %",
                "range": [0, 100],
                "color": "#8b949e",
            },
            xaxis={
                "title": "Time",
                "color": "#8b949e",
                "tickangle": -35,
            },
        )
    else:
        loss_figure = blank_figure(
            f"Waiting for NS-3 trace: {TRACE_FILE}",
            height=300,
        )

    # ------------------------------------------------------------------
    # Event history
    # ------------------------------------------------------------------

    if not current_events:
        event_log = html.Div(
            "Waiting for /cluster/event",
            style={"color": "#8b949e"},
        )
    else:
        event_items = []

        for event in current_events[:12]:
            event_time = time.strftime(
                "%H:%M:%S",
                time.localtime(
                    event.get("_received_at", time.time())
                ),
            )

            old_primary = int(
                event.get("old_primary", 0) or 0
            )
            new_primary = int(
                event.get("new_primary", 0) or 0
            )
            old_backup = int(
                event.get("old_backup", 0) or 0
            )
            new_backup = int(
                event.get("new_backup", 0) or 0
            )
            reason = str(
                event.get("reason", "unknown")
            )

            event_items.append(
                html.Div(
                    style={
                        "padding": "9px 0",
                        "borderBottom": "1px solid #21262d",
                    },
                    children=[
                        html.Div(
                            [
                                html.Span(
                                    event_time,
                                    style={"color": "#8b949e"},
                                ),
                                "  ",
                                html.Span(
                                    reason.replace("_", " ").title(),
                                    style={
                                        "color": "#d2a8ff",
                                        "fontWeight": "700",
                                    },
                                ),
                            ]
                        ),
                        html.Div(
                            (
                                f"Primary: {display_uav(old_primary)} "
                                f"→ {display_uav(new_primary)} | "
                                f"Backup: {display_uav(old_backup)} "
                                f"→ {display_uav(new_backup)}"
                            ),
                            style={
                                "fontSize": "12px",
                                "color": "#8b949e",
                                "marginTop": "4px",
                            },
                        ),
                    ],
                )
            )

        event_log = html.Div(event_items)

    return (
        summary_cards,
        position_figure,
        assignment_table,
        score_table,
        score_history_figure,
        link_table,
        loss_figure,
        event_log,
    )


if __name__ == "__main__":
    threading.Thread(
        target=ros_spin_thread,
        daemon=True,
        name="ros2-dashboard-reader",
    ).start()

    threading.Thread(
        target=ns3_trace_reader,
        daemon=True,
        name="ns3-trace-reader",
    ).start()

    print(f"Dashboard -> http://localhost:{DASH_PORT}")

    app.run(
        debug=False,
        host=DASH_HOST,
        port=DASH_PORT,
    )