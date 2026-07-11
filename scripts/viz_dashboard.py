#!/usr/bin/env python3
"""
Real-time NS-3 + UAV visualization dashboard.
Reads UAV positions from ROS2 and NS-3 trace for channel stats.

Requirements: pip install dash plotly pymavlink
Run AFTER NS-3 and SITL are up:
  python3 scripts/viz_dashboard.py
Then open: http://localhost:8050
"""

import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go
import subprocess, threading, math, time, re
from collections import deque

# ── Config ─────────────────────────────────────────────────────────────────
UAV_IDS   = [1, 2, 3]
TRACE_FILE = "/tmp/ns3_wifi_trace.tr"

# Path loss params (mirror your .cc)
# REMOVED: PATH_LOSS_EXP = 2.7  (urban ground-level exponent)
# ADDED: matches the air-to-air exponent now used in the NS-3 scenario
PATH_LOSS_EXP   = 2.0
REF_DISTANCE    = 1.0
REF_LOSS_DB     = 46.67
TX_POWER_DBM    = 20.0
RX_SENSITIVITY  = -82.0   # dBm — below this = packet loss

# ── Shared state ───────────────────────────────────────────────────────────
positions = {i: {"x": 0.0, "y": 0.0, "z": 10.0, "heading": 0.0} for i in UAV_IDS}
trace_stats = {"tx": deque(maxlen=200), "rx": deque(maxlen=200),
               "loss_pct": deque(maxlen=200), "times": deque(maxlen=200)}
lock = threading.Lock()

# ── ROS2 position reader ───────────────────────────────────────────────────
def ros2_position_reader():
    """Subscribe to /uav{N}/local_position/pose via ros2 topic echo."""
    import subprocess, json
    for uav_id in UAV_IDS:
        topic = f"/uav{uav_id}/mavros/local_position/pose"
        cmd = ["ros2", "topic", "echo", "--once", "--spin-time", "0.1",
               topic, "--no-arr", "--csv"]
        # This is a polling approach — for production use rclpy subscriber
    
    while True:
        for uav_id in UAV_IDS:
            try:
                topic = f"/uav{uav_id}/mavros/local_position/pose"
                result = subprocess.run(
                    ["ros2", "topic", "echo", "--once", "--spin-time", "0.2",
                     topic],
                    capture_output=True, text=True, timeout=1.0
                )
                # Parse position from YAML output
                out = result.stdout
                x_match = re.search(r'x:\s*([\-\d.]+)', out)
                y_match = re.search(r'y:\s*([\-\d.]+)', out)
                z_match = re.search(r'z:\s*([\-\d.]+)', out)
                if x_match and y_match and z_match:
                    with lock:
                        positions[uav_id]["x"] = float(x_match.group(1))
                        positions[uav_id]["y"] = float(y_match.group(1))
                        positions[uav_id]["z"] = float(z_match.group(1))
            except Exception:
                pass
        time.sleep(0.5)

# ── NS-3 trace reader ──────────────────────────────────────────────────────
def ns3_trace_reader():
    """Tail the NS-3 ascii trace and count Tx/Rx events."""
    tx_count = rx_count = 0
    last_report = time.time()
    
    try:
        with open(TRACE_FILE, "r") as f:
            f.seek(0, 2)  # seek to end
            while True:
                line = f.readline()
                if not line:
                    time.sleep(0.05)
                    continue
                if line.startswith("t "):   # transmitted
                    tx_count += 1
                elif line.startswith("r "):  # received
                    rx_count += 1
                
                now = time.time()
                if now - last_report >= 1.0:
                    loss = 0.0
                    if tx_count > 0:
                        loss = max(0.0, (tx_count - rx_count) / tx_count * 100)
                    with lock:
                        trace_stats["tx"].append(tx_count)
                        trace_stats["rx"].append(rx_count)
                        trace_stats["loss_pct"].append(loss)
                        trace_stats["times"].append(now)
                    tx_count = rx_count = 0
                    last_report = now
    except FileNotFoundError:
        pass  # NS-3 not started yet

# ── Path loss calculator ───────────────────────────────────────────────────
def calc_rssi(pos_a, pos_b):
    """Log-distance path loss (mirrors your NS-3 model)."""
    d = math.sqrt((pos_a["x"]-pos_b["x"])**2 +
                  (pos_a["y"]-pos_b["y"])**2 +
                  (pos_a["z"]-pos_b["z"])**2)
    if d < 0.1:
        return TX_POWER_DBM
    if d < REF_DISTANCE:
        d = REF_DISTANCE
    loss = REF_LOSS_DB + 10 * PATH_LOSS_EXP * math.log10(d / REF_DISTANCE)
    return TX_POWER_DBM - loss

def link_color(rssi):
    if rssi > -60:  return "#00ff88"   # strong — green
    if rssi > -72:  return "#ffcc00"   # medium — yellow
    return "#ff4444"                    # weak   — red

# ── Dash app ───────────────────────────────────────────────────────────────
app = dash.Dash(__name__, title="UAV Swarm · NS-3 Channel Monitor")
app.layout = html.Div(style={"backgroundColor":"#0d1117","color":"#e6edf3",
                              "fontFamily":"monospace","padding":"20px"}, children=[
    html.H2("🚁 UAV Swarm — NS-3 Nakagami Channel Monitor",
            style={"textAlign":"center","color":"#58a6ff"}),

    html.Div(style={"display":"grid","gridTemplateColumns":"1fr 1fr","gap":"20px"}, children=[

        # ── 3D Position plot ──────────────────────────────────────────────
        html.Div([
            html.H4("UAV Positions (Live)", style={"color":"#8b949e"}),
            dcc.Graph(id="pos-plot", style={"height":"400px"})
        ], style={"backgroundColor":"#161b22","borderRadius":"8px","padding":"16px"}),

        # ── Link quality table ─────────────────────────────────────────
        html.Div([
            html.H4("Link Quality (Computed RSSI)", style={"color":"#8b949e"}),
            html.Div(id="link-table")
        ], style={"backgroundColor":"#161b22","borderRadius":"8px","padding":"16px"}),

        # ── Packet loss chart ──────────────────────────────────────────
        html.Div([
            html.H4("Packet Loss % (NS-3 Trace)", style={"color":"#8b949e"}),
            dcc.Graph(id="loss-plot", style={"height":"300px"})
        ], style={"backgroundColor":"#161b22","borderRadius":"8px","padding":"16px"}),

        # ── RSSI timeline ──────────────────────────────────────────────
        html.Div([
            html.H4("RSSI Timeline (dBm)", style={"color":"#8b949e"}),
            dcc.Graph(id="rssi-plot", style={"height":"300px"})
        ], style={"backgroundColor":"#161b22","borderRadius":"8px","padding":"16px"}),
    ]),

    dcc.Interval(id="ticker", interval=500, n_intervals=0),
])

rssi_history = {(i,j): deque(maxlen=120) for i in UAV_IDS for j in UAV_IDS if j > i}
rssi_times   = deque(maxlen=120)

@app.callback(
    Output("pos-plot",   "figure"),
    Output("link-table", "children"),
    Output("loss-plot",  "figure"),
    Output("rssi-plot",  "figure"),
    Input("ticker",      "n_intervals")
)
def update(_):
    with lock:
        pos = {k: dict(v) for k, v in positions.items()}
        stats = {k: list(v) for k, v in trace_stats.items()}

    # ── 3D scatter + link lines ───────────────────────────────────────────
    pairs = [(1,2),(1,3),(2,3)]
    rssi_now = {}
    for a, b in pairs:
        rssi_now[(a,b)] = calc_rssi(pos[a], pos[b])

    rssi_times.append(time.time())
    for (a,b), r in rssi_now.items():
        rssi_history[(a,b)].append(r)

    scatter = go.Scatter3d(
        x=[pos[i]["x"] for i in UAV_IDS],
        y=[pos[i]["y"] for i in UAV_IDS],
        z=[pos[i]["z"] for i in UAV_IDS],
        mode="markers+text",
        text=[f"UAV{i}" for i in UAV_IDS],
        textposition="top center",
        marker=dict(size=10, color=["#58a6ff","#3fb950","#f78166"]),
    )
    link_traces = []
    for a, b in pairs:
        rssi = rssi_now[(a,b)]
        link_traces.append(go.Scatter3d(
            x=[pos[a]["x"], pos[b]["x"]],
            y=[pos[a]["y"], pos[b]["y"]],
            z=[pos[a]["z"], pos[b]["z"]],
            mode="lines",
            line=dict(color=link_color(rssi), width=4),
            name=f"UAV{a}↔UAV{b}",
            hovertext=f"{rssi:.1f} dBm",
        ))
    pos_fig = go.Figure(data=[scatter]+link_traces)
    pos_fig.update_layout(
        paper_bgcolor="#161b22", plot_bgcolor="#161b22",
        font_color="#e6edf3", showlegend=False,
        margin=dict(l=0,r=0,t=0,b=0),
        scene=dict(
            bgcolor="#161b22",
            xaxis=dict(title="X (m)", color="#8b949e"),
            yaxis=dict(title="Y (m)", color="#8b949e"),
            zaxis=dict(title="Alt (m)", color="#8b949e"),
        )
    )

    # ── Link quality table ────────────────────────────────────────────────
    rows = []
    for a, b in pairs:
        rssi = rssi_now[(a,b)]
        d = math.sqrt((pos[a]["x"]-pos[b]["x"])**2 +
                      (pos[a]["y"]-pos[b]["y"])**2 +
                      (pos[a]["z"]-pos[b]["z"])**2)
        quality = "🟢 Strong" if rssi > -60 else ("🟡 Medium" if rssi > -72 else "🔴 Weak")
        link_ok = rssi > RX_SENSITIVITY
        rows.append(html.Tr([
            html.Td(f"UAV{a} ↔ UAV{b}"),
            html.Td(f"{d:.1f} m"),
            html.Td(f"{rssi:.1f} dBm"),
            html.Td(quality),
            html.Td("✅ UP" if link_ok else "❌ DOWN",
                    style={"color":"#3fb950" if link_ok else "#f78166"}),
        ]))
    table = html.Table(
        [html.Tr([html.Th(h) for h in
                  ["Link","Distance","RSSI","Quality","Status"]])] + rows,
        style={"width":"100%","borderCollapse":"collapse","fontSize":"14px"}
    )

    # ── Packet loss chart ──────────────────────────────────────────────────
    loss_fig = go.Figure(go.Scatter(
        y=stats.get("loss_pct",[]),
        mode="lines+markers",
        line=dict(color="#f78166"),
        fill="tozeroy",
        fillcolor="rgba(247,129,102,0.15)",
    ))
    loss_fig.update_layout(
        paper_bgcolor="#161b22", plot_bgcolor="#161b22",
        font_color="#e6edf3", margin=dict(l=40,r=10,t=10,b=30),
        yaxis=dict(title="Loss %", range=[0,100], color="#8b949e"),
        xaxis=dict(title="seconds", color="#8b949e"),
    )

    # ── RSSI timeline ──────────────────────────────────────────────────────
    rssi_fig = go.Figure()
    colors = ["#58a6ff","#3fb950","#e3b341"]
    for idx, (a,b) in enumerate(pairs):
        rssi_fig.add_trace(go.Scatter(
            y=list(rssi_history[(a,b)]),
            mode="lines", name=f"UAV{a}↔UAV{b}",
            line=dict(color=colors[idx])
        ))
    rssi_fig.add_hline(y=RX_SENSITIVITY, line_dash="dash",
                       line_color="#f78166", annotation_text="RX floor")
    rssi_fig.update_layout(
        paper_bgcolor="#161b22", plot_bgcolor="#161b22",
        font_color="#e6edf3", margin=dict(l=40,r=10,t=10,b=30),
        yaxis=dict(title="RSSI (dBm)", color="#8b949e"),
        xaxis=dict(title="samples", color="#8b949e"),
        legend=dict(bgcolor="#0d1117"),
    )

    return pos_fig, table, loss_fig, rssi_fig


if __name__ == "__main__":
    threading.Thread(target=ros2_position_reader, daemon=True).start()
    threading.Thread(target=ns3_trace_reader,     daemon=True).start()
    print("Dashboard → http://localhost:8050")
    app.run(debug=False, host="0.0.0.0", port=8050)