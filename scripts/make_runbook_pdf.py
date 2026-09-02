#!/usr/bin/env python3
"""Generate docs/HITL_RUNBOOK.pdf — the full command sequence for a HITL run.

Kept in the repo rather than hand-made so the PDF can be regenerated whenever
the procedure changes:

    python3 scripts/make_runbook_pdf.py
"""
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (BaseDocTemplate, Frame, PageTemplate,
                                Paragraph, Spacer, Table, TableStyle)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "HITL_RUNBOOK.pdf")

INK      = colors.HexColor("#16202c")
INK2     = colors.HexColor("#4a5769")
ACCENT   = colors.HexColor("#1f5f8b")
RULE     = colors.HexColor("#ccd6e0")
CODE_BG  = colors.HexColor("#f2f5f8")
WARN_BG  = colors.HexColor("#fdf0ec")
WARN     = colors.HexColor("#a8402b")
OK_BG    = colors.HexColor("#eaf4ef")
OK       = colors.HexColor("#1c6b4d")

ss = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("title", parent=ss["Title"], fontName="Helvetica-Bold",
                            fontSize=19, leading=23, textColor=INK,
                            alignment=TA_LEFT, spaceAfter=2),
    "sub": ParagraphStyle("sub", fontName="Helvetica", fontSize=9.5, leading=13,
                          textColor=INK2, spaceAfter=10),
    "h1": ParagraphStyle("h1", fontName="Helvetica-Bold", fontSize=12.5, leading=16,
                         textColor=ACCENT, spaceBefore=13, spaceAfter=5),
    "h2": ParagraphStyle("h2", fontName="Helvetica-Bold", fontSize=10, leading=13,
                         textColor=INK, spaceBefore=8, spaceAfter=3),
    "body": ParagraphStyle("body", fontName="Helvetica", fontSize=9, leading=12.5,
                           textColor=INK, spaceAfter=4),
    "note": ParagraphStyle("note", fontName="Helvetica", fontSize=8.4, leading=11.5,
                           textColor=INK2, spaceAfter=4),
    "code": ParagraphStyle("code", fontName="Courier", fontSize=8.1, leading=11,
                           textColor=INK),
}


def para(t, s="body"):
    return Paragraph(t, S[s])


def code(text):
    """Monospace block on a tinted panel."""
    rows = [[Paragraph(l.replace("&", "&amp;").replace("<", "&lt;")
                       .replace(" ", "&nbsp;") or "&nbsp;", S["code"])]
            for l in text.strip("\n").split("\n")]
    t = Table(rows, colWidths=[165 * mm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), CODE_BG),
        ("LINEBEFORE", (0, 0), (0, -1), 1.6, ACCENT),
        ("LEFTPADDING", (0, 0), (-1, -1), 7),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 1),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1),
    ]))
    return [Spacer(1, 3), t, Spacer(1, 5)]


def callout(text, kind="warn"):
    bg, fg, tag = ((WARN_BG, WARN, "!") if kind == "warn" else (OK_BG, OK, "✓"))
    t = Table([[Paragraph(f"<b>{tag}</b>&nbsp;&nbsp;{text}",
                          ParagraphStyle("c", fontName="Helvetica", fontSize=8.4,
                                         leading=11.5, textColor=fg))]],
              colWidths=[165 * mm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), bg),
        ("LEFTPADDING", (0, 0), (-1, -1), 8),
        ("RIGHTPADDING", (0, 0), (-1, -1), 8),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
    ]))
    return [Spacer(1, 2), t, Spacer(1, 5)]


def table(rows, widths):
    data = [[Paragraph(f"<b>{c}</b>" if i == 0 else c,
                       ParagraphStyle("t", fontName="Helvetica", fontSize=8.3,
                                      leading=11, textColor=INK))
             for c in row] for i, row in enumerate(rows)]
    t = Table(data, colWidths=widths)
    t.setStyle(TableStyle([
        ("LINEBELOW", (0, 0), (-1, 0), 0.9, ACCENT),
        ("LINEBELOW", (0, 1), (-1, -2), 0.3, RULE),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return [Spacer(1, 3), t, Spacer(1, 6)]


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(22 * mm, 15 * mm, 188 * mm, 15 * mm)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(INK2)
    canvas.drawString(22 * mm, 10.5 * mm,
                      "HITL Runbook  ·  multi_uav_simulation  ·  "
                      "branch ground-vs-edge-processing-RPi")
    canvas.drawRightString(188 * mm, 10.5 * mm, f"page {doc.page}")
    canvas.restoreState()


story = []
A = story.append
E = story.extend

A(para("HITL Runbook — Edge Detection on the Raspberry Pi 4B", "title"))
A(para("Complete command sequence: host pipeline, Pi edge node, and the "
       "ns-3 impaired link. Commands are marked <b>[HOST]</b> or <b>[PI]</b>.", "sub"))

# ── architecture ──
A(para("Architecture", "h1"))
A(para("One cable carries two logically separate links, split with 802.1Q VLANs. "
       "The camera is treated as the drone's internal camera cable and is never "
       "impaired; only detections cross the simulated radio.", "body"))
E(code("""
  HOST                                              PI 4B  (UAV2)
  Gazebo --- eth-cam  VLAN 10 ==== cable ==== eth0.10 ---> detector
             10.0.0.1                         10.0.0.2     YOLOv8n
                                                              |
  gcsns <-- tap-gcs <-- ns-3 <-- tap-uav2 <-- br-uav2 <-------+
  10.42.0.10            ^        eth-rf VLAN 42     eth0.42
  gcs_receiver     loss, latency, fading            10.42.0.12
"""))
E(table([
    ["Stream", "Path", "Through ns-3?"],
    ["/uav1/camera/image_raw", "host → Pi, VLAN 10", "No — plain cable"],
    ["/uav1/camera/annotated", "Pi → host, VLAN 10", "No — debug only"],
    ["/detections/uav1", "Pi → gcsns, VLAN 42", "YES — measured path"],
], [55 * mm, 60 * mm, 50 * mm]))

# ── 1 ──
A(para("1.  Pre-flight  [HOST]", "h1"))
E(code("""
cd /home/anton/multi_uav_simulation
ip -4 addr show | grep 10.0.0.1        # must return a line
ping -c 2 10.0.0.2                     # Pi must answer
sysctl -n net.core.rmem_max            # must be 536870912
"""))
A(para("If 10.0.0.1 is missing, NetworkManager has taken the adapter:", "note"))
E(code("""
nmcli connection show                  # find the wired profile name
sudo nmcli connection modify "Wired connection 2" \\
     ipv4.method manual ipv4.addresses 10.0.0.1/24 \\
     ipv4.never-default yes ipv6.method disabled
sudo nmcli connection up "Wired connection 2"
"""))
E(callout("10.0.0.1 must exist <b>before</b> Gazebo starts. Fast DDS 2.6 reads the "
          "interface list once at participant creation and has no dynamic detection, "
          "so an address added later can never be used — and the failure is silent."))

# ── 2 ──
A(para("2.  Start the host pipeline  [HOST]", "h1"))
E(code("""
./scripts/netns/sitl_init.sh --gui --view
"""))
A(para("Brings up netns, ns-3, Gazebo, SITL, micro_ros_agent, drone_bridge and the "
       "VLANs. Wait for <b>PIPELINE READY</b>, then start the detectors with "
       "<font face='Courier'>./scripts/netns/detector_start.sh</font> in a second "
       "terminal and fly with "
       "<font face='Courier'>./scripts/netns/run_missions.sh</font>.", "body"))
E(table([
    ["Option", "Effect"],
    ["--mission", "fly uav1_patrol_mission.py automatically"],
    ["--record", "record the camera to a rosbag for offline replay"],
    ["--debug", "detector saves annotated JPEGs (not for timing runs)"],
    ["--pt", "use the PyTorch model instead of NCNN"],
    ["--conf &lt;v&gt;", "detector confidence threshold (default 0.4)"],
], [38 * mm, 127 * mm]))

# ── 3 ──
A(para("3.  Split the Pi's port into two links  [PI]", "h1"))
E(code("""
ssh anton@10.0.0.2
sudo bash ~/pi_hitl_link.sh
"""))
A(para("Creates eth0.10 (10.0.0.2, camera) and eth0.42 (10.42.0.12, radio).", "note"))
E(callout("Not persistent — plain <font face='Courier'>ip</font> commands are lost on "
          "reboot, and the Pi falls back to untagged eth0 while the host still expects "
          "VLAN 10. Re-run this after every Pi reboot until it is moved into netplan."))

# ── 4 ──
A(para("4.  Verify both links  [PI]", "h1"))
E(code("""
ping -c 3 10.0.0.1        # camera link   -> expect ~2 ms
ping -c 30 10.42.0.10     # radio link    -> expect ~59 ms, high jitter
"""))
A(para("Measured reference: camera 2.2 ms avg / 0.8 ms jitter; radio min 3.3, "
       "avg 59.0, max 158.0 ms, 40.8 ms jitter, 0% loss. Use at least 30 pings — "
       "the first packet includes ARP across the simulated channel and skews a "
       "short sample badly.", "note"))
E(callout("If the radio link answers as fast as the cable, detections are NOT crossing "
          "ns-3 and every latency number will be meaningless."))

# ── 5 ──
A(para("5.  Start the edge detector  [PI]", "h1"))
E(code("""
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/uav2_ws/config/fastdds_hitl_eth.xml

~/yolo_env/bin/python ~/yolo_detect_node.py --ros-args \\
    -p model_path:=/home/anton/models/yolov8n.pt \\
    -p show_window:=False
"""))
A(para("<font face='Courier'>show_window:=False</font> is required — the Pi is "
       "headless and cv2.imshow has no display. Swap the model path to "
       "<font face='Courier'>yolov8n_ncnn_model</font> for the NCNN build.", "note"))
E(callout("The Pi's venv is pinned to <font face='Courier'>numpy==1.26.4</font> and "
          "<font face='Courier'>opencv-python==4.10.0.84</font>. Never run "
          "<font face='Courier'>pip install -U</font> there: NumPy 2.x breaks cv_bridge "
          "with <i>_ARRAY_API not found</i>, and OpenCV 5 breaks it with <i>KeyError: 16</i>."))

# ── 6 ──
A(para("6.  Watch  [HOST]", "h1"))
E(code("""
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/anton/multi_uav_simulation/config/fastdds_hitl_eth.xml
source /opt/ros/humble/setup.bash

ros2 topic hz /uav1/camera/image_raw          # must stay steady when the Pi joins
ros2 run rqt_image_view rqt_image_view /uav1/camera/annotated
tail -f /tmp/hitl_gcs.log                     # detections that crossed ns-3
"""))
A(para("Close the annotated view before any timing run — it sends 2.76 MB per frame "
       "back over the cable and costs the Pi a redraw per frame.", "note"))

# ── 7 ──
A(para("7.  Fly the mission  [HOST]", "h1"))
E(code("""
sudo ip netns exec gcsns sudo -H -u anton bash -lc '
    source /opt/ros/humble/setup.bash
    source /home/anton/multi_uav_simulation/ros2/install/setup.bash
    python3 /home/anton/multi_uav_simulation/ros2/uav_controller/uav_controller/uav1_patrol_mission.py
'
"""))
A(para("Must run inside <font face='Courier'>gcsns</font>: drone_bridge lives there, and "
       "from the root namespace its services are invisible and the script hangs on "
       "<i>Waiting for service /uav1/arm</i>.", "note"))

# ── 8 ──
A(para("8.  Shut down  [HOST]", "h1"))
E(code("""
# Ctrl+C in the detector_start.sh, then the sitl_init.sh terminal, or:
bash scripts/netns/kill_all_netns.sh
ssh anton@10.0.0.2 'pkill -f yolo_detect_node'
"""))

# ── reference ──
A(para("Measured reference values", "h1"))
E(table([
    ["Metric", "Value"],
    ["Link throughput (iperf3, steady)", "~834 Mbps, 0 retransmits"],
    ["Camera", "1280×720 RGB, 2.76 MB/frame"],
    ["Camera at 20 Hz / 5 Hz", "481 Mbps / ~111 Mbps"],
    ["Inference — PyTorch @ 960×544", "2,540 ms  (0.39 fps)"],
    ["Inference — PyTorch @ 640×384", "~1,100 ms"],
    ["Inference — NCNN @ 960×544", "~1,000 ms  (0.74 fps)"],
    ["Inference — NCNN @ 640×384", "~470 ms (predicted, not measured)"],
    ["Detection payload", "~118 B vs 2.76 MB/frame image"],
    ["Recall", "2–4 of 5 people per frame"],
], [80 * mm, 85 * mm]))

# ── troubleshooting ──
A(para("Troubleshooting", "h1"))
E(table([
    ["Symptom", "Cause and fix"],
    ["Camera rate collapses when the Pi joins",
     "A RELIABLE subscriber throttles the publisher. The detector must use "
     "BEST_EFFORT / depth 1."],
    ["Camera rate exactly 3.144 s apart",
     "A dead RELIABLE subscriber; Gazebo is retransmitting. Restart the pipeline."],
    ["/clock not ticking",
     "The sim has stalled. Lockstep means Gazebo only steps when SITL sends servo "
     "packets — check SITL, then restart. Always check /clock first: if it is "
     "frozen, every downstream rate is meaningless."],
    ["Topic listed but no data",
     "Socket buffers. rmem_max must be 536870912 on both machines; the default "
     "208 KB holds ~7% of one 2.76 MB frame."],
    ["XMLPARSER Error: realpath failed",
     "FASTRTPS_DEFAULT_PROFILES_FILE points nowhere. Use the absolute path."],
    ["Pi unreachable, Destination Host Unreachable",
     "The Pi rebooted and lost its VLANs; it is untagged while the host expects "
     "VLAN 10. Re-run pi_hitl_link.sh."],
    ["Package 'uav_vision' not found",
     "uav_vision builds into &lt;repo&gt;/install, not &lt;repo&gt;/ros2/install."],
    ["Parameter has not been declared",
     "The Pi has stale code. rsync then colcon build --packages-select uav_vision."],
], [52 * mm, 113 * mm]))

doc = BaseDocTemplate(OUT, pagesize=A4,
                      leftMargin=22 * mm, rightMargin=22 * mm,
                      topMargin=18 * mm, bottomMargin=20 * mm,
                      title="HITL Runbook", author="multi_uav_simulation")
doc.addPageTemplates([PageTemplate(
    id="main",
    frames=[Frame(doc.leftMargin, doc.bottomMargin,
                  doc.width, doc.height, id="f")],
    onPage=footer)])
doc.build(story)
print(f"wrote {OUT}")
