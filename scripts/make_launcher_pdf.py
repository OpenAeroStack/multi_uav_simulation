#!/usr/bin/env python3
"""Generate docs/LAUNCH_2UAV_MECHANISM.pdf — how sitl_init.sh works.

Figures come from scripts/make_launcher_figures.py; run that first if they have
changed:

    python3 scripts/make_launcher_figures.py
    python3 scripts/make_launcher_pdf.py
"""
import os

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.platypus import (BaseDocTemplate, Frame, Image, PageBreak,
                                PageTemplate, Paragraph, Spacer, Table,
                                TableStyle)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIGS = os.path.join(ROOT, "docs", "figures")
OUT  = os.path.join(ROOT, "docs", "LAUNCH_2UAV_MECHANISM.pdf")

INK, INK2 = colors.HexColor("#16202c"), colors.HexColor("#4a5769")
ACCENT, RULE = colors.HexColor("#1f5f8b"), colors.HexColor("#ccd6e0")
CODE_BG = colors.HexColor("#f2f5f8")
WARN_BG, WARN = colors.HexColor("#fdf0ec"), colors.HexColor("#a8402b")
OK_BG, OK = colors.HexColor("#eaf4ef"), colors.HexColor("#1c6b4d")

S = {
    "title": ParagraphStyle("t", fontName="Helvetica-Bold", fontSize=17,
                            leading=20, textColor=INK, alignment=TA_LEFT,
                            spaceAfter=2),
    "sub":  ParagraphStyle("s", fontName="Helvetica", fontSize=9.5, leading=13,
                           textColor=INK2, spaceAfter=9),
    "h1":   ParagraphStyle("h1", fontName="Helvetica-Bold", fontSize=11.5,
                           leading=14, textColor=ACCENT, spaceBefore=8,
                           spaceAfter=3),
    "body": ParagraphStyle("b", fontName="Helvetica", fontSize=8.6, leading=11.6,
                           textColor=INK, spaceAfter=3),
    "cap":  ParagraphStyle("c", fontName="Helvetica-Oblique", fontSize=8.2,
                           leading=11, textColor=INK2, spaceAfter=7),
    "code": ParagraphStyle("cd", fontName="Courier", fontSize=8.1, leading=11,
                           textColor=INK),
}


def para(t, s="body"):
    return Paragraph(t, S[s])


def code(text):
    rows = [[Paragraph(l.replace("&", "&amp;").replace("<", "&lt;")
                       .replace(" ", "&nbsp;") or "&nbsp;", S["code"])]
            for l in text.strip("\n").split("\n")]
    tb = Table(rows, colWidths=[165 * mm])
    tb.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), CODE_BG),
        ("LINEBEFORE", (0, 0), (0, -1), 1.6, ACCENT),
        ("LEFTPADDING", (0, 0), (-1, -1), 7),
        ("TOPPADDING", (0, 0), (-1, -1), 1),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1)]))
    return [Spacer(1, 3), tb, Spacer(1, 5)]


def callout(text, kind="warn"):
    bg, fg, tag = ((WARN_BG, WARN, "!") if kind == "warn" else (OK_BG, OK, "OK"))
    tb = Table([[Paragraph(f"<b>{tag}</b>&nbsp;&nbsp;{text}",
                           ParagraphStyle("x", fontName="Helvetica", fontSize=8.4,
                                          leading=11.5, textColor=fg))]],
               colWidths=[165 * mm])
    tb.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), bg),
        ("LEFTPADDING", (0, 0), (-1, -1), 8),
        ("RIGHTPADDING", (0, 0), (-1, -1), 8),
        ("TOPPADDING", (0, 0), (-1, -1), 5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5)]))
    return [Spacer(1, 2), tb, Spacer(1, 5)]


def table(rows, widths):
    data = [[Paragraph(f"<b>{c}</b>" if i == 0 else c,
                       ParagraphStyle("tt", fontName="Helvetica", fontSize=8.3,
                                      leading=11, textColor=INK))
             for c in row] for i, row in enumerate(rows)]
    tb = Table(data, colWidths=widths)
    tb.setStyle(TableStyle([
        ("LINEBELOW", (0, 0), (-1, 0), 0.9, ACCENT),
        ("LINEBELOW", (0, 1), (-1, -2), 0.3, RULE),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 4),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3)]))
    return [Spacer(1, 3), tb, Spacer(1, 6)]


def figure(name, width_mm, caption):
    """Place a PNG scaled to width_mm, keeping its aspect ratio."""
    path = os.path.join(FIGS, name)
    with PILImage.open(path) as im:
        px_w, px_h = im.size
    w = width_mm * mm
    img = Image(path, width=w, height=w * px_h / px_w)
    img.hAlign = "CENTER"
    return [Spacer(1, 4), img, Spacer(1, 3), para(caption, "cap")]


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(RULE); canvas.setLineWidth(0.5)
    canvas.line(22 * mm, 15 * mm, 188 * mm, 15 * mm)
    canvas.setFont("Helvetica", 7.5); canvas.setFillColor(INK2)
    canvas.drawString(22 * mm, 10.5 * mm,
                      "sitl_init.sh  ·  multi_uav_simulation  ·  "
                      "how the two-UAV HITL pipeline is built")
    canvas.drawRightString(188 * mm, 10.5 * mm, f"page {doc.page}")
    canvas.restoreState()


story = []
A = story.append
def E(xs):
    story.extend(xs)


# ═══════════════════ PAGE 1 — what it builds and why ════════════════════════
A(para("How <font face='Courier'>sitl_init.sh</font> works", "title"))
A(para("One script builds a two-aircraft hardware-in-the-loop testbed: two "
       "simulated drones, a ground station, a simulated radio channel, and a "
       "real Raspberry Pi doing the vision work. This is what it assembles, in "
       "what order, and why the order matters.", "sub"))

A(para("The problem it solves", "h1"))
A(para("A multi-UAV experiment needs four things running at once, and they all "
       "want to talk to each other: <b>Gazebo</b> for physics, <b>ArduPilot "
       "SITL</b> for the autopilot, <b>ns-3</b> for the radio, and <b>ROS 2</b> "
       "to carry commands and telemetry. Left to themselves on one machine they "
       "would simply talk over loopback at full speed — and the network "
       "simulation would be bypassed without anyone noticing."))
A(para("The launcher's job is to make that impossible. It puts each aircraft in "
       "its own <b>Linux network namespace</b>, so the only route between them "
       "runs through ns-3's simulated Wi-Fi. If the plumbing is wrong, nothing "
       "connects at all — which is much safer than silently measuring a perfect "
       "network."))

E(figure("launcher_overview.png", 158,
         "Three isolated worlds. Everything above the dashed line is the real "
         "machine; the three boxes below are private network stacks that can "
         "only reach one another through the simulated channel."))

A(para("Why a network namespace", "h1"))
A(para("A namespace gives a group of processes their own interfaces, routes, "
       "firewall rules, port numbers and loopback. It is the same kernel feature "
       "Docker is built on, used here directly. Two consequences matter:"))
A(para("<b>1. Ports stop colliding.</b> Every ArduPilot SITL wants the same "
       "default ports. In one shared namespace you must hand-allocate them and "
       "hope; in separate namespaces each aircraft reuses identical numbers, "
       "exactly as separate physical drones would."))
A(para("<b>2. Traffic cannot take a shortcut.</b> A fresh namespace has no route "
       "to anything. The only path out is the one the script builds, and that "
       "path goes through ns-3."))

E(callout("A namespace's loopback is private too, which is why "
          "<font face='Courier'>--sim-address</font> must be a real IP and never "
          "127.0.0.1: the two sides live in different namespaces and do not "
          "share a loopback.", "ok"))

A(PageBreak())

# ═══════════════════ PAGE 2 — the fabric ════════════════════════════════════
A(para("The network fabric", "h1"))
A(para("Each namespace is wired to ns-3 through the same four-part chain. A "
       "<b>veth</b> is a virtual cable with two ends; one end is pushed inside "
       "the namespace, the other stays outside. A <b>bridge</b> is a virtual "
       "switch. A <b>TAP</b> is a device ns-3 can attach to and read raw frames "
       "from."))

E(figure("launcher_packet_path.png", 165,
         "Every hop a telemetry packet takes from the autopilot to the ground "
         "station. Nine of the ten are ordinary Linux plumbing; only one applies "
         "the physics being studied."))

A(para("Node numbering", "h1"))
A(para("ns-3 knows nothing about aircraft — it has numbered nodes. The mapping "
       "interleaves aircraft with their companion computers, because on a real "
       "airframe the Pi flies bolted to the autopilot:"))

E(figure("launcher_node_map.png", 150,
         "Node ids are shared by ns-3, the position publisher and the obstacle "
         "model. Getting one wrong makes a link's measurements fiction while "
         "everything still appears to run."))

A(para("The two links to the Raspberry Pi", "h1"))
A(para("The Pi has one Ethernet port but needs two logically separate links, so "
       "the script uses 802.1Q VLAN tagging to split the single cable:"))

E(table([
    ["", "VLAN 10 — camera", "VLAN 42/43 — radio"],
    ["Address", "10.0.0.x", "10.42.0.x"],
    ["Carries", "camera frames <i>into</i> the Pi",
     "detections <i>out</i> of the Pi"],
    ["Path", "direct to Gazebo, never enters a bridge ns-3 can see",
     "br-uav2 &rarr; tap-uav2 &rarr; ns-3 &rarr; tap-gcs &rarr; gcsns"],
    ["Impaired?", "<b>No</b> — it stands in for the ribbon cable inside one "
     "airframe", "<b>Yes</b> — this is the radio under study"],
], [24 * mm, 68 * mm, 73 * mm]))

A(para("The routing then enforces itself: Gazebo can only reach the Pi at "
       "10.0.0.2 and the ground station only at 10.42.0.12, so neither can take "
       "the wrong path even by accident."))

A(PageBreak())

# ═══════════════════ PAGE 3 — the startup ladder ════════════════════════════
A(para("Startup, stage by stage", "h1"))
A(para("Eleven stages run in a fixed order. Three of them are hard ordering "
       "constraints rather than preferences — get those wrong and the run fails "
       "in ways that look like something else entirely."))

E(figure("launcher_stages.png", 138,
         "The launch ladder. Shaded steps must happen when they happen; the "
         "others are merely sensible."))

A(para("The three constraints", "h1"))
A(para("<b>ns-3 before any traffic (stage 3).</b> ns-3 must own the TAP devices "
       "before anything tries to send. The script waits for carrier on all four "
       "TAPs and aborts if ns-3 exits first."))
A(para("<b>Agents before SITL (stage 5).</b> ArduPilot's DDS client dials out to "
       "a fixed address. If nothing is listening, it retries and may give up; "
       "the agents must already be bound."))
A(para("<b>Gates before flying (stage 7).</b> The script waits for a real "
       "<font face='Courier'>navsat</font> message on <i>both</i> aircraft — not "
       "merely a publisher count, which can read healthy while nothing arrives."))

A(para("Every address and port in one place", "h1"))
E(table([
    ["", "UAV1", "UAV2", "carried over"],
    ["Namespace", "uav1ns", "uav2ns", "—"],
    ["Radio address", "10.42.0.11", "10.42.0.13", "ns-3 Wi-Fi"],
    ["MAVLink (commands)", "TCP 5760", "TCP 5770", "ns-3 Wi-Fi"],
    ["DDS to agent (telemetry)", "UDP 2019", "UDP 2020", "ns-3 Wi-Fi"],
    ["Gazebo FDM (physics)", "UDP 9002", "UDP 9012", "direct veth"],
    ["Management link", "172.31.1.1 / .2", "172.31.2.1 / .2", "direct veth"],
    ["Raspberry Pi radio", "10.42.0.12", "10.42.0.14", "ns-3 Wi-Fi"],
    ["Raspberry Pi camera", "10.0.0.2", "10.0.1.2", "VLAN, unimpaired"],
], [42 * mm, 41 * mm, 41 * mm, 41 * mm]))

A(para("The ground station sits at <font face='Courier'>10.42.0.10</font> inside "
       "<font face='Courier'>gcsns</font> and runs both micro-ROS agents, both "
       "<font face='Courier'>drone_bridge</font> nodes and the mission scripts. "
       "Note the pattern in the last three rows: anything to do with <i>flying</i> "
       "avoids ns-3, and anything to do with <i>communicating</i> goes through it."))

A(PageBreak())

# ═══════════════════ PAGE 4 — in flight, and what bites ═════════════════════
A(para("What happens once it is running", "h1"))

E(figure("launcher_dataflow.png", 158,
         "Three conversations at once. Commands and telemetry cross the "
         "simulated channel; flight physics deliberately does not."))

A(para("Reading the output", "h1"))
E(table([
    ["Line you should see", "What it confirms"],
    ["<font face='Courier'>All TAPs attached</font>", "ns-3 owns the devices; the simulated channel exists"],
    ["<font face='Courier'>/ap/v1/navsat is delivering messages</font>",
     "real telemetry, not just a registered publisher"],
    ["<font face='Courier'>Letting AP_DDS settle (15s)</font>",
     "handshake headroom before anything talks to SITL"],
    ["<font face='Courier'>PIPELINE READY</font>", "safe to fly a mission"],
], [72 * mm, 93 * mm]))

A(para("Two things that bite", "h1"))
E(callout("<b>Duplicate node names.</b> Both bridges are the same executable, so "
          "without <font face='Courier'>-r __node:=drone_bridge_uavN</font> both "
          "register as <font face='Courier'>/drone_bridge</font>. Two ROS 2 nodes "
          "sharing a name is undefined behaviour: the one that starts second "
          "displaces the first, so UAV1 silently loses its topics — every run, "
          "never UAV2."))
E(callout("<b>A comment after a line continuation.</b> Bash joins the next line "
          "literally, so a <font face='Courier'>#</font> there comments out the "
          "arguments. It cost a run where the position publisher came up on its "
          "defaults and nobody could see why."))

A(para("Running it", "h1"))
E(code("""
# the full two-board run, with cameras and the Gazebo viewer
./scripts/netns/run_hitl.sh --uavs 2 --view --gui --mission

# host only, no Raspberry Pi attached
./scripts/netns/run_hitl.sh --uavs 2 --no-pi --mission

# tear everything down
sudo bash scripts/netns/kill_all_netns.sh
"""))

A(para("Logs live in <font face='Courier'>/tmp</font>: "
       "<font face='Courier'>ns3_2uav.log</font>, "
       "<font face='Courier'>gazebo_2uav.log</font>, "
       "<font face='Courier'>agent_2uav_uavN.log</font>, "
       "<font face='Courier'>bridge_2uav_uavN.log</font>, "
       "<font face='Courier'>hitl_mission_uavN.log</font>. When a mission "
       "misbehaves, read them in that order — it follows the direction the data "
       "flows."))

doc = BaseDocTemplate(OUT, pagesize=A4,
                      leftMargin=22 * mm, rightMargin=22 * mm,
                      topMargin=18 * mm, bottomMargin=20 * mm,
                      title="sitl_init.sh — mechanism",
                      author="anton")
doc.addPageTemplates([PageTemplate(
    id="p", frames=[Frame(doc.leftMargin, doc.bottomMargin,
                          doc.width, doc.height, id="f")], onPage=footer)])
doc.build(story)
print(f"wrote {OUT}")
