#!/usr/bin/env python3
"""Generate docs/SCALING_GUIDE.pdf — how to add a UAV, a node, and a Pi.

A learning document: what an ns-3 node and a TAP actually are, how ROS 2
reaches across two machines, and the exact checklist for adding one more
drone or one more Raspberry Pi.

Kept in the repo so it can be regenerated when the procedure changes:

    python3 scripts/make_scaling_guide_pdf.py
"""
import os

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (BaseDocTemplate, Frame, PageBreak, PageTemplate,
                                Paragraph, Spacer, Table, TableStyle)

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "docs", "SCALING_GUIDE.pdf")

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
    "code": ParagraphStyle("code", fontName="Courier", fontSize=7.6, leading=10,
                           textColor=INK),
}


def para(t, s="body"):
    return Paragraph(t, S[s])


def code(text):
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
    bg, fg, tag = ((WARN_BG, WARN, "!") if kind == "warn" else (OK_BG, OK, "OK"))
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
                      "Scaling Guide  ·  multi_uav_simulation  ·  "
                      "adding UAVs, ns-3 nodes and Raspberry Pi boards")
    canvas.drawRightString(188 * mm, 10.5 * mm, f"page {doc.page}")
    canvas.restoreState()


story = []
A = story.append
E = story.extend

# ═══════════════════════════════ PAGE 1 ═══════════════════════════════
A(para("Scaling Guide — Adding UAVs, Nodes and Boards", "title"))
A(para("What an ns-3 node and a TAP actually are, how ROS 2 reaches across two "
       "machines, and the exact checklist for adding one more drone or one more "
       "Raspberry Pi.", "sub"))

A(para("1.  The four layers", "h1"))
A(para("Every drone in this testbed exists in four places at once. Adding a UAV "
       "means adding it to all four — miss one and the failure is usually silent.", "body"))

E(table([
    ["Layer", "What it is", "Where it lives"],
    ["Gazebo model", "the airframe, its camera, its FDM ports", "models/iris_N_netns/"],
    ["SITL", "the autopilot — ArduPilot process", "one process per UAV"],
    ["ROS 2", "topics, services, the bridge to the GCS", "micro_ros_agent + drone_bridge"],
    ["ns-3 node", "a radio with a position in 3D space", "the .cc scenario file"],
], [32 * mm, 78 * mm, 55 * mm]))

A(para("2.  Node, TAP, bridge — what they actually are", "h1"))

A(para("<b>Node</b> — inside ns-3, a simulated computer holding a WifiNetDevice "
       "(a simulated 802.11a radio) and a mobility model (its position). All radios "
       "share one channel. When one transmits, ns-3 computes for every other node: "
       "distance, path loss, obstacle shadowing, fading, SNR — then decides whether "
       "the packet decodes. That calculation is the whole point of the simulator, "
       "and it depends entirely on position.", "body"))

A(para("<b>TAP</b> — a virtual network interface on Linux. The kernel treats it like "
       "a real network card, but instead of a wire on the other side, packets are "
       "handed to a userspace program. Here that program is ns-3.", "body"))

A(para("<b>TapBridge</b> — the ns-3 component that ties one TAP to one node's radio. "
       "It is the airlock: a real Ethernet frame goes in, crosses the simulated air "
       "with real loss and delay, and comes out as a real frame on the other side.", "body"))

A(para("<b>Bridge (br-uavN)</b> — a Linux software switch. It joins the TAP to "
       "whatever should sit behind that radio: a network namespace, a physical VLAN, "
       "or both.", "body"))

E(code("""
  THE FULL PATH OF ONE DETECTION

  Pi: detector publishes /detections/uav1
        |  real UDP packet, VLAN 43 tagged
        v
    switch --- host NIC --- eth-rf --- br-uav3 --- tap-uav3       REAL LINUX
                                                       |
                                            TapBridge (the airlock)
                                                       |
    ns-3 node 3 radio  ===>  simulated 802.11a air  ===>  ns-3 node 0 radio
                        distance / buildings / fading / contention
                                                       |
    tap-gcs --- br-gcs --- veth --- gcsns --- gcs_receiver        REAL LINUX

  Everything outside the airlock happens on real hardware.
  Everything inside it is arithmetic.
"""))

A(PageBreak())

# ═══════════════════════════════ PAGE 2 ═══════════════════════════════
A(para("3.  How ROS 2 connects across two machines", "h1"))

A(para("ROS 2 has no central server. Nodes find each other with <b>DDS discovery</b>: "
       "each participant announces itself by multicast, and any node interested in a "
       "matching topic connects directly. Nothing needs configuring for two nodes on "
       "one machine to find each other — which is exactly why it is easy to get wrong "
       "across two.", "body"))

E(code("""
  DISCOVERY                              DATA

  detector (Pi)                          detector (Pi)
      |  "I publish /detections/uav1"        |
      |  multicast announcement              |  unicast, direct
      v                                      v
  gcs_receiver (host)                    gcs_receiver (host)
      "I subscribe to /detections/uav1"

  Discovery is multicast. Once matched, data flows point to point.
"""))

A(para("<b>The interface whitelist.</b> A machine with WiFi and Ethernet has several "
       "addresses, and Fast DDS announces a locator on every one of them. The Pi can "
       "then advertise its WiFi address, the host connects to it, and the camera "
       "stream silently takes the WiFi path — where large samples fragment and drop. "
       "config/fastdds_hitl_eth.xml pins DDS to the wired addresses only.", "body"))

E(callout("Fast DDS 2.6 (ROS 2 Humble) reads the interface list ONCE, at participant "
          "creation, and has no dynamic interface detection. An address that appears "
          "after a process starts can never be used by it. Bring the addresses up "
          "<b>before</b> launching Gazebo.", "warn"))

A(para("4.  The two networks, and why they are separate", "h1"))

A(para("Each drone has two data paths that must never be confused.", "body"))

E(table([
    ["", "Sensor / management", "Wireless"],
    ["Carries", "camera frames, SSH, chrony", "detections, telemetry"],
    ["Subnet", "10.0.N.x  (VLAN 10, 11, 12)", "10.42.0.x  (VLAN 42, 43, 44)"],
    ["Through ns-3", "No — plain cable", "YES — this is what is measured"],
    ["Represents", "the camera ribbon cable inside one airframe", "the radio link"],
    ["Measured", "1.4 ms, 0% loss", "74.5 ms, 38 ms jitter"],
], [24 * mm, 71 * mm, 70 * mm]))

A(para("Degrading the sensor link would be physically meaningless — on a real drone "
       "the camera is bolted to the companion computer. Routing enforces the split "
       "with no firewall rules: Gazebo lives on 10.0.N.1 and can only reach the Pi at "
       "10.0.N.2; gcsns lives on 10.42.0.10 and can only reach it at 10.42.0.1x.", "body"))

E(callout("Give every board its own VLAN ids. Two Pis sharing a VLAN can reach each "
          "other directly through the switch at 1.4 ms, bypassing ns-3 entirely — and "
          "every UAV-to-UAV latency number becomes meaningless with no error anywhere.",
          "warn"))

A(PageBreak())

# ═══════════════════════════════ PAGE 3 ═══════════════════════════════
A(para("5.  Adding one more UAV (software only)", "h1"))

A(para("Every UAV needs its own ports at four levels. The numbers below are the "
       "existing convention — follow it and nothing collides.", "body"))

E(table([
    ["", "UAV 1", "UAV 2", "UAV 3"],
    ["MAVLink TCP", "5760", "5770", "5780"],
    ["FDM in / out", "9002 / 9003", "9012 / 9013", "9022 / 9023"],
    ["DDS UDP port", "2019", "2020", "2021"],
    ["FDM address", "172.31.1.1", "172.31.2.1", "172.31.3.1"],
    ["ns-3 node / tap", "1 / tap-uav1", "2 / tap-uav2", "3 / tap-uav3"],
    ["Radio address", "10.42.0.11", "10.42.0.12", "10.42.0.13"],
], [34 * mm, 43 * mm, 43 * mm, 43 * mm]))

A(para("Step by step", "h2"))

A(para("<b>1. Gazebo model.</b> models/iris_N_netns/ already exists for N = 1,2,3 with "
       "the correct FDM ports and camera topic. Adding a fourth means copying the "
       "directory and changing fdm_port_in, listen_addr and the camera topic name.", "body"))

A(para("<b>2. Add it to the world.</b> The netns world currently includes only "
       "iris_1_netns. A multi-UAV world includes each model with its own pose.", "body"))

A(para("<b>3. DDS parameters.</b> Copy params/uav1_dds_netns.parm and change "
       "DDS_UDP_PORT. It points the autopilot at the micro_ros_agent in gcsns.", "body"))

A(para("<b>4. SITL instance</b> — one process per UAV, each in its own namespace:", "body"))

E(code("""
  sudo ip netns exec uavNns strace -f -e trace=none -o /dev/null \\
       <ardupilot>/build/sitl/bin/arducopter \\
       --model gazebo-iris --sysid N --sim-address=172.31.N.1 -I<N-1> \\
       --defaults params/uavN_dds_netns.parm

  # strace is NOT for logging. Being ptrace-traced suppresses SITL's
  # auto-reboot loop, which otherwise restarts ~25 times in 15 s when DDS
  # is enabled inside a namespace. The reboot preserves the PID, so a
  # liveness check reports "alive" throughout and sees nothing wrong.
"""))

A(para("<b>5. ROS 2 side</b> — one micro_ros_agent and one drone_bridge per UAV, "
       "both inside gcsns:", "body"))

E(code("""
  ros2 run micro_ros_agent micro_ros_agent udp4 --port 2020
  ros2 run uav_controller drone_bridge --ros-args -p uav_id:=2 -p mavlink_port:=5770
"""))

A(para("<b>6. ns-3 node</b> — a TAP and a bridge, created by the launcher:", "body"))

E(code("""
  ip tuntap add dev tap-uavN mode tap user $USER
  ip link add name br-uavN type bridge
  ip link set tap-uavN master br-uavN
  ip link set br-uavN up
  # then attach whatever sits behind that radio:
  #   a namespace  -> veth pair into uavNns
  #   a Pi         -> the physical VLAN leg (eth-rf.4X)
"""))

A(PageBreak())

# ═══════════════════════════════ PAGE 4 ═══════════════════════════════
A(para("6.  Adding one more Raspberry Pi (hardware node)", "h1"))

A(para("A Pi replaces the software namespace with real silicon. The addressing is "
       "the part that must be right — every board gets its own VLAN ids so no two "
       "boards can reach each other except through ns-3.", "body"))

E(table([
    ["Board", "Sensor VLAN", "Pi address", "Host address", "Radio VLAN", "Radio address"],
    ["uav-pi-01", "10", "10.0.0.2", "10.0.0.1", "42", "10.42.0.12"],
    ["uav-pi-02", "11", "10.0.1.2", "10.0.1.1", "43", "10.42.0.13"],
    ["uav-pi-03", "12", "10.0.2.2", "10.0.2.1", "44", "10.42.0.14"],
], [24 * mm, 24 * mm, 26 * mm, 26 * mm, 22 * mm, 28 * mm]))

E(code("""
  ONE CABLE, TWO LOGICAL LINKS, PER BOARD

     HOST                          SWITCH                       PI 4B
     ----                          ------                       -----
   eth-cam2 (VLAN 11) --+                        +-- eth0.11  10.0.1.2
     10.0.1.1           |                        |     ^  camera in
                        +==== one cable =========+     |
   eth-rf.43 (VLAN 43)--+                        +-- eth0.43  10.42.0.13
        |                                                 |  detections out
        v                                                 |
     br-uav3 --- tap-uav3 --- ns-3 node 3  <--------------+

  The switch does not need to understand VLANs. Tagging happens at the
  endpoints; the switch only has to forward tagged frames, which any
  gigabit switch does. Measured through an unmanaged LS1005G:
  1.4 ms, 0% loss, 942 Mbps, 0 retransmits.
"""))

A(para("Host side (GUI)", "h2"))
A(para("Network Connections &rarr; + &rarr; VLAN &rarr; Create. Parent = the USB "
       "adapter <i>by MAC</i>, VLAN id = 11, name = eth-cam2. IPv4 Manual, "
       "10.0.1.1/24, no gateway, and tick <i>Use this connection only for resources "
       "on its network</i>. IPv6 Disabled.", "body"))

A(para("Pi side (netplan)", "h2"))
E(code("""
  # /etc/netplan/60-hitl-vlans.yaml     chmod 600
  network:
    version: 2
    ethernets:
      eth0: {dhcp4: no, optional: true}
    vlans:
      eth0.11: {id: 11, link: eth0, addresses: [10.0.1.2/24]}
      eth0.43: {id: 43, link: eth0, addresses: [10.42.0.13/24]}

  sudo netplan generate     # parses only — this is the syntax check
  sudo netplan try          # applies, auto-reverts after 120 s
"""))

E(callout("Only ONE netplan file may define eth0. Two files giving the same address "
          "to different interfaces both take effect — netplan merges without warning. "
          "Ping still works, multicast silently never arrives, and Fast DDS segfaults "
          "at participant creation with no message at all. Check with: "
          "grep -l eth0 /etc/netplan/*.yaml", "warn"))

A(PageBreak())

# ═══════════════════════════════ PAGE 5 ═══════════════════════════════
A(para("7.  Limits you will hit", "h1"))

E(table([
    ["Limit", "Value", "Why"],
    ["ns-3 nodes", "4 (GCS + 3)", "hardcoded array in the .cc — one TAP name each"],
    ["Machines per node", "1", "TapBridge UseLocal tracks a single MAC address"],
    ["Switch ports", "5 (LS1005G)", "host + 4 boards"],
    ["Camera bandwidth", "29.5 Mbps per Pi", "640x384 RGB at 5 Hz, plus 5% overhead"],
    ["Host uplink", "~940 Mbps", "measured — about 30 Pis before saturation"],
], [40 * mm, 34 * mm, 91 * mm]))

A(para("The node budget is the real constraint. Four nodes means GCS plus three "
       "radios. Giving SITL and its Pi separate nodes spends two on one airframe — "
       "which also models one aircraft as two contending radios, an artifact that does "
       "not exist in reality.", "body"))

A(para("Three ways past it", "h2"))
A(para("<b>a. Route through the namespace.</b> Put the Pi behind uavNns instead of "
       "beside it: uavNns gets a second interface, IP forwarding on, and the Pi uses it "
       "as gateway. One MAC on the channel, so UseLocal is satisfied, and one node "
       "genuinely equals one aircraft. Cost: DDS multicast discovery does not cross a "
       "router, so unicast discovery peers become necessary.", "body"))
A(para("<b>b. Patch AdhocWifiMac::SupportsSendFrom() to return true</b>, then use "
       "TapBridge UseBridge and put both behind one bridge. Mechanically sound — ad-hoc "
       "frames carry the source in addr2 — but it is a simulator modification that has "
       "to be disclosed.", "body"))
A(para("<b>c. Extend the scenario past four nodes.</b> The tapNames array and the "
       "loops around it become N-parameterised. The largest change, and the right one "
       "if the swarm grows.", "body"))

A(para("8.  Verification checklist", "h1"))

E(table([
    ["Check", "Command", "Expected"],
    ["One address, one interface", "ip -4 addr show | grep -c 10.0.1.2/24", "1, never 2"],
    ["Sensor link", "ping -c 30 10.0.N.1", "~1.4 ms, 0% loss"],
    ["Throughput", "iperf3 -c 10.0.N.2 -t 10", "~940 Mbps, 0 retr"],
    ["Clock", "chronyc tracking | grep 'System time'", "a few microseconds"],
    ["Socket buffers", "sysctl net.core.rmem_max net.core.wmem_max", "both 536870912"],
    ["ns-3 sees the node", "grep 'missing node IDs' /tmp/ns3_single.log", "your node absent"],
    ["Node is moving", "grep '^t=' /tmp/ns3_single.log", "coordinates change"],
    ["ISOLATION", "from Pi 1:  ping -c 30 10.42.0.13", "~75 ms, NOT 1.4 ms"],
], [42 * mm, 78 * mm, 45 * mm]))

E(callout("The isolation check is the one that matters most. If two Pis answer each "
          "other in 1.4 ms, they are talking through the switch and skipping ns-3 — "
          "the same class of fault as the shared-memory bypass that made DDS deliver "
          "detections without crossing the simulated radio at all. Nothing errors; "
          "the numbers simply describe a link that was never simulated.", "warn"))

E(callout("Use at least 30 pings on the radio path. The first packet carries an ARP "
          "exchange across the simulated channel, and a 2-ping sample once reported "
          "167 ms for a link that measures ~75 ms over 30.", "ok"))

doc = BaseDocTemplate(OUT, pagesize=A4,
                      leftMargin=22 * mm, rightMargin=22 * mm,
                      topMargin=18 * mm, bottomMargin=20 * mm,
                      title="Scaling Guide — multi_uav_simulation",
                      author="anton")
frame = Frame(doc.leftMargin, doc.bottomMargin,
              doc.width, doc.height, id="f")
doc.addPageTemplates([PageTemplate(id="p", frames=[frame], onPage=footer)])
doc.build(story)
print(f"wrote {OUT}")
