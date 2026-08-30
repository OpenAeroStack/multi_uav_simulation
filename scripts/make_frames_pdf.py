#!/usr/bin/env python3
"""Generate docs/COORDINATE_FRAMES.pdf — why waypoints landed 90 degrees off.

Figures come from scripts/make_frames_figures.py; run that first if they have
changed:

    python3 scripts/make_frames_figures.py
    python3 scripts/make_frames_pdf.py
"""
import os

from PIL import Image as PILImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (BaseDocTemplate, Frame, Image, PageBreak,
                                PageTemplate, Paragraph, Spacer, Table,
                                TableStyle)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIGS = os.path.join(ROOT, "docs", "figures")
OUT  = os.path.join(ROOT, "docs", "COORDINATE_FRAMES.pdf")

INK, INK2 = colors.HexColor("#16202c"), colors.HexColor("#4a5769")
ACCENT, RULE = colors.HexColor("#1f5f8b"), colors.HexColor("#ccd6e0")
CODE_BG = colors.HexColor("#f2f5f8")
WARN_BG, WARN = colors.HexColor("#fdf0ec"), colors.HexColor("#a8402b")
OK_BG, OK = colors.HexColor("#eaf4ef"), colors.HexColor("#1c6b4d")

ss = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("t", parent=ss["Title"], fontName="Helvetica-Bold",
                            fontSize=18, leading=22, textColor=INK,
                            alignment=TA_LEFT, spaceAfter=2),
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
    """Place a PNG scaled to width_mm, keeping its aspect ratio.

    The size must be given to the constructor. Setting drawWidth afterwards is
    ignored, and the image renders at its native pixel size -- which for a
    160 dpi figure is far wider than the text column.
    """
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
                      "Coordinate Frames  ·  multi_uav_simulation  ·  "
                      "Gazebo, ArduPilot and the 90-degree error")
    canvas.drawRightString(188 * mm, 10.5 * mm, f"page {doc.page}")
    canvas.restoreState()


story, A, E = [], None, None
story = []
A = story.append
E = story.extend

# ════════════════════════════ PAGE 1 ════════════════════════════
A(para("Coordinate Frames — and how a waypoint went 90° wrong", "title"))
A(para("Gazebo, ArduPilot and ROS each describe the world differently. Reading a "
       "position out of one and giving it to another without converting is the "
       "single easiest way to send a drone somewhere it was never meant to go.", "sub"))

A(para("1.  Three frames, three conventions", "h1"))
A(para("A <b>frame</b> is just an agreement about which way x, y and z point. "
       "Nothing is more correct than anything else — but two programs must agree, "
       "or the numbers mean different things.", "body"))

E(table([
    ["Used by", "Convention", "x", "y", "z"],
    ["ROS 2, Gazebo (default)", "<b>ENU</b>", "East", "North", "Up"],
    ["ArduPilot, PX4, aviation", "<b>NED</b>", "North", "East", "Down"],
    ["This project's world", "see below", "North", "West", "Up"],
], [42 * mm, 32 * mm, 30 * mm, 30 * mm, 30 * mm]))

A(para("ENU is the ROS standard, fixed by <b>REP-103</b>. NED is the aviation "
       "standard. Both are everywhere; neither is going away.", "body"))

E(callout("The trap: <b>both use the letters x and y</b>. A file saying "
          "<i>x = 40.5</i> tells you nothing until you know which convention it "
          "follows. That single ambiguity caused every failed flight in this "
          "project until 2026-08-30.", "warn"))

E(figure("frames_two.png", 122,
         "Figure 1 — the world file's axes (left) and what the autopilot actually "
         "receives (right). The link between them is set by one line of the model SDF."))

A(para("2.  What connects them", "h1"))
A(para("The ArduPilot Gazebo plugin is told how to convert, in the model SDF:", "body"))
E(code("""
  <plugin name="arducopter_plugin" filename="libArduPilotPlugin.so">
    <gazeboXYZToNED>0 0 0 3.141593 0 0</gazeboXYZToNED>
"""))
A(para("Those six numbers are x y z roll pitch yaw. Here it is a roll of "
       "&#960; radians — 180° about the x axis — which flips the sign of y and z:", "body"))
E(code("""
  north =  gazebo_x
  east  = -gazebo_y
  down  = -gazebo_z
"""))
A(para("So in this project Gazebo's x is <b>north</b> and its y is <b>west</b>, "
       "not the ENU default. That is a property of this world and this plugin "
       "setting, not of Gazebo in general — which is exactly why it must be "
       "measured rather than assumed.", "body"))

A(PageBreak())

# ════════════════════════════ PAGE 2 ════════════════════════════
A(para("3.  The error, in one picture", "h1"))
A(para("The mission needed to fly from the drone's spawn point to five people "
       "standing in a city square. Both positions were read from the same "
       "<i>.world</i> file:", "body"))

E(code("""
  <model name="iris_1_demo">    <pose>-70   -22    0  0 0 0</pose>
  <model name="person_center">  <pose> 40.5 -187.9 0  0 0 1.57</pose>
"""))

A(para("Subtracting gives +110.5 and −165.9. The mistake was assuming those were "
       "<i>east</i> and <i>north</i>. They are <i>north</i> and <i>west</i>, so the "
       "correct autopilot offsets are +110.5 north and +165.9 east — the two "
       "numbers swapped, and one sign flipped.", "body"))

E(figure("frames_error.png", 88,
         "Figure 2 — same distance, wrong direction. The drone flew 199 m on a "
         "bearing of 146° instead of 56°, ending 218 m from anybody. Every "
         "detection run before the fix searched empty ground."))

E(callout("The failure was silent. The drone armed, climbed, flew a sensible "
          "distance, held station and returned home. Every log line said success. "
          "Only the detector's <i>0 humans</i>, run after run, showed anything was "
          "wrong — and that looked like a camera problem, not a navigation one.", "warn"))

A(PageBreak())

# ════════════════════════════ PAGE 3 ════════════════════════════
A(para("4.  World frame or drone frame? Both — for different things", "h1"))
A(para("This simulation uses <b>both</b>, and knowing which applies where is what "
       "makes the geometry predictable.", "body"))

E(table([
    ["What", "Frame", "Why"],
    ["<b>goto</b> waypoints", "WORLD (global lat/lon)",
     "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT — an absolute place on Earth. "
     "Heading has no effect on where the drone ends up."],
    ["<b>GPS telemetry</b>", "WORLD (global lat/lon)",
     "what /uavN/gps reports"],
    ["<b>ns-3 node positions</b>", "WORLD (Gazebo metres)",
     "/uav_world_positions feeds the channel model in Gazebo coordinates"],
    ["<b>Camera field of view</b>", "BODY (moves with the drone)",
     "the sensor is bolted to the airframe, so what it sees depends on where the "
     "aircraft is pointing"],
    ["<b>FDM / IMU</b>", "BODY", "accelerations and rates are felt in the airframe"],
], [34 * mm, 42 * mm, 89 * mm]))

E(figure("frames_world_body.png", 128,
         "Figure 3 — a waypoint is a fixed point on the ground; the camera's view "
         "is a cone attached to the aircraft. Turn the drone and the waypoint does "
         "not move, but everything the camera sees does."))

A(para("5.  Why the camera never sees straight down", "h1"))
A(para("The camera is pitched 45° <b>forward</b>-down, so at altitude A it sees "
       "the ground roughly 0.7A to 1.5A metres ahead. Directly beneath the "
       "aircraft is a blind spot. A drone hovering exactly over its subjects "
       "cannot see them.", "body"))

E(table([
    ["Altitude", "Ground band ahead", "Stand off by"],
    ["25 m", "17 – 36 m", "25 m"],
    ["15 m", "10 – 22 m", "15 m"],
    ["10 m", "7 – 15 m", "10 m"],
], [40 * mm, 60 * mm, 65 * mm]))

A(para("Keep the stand-off roughly equal to the altitude. The 45° pitch is "
       "deliberate: a person's pixel height scales as 0.5&#183;sin(2&#183;pitch), which "
       "peaks exactly there; at nadir a standing person has no vertical extent.", "body"))

A(PageBreak())

# ════════════════════════════ PAGE 4 ════════════════════════════
A(para("6.  How to get it right: ask, don't derive", "h1"))
A(para("The mapping was eventually found not by reasoning about conventions but "
       "by putting the vehicle at a known place and reading back what it thought "
       "its position was.", "body"))

E(figure("frames_verify.png", 130,
         "Figure 4 — four steps, no arithmetic. The vehicle is the authority on "
         "its own frame."))

E(code("""
  # 1. the world file says where it spawns
  grep -A1 'iris_1_demo' worlds/*.world          ->  pose -70 -22 0

  # 2. ask the autopilot where it thinks it is
  ros2 topic echo /uav1/gps --once               ->  6.078440, 80.191727

  # 3. compare with SITL --home                      6.079068, 80.191528
  #    difference: 70 m SOUTH, 22 m EAST

  # 4. therefore   north = gazebo_x    east = -gazebo_y
"""))

E(callout("A derivation can be self-consistent and still wrong. An earlier check "
          "using the world's &lt;spherical_coordinates&gt; agreed to 0.14 m — but it "
          "converted both sides the same incorrect way, so it confirmed nothing. "
          "One measurement from the vehicle settled it immediately.", "warn"))

A(para("7.  Where to read more", "h1"))

E(table([
    ["Source", "What it gives you"],
    ["<b>REP-103</b> — Standard Units and Coordinate Conventions<br/>"
     "<font size=7.5 color='#4a5769'>ros.org/reps/rep-0103.html</font>",
     "The ROS rule: ENU for the world, x-forward/y-left/z-up for a body. "
     "The document everything ROS is measured against."],
    ["<b>REP-105</b> — Coordinate Frames for Mobile Platforms<br/>"
     "<font size=7.5 color='#4a5769'>ros.org/reps/rep-0105.html</font>",
     "map, odom and base_link, and how they relate."],
    ["<b>ArduPilot — SITL with Gazebo</b><br/>"
     "<font size=7.5 color='#4a5769'>ardupilot.org/dev/docs/sitl-with-gazebo.html</font>",
     "The plugin, its SDF parameters, and the frame conversion it performs."],
    ["<b>MAVLink — MAV_FRAME</b><br/>"
     "<font size=7.5 color='#4a5769'>mavlink.io/en/messages/common.html#MAV_FRAME</font>",
     "Every frame a command can be expressed in. GLOBAL_RELATIVE_ALT_INT is the "
     "one this project's goto uses."],
    ["<b>Gazebo Classic — model SDF</b><br/>"
     "<font size=7.5 color='#4a5769'>sdformat.org/spec</font>",
     "What a &lt;pose&gt; means: six numbers, x y z roll pitch yaw, relative to the "
     "parent frame."],
], [64 * mm, 101 * mm]))

E(callout("Rules of thumb worth keeping: (1) a pose is meaningless without its "
          "frame; (2) prefer relative offsets over absolute coordinates, because "
          "a frame error cancels; (3) when two systems disagree, believe the one "
          "that is flying.", "ok"))

doc = BaseDocTemplate(OUT, pagesize=A4,
                      leftMargin=22 * mm, rightMargin=22 * mm,
                      topMargin=18 * mm, bottomMargin=20 * mm,
                      title="Coordinate Frames — multi_uav_simulation",
                      author="anton")
doc.addPageTemplates([PageTemplate(
    id="p", frames=[Frame(doc.leftMargin, doc.bottomMargin,
                          doc.width, doc.height, id="f")], onPage=footer)])
doc.build(story)
print(f"wrote {OUT}")
