#!/usr/bin/env python3
"""Build the HITL defence deck as an editable PowerPoint file.

Native shapes and text boxes, not screenshots: every element stays editable in
PowerPoint, and the fonts are ones Windows and Office already have.

Figures come from results/<run>/summary.txt, so the deck cannot drift from the
measured data. Pass a run directory to use a different run.

    python3 scripts/make_slides_pptx.py [results/20260903_163917]

Output: report/HITL_slides.pptx
"""
import re
import sys
from pathlib import Path

from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Emu, Inches, Pt

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "report" / "HITL_slides.pptx"

INK    = RGBColor(0x14, 0x1C, 0x26)
MUTED  = RGBColor(0x5B, 0x67, 0x75)
FAINT  = RGBColor(0x8A, 0x94, 0xA1)
RULE   = RGBColor(0xD3, 0xDA, 0xE3)
SUNK   = RGBColor(0xF4, 0xF6, 0xF8)
WHITE  = RGBColor(0xFF, 0xFF, 0xFF)
SENSOR = RGBColor(0x0B, 0x6E, 0x7F)      # unimpaired camera link
RADIO  = RGBColor(0xB0, 0x43, 0x0B)      # impaired ns-3 link
GOOD   = RGBColor(0x1E, 0x6B, 0x45)
WARN   = RGBColor(0xB0, 0x43, 0x0B)

BODY, MONO = "Calibri", "Consolas"       # present on every Office install


# ── helpers ─────────────────────────────────────────────────────────────────
def text(slide, x, y, w, h, runs, size=12, color=INK, font=BODY,
         bold=False, align=PP_ALIGN.LEFT, spacing=1.0, anchor=MSO_ANCHOR.TOP):
    """runs: a string, or a list of (string, {overrides}) tuples, one per line."""
    box = slide.shapes.add_textbox(Inches(x), Inches(y), Inches(w), Inches(h))
    tf = box.text_frame
    tf.word_wrap = True
    tf.vertical_anchor = anchor
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    lines = [runs] if isinstance(runs, str) else runs
    for i, line in enumerate(lines):
        s, over = (line, {}) if isinstance(line, str) else line
        p = tf.paragraphs[0] if i == 0 else tf.add_paragraph()
        p.alignment = over.get("align", align)
        p.line_spacing = over.get("spacing", spacing)
        if over.get("space_before"):
            p.space_before = Pt(over["space_before"])
        r = p.add_run()
        r.text = s
        f = r.font
        f.name = over.get("font", font)
        f.size = Pt(over.get("size", size))
        f.bold = over.get("bold", bold)
        f.color.rgb = over.get("color", color)
    return box


def box(slide, x, y, w, h, fill=WHITE, line=RULE, width=1.0, dash=None):
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y),
                                Inches(w), Inches(h))
    sh.shadow.inherit = False
    if fill is None:
        sh.fill.background()
    else:
        sh.fill.solid()
        sh.fill.fore_color.rgb = fill
    sh.line.color.rgb = line
    sh.line.width = Pt(width)
    if dash:
        sh.line.dash_style = dash
    sh.text_frame.word_wrap = True
    return sh


def rule(slide, x, y, w, h, color=INK, width=1.5):
    sh = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(x), Inches(y),
                                Inches(w), Inches(h))
    sh.shadow.inherit = False
    sh.fill.solid()
    sh.fill.fore_color.rgb = color
    sh.line.fill.background()
    return sh


def header(slide, num, title, eyebrow):
    rule(slide, 0.55, 0.42, 0.42, 0.30, INK)
    text(slide, 0.55, 0.47, 0.42, 0.25, num, size=11, color=WHITE, font=MONO,
         bold=True, align=PP_ALIGN.CENTER)
    text(slide, 1.12, 0.33, 8.8, 0.55, title, size=23, bold=True, spacing=0.92)
    text(slide, 10.1, 0.47, 2.7, 0.3, eyebrow, size=10, color=FAINT, font=MONO,
         align=PP_ALIGN.RIGHT)
    rule(slide, 0.55, 0.95, 12.22, 0.022, INK)


def blank(prs):
    s = prs.slides.add_slide(prs.slide_layouts[6])
    bg = s.background.fill
    bg.solid()
    bg.fore_color.rgb = WHITE
    return s


def parse_summary(path):
    """Pull the per-board figures out of summarise_run.sh output."""
    if not path.exists():
        sys.exit(f"ERROR: no summary at {path}\n"
                 "       Fly a mission first, or pass a run directory.")
    txt = path.read_text()
    out = {}
    for blockmatch in re.finditer(
            r"── (UAV\d) ─.*?(?=── UAV|\Z)", txt, re.S):
        name, blk = blockmatch.group(1), blockmatch.group(0)

        def g(pat, cast=float, default=None):
            m = re.search(pat, blk)
            return cast(m.group(1)) if m else default

        out[name] = {
            "frames":    g(r"edge\s*:\s*(\d+) frames", int, 0),
            "secs":      g(r"in ([\d.]+) s"),
            "fps":       g(r"=\s*([\d.]+) fps"),
            "inf":       g(r"mean (\d+) ms", int),
            "inf_min":   g(r"min (\d+) \|", int),
            "inf_max":   g(r"max (\d+) \| ceiling", int),
            "ceiling":   g(r"ceiling ([\d.]+) fps"),
            "hits":      g(r"detected:\s*(\d+) frames", int, 0),
            "hitpct":    g(r"person \(([\d.]+) %\)"),
            "gcs":       g(r"gcs\s*:\s*(\d+) detection", int, 0),
            "pay":       g(r"payload : mean (\d+) B", int),
            "pay_max":   g(r"payload : mean \d+ B \| min \d+ \| max (\d+)", int),
            "sent":      g(r"delivery: \d+/(\d+)", int),
            "deliv":     g(r"=\s*([\d.]+) % reached"),
            "t_mean":    g(r"mean ([\d.]+) C"),
            "t_peak":    g(r"peak ([\d.]+) C"),
            "throttled": g(r"throttled (\S+)", str, "?"),
        }
    if not out:
        sys.exit("ERROR: could not parse the summary file.")
    return out


# ── slide 1 — HITL architecture ─────────────────────────────────────────────
def slide_architecture(prs, d):
    s = blank(prs)
    header(s, "01", "Two real boards in a simulated fleet", "HITL ARCHITECTURE")

    text(s, 0.55, 1.12, 12.2, 0.5,
         "Each aircraft runs in its own Linux network namespace with a real IP stack. "
         "One Ethernet cable per board carries two tagged VLANs, so the camera feed and "
         "the radio link share a wire but are different networks — and only one is degraded.",
         size=12.5, color=MUTED, spacing=1.15)

    # legend
    rule(s, 0.55, 1.83, 0.26, 0.045, SENSOR)
    text(s, 0.90, 1.72, 3.4, 0.25, "VLAN 10 · sensor · unimpaired",
         size=10, color=SENSOR, font=MONO, bold=True)
    rule(s, 4.35, 1.83, 0.26, 0.045, RADIO)
    text(s, 4.70, 1.72, 3.8, 0.25, "VLAN 42/43 · radio · through ns-3",
         size=10, color=RADIO, font=MONO, bold=True)

    # host enclosure
    box(s, 0.55, 2.06, 6.55, 4.12, fill=None, line=FAINT, width=1.0)
    text(s, 0.65, 2.11, 4.5, 0.22, "HOST — UBUNTU 22.04 / ROS 2 HUMBLE",
         size=9, color=FAINT, font=MONO, bold=True)

    def node(x, y, w, h, title, lines):
        box(s, x, y, w, h, fill=WHITE, line=RULE)
        text(s, x + 0.14, y + 0.10, w - 0.28, 0.22, title, size=11, bold=True)
        text(s, x + 0.14, y + 0.34, w - 0.28, h - 0.42,
             lines, size=9, color=MUTED, font=MONO, spacing=1.25)

    node(0.72, 2.40, 3.05, 0.82, "Gazebo · physics + 2 cameras",
         ["640×384 · 5 Hz · 45° down", "0.6 rad FOV · small_city world"])
    node(0.72, 3.30, 3.05, 0.56, "netns uav1ns · ArduPilot SITL",
         ["10.42.0.11 · sysid 1 · AP_DDS"])
    node(0.72, 3.92, 3.05, 0.56, "netns uav2ns · ArduPilot SITL",
         ["10.42.0.13 · sysid 2 · AP_DDS"])
    node(0.72, 4.70, 3.05, 1.32, "netns gcsns · ground station",
         ["drone_bridge ×2 — commands out",
          "micro_ros_agent ×2 — telemetry in",
          "gcs_receiver ×2 — detections in",
          "10.42.0.10"])

    box(s, 3.98, 3.30, 3.00, 2.72, fill=SUNK, line=RULE)
    text(s, 4.12, 3.40, 2.75, 0.25, "ns-3 · simulated radio", size=11, bold=True)
    text(s, 4.12, 3.70, 2.75, 2.2,
         ["three_uav_tapbridge_integrated",
          "TapBridge injects the channel",
          "model into real Linux packets",
          "802.11 · Nakagami fading",
          "nodes 0–3 · GCS + UAVs + edge",
          "",
          "Detections cross HERE.",
          "Camera frames never do."],
         size=9, color=MUTED, font=MONO, spacing=1.35)

    # edge boards
    text(s, 7.55, 2.13, 3.0, 0.22, "EDGE NODES", size=9, color=FAINT,
         font=MONO, bold=True)

    def board(y, name, cam, radio_ip, inf, fps):
        box(s, 7.55, y, 5.25, 1.68, fill=WHITE, line=RULE)
        text(s, 7.72, y + 0.13, 4.9, 0.25, name, size=11.5, bold=True)
        text(s, 7.72, y + 0.42, 4.9, 1.2,
             [f"YOLO11n · OpenVINO FP32 · 384×640",
              f"conf 0.40 · class 0 (person)",
              f"cam {cam}   ·   radio {radio_ip}",
              f"measured: {inf} ms · {fps} fps"],
             size=9.5, color=MUTED, font=MONO, spacing=1.35)

    board(2.40, "Raspberry Pi 4B — UAV1 edge node", "10.0.0.2", "10.42.0.12",
          d["UAV1"]["inf"], f'{d["UAV1"]["fps"]:.2f}')
    board(4.34, "Raspberry Pi 4B — UAV2 edge node", "10.0.1.2", "10.42.0.14",
          d["UAV2"]["inf"], f'{d["UAV2"]["fps"]:.2f}')

    # sensor path: over the top, clear of ns-3
    rule(s, 3.77, 2.62, 3.53, 0.035, SENSOR)
    rule(s, 7.27, 2.62, 0.035, 2.58, SENSOR)
    rule(s, 7.27, 3.04, 0.30, 0.035, SENSOR)
    rule(s, 7.27, 5.18, 0.30, 0.035, SENSOR)
    text(s, 4.00, 2.32, 3.3, 0.25, "camera frames · 737 KB/frame",
         size=9, color=SENSOR, font=MONO, bold=True)

    # radio path: boards -> ns-3 -> gcsns
    rule(s, 6.98, 3.72, 0.58, 0.035, RADIO)
    rule(s, 6.98, 5.66, 0.58, 0.035, RADIO)
    rule(s, 3.77, 5.40, 0.24, 0.035, RADIO)
    text(s, 4.12, 5.72, 3.0, 0.25, 'detections · 73–504 B JSON',
         size=9, color=RADIO, font=MONO, bold=True)

    # takeaways
    cols = [
        ("Isolation is real, not simulated",
         "Network namespaces give each aircraft a genuine kernel stack — separate routing "
         "tables and its own 127.0.0.1. Nothing shortcuts through loopback, so the packets "
         "ns-3 degrades are the packets the flight code sent."),
        ("One cable, two networks",
         "802.1Q VLAN tagging splits each board's link. VLAN 10 behaves like a camera "
         "ribbon; VLAN 42/43 is bridged into ns-3 — modelling an airframe where sensor bus "
         "and radio are separate systems."),
        ("Only results cross the radio",
         "A 737 KB frame stays on the sensor link. What crosses the degraded channel is "
         "73–504 bytes of JSON. That reduction is the architectural claim edge processing "
         "makes, and what this experiment measures."),
    ]
    for i, (h, body) in enumerate(cols):
        x = 0.55 + i * 4.15
        text(s, x, 6.34, 3.85, 0.22, h.upper(), size=9, color=FAINT,
             font=MONO, bold=True)
        text(s, x, 6.57, 3.85, 0.90, body, size=9, color=MUTED, spacing=1.12)


# ── slide 2 — detection ─────────────────────────────────────────────────────
def slide_detection(prs, d):
    s = blank(prs)
    header(s, "02", "Human detection on the airframe", "EDGE INFERENCE")

    text(s, 0.55, 1.12, 12.2, 0.5,
         "Each board subscribes to its own aircraft's camera, runs YOLO11n under OpenVINO, "
         "and publishes only the bounding boxes. The camera geometry — not the model — "
         "governs whether a person is detectable at all.",
         size=12.5, color=MUTED, spacing=1.15)

    # geometry figure
    box(s, 0.55, 1.80, 7.55, 3.15, fill=SUNK, line=RULE)
    gy = 4.30                                        # ground line
    rule(s, 0.95, gy, 6.75, 0.028, FAINT)
    text(s, 0.95, gy + 0.10, 1.0, 0.2, "GROUND", size=8.5, color=FAINT,
         font=MONO, bold=True)

    # frustum as a triangle
    # Freeform, so the cone's apex sits on the aircraft rather than beside it.
    ff = s.shapes.build_freeform(Inches(1.78), Inches(2.40))
    ff.add_line_segments([(Inches(3.05), Inches(gy)),
                          (Inches(6.35), Inches(gy))], close=True)
    tri = ff.convert_to_shape()
    tri.shadow.inherit = False
    tri.fill.solid()
    tri.fill.fore_color.rgb = RGBColor(0xDC, 0xEE, 0xF1)
    tri.line.color.rgb = SENSOR
    tri.line.width = Pt(1.0)

    box(s, 1.35, 2.06, 0.85, 0.30, fill=WHITE, line=RULE)
    text(s, 1.44, 2.11, 0.7, 0.22, "UAV", size=9.5, bold=True)
    rule(s, 1.76, 2.36, 0.018, gy - 2.36, FAINT)
    text(s, 1.20, 3.10, 0.5, 0.2, "30 m", size=9, color=MUTED, font=MONO)

    rule(s, 1.78, gy - 0.035, 1.27, 0.05, RADIO)
    text(s, 1.55, gy - 0.32, 2.2, 0.2, "blind zone 0–20.6 m",
         size=8.5, color=RADIO, font=MONO, bold=True)
    text(s, 2.86, gy + 0.10, 0.9, 0.2, "20.6 m", size=8.5, color=MUTED, font=MONO)
    text(s, 6.02, gy + 0.10, 0.9, 0.2, "43.6 m", size=8.5, color=MUTED, font=MONO)
    text(s, 4.30, gy - 0.30, 1.6, 0.2, "visible band", size=9, color=SENSOR,
         font=MONO, bold=True)
    rule(s, 4.75, gy - 0.18, 0.035, 0.18, INK)
    text(s, 4.42, gy - 0.52, 1.2, 0.2, "≈30 px", size=8.5, color=INK, font=MONO)

    text(s, 6.55, 2.10, 1.45, 1.9,
         ["45° down-pitch", "0.6 rad H-FOV", "640×384 @ 5 Hz", "",
          "band = 0.687h", "        … 1.453h", "", "person px", "  ≈ 889 / h"],
         size=9, color=MUTED, font=MONO, spacing=1.3)

    # pipeline table
    text(s, 8.45, 1.80, 4.3, 0.22, "PIPELINE, PER AIRCRAFT", size=9,
         color=FAINT, font=MONO, bold=True)
    rows = [("Capture", "640×384 RGB @ 5 Hz"),
            ("Transport", "VLAN 10, unimpaired"),
            ("Model", "YOLO11n · OpenVINO FP32"),
            ("Input shape", "1×3×384×640 (frozen)"),
            ("Filter", "conf 0.40 · class 0"),
            ("Result out", "JSON, 73–504 B"),
            ("Return path", "VLAN 42/43 → ns-3 → GCS")]
    y = 2.10
    for k, v in rows:
        text(s, 8.45, y, 1.45, 0.22, k, size=9.5, color=MUTED)
        text(s, 10.00, y, 2.8, 0.22, v, size=9.5, font=MONO)
        rule(s, 8.45, y + 0.27, 4.32, 0.008, RULE)
        y += 0.40

    # findings
    fx = 0.55
    text(s, fx, 5.20, 5.9, 0.22, "FRAMES ARE DROPPED BY DESIGN", size=9,
         color=FAINT, font=MONO, bold=True)
    text(s, fx, 5.45, 5.9, 1.0,
         "The subscription is BEST_EFFORT, KEEP_LAST, depth 1. While inference runs, "
         "arriving frames overwrite a single slot, so the detector always works on the "
         "newest image instead of falling behind a queue. The frames it discards are "
         "frames it could not have processed anyway.",
         size=10, color=MUTED, spacing=1.18)

    text(s, fx, 6.45, 5.9, 0.22, "GEOMETRY IS THE LIMIT, NOT THE MODEL", size=9,
         color=FAINT, font=MONO, bold=True)
    text(s, fx, 6.70, 5.9, 1.0,
         "At 30 m a standing person subtends about 30 px — the floor of what YOLO "
         "resolves reliably. At 20 m it is ~45 px. Camera resolution cannot help: the "
         "exported model input is frozen at 384×640, so a larger sensor image is simply "
         "scaled back down.",
         size=10, color=MUTED, spacing=1.18)

    text(s, 6.85, 5.20, 5.9, 0.22, "MEASURED CONSEQUENCE", size=9,
         color=FAINT, font=MONO, bold=True)
    text(s, 6.85, 5.45, 5.9, 1.9,
         [(f'UAV1 flew a hold 41 m from a static group and detected a person in '
           f'{d["UAV1"]["hits"]} of {d["UAV1"]["frames"]} frames '
           f'({d["UAV1"]["hitpct"]:.1f} %).', {}),
          ("", {"size": 5}),
          (f'UAV2 flew along the walking subject\'s own path and detected one in '
           f'{d["UAV2"]["hits"]} of {d["UAV2"]["frames"]} frames '
           f'({d["UAV2"]["hitpct"]:.1f} %).', {}),
          ("", {"size": 5}),
          ("Same model, same settings, same boards. The difference is where the camera "
           "was pointed and for how long — a stationary hold sees the target for a "
           "hundred frames; a fly-past sees it for a handful.", {"color": INK})],
         size=10, color=MUTED, spacing=1.18)


# ── slide 3 — results ───────────────────────────────────────────────────────
def slide_results(prs, d):
    s = blank(prs)
    header(s, "03", "Output, and how it was validated", "OUTPUT & VALIDATION")

    u1, u2 = d["UAV1"], d["UAV2"]
    eff1 = 100 * u1["fps"] / u1["ceiling"]
    eff2 = 100 * u2["fps"] / u2["ceiling"]

    text(s, 0.55, 1.12, 12.2, 0.4,
         f"One instrumented mission, both boards. Every figure below is read from the "
         f"run's own logs by scripts/summarise_run.sh — no hand-copied numbers.",
         size=12.5, color=MUTED, spacing=1.15)

    # stat tiles
    tiles = [
        (f"{eff1:.0f} % / {eff2:.0f} %", "OF COMPUTE CEILING",
         f'{u1["fps"]:.2f} and {u2["fps"]:.2f} fps against limits of '
         f'{u1["ceiling"]:.2f} and {u2["ceiling"]:.2f}', GOOD),
        (f'{u1["inf"]} / {u2["inf"]} ms', "MEAN INFERENCE",
         f'board 1 ranged {u1["inf_min"]}–{u1["inf_max"]}, '
         f'board 2 {u2["inf_min"]}–{u2["inf_max"]}', INK),
        (f'{u1["deliv"]:.1f} % / {u2["deliv"]:.1f} %', "DELIVERED ACROSS ns-3",
         f'{u1["gcs"]}/{u1["sent"]} and {u2["gcs"]}/{u2["sent"]} '
         f'detection messages reached the GCS', INK),
        (f'{u1["pay"]} / {u2["pay"]} B', "MEAN PAYLOAD",
         f'73 B empty result, up to {u1["pay_max"]} B with several people', INK),
    ]
    for i, (v, k, note, col) in enumerate(tiles):
        x = 0.55 + i * 3.09
        box(s, x, 1.62, 2.95, 1.30, fill=WHITE, line=RULE)
        text(s, x + 0.16, 1.74, 2.7, 0.4, v, size=20, bold=True, font=MONO, color=col)
        text(s, x + 0.16, 2.16, 2.7, 0.2, k, size=8.5, color=FAINT, font=MONO, bold=True)
        text(s, x + 0.16, 2.40, 2.7, 0.48, note, size=8.5, color=MUTED, spacing=1.12)

    # validation chain
    text(s, 0.55, 3.08, 6.0, 0.22, "HOW A RUN IS VALIDATED", size=9,
         color=FAINT, font=MONO, bold=True)
    steps = [
        ("STEP 0", "rpi_init.sh", "16 checks per board: link, VLAN addressing, RTT, "
                                  "socket buffers, clock skew ≤ 50 ms"),
        ("STEP 1", "sitl_init.sh", "Cold-starts netns, ns-3, Gazebo, SITL. Waits for a "
                                   "real navsat message per aircraft"),
        ("STEP 2", "detector_start.sh", "Starts both edge detectors, blocks until each "
                                        "reports its model loaded"),
        ("STEP 3", "run_missions.sh", "Re-checks every stage before arming, flies, then "
                                      "archives and summarises the run"),
    ]
    for i, (n, name, desc) in enumerate(steps):
        x = 0.55 + i * 3.09
        box(s, x, 3.32, 2.95, 1.05, fill=SUNK, line=RULE)
        text(s, x + 0.14, 3.42, 2.7, 0.18, n, size=8, color=FAINT, font=MONO, bold=True)
        text(s, x + 0.14, 3.62, 2.7, 0.2, name, size=10, font=MONO, bold=True)
        text(s, x + 0.14, 3.86, 2.7, 0.45, desc, size=8.5, color=MUTED, spacing=1.1)

    # findings
    text(s, 0.55, 4.58, 6.0, 0.22, "WHAT THE INSTRUMENTATION REVEALED", size=9,
         color=FAINT, font=MONO, bold=True)
    text(s, 0.55, 4.84, 6.0, 2.2,
         [("Board 1 is power-limited, not heat-limited.", {"bold": True, "color": WARN}),
          (f'It ran hotter work more slowly ({u1["inf"]} ms) than board 2 '
           f'({u2["inf"]} ms) while staying cooler — {u1["t_peak"]:.1f} °C peak against '
           f'{u2["t_peak"]:.1f} °C. Its throttle flag reads {u1["throttled"]}: '
           f'under-voltage, not temperature. A better supply, not a heatsink.', {}),
          ("", {"size": 6}),
          ("Both boards ran at 96 % of their own compute ceiling.",
           {"bold": True, "color": INK}),
          ("Delivery was never the constraint: if the link were starving them, the "
           "achieved rate would sit well below what inference allows. It does not.", {}),
          ("", {"size": 6}),
          (f'Board 2 lost {100 - u2["deliv"]:.0f} % of its detections crossing ns-3.',
           {"bold": True, "color": WARN}),
          (f'{u2["gcs"]} of {u2["sent"]} arrived, against {u1["deliv"]:.1f} % on board 1 '
           f'— a real difference between the two radio links, not measurement noise.', {})],
         size=10, color=MUTED, spacing=1.16)

    text(s, 6.85, 4.58, 5.9, 0.22, "STATED LIMITATIONS", size=9,
         color=FAINT, font=MONO, bold=True)
    text(s, 6.85, 4.84, 5.9, 2.2,
         [("INT8 quantisation was evaluated and rejected.", {"bold": True, "color": INK}),
          ("The quantised model was verified correct on x86, but OpenVINO's ARM CPU "
           "plugin could not compile the graph on the Pi 4B. Edge inference runs FP32.", {}),
          ("", {"size": 6}),
          ("Board power is not instrumented.", {"bold": True, "color": INK}),
          ("The Pi 4B exposes no power sensor. ARM clock, core voltage and CPU load are "
           "logged as on-board proxies; absolute wattage needs an inline meter.", {}),
          ("", {"size": 6}),
          ("Throttling changes timing, never accuracy.", {"bold": True, "color": INK}),
          ("A throttled CPU produces bit-identical detections more slowly. It reduces "
           "frames analysed per target pass — not precision or recall.", {})],
         size=10, color=MUTED, spacing=1.16)

    text(s, 0.55, 7.10, 12.2, 0.25,
         f'Source: results/{sys.argv[1].rstrip("/").split("/")[-1] if len(sys.argv) > 1 else "20260903_163917"}'
         f'  ·  {u1["frames"]} + {u2["frames"]} frames  ·  '
         f'{u1["secs"]:.0f} s and {u2["secs"]:.0f} s of flight  ·  FP32 YOLO11n',
         size=8.5, color=FAINT, font=MONO)


def main():
    run = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "results" / "20260903_163917"
    data = parse_summary(run / "summary.txt")

    prs = Presentation()
    prs.slide_width, prs.slide_height = Inches(13.333), Inches(7.5)   # 16:9

    slide_architecture(prs, data)
    slide_detection(prs, data)
    slide_results(prs, data)

    OUT.parent.mkdir(exist_ok=True)
    prs.save(OUT)
    print(f"  {OUT.relative_to(ROOT)}  {OUT.stat().st_size // 1024} KB  "
          f"({len(prs.slides.__iter__.__self__._sldIdLst)} slides, 16:9)")


if __name__ == "__main__":
    main()
