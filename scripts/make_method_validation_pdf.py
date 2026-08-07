#!/usr/bin/env python3
"""Generate docs/HITL_METHOD_VALIDATION.pdf — literature check on the HITL
edge-processing approach, with citable sources.

    python3 scripts/make_method_validation_pdf.py
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
                   "docs", "HITL_METHOD_VALIDATION.pdf")

INK     = colors.HexColor("#16202c")
INK2    = colors.HexColor("#4a5769")
ACCENT  = colors.HexColor("#1f5f8b")
RULE    = colors.HexColor("#ccd6e0")
QUOTE_BG = colors.HexColor("#f2f5f8")
OK_BG   = colors.HexColor("#eaf4ef")
OK      = colors.HexColor("#1c6b4d")
LINK    = colors.HexColor("#1a5c8a")

ss = getSampleStyleSheet()
S = {
    "title": ParagraphStyle("title", parent=ss["Title"], fontName="Helvetica-Bold",
                            fontSize=18, leading=22, textColor=INK,
                            alignment=TA_LEFT, spaceAfter=2),
    "sub": ParagraphStyle("sub", fontName="Helvetica", fontSize=9.5, leading=13,
                          textColor=INK2, spaceAfter=11),
    "h1": ParagraphStyle("h1", fontName="Helvetica-Bold", fontSize=12.5, leading=16,
                         textColor=ACCENT, spaceBefore=13, spaceAfter=5),
    "body": ParagraphStyle("body", fontName="Helvetica", fontSize=9.2, leading=13,
                           textColor=INK, spaceAfter=5),
    "note": ParagraphStyle("note", fontName="Helvetica", fontSize=8.4, leading=11.5,
                           textColor=INK2, spaceAfter=4),
    "src": ParagraphStyle("src", fontName="Helvetica", fontSize=8, leading=11.5,
                          textColor=INK, spaceAfter=4, leftIndent=13,
                          firstLineIndent=-13),
}


def para(t, s="body"):
    return Paragraph(t, S[s])


def quote(text, cite):
    t = Table([[Paragraph(f'<i>"{text}"</i> &nbsp;&mdash;&nbsp; {cite}',
                          ParagraphStyle("q", fontName="Helvetica", fontSize=8.6,
                                         leading=12, textColor=INK))]],
              colWidths=[165 * mm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), QUOTE_BG),
        ("LINEBEFORE", (0, 0), (0, -1), 1.8, ACCENT),
        ("LEFTPADDING", (0, 0), (-1, -1), 9),
        ("RIGHTPADDING", (0, 0), (-1, -1), 9),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
    ]))
    return [Spacer(1, 3), t, Spacer(1, 6)]


def callout(text):
    t = Table([[Paragraph(text, ParagraphStyle(
        "c", fontName="Helvetica", fontSize=8.6, leading=12, textColor=OK))]],
        colWidths=[165 * mm])
    t.setStyle(TableStyle([
        ("BACKGROUND", (0, 0), (-1, -1), OK_BG),
        ("LEFTPADDING", (0, 0), (-1, -1), 9),
        ("RIGHTPADDING", (0, 0), (-1, -1), 9),
        ("TOPPADDING", (0, 0), (-1, -1), 6),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
    ]))
    return [Spacer(1, 3), t, Spacer(1, 6)]


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
    return [Spacer(1, 3), t, Spacer(1, 7)]


def footer(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(RULE)
    canvas.setLineWidth(0.5)
    canvas.line(22 * mm, 15 * mm, 188 * mm, 15 * mm)
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(INK2)
    canvas.drawString(22 * mm, 10.5 * mm,
                      "HITL Method Validation  ·  multi_uav_simulation")
    canvas.drawRightString(188 * mm, 10.5 * mm, f"page {doc.page}")
    canvas.restoreState()


story = []
A, E = story.append, story.extend

A(para("Method Validation: HITL Edge Processing", "title"))
A(para("Literature check on the testbed design, with citable sources. "
       "Searched August 2026.", "sub"))

# ── verdict ──
A(para("Verdict", "h1"))
A(para("The approach follows established practice. ns-3 TapBridge is the "
       "documented mechanism for attaching physical hardware to an emulated "
       "network, and three published systems perform the same ns-3 + Gazebo + "
       "ROS co-simulation.", "body"))
E(quote("TapBridge allows a real host to participate in an ns-3 simulation as "
        "if it were one of the simulated nodes", "ns-3 Model Library [1]"))
A(para("This is exactly the mechanism by which the Raspberry Pi occupies "
       "aircraft node 2 of the scenario.", "note"))

# ── prior work ──
A(para("Prior work worth citing", "h1"))
E(table([
    ["System", "What it does", "Relation to this work"],
    ["CORNET [3]", "Connects ns-3 to Gazebo for robot networks",
     "Same pairing as the baseline stack"],
    ["FlyNetSim [4]", "Synchronises ns-3 with ArduPilot",
     "Close to the SITL arrangement used here"],
    ["FANS [5]", "ns-3 + ROS + Gazebo, extended to embedded hardware "
                 "(Jetson Nano + Pixhawk 4)",
     "Nearest match: also puts a real board in the loop"],
], [26 * mm, 74 * mm, 65 * mm]))
A(para("FANS is the strongest comparison point. It does the same thing — a real "
       "embedded board in the loop — but uses an NVIDIA Jetson Nano rather than "
       "a Raspberry Pi 4B. That hardware difference is the main methodological "
       "distinction to acknowledge.", "body"))

# ── distinctive ──
A(para("What is distinctive in this work", "h1"))
A(para("No published description was found of <b>separating the sensor and "
       "radio paths</b> in this context. The co-simulation frameworks above "
       "route traffic through the emulated network without distinguishing which "
       "links would physically be radios. The VLAN-based split used here is "
       "therefore a contribution, and should be stated as such rather than "
       "presented as routine setup.", "body"))

# ── second justification ──
A(para("A second justification for the split", "h1"))
A(para("The ns-3 documentation records a real limitation of real-time mode:", "body"))
E(quote("ns-3's real-time mode cannot cope with large and complex networks, and "
        "its accuracy decreases with higher traffic volumes or particularly "
        "bursty traffic", "Performance Evaluation of ns-3 Real-Time Emulation [6]"))
A(para("The camera stream is 481 Mbps. Had it crossed ns-3, the experiment would "
       "have been operating well outside the regime in which the emulator is "
       "accurate, and the channel model itself would have become a source of "
       "error. By keeping imagery off ns-3 and passing only 118-byte messages "
       "through it, the testbed stays inside the accurate operating region.", "body"))
E(callout("<b>The two-link split is not only physically faithful — it is "
          "required for the emulation to be trustworthy.</b> This is a stronger "
          "argument than the physical one alone, and it is citable."))

# ── results agree ──
A(para("Measured results agree with published work", "h1"))
E(quote("the RPI5 failed to satisfy the real-time processing needs in spite of "
        "its suitability for low-energy consumption applications",
        "YOLO on a standalone Raspberry Pi 5 [7]"))
E(quote("On Raspberry Pi 5, CPU-only execution of large models is impractical "
        "due to multi-second per-frame latency",
        "YOLOv8 / RT-DETR energy efficiency on edge devices [8]"))
A(para("The figures measured here — 2,540 ms with PyTorch and approximately "
       "1,000 ms with NCNN, on a Pi 4B, which is slower than the Pi 5 used in "
       "both studies — sit exactly where that literature predicts. The results "
       "are therefore consistent with independent work rather than anomalous, "
       "and the report can say so.", "body"))
A(para("The same body of work also characterises the offloading side, reporting "
       "end-to-end latency rising from 0.123 s to 2.317 s as available bandwidth "
       "falls from 1 Mbps to 50 Kbps [9] — the curve the ground arm of this "
       "experiment will trace.", "body"))

# ── optimisation ──
A(para("Optimisation options", "h1"))
E(table([
    ["Option", "Expected effect", "Cost"],
    ["NCNN at 640x384", "~470 ms, from ~1,000 ms", "One export; already planned"],
    ["Hailo or Coral accelerator", "10-30x", "Hardware, approx. USD 70"],
    ["Pi 5 with AI HAT (NPU)", "Large; benchmarked in [8]", "New board"],
    ["Jetson Orin Nano", "GPU inference; what FANS uses", "Expensive"],
    ["INT8 quantisation", "~2x", "Accuracy loss; needs calibration"],
], [46 * mm, 62 * mm, 57 * mm]))
E(callout("<b>Recommendation: do not change hardware.</b> The contribution is "
          "the testbed and the measurement, and \"the Pi 4B is too slow for "
          "real-time detection\" is itself a finding supported by [7] and [8]. "
          "Substituting a Jetson would remove the very constraint the experiment "
          "set out to measure. The NCNN 640 export is worth doing: it is free and "
          "completes the results table."))

# ── suggested text ──
A(para("Suggested wording for the report", "h1"))
E(quote("The use of ns-3 TapBridge to attach physical hardware to an emulated "
        "network follows the approach documented in the ns-3 model library [1] "
        "and applied in prior UAV co-simulation work such as CORNET [3] and "
        "FANS [5]. Unlike those systems, the present testbed separates the "
        "sensor and radio paths, so that only traffic which would physically "
        "traverse a radio is emulated — a distinction which also keeps ns-3 "
        "within the traffic regime in which its real-time mode remains "
        "accurate [6].", "adapt as needed"))

# ── sources ──
A(para("Sources", "h1"))
SRC = [
    ("[1]", "Emulation Overview — ns-3 Model Library",
     "https://www.nsnam.org/docs/models/html/emulation-overview.html"),
    ("[2]", "HOWTO make ns-3 interact with the real world",
     "https://www.nsnam.org/wiki/HOWTO_make_ns-3_interact_with_the_real_world"),
    ("[3]", "CORNET: A Co-Simulation Middleware for Robot Networks",
     "https://ece.iisc.ac.in/~parimal/papers/2020/comsnets.pdf"),
    ("[4]", "FlyNetSim: An Open Source Synchronized UAV Network Simulator "
            "based on ns-3 and Ardupilot",
     "https://ar5iv.labs.arxiv.org/html/1808.04967"),
    ("[5]", "Hardware Implementation of FANET Using FANS "
            "(Proc. ns-3 Conference 2025)",
     "https://dl.acm.org/doi/10.1145/3747204.3747223"),
    ("[6]", "Performance Evaluation of ns-3 Real-Time Emulation",
     "https://www.researchgate.net/publication/390270733_Performance_Evaluation_of_ns-3_Real-Time_Emulation"),
    ("[7]", "UAV detection with YOLO on a standalone Raspberry Pi 5 system",
     "https://ceur-ws.org/Vol-3970/PAPER1.pdf"),
    ("[8]", "Review of large YOLOv8 and RT-DETR energy efficiency on edge "
            "devices for real-time detection",
     "https://www.nature.com/articles/s41598-026-46453-6"),
    ("[9]", "Communication-Computation Trade-Off in Resource-Constrained "
            "Edge Inference",
     "https://arxiv.org/pdf/2006.02166"),
]
for n, t, u in SRC:
    A(Paragraph(f'<b>{n}</b>&nbsp; {t}<br/>'
                f'<font color="#1a5c8a" size="7.4">'
                f'<link href="{u}">{u}</link></font>', S["src"]))

doc = BaseDocTemplate(OUT, pagesize=A4,
                      leftMargin=22 * mm, rightMargin=22 * mm,
                      topMargin=18 * mm, bottomMargin=20 * mm,
                      title="HITL Method Validation",
                      author="multi_uav_simulation")
doc.addPageTemplates([PageTemplate(
    id="main",
    frames=[Frame(doc.leftMargin, doc.bottomMargin, doc.width, doc.height, id="f")],
    onPage=footer)])
doc.build(story)
print(f"wrote {OUT}")
