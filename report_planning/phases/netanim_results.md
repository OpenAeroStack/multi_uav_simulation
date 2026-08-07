# NetAnim Results

## Purpose
Provide visual evidence of simulated GCS/UAV mobility.

## Source directories
`results_02/netanim/`.

## Experiment design
Two four-node timestamped mobility recordings.

## Independent experimental unit
One complete trace.

## Available runs
`three_uav_20260725_171432_82636.xml` (~67.28 s) and `three_uav_20260725_174947_95620.xml` (~40.10 s), each with an incomplete companion; `latest.xml` symlinks to the latter.

## Main files
Two final XMLs, two `_incomplete.xml` files, and the `latest.xml` symbolic link.

## Main measured metrics
Node IDs and time-indexed x/y positions. No packet or explicit link records were found.

## Verified numerical results
Each trace defines four nodes. Final XMLs pass syntax parsing but end with a malformed position value: `y="-"` in the older trace and an empty `id` in the newer trace. Incomplete XMLs fail parsing.

## Results suitable for the main report
At most a small implementation illustration if required.

## Results better suited to an appendix
One representative screenshot or animation reference.

## Known issues and limitations
Malformed endpoints, truncated companions, two runs only, and no packet/link evidence.

## Recommended Chapter 5 destination
Section 2 or appendix; demonstration only.

## Processing still required
Visually validate playback and capture before the malformed last update; do not modify source XML.

## Suggested tables
Trace name, duration, nodes, syntax/end-record status.

## Suggested figures
At most one representative NetAnim screenshot.

## Claims supported by the evidence
Four-node mobility updates were produced.

## Claims not supported by the evidence
Packet delivery, link performance, clustering correctness, or complete clean trace generation.
