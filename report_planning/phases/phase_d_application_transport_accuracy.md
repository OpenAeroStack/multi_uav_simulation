# Phase D — Application Transport and Accuracy

## Purpose
Evaluate image bandwidth/delivery versus JPEG quality and raw-versus-Q5 detection accuracy.

## Source directories
`results/phase_d_application/` and Phase-D scripts in `results/scripts/`.

## Experiment design
Edge/Ground TAP-counter transport runs plus a manually labelled D4 evaluation applying the same detector to raw PNG and JPEG-Q5 representations.

## Independent experimental unit
A complete transport run for D1; D4 v2 is one 60-unique-frame labelled evaluation set. Frames are evaluation cases, not system replications.

## Available runs
Ten transport rows/configurations. D4 raw provenance stores 60 PNGs, 60 capture-index rows and an original duplicated 61-label/details analysis. Report-authoritative v2 contains 60 ordered unique rows per mode.

## Main files
D1 bandwidth/delivery summaries and TAP logs; original D4 sources; `d4_unique_frames_v2/` audit, unique index, per-mode details, corrected summary/comparison and figures; versioned recomputation script.

## Main measured metrics
Bandwidth, byte/packet delivery, precision, recall, F1, exact-count rate, inference, JPEG payload/encode/decode time.

## Verified numerical results
Corrected v2 (60 unique frames): Edge/raw TP=135, FP=19, FN=29, precision 0.876623, recall 0.823171, F1 0.849057, exact count 34/60=0.566667. Ground/Q5 TP=29, FP=0, FN=135, precision 1.000000, recall 0.176829, F1 0.300518, exact count 26/60=0.433333. D1 byte delivery ranges from ~29.6–30.0% at q50 to 100% at q5; byte delivery is not complete-frame delivery.

## Results suitable for the main report
Corrected D4 v2 comparison/figure and concise JPEG transport table. The primary final dataset's older D4 figure is superseded for report accuracy values.

## Results better suited to an appendix
Original 61-row D4 outputs/figure, v2 audit details, source images, TAP counters and rerun provenance.

## Known issues and limitations
The original analysis duplicated frame 33 and is superseded; all original artifacts remain for provenance. Received interface counts can slightly exceed sent counts due to window/background alignment.

## Recommended Chapter 5 destination
Section 6, already drafted; consistency check only.

## Processing still required
No D4 recomputation remains; use v2 for final report consistency review. Do not alter original source/provenance files.

## Suggested tables
D4 metrics and JPEG transport delivery.

## Suggested figures
Versioned `d4_detection_accuracy_comparison_unique`; other existing final application figures.

## Claims supported by the evidence
Corrected 60-frame D4 metrics show substantially higher raw than Q5 recall; compression reduces payload and improves byte delivery as quality falls.

## Claims not supported by the evidence
That the original D4 contains 61 unique frames, that the small metric correction is statistically significant, or that byte delivery equals complete-frame delivery.
