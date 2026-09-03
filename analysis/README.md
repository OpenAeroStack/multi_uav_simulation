# Real-flight versus simulation analysis

Run the full pipeline from the repository root:

```bash
python3 analysis/run_all_comparisons.py
```

The runner reads the original real and simulation result directories without
modifying them. A versioned comparison directory is created under `results/`;
if the normal output name exists, `_2`, `_3`, and so on are used safely.

The analysis uses the `.tlog`/logger arrival timelines, aligns both runs at the
first two consecutive GPS samples at or above 0.5 m relative altitude, and stops
the shared comparison at the earlier mission-end event. GPS coordinates are
reprojected into one common ENU frame using the origin recorded in simulation
metadata. No comparison is performed by CSV row index and no extrapolation is
used.

The trajectory phase also reads the source flight's QGC WPL 110 mission from
`analysis/data/real_2026-08-31_18-03-14.waypoints`. It converts each unique,
valid horizontal waypoint through the same ENU transformation and reports
separate, geometry-only real-to-commanded and simulation-to-commanded
point-to-polyline errors. These do not replace the time-aligned trajectory
metrics.

Each numbered script can also be run separately. With no `--output-dir`, an
independent version-safe result directory is created. NumPy, Matplotlib, and
pyproj are required; pandas is not required.
