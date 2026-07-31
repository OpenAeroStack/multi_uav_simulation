#!/usr/bin/env python3
"""
Reads spawn poses directly out of a Gazebo world SDF and converts each one to
a geodetic home location, using world_origin.yaml as the single source of
truth for the world's (0,0,0) -> lat/lon/alt/heading mapping.

Usage:
    python3 compute_home_locations.py my_world.world world_origin.yaml
"""
import sys
import re
import yaml
import pymap3d as pm

MODEL_POSE_RE = re.compile(
    r'<model\s+name=["\'](?P<name>[^"\']+)["\']>\s*'
    r'(?:<static>[^<]*</static>\s*)?'
    r'<pose[^>]*>(?P<pose>[^<]+)</pose>'
)


def load_origin(path):
    with open(path) as f:
        o = yaml.safe_load(f)
    return o["latitude_deg"], o["longitude_deg"], o["elevation_m"], o.get("heading_deg", 0.0)


def extract_poses(world_path):
    text = open(world_path).read()
    poses = {}
    for m in MODEL_POSE_RE.finditer(text):
        name = m.group("name")
        vals = [float(v) for v in m.group("pose").split()]
        poses[name] = (vals[0], vals[1], vals[2])
    return poses


def main():
    world_path, origin_path = sys.argv[1], sys.argv[2]
    lat0, lon0, alt0, heading = load_origin(origin_path)
    if abs(heading) > 1e-6:
        print(f"WARNING: heading_deg={heading} nonzero — this script does NOT "
              f"rotate for heading. Add a rotation step before trusting these numbers.")

    poses = extract_poses(world_path)
    if not poses:
        print("No model poses matched — check MODEL_POSE_RE against your SDF.")
        return

    print(f"{'model':20s} {'xyz':22s} {'lat':>12s} {'lon':>12s} {'alt':>7s}  custom-location")
    for name, (x, y, z) in poses.items():
        lat, lon, alt = pm.enu2geodetic(x, y, z, lat0, lon0, alt0)
        print(f"{name:20s} ({x:>5.1f},{y:>5.1f},{z:>4.1f})   {lat:12.7f} {lon:12.7f} {alt:7.2f}  {lat:.7f},{lon:.7f},{alt:.2f},0")


if __name__ == "__main__":
    main()