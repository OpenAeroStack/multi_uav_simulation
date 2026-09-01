#!/usr/bin/env python3
"""Run the complete, version-safe real-versus-simulation analysis pipeline."""

import argparse
import subprocess
import sys
from pathlib import Path
from common import (DEFAULT_REAL, DEFAULT_SIM, OUTPUT_BASENAME, RESULTS_ROOT,
                    prepare_output, unique_dir, validate_inputs)


SCRIPTS=("01_inspect_datasets.py","02_compare_trajectory.py","03_compare_altitude.py","04_compare_speed.py",
         "05_compare_heading.py","06_compare_mission.py","07_compare_telemetry.py","08_analyze_network.py","09_generate_summary.py")


def main():
    p=argparse.ArgumentParser(description="Run every real-vs-simulation comparison phase")
    p.add_argument("--real-dir",type=Path,default=DEFAULT_REAL); p.add_argument("--sim-dir",type=Path,default=DEFAULT_SIM); p.add_argument("--output-dir",type=Path)
    x=p.parse_args(); real,sim=validate_inputs(x.real_dir,x.sim_dir)
    requested=x.output_dir.resolve() if x.output_dir else RESULTS_ROOT/OUTPUT_BASENAME
    output=prepare_output(unique_dir(requested))
    here=Path(__file__).resolve().parent
    for script in SCRIPTS:
        print(f"Running {script}...",flush=True)
        subprocess.run([sys.executable,str(here/script),"--real-dir",str(real),"--sim-dir",str(sim),"--output-dir",str(output)],check=True)
    print(f"\nComparison complete: {output}")


if __name__=="__main__": main()
