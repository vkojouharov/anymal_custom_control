#!/usr/bin/env python3
"""Generate all standard egocentric servo visualizations for one or more runs."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


RUNS = [
    "roverscape_test2",
    "roverscape_test3",
    "roverscape_test4",
    "roverscape_test6",
    "roverscape_test10",
    "roverscape_test11",
    "roverscape_test12",
    "roverscape_test14"
]

root = Path(__file__).resolve().parent
scripts = [
    # ("trajectory plot", root / "visualize_trajectory.py"),
    # ("MPC rollout", root / "visualize_MPC_planning.py"),
    ("synced POV", root / "visualize_synced_mpc_pov.py"),
]

for run in RUNS:
    print(f"\n=== {run} ===", flush=True)
    for label, script in scripts:
        command = [sys.executable, str(script), run, "--no-show"] if label == "trajectory plot" else [sys.executable, str(script), run]
        print(f"running {label}...", flush=True)
        result = subprocess.run(command)
        status = "done" if result.returncode == 0 else f"failed ({result.returncode})"
        print(f"{label}: {status}", flush=True)
        if result.returncode != 0:
            raise SystemExit(result.returncode)
