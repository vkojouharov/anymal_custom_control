#!/usr/bin/env python3
"""Fit BOTA boom compliance data."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import numpy as np

from src.pipeline_paths import CONVERTED_AXES_DATA_DIR, compliance_matrix_path, length_from_stem, select_input_csv


DEFAULT_INPUT_PATTERN = "bota_*_shear_center_converted_axes.csv"
# Optional manual input. Leave as "" to use the newest matching converted_axes_data CSV.
# You can type either a filename like "bota_1p2m_shear_center_converted_axes.csv" or a path like "data/converted_axes_data/bota_1p2m_shear_center_converted_axes.csv".
MANUAL_INPUT_CSV = "bota_0p75m_shear_center_converted_axes.csv"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--boom-length-m", type=float, default=None, help="Override boom length used for residual scaling")
    args = parser.parse_args()

    csv_path = select_input_csv(args.input, MANUAL_INPUT_CSV, CONVERTED_AXES_DATA_DIR, DEFAULT_INPUT_PATTERN)
    output_path = args.output.expanduser() if args.output is not None else compliance_matrix_path(csv_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    boom_length_m = args.boom_length_m if args.boom_length_m is not None else length_from_stem(csv_path.stem)

    print(f"Loading {csv_path}")
    print(f"Boom length: {boom_length_m:g} m")
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    W = data[:, 0:6].T  # [Fx, Fy, Fz, Mx, My, Mz], 6 x N
    X = data[:, 6:12].T  # [ux, uy, uz, theta_x, theta_y, theta_z], 6 x N

    H_sqrt = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])

    import cvxpy as cp

    C = cp.Variable((6, 6), symmetric=True)
    residual = X - C @ W
    objective = cp.sum_squares(H_sqrt @ residual)
    constraints = [
        C >> 0,
    ]

    


    problem = cp.Problem(cp.Minimize(objective), constraints)

    problem.solve()
    print("status:", problem.status)
    print("objective:", problem.value)
    print(C.value)
    if C.value is not None:
        C.value[np.abs(C.value) < 1e-6] = 0.0
        np.savetxt(output_path, C.value, delimiter=",", header="ux,uy,uz,theta_x,theta_y,theta_z", comments="")
        print(f"saved: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
