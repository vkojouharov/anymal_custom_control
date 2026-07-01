#!/usr/bin/env python3
"""Convert shear-center CSV axes to the final beam naming convention.

Axis convention:
- old +Z becomes new +X
- old +Y becomes new -Y
- old +X becomes new +Z

So every 3-vector is transformed as:
    [new_x, new_y, new_z] = [old_z, -old_y, old_x]

The same transform is applied to force, moment, displacement, and rotation
vector columns.
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import numpy as np

from src.pipeline_paths import SHEAR_CENTER_DATA_DIR, converted_axes_path, select_input_csv


DEFAULT_INPUT_PATTERN = "bota_*_shear_center.csv"
# Optional manual input. Leave as "" to use the newest matching shear_center_data CSV.
# You can type either a filename like "bota_1p2m_shear_center.csv" or a path like "data/shear_center_data/bota_1p2m_shear_center.csv".
MANUAL_INPUT_CSV = "bota_0p75m_shear_center.csv"

OUTPUT_COLUMNS = [
    "Fx_N",
    "Fy_N",
    "Fz_N",
    "Mx_Nm",
    "My_Nm",
    "Mz_Nm",
    "ux_m",
    "uy_m",
    "uz_m",
    "theta_x_rad",
    "theta_y_rad",
    "theta_z_rad",
]

R_NEW_FROM_OLD = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)


def vector_from_row(row: dict[str, str], keys: tuple[str, str, str]) -> np.ndarray:
    return np.array([float(row[key]) for key in keys], dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    input_csv = select_input_csv(args.input, MANUAL_INPUT_CSV, SHEAR_CENTER_DATA_DIR, DEFAULT_INPUT_PATTERN)
    output_csv = args.output.expanduser() if args.output is not None else converted_axes_path(input_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    rows_written = 0
    with input_csv.open(newline="") as input_handle, output_csv.open("w", newline="") as output_handle:
        reader = csv.DictReader(input_handle)
        writer = csv.writer(output_handle)
        writer.writerow(OUTPUT_COLUMNS)

        for row in reader:
            force_old = vector_from_row(row, ("Fx_N", "Fy_N", "Fz_N"))
            moment_old = vector_from_row(row, ("Mx_Nm", "My_Nm", "Mz_Nm"))
            displacement_old = vector_from_row(row, ("ux_m", "uy_m", "uz_m"))
            theta_old = vector_from_row(row, ("theta_x_rad", "theta_y_rad", "theta_z_rad"))

            force_new = R_NEW_FROM_OLD @ force_old
            moment_new = R_NEW_FROM_OLD @ moment_old
            displacement_new = R_NEW_FROM_OLD @ displacement_old
            theta_new = R_NEW_FROM_OLD @ theta_old

            writer.writerow(
                [
                    *force_new,
                    *moment_new,
                    *displacement_new,
                    *theta_new,
                ]
            )
            rows_written += 1

    print(f"Wrote {rows_written} rows to {output_csv}")
    print("Axis mapping: new [X,Y,Z] = old [Z,-Y,X]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
