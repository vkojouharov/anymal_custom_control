#!/usr/bin/env python3
"""Convert compliance capture data to shear-center wrench/displacement CSV.

Frame convention:
- Input rows come from collect_compliance_data.py.
- deflection_*_m is the displacement of the rigid-body center, expressed in
  the initial body-34 frame.
- rel_q* is the reorientation from the initial body-34 frame to the current
  body-34 frame.
- BOTA force/torque are assumed to already be expressed in the body-34 frame.
- The shear center is 33 mm in +Y from the rigid-body center.
- Output wrench and displacement are both about the shear center.
"""

from __future__ import annotations

import argparse
import csv
import sys
import math
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import numpy as np

from src.pipeline_paths import RAW_DATA_DIR, select_input_csv, shear_center_path


DEFAULT_INPUT_PATTERN = "bota_*.csv"
# Optional manual input. Leave as "" to use the newest matching raw_data CSV.
# You can type either a filename like "bota_1p2m.csv" or a path like "data/raw_data/bota_1p2m.csv".
MANUAL_INPUT_CSV = "bota_0p75m.csv"
SHEAR_CENTER_Y_MM = 33.24

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


def parse_float(row: dict[str, str], key: str) -> float:
    value = row.get(key, "")
    if value == "":
        raise ValueError(f"missing {key}")
    return float(value)


def quat_to_rotmat(q_xyzw: np.ndarray) -> np.ndarray:
    q = q_xyzw / np.linalg.norm(q_xyzw)
    x, y, z, w = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotmat_to_rotvec(r: np.ndarray) -> np.ndarray:
    angle = math.acos(float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-12:
        return np.zeros(3)

    axis = np.array(
        [
            r[2, 1] - r[1, 2],
            r[0, 2] - r[2, 0],
            r[1, 0] - r[0, 1],
        ],
        dtype=float,
    )
    axis /= 2.0 * math.sin(angle)
    return axis * angle


def vector_from_row(row: dict[str, str], keys: tuple[str, str, str]) -> np.ndarray:
    return np.array([parse_float(row, key) for key in keys], dtype=float)


def quat_from_row(row: dict[str, str], prefix: str) -> np.ndarray:
    return np.array(
        [
            parse_float(row, f"{prefix}_qx"),
            parse_float(row, f"{prefix}_qy"),
            parse_float(row, f"{prefix}_qz"),
            parse_float(row, f"{prefix}_qw"),
        ],
        dtype=float,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--y-sc-mm", type=float, default=SHEAR_CENTER_Y_MM, help="Shear-center offset from rigid-body center in +Y")
    args = parser.parse_args()

    input_csv = select_input_csv(args.input, MANUAL_INPUT_CSV, RAW_DATA_DIR, DEFAULT_INPUT_PATTERN)
    output_csv = args.output.expanduser() if args.output is not None else shear_center_path(input_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)

    # Shear center is at +Y from the rigid-body center, so this vector points
    # from the shear center back to the rigid-body center.
    r_sc_to_center = np.array([0.0, -args.y_sc_mm / 1000.0, 0.0], dtype=float)

    rows_written = 0
    rows_skipped = 0
    with input_csv.open(newline="") as input_handle, output_csv.open("w", newline="") as output_handle:
        reader = csv.DictReader(input_handle)
        writer = csv.writer(output_handle)
        writer.writerow(OUTPUT_COLUMNS)

        for row_number, row in enumerate(reader, start=1):
            try:
                force = vector_from_row(row, ("fx", "fy", "fz"))
                moment_center = vector_from_row(row, ("tx", "ty", "tz"))
                u_center = vector_from_row(row, ("deflection_x_m", "deflection_y_m", "deflection_z_m"))
                rel_quat = quat_from_row(row, "rel")
            except ValueError as exc:
                rows_skipped += 1
                print(f"Skipping row {row_number}: {exc}")
                continue

            theta = rotmat_to_rotvec(quat_to_rotmat(rel_quat))
            moment_shear_center = moment_center + np.cross(r_sc_to_center, force)
            u_shear_center = u_center - np.cross(theta, r_sc_to_center)

            writer.writerow(
                [
                    *force,
                    *moment_shear_center,
                    *u_shear_center,
                    *theta,
                ]
            )
            rows_written += 1

    print(f"Wrote {rows_written} rows to {output_csv}")
    if rows_skipped:
        print(f"Skipped {rows_skipped} rows with missing data")
    print(f"Signed shear-center offset from rigid-body center: y_sc = {args.y_sc_mm:+.3f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
