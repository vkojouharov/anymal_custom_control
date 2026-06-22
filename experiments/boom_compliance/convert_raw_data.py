#!/usr/bin/env python3
"""Convert raw boom compliance capture data with LOAD CELL measurements 
to shear-center wrench/displacement CSV.

Frame convention:
- id34_t1 is the undeformed endpoint reference frame.
- In the raw id34 rigid-body frame, +Z is the boom longitudinal axis and -Y
  points toward the open side of the cross section.
- The output CSV is converted to a beam frame where +X is longitudinal, +Y is
  the opening direction, and +Z completes the right-handed frame.
- The input force is applied at the circle center.
- The output wrench and displacement are both about the shear center.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import numpy as np


DEFAULT_INPUT_CSV = Path(__file__).resolve().with_name("length_1p5m_force_5N.csv")
DEFAULT_OUTPUT_CSV = Path(__file__).resolve().with_name("length_1p5m_force_5N_shear_center.csv")
DEFAULT_FORCE_N = 5.0
DEFAULT_DIAMETER_MM = 40.0
DEFAULT_SUBTENDED_ANGLE_DEG = 270.0
R_BEAM_FROM_ID34 = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=float,
)

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


def pose_from_row(row: dict[str, str], prefix: str) -> tuple[np.ndarray, np.ndarray]:
    p = np.array(
        [float(row[f"{prefix}_px"]), float(row[f"{prefix}_py"]), float(row[f"{prefix}_pz"])],
        dtype=float,
    )
    q = np.array(
        [float(row[f"{prefix}_qx"]), float(row[f"{prefix}_qy"]), float(row[f"{prefix}_qz"]), float(row[f"{prefix}_qw"])],
        dtype=float,
    )
    return p, q


def shear_center_y_m(diameter_mm: float, subtended_angle_deg: float) -> float:
    """Signed shear-center y coordinate for an open thin-wall circular arc."""
    radius_m = diameter_mm / 2000.0
    alpha = math.radians(subtended_angle_deg / 2.0)
    numerator = math.sin(alpha) - alpha * math.cos(alpha)
    denominator = alpha - math.sin(alpha) * math.cos(alpha)
    return -2.0 * radius_m * numerator / denominator


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_CSV)
    parser.add_argument("--force-n", type=float, default=DEFAULT_FORCE_N)
    parser.add_argument("--diameter-mm", type=float, default=DEFAULT_DIAMETER_MM)
    parser.add_argument("--subtended-angle-deg", type=float, default=DEFAULT_SUBTENDED_ANGLE_DEG)
    parser.add_argument(
        "--y-sc-mm",
        type=float,
        default=None,
        help="Override signed shear-center y coordinate. If omitted, compute from diameter and subtended angle.",
    )
    args = parser.parse_args()

    if args.y_sc_mm is None:
        y_sc_m = shear_center_y_m(args.diameter_mm, args.subtended_angle_deg)
    else:
        y_sc_m = args.y_sc_mm / 1000.0

    # Circle center is y=0. The shear center is at y=y_sc_m, so this vector
    # points from the shear center to the circle center.
    r_sc_to_circle_center = np.array([0.0, -y_sc_m, 0.0], dtype=float)

    rows_written = 0
    with args.input.open(newline="") as input_handle, args.output.open("w", newline="") as output_handle:
        reader = csv.DictReader(input_handle)
        writer = csv.writer(output_handle)
        writer.writerow(OUTPUT_COLUMNS)

        for row_number, row in enumerate(reader, start=1):
            p34_t1, q34_t1 = pose_from_row(row, "id34_t1")
            p34_t2, q34_t2 = pose_from_row(row, "id34_t2")
            p35_t2, _q35_t2 = pose_from_row(row, "id35_t2")

            r_world_from_ref = quat_to_rotmat(q34_t1)
            r_ref_from_world = r_world_from_ref.T

            # First express vectors in raw id34_t1, then convert to beam frame:
            # x_beam = z_id34, y_beam = -y_id34, z_beam = x_id34.
            u_circle_center_id34 = r_ref_from_world @ (p34_t2 - p34_t1)
            r_world_from_deformed = quat_to_rotmat(q34_t2)
            theta_id34 = rotmat_to_rotvec(r_ref_from_world @ r_world_from_deformed)
            u_circle_center = R_BEAM_FROM_ID34 @ u_circle_center_id34
            theta = R_BEAM_FROM_ID34 @ theta_id34

            # Convert measured circle-center displacement to shear-center displacement.
            u_shear_center = u_circle_center - np.cross(theta, r_sc_to_circle_center)

            # Force direction is from the deformed endpoint marker toward the load cell.
            force_direction_world = p35_t2 - p34_t2
            direction_norm = np.linalg.norm(force_direction_world)
            if direction_norm <= 1e-12:
                print(f"Skipping row {row_number}: id35_t2 position equals id34_t2 position")
                continue

            force_id34 = r_ref_from_world @ (args.force_n * force_direction_world / direction_norm)
            force = R_BEAM_FROM_ID34 @ force_id34
            moment_about_shear_center = np.cross(r_sc_to_circle_center, force)

            writer.writerow(
                [
                    *force,
                    *moment_about_shear_center,
                    *u_shear_center,
                    *theta,
                ]
            )
            rows_written += 1

    print(f"Wrote {rows_written} rows to {args.output}")
    print(f"Force magnitude: {args.force_n:.3f} N")
    print(f"Signed shear-center offset: y_sc = {1000.0 * y_sc_m:+.3f} mm")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
