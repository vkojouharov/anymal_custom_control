#!/usr/bin/env python3
"""Visualize compliance fit quality by force direction for each trial."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable

# Edit these before running.
TRIAL_CSV_NAME = "../length_1p5m_force_5N.csv"
COMPLIANCE_CSV_NAME = "../compliance_matrix.csv"
FORCE_MAGNITUDE_N = 5.0
ARROW_LENGTH = 1.0  # Display length for normalized direction arrows.
ERROR_MODE = "relative"  # "relative" or "absolute"


@dataclass(frozen=True)
class Pose:
    p: np.ndarray
    q: np.ndarray  # xyzw


def resolve_path(name: str) -> Path:
    path = Path(name).expanduser()
    if path.is_absolute():
        return path
    return Path(__file__).resolve().parent / path


def quat_normalize(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm <= 1e-12:
        raise ValueError("zero-length quaternion")
    return q / norm


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ],
        dtype=float,
    )


def quat_to_rotmat(q_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quat_normalize(q_xyzw)
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=float,
    )


def quat_to_rotvec(q_xyzw: np.ndarray) -> np.ndarray:
    q = quat_normalize(q_xyzw)
    if q[3] < 0.0:
        q = -q
    vector = q[:3]
    scalar = q[3]
    vector_norm = np.linalg.norm(vector)
    if vector_norm <= 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, scalar)
    return vector / vector_norm * angle


def read_pose(row: dict[str, str], prefix: str) -> Pose:
    return Pose(
        p=np.array(
            [float(row[f"{prefix}_px"]), float(row[f"{prefix}_py"]), float(row[f"{prefix}_pz"])],
            dtype=float,
        ),
        q=np.array(
            [float(row[f"{prefix}_qx"]), float(row[f"{prefix}_qy"]), float(row[f"{prefix}_qz"]), float(row[f"{prefix}_qw"])],
            dtype=float,
        ),
    )


def load_compliance_matrix(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open(newline="") as handle:
        reader = csv.reader(handle)
        header = next(reader, None)
        if header is None:
            raise ValueError(f"empty compliance matrix CSV: {path}")
        for row in reader:
            if not row:
                continue
            rows.append([float(value) for value in row[1:7]])
    matrix = np.array(rows, dtype=float)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape}")
    return matrix


def row_to_trial_vectors(row: dict[str, str], compliance: np.ndarray) -> tuple[np.ndarray, float, np.ndarray, np.ndarray]:
    id34_t1 = read_pose(row, "id34_t1")
    id34_t2 = read_pose(row, "id34_t2")
    id35_t2 = read_pose(row, "id35_t2")

    r_world_from_body_to_world = quat_to_rotmat(id34_t1.q)
    r_body_from_world = r_world_from_body_to_world.T

    dp_body_t1 = r_body_from_world @ (id34_t2.p - id34_t1.p)
    q_rel_world = quat_multiply(id34_t2.q, quat_conjugate(id34_t1.q))
    drot_body_t1 = r_body_from_world @ quat_to_rotvec(q_rel_world)
    measured_x = np.concatenate([dp_body_t1, drot_body_t1])

    direction_world = id35_t2.p - id34_t2.p
    direction_norm = np.linalg.norm(direction_world)
    if direction_norm <= 1e-12:
        raise ValueError("ID35 time-2 position equals ID34 time-2 position; direction is undefined")

    unit_direction_world = direction_world / direction_norm
    force_body_t1 = r_body_from_world @ (FORCE_MAGNITUDE_N * unit_direction_world)
    wrench = np.concatenate([force_body_t1, np.zeros(3)])
    predicted_x = compliance @ wrench
    residual = measured_x - predicted_x

    if ERROR_MODE == "absolute":
        error = float(np.linalg.norm(residual))
    elif ERROR_MODE == "relative":
        error = float(np.linalg.norm(residual) / max(np.linalg.norm(measured_x), 1e-12))
    else:
        raise ValueError('ERROR_MODE must be "relative" or "absolute"')

    return unit_direction_world, error, measured_x, predicted_x


def load_trials(path: Path, compliance: np.ndarray) -> tuple[list[np.ndarray], list[float]]:
    directions: list[np.ndarray] = []
    errors: list[float] = []
    skipped = 0

    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for row_index, row in enumerate(reader, start=1):
            try:
                direction, error, _measured, _predicted = row_to_trial_vectors(row, compliance)
            except Exception as exc:
                skipped += 1
                print(f"Skipping row {row_index}: {exc}")
                continue
            directions.append(direction)
            errors.append(error)

    if not directions:
        raise RuntimeError("no usable trial rows found")
    if skipped:
        print(f"Skipped {skipped} row(s)")
    return directions, errors


def set_equal_bounds(ax, radius: float) -> None:
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_zlim(-radius, radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def main() -> int:
    trial_csv = resolve_path(TRIAL_CSV_NAME)
    compliance_csv = resolve_path(COMPLIANCE_CSV_NAME)
    compliance = load_compliance_matrix(compliance_csv)
    directions, errors = load_trials(trial_csv, compliance)

    error_array = np.array(errors, dtype=float)
    best = float(np.min(error_array))
    worst = float(np.max(error_array))
    mean = float(np.mean(error_array))

    cmap = LinearSegmentedColormap.from_list(
        "green_yellow_orange_red",
        ["#128a30", "#f2e85c", "#f28c28", "#c92525"],
    )
    norm = Normalize(vmin=best, vmax=worst if worst > best else best + 1e-12)

    fig = plt.figure("Compliance Fit By Force Direction")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"Compliance fit by direction ({ERROR_MODE} error)")
    ax.set_xlabel("World X")
    ax.set_ylabel("World Y")
    ax.set_zlabel("World Z")

    origin = np.zeros(3)
    for index, (direction, error) in enumerate(zip(directions, errors), start=1):
        end = direction * ARROW_LENGTH
        color = cmap(norm(error))
        ax.quiver(
            origin[0], origin[1], origin[2],
            end[0], end[1], end[2],
            color=color,
            linewidth=2.2,
            arrow_length_ratio=0.12,
            normalize=False,
        )
        ax.text(end[0] * 1.04, end[1] * 1.04, end[2] * 1.04, str(index), color=color, fontsize=8)

    set_equal_bounds(ax, ARROW_LENGTH * 1.25)
    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    colorbar = fig.colorbar(sm, ax=ax, shrink=0.72, pad=0.10)
    colorbar.set_label(f"Fit error ({ERROR_MODE})")

    print(f"Trial CSV: {trial_csv}")
    print(f"Compliance CSV: {compliance_csv}")
    print(f"Trials plotted: {len(directions)}")
    print(f"Fit error min/mean/max: {best:.6e} / {mean:.6e} / {worst:.6e}")
    print("Green is lower error; red is higher error.")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
