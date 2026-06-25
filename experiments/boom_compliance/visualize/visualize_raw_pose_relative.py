#!/usr/bin/env python3
"""Visualize raw id34/id35 pose data relative to the id34_t1 frame."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

from convert_raw_data import pose_from_row, quat_to_rotmat


DEFAULT_INPUT_CSV = EXPERIMENT_DIR / "data_2p25m_5N.csv"


def draw_vector(ax, start: np.ndarray, vector: np.ndarray, *, color: str, label: str, linewidth: float = 2.5) -> None:
    ax.quiver(
        start[0],
        start[1],
        start[2],
        vector[0],
        vector[1],
        vector[2],
        color=color,
        linewidth=linewidth,
        arrow_length_ratio=0.12,
        label=label,
    )


def draw_frame(ax, origin: np.ndarray, r_ref_from_frame: np.ndarray, *, prefix: str, linewidth: float, linestyle: str) -> None:
    axis_len = 0.08
    axes = [
        ("X", np.array([axis_len, 0.0, 0.0]), "red"),
        ("Y", np.array([0.0, axis_len, 0.0]), "green"),
        ("Z", np.array([0.0, 0.0, axis_len]), "blue"),
    ]
    for name, axis, color in axes:
        vector = r_ref_from_frame @ axis
        line = ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            vector[0],
            vector[1],
            vector[2],
            color=color,
            linewidth=linewidth,
            arrow_length_ratio=0.12,
            label=f"{prefix} {name}",
        )
        line.set_linestyle(linestyle)


def set_equal_bounds(ax, points: list[np.ndarray]) -> None:
    stacked = np.vstack(points)
    center = 0.5 * (stacked.min(axis=0) + stacked.max(axis=0))
    radius = max(float(np.max(stacked.max(axis=0) - stacked.min(axis=0))) * 0.65, 0.10)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--row", type=int, default=1)
    args = parser.parse_args()

    with args.input.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if args.row < 1 or args.row > len(rows):
        raise IndexError(f"--row must be between 1 and {len(rows)}")
    row = rows[args.row - 1]

    p34_t1, q34_t1 = pose_from_row(row, "id34_t1")
    p34_t2, q34_t2 = pose_from_row(row, "id34_t2")
    p35_t2, _q35_t2 = pose_from_row(row, "id35_t2")

    r_world_from_t1 = quat_to_rotmat(q34_t1)
    r_world_from_t2 = quat_to_rotmat(q34_t2)
    r_t1_from_world = r_world_from_t1.T

    origin_t1 = np.zeros(3)
    origin_t2 = r_t1_from_world @ (p34_t2 - p34_t1)
    load_cell_t2 = r_t1_from_world @ (p35_t2 - p34_t1)
    force_direction = load_cell_t2 - origin_t2
    force_direction_unit = force_direction / np.linalg.norm(force_direction)
    r_t1_from_t2 = r_t1_from_world @ r_world_from_t2

    fig = plt.figure(f"Raw Pose Relative Row {args.row}")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"Raw id34/id35 data relative to id34_t1, row {args.row}")
    ax.set_xlabel("id34_t1 X (m)")
    ax.set_ylabel("id34_t1 Y (m)")
    ax.set_zlabel("id34_t1 Z (m)")

    draw_frame(ax, origin_t1, np.eye(3), prefix="id34_t1 dashed", linewidth=1.6, linestyle="--")
    draw_frame(ax, origin_t2, r_t1_from_t2, prefix="id34_t2 solid", linewidth=2.8, linestyle="-")
    draw_vector(ax, origin_t1, origin_t2, color="#ff7f0e", label="id34 displacement")
    draw_vector(ax, origin_t2, force_direction_unit * 0.12, color="#d62728", label="id35_t2 - id34_t2")

    ax.scatter(*origin_t1, color="black", marker="+", s=120, linewidths=2.0, label="id34_t1")
    ax.scatter(*origin_t2, color="black", marker="o", s=45, label="id34_t2")
    ax.scatter(*load_cell_t2, color="#d62728", marker="x", s=65, linewidths=2.0, label="id35_t2")
    ax.plot(
        [origin_t2[0], load_cell_t2[0]],
        [origin_t2[1], load_cell_t2[1]],
        [origin_t2[2], load_cell_t2[2]],
        color="#d62728",
        linestyle=":",
        linewidth=1.4,
    )

    set_equal_bounds(ax, [origin_t1, origin_t2, load_cell_t2, origin_t2 + force_direction_unit * 0.12])
    ax.legend(loc="upper left", fontsize=8)

    print(f"row: {args.row}")
    print(f"id34_t2 origin in id34_t1 frame [m]: {origin_t2}")
    print(f"id35_t2 origin in id34_t1 frame [m]: {load_cell_t2}")
    print(f"force direction unit in id34_t1 frame: {force_direction_unit}")
    print("id34_t2 axes as columns in id34_t1 frame:")
    print(r_t1_from_t2)

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
