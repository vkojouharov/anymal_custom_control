#!/usr/bin/env python3
"""Visualize one captured OptiTrack CSV row in 3D."""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt

# Edit these before running.
CSV_NAME = "../optitrack_capture.csv"
ROW_NUMBER = 3  # 1 means the first data row, not the CSV header.
AXIS_LENGTH = 0.20


@dataclass(frozen=True)
class Pose:
    px: float
    py: float
    pz: float
    qx: float
    qy: float
    qz: float
    qw: float

    @property
    def origin(self) -> tuple[float, float, float]:
        return self.px, self.py, self.pz


def body_axes_from_quaternion(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)

    x = qx / norm
    y = qy / norm
    z = qz / norm
    w = qw / norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    x_axis = (1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy))
    y_axis = (2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx))
    z_axis = (2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy))
    return x_axis, y_axis, z_axis


def resolve_csv_path() -> Path:
    csv_path = Path(CSV_NAME).expanduser()
    if csv_path.is_absolute():
        return csv_path
    return Path(__file__).resolve().parent / csv_path


def read_pose(row: dict[str, str], prefix: str) -> Pose:
    return Pose(
        px=float(row[f"{prefix}_px"]),
        py=float(row[f"{prefix}_py"]),
        pz=float(row[f"{prefix}_pz"]),
        qx=float(row[f"{prefix}_qx"]),
        qy=float(row[f"{prefix}_qy"]),
        qz=float(row[f"{prefix}_qz"]),
        qw=float(row[f"{prefix}_qw"]),
    )


def read_row(csv_path: Path, row_number: int) -> dict[str, str]:
    if row_number < 1:
        raise ValueError("ROW_NUMBER must be 1 or greater")

    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for index, row in enumerate(reader, start=1):
            if index == row_number:
                return row

    raise IndexError(f"CSV only has {index if 'index' in locals() else 0} data rows; requested row {row_number}")


def draw_pose_axes(ax, pose: Pose, *, alpha: float, linewidth: float, label_prefix: str) -> list[tuple[float, float, float]]:
    colors = (("X", "red"), ("Y", "green"), ("Z", "blue"))
    axes = body_axes_from_quaternion(pose.qx, pose.qy, pose.qz, pose.qw)
    origin = pose.origin
    plotted_points = [origin]

    for (axis_name, color), vector in zip(colors, axes):
        end = (
            origin[0] + vector[0] * AXIS_LENGTH,
            origin[1] + vector[1] * AXIS_LENGTH,
            origin[2] + vector[2] * AXIS_LENGTH,
        )
        ax.plot(
            [origin[0], end[0]],
            [origin[1], end[1]],
            [origin[2], end[2]],
            color=color,
            alpha=alpha,
            linewidth=linewidth,
            label=f"{label_prefix} {axis_name}",
        )
        plotted_points.append(end)
    return plotted_points


def set_equal_bounds(ax, points: list[tuple[float, float, float]]) -> None:
    xs, ys, zs = zip(*points)
    cx = (min(xs) + max(xs)) / 2.0
    cy = (min(ys) + max(ys)) / 2.0
    cz = (min(zs) + max(zs)) / 2.0
    span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), AXIS_LENGTH) * 0.65
    margin = AXIS_LENGTH * 0.75
    radius = span + margin

    ax.set_xlim(cx - radius, cx + radius)
    ax.set_ylim(cy - radius, cy + radius)
    ax.set_zlim(cz - radius, cz + radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def main() -> int:
    csv_path = resolve_csv_path()
    row = read_row(csv_path, ROW_NUMBER)

    id34_t1 = read_pose(row, "id34_t1")
    id34_t2 = read_pose(row, "id34_t2")
    id35_t2 = read_pose(row, "id35_t2")

    fig = plt.figure(f"OptiTrack CSV Row {ROW_NUMBER}")
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"{csv_path.name} row {ROW_NUMBER}")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")

    points: list[tuple[float, float, float]] = []
    points.extend(draw_pose_axes(ax, id34_t1, alpha=0.25, linewidth=1.5, label_prefix="ID34 t1"))
    points.extend(draw_pose_axes(ax, id34_t2, alpha=1.0, linewidth=3.0, label_prefix="ID34 t2"))

    ax.scatter(
        [id35_t2.px],
        [id35_t2.py],
        [id35_t2.pz],
        color="black",
        s=45,
        depthshade=False,
        label="ID35 t2",
    )
    ax.plot(
        [id34_t2.px, id35_t2.px],
        [id34_t2.py, id35_t2.py],
        [id34_t2.pz, id35_t2.pz],
        color="black",
        linewidth=1.8,
        label="ID34 t2 to ID35 t2",
    )
    points.extend([id35_t2.origin])

    set_equal_bounds(ax, points)
    ax.legend(loc="upper left", fontsize=8)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
