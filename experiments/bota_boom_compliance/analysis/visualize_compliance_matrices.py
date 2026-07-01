#!/usr/bin/env python3
"""Visualize predicted deflection frames for all fitted compliance matrices."""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Button, Slider

from src.pipeline_paths import COMPLIANCE_MATRIX_DIR, length_from_stem


WRENCH_LABELS = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")
WRENCH_UNITS = ("N", "N", "N", "Nm", "Nm", "Nm")
AXIS_COLORS = ("red", "green", "blue")
ORIGIN_FRAME_ALPHA = 0.22
DEFLECTED_FRAME_ALPHA = 0.95


@dataclass(frozen=True)
class ComplianceMatrix:
    path: Path
    label: str
    length_m: float
    matrix: np.ndarray


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def matrix_label(path: Path) -> tuple[str, float]:
    suffix = "_compliance_matrix"
    stem = path.stem
    trial_stem = stem[: -len(suffix)] if stem.endswith(suffix) else stem
    length_m = length_from_stem(trial_stem)
    return f"{length_m:g} m", length_m


def load_matrices(directory: Path, pattern: str) -> list[ComplianceMatrix]:
    paths = sorted(
        directory.glob(pattern),
        key=lambda path: matrix_label(path)[1],
    )
    if not paths:
        raise FileNotFoundError(f"no compliance matrices matching {directory / pattern}")

    matrices = []
    for path in paths:
        label, length_m = matrix_label(path)
        matrices.append(
            ComplianceMatrix(
                path=path,
                label=label,
                length_m=length_m,
                matrix=load_compliance(path),
            )
        )
    return matrices


def rotvec_to_rotmat(rotvec: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return np.eye(3)

    axis = rotvec / angle
    x, y, z = axis
    skew = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=float,
    )
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def set_equal_axes(ax, limit: float) -> None:
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_box_aspect((1.0, 1.0, 1.0))


def draw_pose_frame(
    ax,
    *,
    position: np.ndarray,
    rotation_vector: np.ndarray,
    frame_scale: float,
    alpha: float,
    linewidth: float,
) -> None:
    rotation = rotvec_to_rotmat(rotation_vector)
    for axis_index, color in zip(range(3), AXIS_COLORS, strict=True):
        direction = rotation[:, axis_index] * frame_scale
        ax.quiver(
            position[0],
            position[1],
            position[2],
            direction[0],
            direction[1],
            direction[2],
            color=color,
            alpha=alpha,
            linewidth=linewidth,
            arrow_length_ratio=0.18,
        )


def predict(matrix: np.ndarray, wrench: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    deflection = matrix @ wrench
    return deflection[0:3], deflection[3:6]


def default_axis_limit(matrices: list[ComplianceMatrix], wrench_ranges: np.ndarray) -> float:
    max_displacement = 0.0
    for item in matrices:
        translation_map = item.matrix[0:3, :]
        # For a linear map over a box, the maximum absolute coordinate is bounded
        # by the row-wise absolute weighted sum.
        max_for_matrix = float(np.max(np.abs(translation_map) @ wrench_ranges))
        max_displacement = max(max_displacement, max_for_matrix)
    return max(0.01, 1.15 * max_displacement)


def slider_axis(fig, index: int, count: int):
    left = 0.10
    right = 0.88
    top = 0.19
    bottom = 0.035
    gap = 0.012
    height = (top - bottom - gap * (count - 1)) / count
    y = top - (index + 1) * height - index * gap
    return fig.add_axes([left, y, right - left, height])


def draw_dashboard(
    *,
    matrices: list[ComplianceMatrix],
    force_range: float,
    torque_range: float,
    axis_limit: float | None,
) -> None:
    wrench_ranges = np.array(
        [force_range, force_range, force_range, torque_range, torque_range, torque_range],
        dtype=float,
    )
    limit = axis_limit if axis_limit is not None else default_axis_limit(matrices, wrench_ranges)
    frame_scale = 0.20 * limit

    count = len(matrices)
    cols = math.ceil(math.sqrt(count))
    rows = math.ceil(count / cols)
    fig = plt.figure(figsize=(3.4 * cols, 3.25 * rows + 2.1))
    fig.subplots_adjust(left=0.03, right=0.98, top=0.94, bottom=0.27, wspace=0.08, hspace=0.16)
    fig.suptitle("Compliance Matrix Predicted Deflection", fontsize=14)

    axes = [fig.add_subplot(rows, cols, idx + 1, projection="3d") for idx in range(count)]
    unused_slots = rows * cols - count
    for idx in range(unused_slots):
        ax = fig.add_subplot(rows, cols, count + idx + 1)
        ax.axis("off")

    slider_ranges = {
        "Fx": force_range,
        "Fy": force_range,
        "Fz": force_range,
        "Tx": torque_range,
        "Ty": torque_range,
        "Tz": torque_range,
    }
    sliders = []
    for idx, (label, unit) in enumerate(zip(WRENCH_LABELS, WRENCH_UNITS, strict=True)):
        max_value = slider_ranges[label]
        slider = Slider(
            ax=slider_axis(fig, idx, len(WRENCH_LABELS)),
            label=f"{label} ({unit})",
            valmin=-max_value,
            valmax=max_value,
            valinit=0.0,
            valstep=max_value / 200.0 if max_value > 0.0 else None,
        )
        sliders.append(slider)

    reset_ax = fig.add_axes([0.90, 0.035, 0.08, 0.035])
    reset_button = Button(reset_ax, "Reset")

    def current_wrench() -> np.ndarray:
        return np.array([slider.val for slider in sliders], dtype=float)

    def redraw(_event=None) -> None:
        wrench = current_wrench()
        for ax, item in zip(axes, matrices, strict=True):
            position, rotation_vector = predict(item.matrix, wrench)
            ax.clear()
            set_equal_axes(ax, limit)
            ax.scatter([0.0], [0.0], [0.0], color="black", s=10)
            ax.plot(
                [0.0, position[0]],
                [0.0, position[1]],
                [0.0, position[2]],
                color="black",
                linewidth=1.0,
                alpha=0.65,
            )
            draw_pose_frame(
                ax,
                position=np.zeros(3),
                rotation_vector=np.zeros(3),
                frame_scale=frame_scale,
                alpha=ORIGIN_FRAME_ALPHA,
                linewidth=1.4,
            )
            draw_pose_frame(
                ax,
                position=position,
                rotation_vector=rotation_vector,
                frame_scale=frame_scale,
                alpha=DEFLECTED_FRAME_ALPHA,
                linewidth=2.3,
            )
            ax.set_title(
                f"{item.label}\n"
                f"u=({position[0]:+.3e}, {position[1]:+.3e}, {position[2]:+.3e}) m\n"
                f"r=({rotation_vector[0]:+.3e}, {rotation_vector[1]:+.3e}, {rotation_vector[2]:+.3e}) rad",
                fontsize=8,
            )
            ax.set_xlabel("x (m)", fontsize=7)
            ax.set_ylabel("y (m)", fontsize=7)
            ax.set_zlabel("z (m)", fontsize=7)
            ax.tick_params(axis="both", labelsize=6, pad=0)
        fig.canvas.draw_idle()

    def reset(_event) -> None:
        for slider in sliders:
            slider.reset()

    for slider in sliders:
        slider.on_changed(redraw)
    reset_button.on_clicked(reset)

    redraw()
    print("Loaded compliance matrices:")
    for item in matrices:
        print(f"  {item.label}: {item.path}")
    print("Slider order is Fx, Fy, Fz, Tx, Ty, Tz; torques are applied as the matrix Mx, My, Mz inputs.")
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--matrix-dir",
        type=Path,
        default=COMPLIANCE_MATRIX_DIR,
        help="Directory containing bota_*_compliance_matrix.csv files",
    )
    parser.add_argument(
        "--pattern",
        default="bota_*_compliance_matrix.csv",
        help="Glob pattern used inside --matrix-dir",
    )
    parser.add_argument("--force-range", type=float, default=30.0, help="Absolute Fx/Fy/Fz slider range in N")
    parser.add_argument("--torque-range", type=float, default=5.0, help="Absolute Tx/Ty/Tz slider range in Nm")
    parser.add_argument("--axis-limit", type=float, default=None, help="Fixed symmetric 3D axis limit in meters")
    args = parser.parse_args()

    if args.force_range <= 0.0:
        raise ValueError("--force-range must be positive")
    if args.torque_range <= 0.0:
        raise ValueError("--torque-range must be positive")
    if args.axis_limit is not None and args.axis_limit <= 0.0:
        raise ValueError("--axis-limit must be positive")

    matrices = load_matrices(args.matrix_dir.expanduser(), args.pattern)
    draw_dashboard(
        matrices=matrices,
        force_range=args.force_range,
        torque_range=args.torque_range,
        axis_limit=args.axis_limit,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
