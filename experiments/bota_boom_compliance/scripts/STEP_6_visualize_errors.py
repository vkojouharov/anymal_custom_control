#!/usr/bin/env python3
"""Step through measured and predicted compliance deflections in 3D."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import matplotlib.pyplot as plt
import numpy as np

from src.pipeline_paths import (
    CONVERTED_AXES_DATA_DIR,
    compliance_matrix_path,
    length_from_stem,
    select_input_csv,
)


DEFAULT_INPUT_PATTERN = "bota_*_shear_center_converted_axes.csv"
# Optional manual input. Leave as "" to use the newest matching converted_axes_data CSV.
# You can type either a filename like "bota_2p25m_shear_center_converted_axes.csv" or a path like "data/converted_axes_data/bota_2p25m_shear_center_converted_axes.csv".
MANUAL_INPUT_CSV = "bota_2p25m_shear_center_converted_axes.csv"

FORCE_NAMES = ("Fx_N", "Fy_N", "Fz_N")
MOMENT_NAMES = ("Mx_Nm", "My_Nm", "Mz_Nm")
DISPLACEMENT_NAMES = ("ux_m", "uy_m", "uz_m")
ROTATION_NAMES = ("theta_x_rad", "theta_y_rad", "theta_z_rad")
AXIS_COLORS = ("red", "green", "blue")


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def trial_stem_from_converted_axes(path: Path) -> str:
    suffix = "_shear_center_converted_axes"
    stem = path.stem
    if not stem.endswith(suffix):
        raise ValueError(f"input stem must end with {suffix!r}: {path.name}")
    return stem[: -len(suffix)]


def set_equal_axes(ax, limit: float) -> None:
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_box_aspect((1.0, 1.0, 1.0))


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


def draw_pose_frame(
    ax,
    *,
    position: np.ndarray,
    rotation_vector: np.ndarray,
    frame_scale: float,
    alpha: float,
    linewidth: float,
    label_prefix: str,
) -> None:
    rotation = rotvec_to_rotmat(rotation_vector)
    labels = (f"{label_prefix} +X", f"{label_prefix} +Y", f"{label_prefix} +Z")
    for axis_index, color, label in zip(range(3), AXIS_COLORS, labels, strict=True):
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
            label=label,
        )


def print_sample(
    *,
    index: int,
    sample_count: int,
    wrench: np.ndarray,
    measured: np.ndarray,
    predicted: np.ndarray,
    weighted_error_norm: float,
) -> None:
    residual = measured - predicted
    print()
    print(f"sample {index + 1}/{sample_count}")
    print("wrench:")
    for name, value in zip((*FORCE_NAMES, *MOMENT_NAMES), wrench, strict=True):
        print(f"  {name}: {float(value): .6e}")
    print("measured vs predicted:")
    for name, measured_value, predicted_value, residual_value in zip(
        (*DISPLACEMENT_NAMES, *ROTATION_NAMES),
        measured,
        predicted,
        residual,
        strict=True,
    ):
        print(
            f"  {name}: measured={float(measured_value): .6e} "
            f"predicted={float(predicted_value): .6e} residual={float(residual_value): .6e}"
        )
    print(f"weighted residual norm: {float(weighted_error_norm):.6e}")


def wait_for_enter(is_last_displayed_sample: bool) -> bool:
    if is_last_displayed_sample:
        input("End of data. Press Enter to close...")
        return False
    text = input("Press Enter for next sample, or type q then Enter to quit: ")
    return text.strip().lower() not in {"q", "quit", "exit"}


def visualize(
    *,
    data_csv: Path,
    compliance_csv: Path,
    boom_length_m: float,
    start_index: int,
    stride: int,
    axis_limit: float | None,
) -> None:
    data = np.loadtxt(data_csv, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 12:
        raise ValueError(f"expected at least 12 columns in {data_csv}, got {data.shape[1]}")

    wrench_all = data[:, 0:6]
    measured_all = data[:, 6:12]
    compliance = load_compliance(compliance_csv)
    predicted_all = (compliance @ wrench_all.T).T

    sqrt_h = np.diag([1.0, 1.0, 1.0, boom_length_m, boom_length_m, boom_length_m])
    weighted_residual = (sqrt_h @ (measured_all - predicted_all).T).T
    weighted_error_norm = np.linalg.norm(weighted_residual, axis=1)

    displacement_values = np.vstack([measured_all[:, 0:3], predicted_all[:, 0:3]])
    max_abs_displacement = float(np.max(np.abs(displacement_values))) if displacement_values.size else 0.0
    limit = axis_limit if axis_limit is not None else max(0.01, 1.2 * max_abs_displacement)
    frame_scale = 0.18 * limit

    print(f"data: {data_csv}")
    print(f"compliance: {compliance_csv}")
    print(f"boom length: {boom_length_m:g} m")
    print(f"samples: {data.shape[0]}")
    print("origin frame is fixed; actual pose frame is solid RGB; predicted pose frame is translucent RGB.")

    fig = plt.figure(figsize=(8.0, 7.0))
    ax = fig.add_subplot(111, projection="3d")
    plt.ion()

    indices = list(range(start_index, data.shape[0], stride))
    if not indices:
        raise ValueError(f"--start-index {start_index} is outside the data range 0..{data.shape[0] - 1}")

    for display_index, index in enumerate(indices):
        measured = measured_all[index]
        predicted = predicted_all[index]
        measured_disp = measured[0:3]
        predicted_disp = predicted[0:3]
        measured_rot = measured[3:6]
        predicted_rot = predicted[3:6]

        ax.clear()
        set_equal_axes(ax, limit)
        ax.scatter([0.0], [0.0], [0.0], color="black", s=40, label="origin")
        draw_pose_frame(
            ax,
            position=np.zeros(3),
            rotation_vector=np.zeros(3),
            frame_scale=frame_scale,
            alpha=0.22,
            linewidth=1.8,
            label_prefix="origin",
        )
        ax.plot(
            [0.0, measured_disp[0]],
            [0.0, measured_disp[1]],
            [0.0, measured_disp[2]],
            color="black",
            linewidth=1.6,
            label="actual deflection",
        )
        ax.plot(
            [0.0, predicted_disp[0]],
            [0.0, predicted_disp[1]],
            [0.0, predicted_disp[2]],
            color="gray",
            alpha=0.45,
            linewidth=1.6,
            linestyle="--",
            label="predicted deflection",
        )
        draw_pose_frame(
            ax,
            position=measured_disp,
            rotation_vector=measured_rot,
            frame_scale=frame_scale,
            alpha=1.0,
            linewidth=3.2,
            label_prefix="actual",
        )
        draw_pose_frame(
            ax,
            position=predicted_disp,
            rotation_vector=predicted_rot,
            frame_scale=frame_scale,
            alpha=0.30,
            linewidth=5.0,
            label_prefix="predicted",
        )

        ax.set_xlabel("x deflection (m)")
        ax.set_ylabel("y deflection (m)")
        ax.set_zlabel("z deflection (m)")
        ax.set_title(f"Compliance Prediction Error - sample {index + 1}/{data.shape[0]}")
        ax.legend(loc="upper left")
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

        print_sample(
            index=index,
            sample_count=data.shape[0],
            wrench=wrench_all[index],
            measured=measured,
            predicted=predicted,
            weighted_error_norm=weighted_error_norm[index],
        )
        if not wait_for_enter(display_index == len(indices) - 1):
            break

    plt.ioff()
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=None, help="Converted-axis CSV to visualize")
    parser.add_argument("--compliance", type=Path, default=None, help="Compliance matrix CSV to use")
    parser.add_argument("--boom-length-m", type=float, default=None, help="Override boom length used for residual scaling")
    parser.add_argument("--start-index", type=int, default=0, help="Zero-based sample index to start at")
    parser.add_argument("--stride", type=int, default=1, help="Step through every Nth sample")
    parser.add_argument("--axis-limit", type=float, default=None, help="Symmetric 3D axis limit in meters")
    args = parser.parse_args()

    if args.start_index < 0:
        raise ValueError("--start-index must be nonnegative")
    if args.stride <= 0:
        raise ValueError("--stride must be positive")

    data_csv = select_input_csv(args.input, MANUAL_INPUT_CSV, CONVERTED_AXES_DATA_DIR, DEFAULT_INPUT_PATTERN)
    compliance_csv = args.compliance.expanduser() if args.compliance is not None else compliance_matrix_path(data_csv)
    trial_stem = trial_stem_from_converted_axes(data_csv)
    boom_length_m = args.boom_length_m if args.boom_length_m is not None else length_from_stem(trial_stem)

    visualize(
        data_csv=data_csv,
        compliance_csv=compliance_csv,
        boom_length_m=boom_length_m,
        start_index=args.start_index,
        stride=args.stride,
        axis_limit=args.axis_limit,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
