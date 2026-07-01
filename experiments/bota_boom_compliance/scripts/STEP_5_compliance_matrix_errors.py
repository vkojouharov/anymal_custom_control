#!/usr/bin/env python3
"""Plot compliance-matrix residuals for converted boom data."""

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
    COMPLIANCE_MATRIX_DIR,
    CONVERTED_AXES_DATA_DIR,
    DATA_DIR,
    compliance_matrix_path,
    length_from_stem,
    select_input_csv,
)


DEFAULT_INPUT_PATTERN = "bota_*_shear_center_converted_axes.csv"
# Optional manual input. Leave as "" to use the newest matching converted_axes_data CSV.
# You can type either a filename like "bota_2p25m_shear_center_converted_axes.csv" or a path like "data/converted_axes_data/bota_2p25m_shear_center_converted_axes.csv".
MANUAL_INPUT_CSV = "bota_0p75m_shear_center_converted_axes.csv"
ERROR_PLOT_DIR = DATA_DIR / "compliance_matrix_error_plots"
DEFAULT_X_LIMITS = (0.0, 2.0)

COMPONENT_NAMES = ("ux_m", "uy_m", "uz_m", "theta_x_rad", "theta_y_rad", "theta_z_rad")


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def converted_axes_path_from_stem(stem: str) -> Path:
    return CONVERTED_AXES_DATA_DIR / f"{stem}_shear_center_converted_axes.csv"


def compliance_path_from_stem(stem: str) -> Path:
    return COMPLIANCE_MATRIX_DIR / f"{stem}_compliance_matrix.csv"


def trial_stem_from_converted_axes(path: Path) -> str:
    suffix = "_shear_center_converted_axes"
    stem = path.stem
    if not stem.endswith(suffix):
        raise ValueError(f"input stem must end with {suffix!r}: {path.name}")
    return stem[: -len(suffix)]


def default_output_path(data_csv: Path) -> Path:
    trial_stem = trial_stem_from_converted_axes(data_csv)
    return ERROR_PLOT_DIR / f"{trial_stem}_compliance_matrix_error.png"


def plot_one(
    *,
    data_csv: Path,
    compliance_csv: Path,
    output: Path,
    boom_length_m: float,
    x_limits: tuple[float, float],
    show: bool = False,
) -> None:
    data = np.loadtxt(data_csv, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 12:
        raise ValueError(f"expected at least 12 columns in {data_csv}, got {data.shape[1]}")

    wrench = data[:, 0:6].T
    measured = data[:, 6:12].T
    compliance = load_compliance(compliance_csv)

    predicted = compliance @ wrench
    residual = measured - predicted
    sqrt_h = np.diag([1.0, 1.0, 1.0, boom_length_m, boom_length_m, boom_length_m])
    weighted_residual = sqrt_h @ residual
    weighted_error_norm = np.linalg.norm(weighted_residual, axis=0)
    per_sample_cost = weighted_error_norm**2

    fig, (hist_ax, box_ax) = plt.subplots(
        2,
        1,
        figsize=(8.0, 4.8),
        sharex=True,
        gridspec_kw={"height_ratios": [3.0, 0.65], "hspace": 0.06},
    )

    hist_ax.hist(
        weighted_error_norm,
        bins="auto",
        color="#6aaed6",
        edgecolor="white",
        alpha=0.85,
    )
    hist_ax.axvline(np.median(weighted_error_norm), color="#f28e2b", linewidth=2.0, label="median")
    hist_ax.axvline(np.mean(weighted_error_norm), color="#59a14f", linewidth=2.0, linestyle="--", label="mean")
    hist_ax.set_ylabel("count")
    hist_ax.set_title("Compliance Matrix Residuals")
    hist_ax.grid(True, axis="y", alpha=0.30)
    hist_ax.legend(loc="upper right")

    box_ax.boxplot(
        weighted_error_norm,
        vert=False,
        widths=0.45,
        showmeans=True,
        patch_artist=True,
        boxprops={"facecolor": "#d7e8f5", "edgecolor": "black"},
        medianprops={"color": "#f28e2b", "linewidth": 2.0},
        meanprops={"marker": "^", "markerfacecolor": "#59a14f", "markeredgecolor": "#59a14f"},
    )
    box_ax.scatter(
        weighted_error_norm,
        np.ones_like(weighted_error_norm),
        color="#1f77b4",
        alpha=0.55,
        s=26,
    )
    box_ax.set_xlim(*x_limits)
    box_ax.set_yticks([])
    box_ax.set_xlabel(r"weighted residual norm $\|H^{1/2}(X - CW)\|_2$")
    box_ax.grid(True, axis="x", alpha=0.30)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")

    rmse = np.sqrt(np.mean(residual**2, axis=1))
    weighted_rmse = np.sqrt(np.mean(weighted_residual**2, axis=1))

    print(f"data: {data_csv}")
    print(f"compliance: {compliance_csv}")
    print(f"boom length: {boom_length_m:g} m")
    print(f"samples: {weighted_error_norm.size}")
    print(f"total weighted cost: {float(np.sum(per_sample_cost)):.6e}")
    print(
        "weighted error norm min/median/mean/max: "
        f"{float(np.min(weighted_error_norm)):.6e} / "
        f"{float(np.median(weighted_error_norm)):.6e} / "
        f"{float(np.mean(weighted_error_norm)):.6e} / "
        f"{float(np.max(weighted_error_norm)):.6e}"
    )
    print("component RMSE:")
    for name, raw_value, weighted_value in zip(COMPONENT_NAMES, rmse, weighted_rmse, strict=True):
        print(f"  {name}: {float(raw_value):.6e} weighted={float(weighted_value):.6e}")
    print(f"saved: {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=None, help="Converted-axis CSV to evaluate")
    parser.add_argument("--compliance", type=Path, default=None, help="Compliance matrix CSV to evaluate")
    parser.add_argument("--output", type=Path, default=None, help="Output plot path")
    parser.add_argument(
        "--datasets",
        nargs="+",
        default=None,
        help="Trial stems such as bota_2p25m. Overrides --input/--compliance/--output.",
    )
    parser.add_argument("--boom-length-m", type=float, default=None, help="Override boom length used for residual scaling")
    parser.add_argument("--x-min", type=float, default=DEFAULT_X_LIMITS[0])
    parser.add_argument("--x-max", type=float, default=DEFAULT_X_LIMITS[1])
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    x_limits = (args.x_min, args.x_max)

    if args.datasets is not None:
        for dataset_stem in args.datasets:
            data_csv = converted_axes_path_from_stem(dataset_stem)
            plot_one(
                data_csv=data_csv,
                compliance_csv=compliance_path_from_stem(dataset_stem),
                output=default_output_path(data_csv),
                boom_length_m=args.boom_length_m if args.boom_length_m is not None else length_from_stem(dataset_stem),
                x_limits=x_limits,
                show=args.show,
            )
        return 0

    data_csv = select_input_csv(args.input, MANUAL_INPUT_CSV, CONVERTED_AXES_DATA_DIR, DEFAULT_INPUT_PATTERN)
    compliance_csv = args.compliance.expanduser() if args.compliance is not None else compliance_matrix_path(data_csv)
    output = args.output.expanduser() if args.output is not None else default_output_path(data_csv)
    trial_stem = trial_stem_from_converted_axes(data_csv)
    boom_length_m = args.boom_length_m if args.boom_length_m is not None else length_from_stem(trial_stem)

    plot_one(
        data_csv=data_csv,
        compliance_csv=compliance_csv,
        output=output,
        boom_length_m=boom_length_m,
        x_limits=x_limits,
        show=args.show,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
