#!/usr/bin/env python3
"""Evaluate the simplified boom compliance model against converted raw data.

This uses the same residual definition as scripts/STEP_5_compliance_matrix_errors.py:

    residual = measured_deflection - C(L) @ wrench

where C(L) is reconstructed from the simplified compliance coefficients and the
boom length parsed from each converted-axis CSV filename. The plotted error is
the norm of the residual after multiplying rotations by the boom length.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import matplotlib.pyplot as plt
import numpy as np

from src.pipeline_paths import CONVERTED_AXES_DATA_DIR, DATA_DIR, length_from_stem


DEFAULT_INPUT_PATTERN = "bota_*_shear_center_converted_axes.csv"
DEFAULT_MODEL_DIR = DATA_DIR / "simplified_compliance_model"
DEFAULT_COEFFICIENT_CSV = DEFAULT_MODEL_DIR / "simplified_compliance_coefficients.csv"
DEFAULT_OUTPUT_DIR = DEFAULT_MODEL_DIR / "raw_data_error_plots"
DEFAULT_SUMMARY_CSV = DEFAULT_MODEL_DIR / "raw_data_error_summary.csv"
DEFAULT_X_LIMITS = (0.0, 2.0)

COMPONENT_NAMES = ("ux_m", "uy_m", "uz_m", "theta_x_rad", "theta_y_rad", "theta_z_rad")


@dataclass(frozen=True)
class ModelTerm:
    name: str
    entries: tuple[tuple[int, int], ...]
    power: int


@dataclass(frozen=True)
class DatasetResult:
    stem: str
    length_m: float
    sample_count: int
    total_weighted_cost: float
    min_weighted_error: float
    median_weighted_error: float
    mean_weighted_error: float
    max_weighted_error: float
    component_rmse: np.ndarray
    weighted_component_rmse: np.ndarray


MODEL_TERMS = (
    ModelTerm("a11", ((0, 0),), 1),
    ModelTerm("a22", ((1, 1),), 3),
    ModelTerm("a33", ((2, 2),), 3),
    ModelTerm("a44", ((3, 3),), 1),
    ModelTerm("a55", ((4, 4),), 0),
    ModelTerm("a66", ((5, 5),), 0),
    ModelTerm("a35", ((2, 4), (4, 2)), 2),
    ModelTerm("a26", ((1, 5), (5, 1)), 2),
)


def trial_stem_from_converted_axes(path: Path) -> str:
    suffix = "_shear_center_converted_axes"
    stem = path.stem
    if not stem.endswith(suffix):
        raise ValueError(f"input stem must end with {suffix!r}: {path.name}")
    return stem[: -len(suffix)]


def load_coefficients(path: Path) -> dict[str, float]:
    coefficients: dict[str, float] = {}
    with path.open(newline="") as file:
        reader = csv.DictReader(file)
        for row in reader:
            coefficients[row["coefficient"]] = float(row["value"])

    missing = [term.name for term in MODEL_TERMS if term.name not in coefficients]
    if missing:
        raise ValueError(f"missing simplified coefficients in {path}: {', '.join(missing)}")
    return coefficients


def simplified_compliance_matrix(coefficients: dict[str, float], length_m: float) -> np.ndarray:
    matrix = np.zeros((6, 6), dtype=float)
    for term in MODEL_TERMS:
        value = coefficients[term.name] * length_m**term.power
        for row, col in term.entries:
            matrix[row, col] = value
    return matrix


def load_converted_data(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    if data.shape[1] < 12:
        raise ValueError(f"expected at least 12 columns in {path}, got {data.shape[1]}")
    return data[:, 0:6].T, data[:, 6:12].T


def residuals_for_dataset(
    *,
    data_csv: Path,
    coefficients: dict[str, float],
    boom_length_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    wrench, measured = load_converted_data(data_csv)
    compliance = simplified_compliance_matrix(coefficients, boom_length_m)
    predicted = compliance @ wrench
    residual = measured - predicted
    sqrt_h = np.diag([1.0, 1.0, 1.0, boom_length_m, boom_length_m, boom_length_m])
    weighted_residual = sqrt_h @ residual
    weighted_error_norm = np.linalg.norm(weighted_residual, axis=0)
    return residual, weighted_residual, weighted_error_norm, compliance


def plot_error_distribution(
    *,
    output: Path,
    stem: str,
    boom_length_m: float,
    weighted_error_norm: np.ndarray,
    x_limits: tuple[float, float],
    show: bool,
) -> None:
    fig, (hist_ax, box_ax) = plt.subplots(
        2,
        1,
        figsize=(8.0, 4.8),
        sharex=True,
        gridspec_kw={"height_ratios": [3.0, 0.65], "hspace": 0.06},
    )

    hist_ax.hist(weighted_error_norm, bins="auto", color="#9c755f", edgecolor="white", alpha=0.85)
    hist_ax.axvline(np.median(weighted_error_norm), color="#f28e2b", linewidth=2.0, label="median")
    hist_ax.axvline(np.mean(weighted_error_norm), color="#59a14f", linewidth=2.0, linestyle="--", label="mean")
    hist_ax.set_ylabel("count")
    hist_ax.set_title(f"Simplified Compliance Raw-Data Residuals - {stem} ({boom_length_m:g} m)")
    hist_ax.grid(True, axis="y", alpha=0.30)
    hist_ax.legend(loc="upper right")

    box_ax.boxplot(
        weighted_error_norm,
        vert=False,
        widths=0.45,
        showmeans=True,
        patch_artist=True,
        boxprops={"facecolor": "#eadbd3", "edgecolor": "black"},
        medianprops={"color": "#f28e2b", "linewidth": 2.0},
        meanprops={"marker": "^", "markerfacecolor": "#59a14f", "markeredgecolor": "#59a14f"},
    )
    box_ax.scatter(weighted_error_norm, np.ones_like(weighted_error_norm), color="#8c564b", alpha=0.55, s=26)
    box_ax.set_xlim(*x_limits)
    box_ax.set_yticks([])
    box_ax.set_xlabel(r"weighted residual norm $\|H^{1/2}(X - C_{simple}(L)W)\|_2$")
    box_ax.grid(True, axis="x", alpha=0.30)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")
    if show:
        plt.show()
    else:
        plt.close(fig)


def evaluate_one(
    *,
    data_csv: Path,
    coefficients: dict[str, float],
    output_dir: Path,
    x_limits: tuple[float, float],
    show: bool,
) -> DatasetResult:
    stem = trial_stem_from_converted_axes(data_csv)
    boom_length_m = length_from_stem(stem)
    residual, weighted_residual, weighted_error_norm, _compliance = residuals_for_dataset(
        data_csv=data_csv,
        coefficients=coefficients,
        boom_length_m=boom_length_m,
    )

    output = output_dir / f"{stem}_simplified_compliance_error.png"
    plot_error_distribution(
        output=output,
        stem=stem,
        boom_length_m=boom_length_m,
        weighted_error_norm=weighted_error_norm,
        x_limits=x_limits,
        show=show,
    )

    component_rmse = np.sqrt(np.mean(residual**2, axis=1))
    weighted_component_rmse = np.sqrt(np.mean(weighted_residual**2, axis=1))
    result = DatasetResult(
        stem=stem,
        length_m=boom_length_m,
        sample_count=weighted_error_norm.size,
        total_weighted_cost=float(np.sum(weighted_error_norm**2)),
        min_weighted_error=float(np.min(weighted_error_norm)),
        median_weighted_error=float(np.median(weighted_error_norm)),
        mean_weighted_error=float(np.mean(weighted_error_norm)),
        max_weighted_error=float(np.max(weighted_error_norm)),
        component_rmse=component_rmse,
        weighted_component_rmse=weighted_component_rmse,
    )

    print(f"{stem} ({boom_length_m:g} m)")
    print(f"  samples: {result.sample_count}")
    print(f"  weighted cost: {result.total_weighted_cost:.6e}")
    print(
        "  weighted error norm min/median/mean/max: "
        f"{result.min_weighted_error:.6e} / "
        f"{result.median_weighted_error:.6e} / "
        f"{result.mean_weighted_error:.6e} / "
        f"{result.max_weighted_error:.6e}"
    )
    print(f"  saved: {output}")
    return result


def write_summary(path: Path, results: list[DatasetResult]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        header = [
            "stem",
            "length_m",
            "sample_count",
            "total_weighted_cost",
            "min_weighted_error",
            "median_weighted_error",
            "mean_weighted_error",
            "max_weighted_error",
        ]
        header += [f"{name}_rmse" for name in COMPONENT_NAMES]
        header += [f"{name}_weighted_rmse" for name in COMPONENT_NAMES]
        writer.writerow(header)
        for result in results:
            writer.writerow(
                [
                    result.stem,
                    f"{result.length_m:.18e}",
                    result.sample_count,
                    f"{result.total_weighted_cost:.18e}",
                    f"{result.min_weighted_error:.18e}",
                    f"{result.median_weighted_error:.18e}",
                    f"{result.mean_weighted_error:.18e}",
                    f"{result.max_weighted_error:.18e}",
                    *[f"{value:.18e}" for value in result.component_rmse],
                    *[f"{value:.18e}" for value in result.weighted_component_rmse],
                ]
            )
    print(f"saved summary: {path}")


def plot_summary(output_dir: Path, results: list[DatasetResult]) -> None:
    lengths = np.array([result.length_m for result in results], dtype=float)
    medians = np.array([result.median_weighted_error for result in results], dtype=float)
    means = np.array([result.mean_weighted_error for result in results], dtype=float)
    maxes = np.array([result.max_weighted_error for result in results], dtype=float)

    fig, ax = plt.subplots(figsize=(8.0, 4.4), constrained_layout=True)
    ax.plot(lengths, medians, marker="o", label="median")
    ax.plot(lengths, means, marker="s", label="mean")
    ax.plot(lengths, maxes, marker="^", label="max")
    ax.set_xlabel("Boom length L (m)")
    ax.set_ylabel(r"weighted residual norm $\|H^{1/2}(X - C_{simple}(L)W)\|_2$")
    ax.set_title("Simplified Compliance Raw-Data Error Summary")
    ax.grid(True, alpha=0.30)
    ax.legend()

    output = output_dir / "simplified_compliance_raw_data_error_summary.png"
    fig.savefig(output, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(f"saved summary plot: {output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=CONVERTED_AXES_DATA_DIR)
    parser.add_argument("--pattern", default=DEFAULT_INPUT_PATTERN)
    parser.add_argument("--coefficients", type=Path, default=DEFAULT_COEFFICIENT_CSV)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--summary-csv", type=Path, default=DEFAULT_SUMMARY_CSV)
    parser.add_argument("--datasets", nargs="+", default=None, help="Trial stems such as bota_2p25m")
    parser.add_argument("--x-min", type=float, default=DEFAULT_X_LIMITS[0])
    parser.add_argument("--x-max", type=float, default=DEFAULT_X_LIMITS[1])
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    coefficients = load_coefficients(args.coefficients.expanduser())
    data_dir = args.data_dir.expanduser()
    if args.datasets is None:
        data_paths = sorted(data_dir.glob(args.pattern), key=lambda path: length_from_stem(trial_stem_from_converted_axes(path)))
    else:
        data_paths = [data_dir / f"{stem}_shear_center_converted_axes.csv" for stem in args.datasets]

    if not data_paths:
        raise FileNotFoundError(f"no converted-axis data matching {data_dir / args.pattern}")

    output_dir = args.output_dir.expanduser()
    x_limits = (args.x_min, args.x_max)
    results = [
        evaluate_one(
            data_csv=path,
            coefficients=coefficients,
            output_dir=output_dir,
            x_limits=x_limits,
            show=args.show,
        )
        for path in data_paths
    ]
    write_summary(args.summary_csv.expanduser(), results)
    plot_summary(output_dir, results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
