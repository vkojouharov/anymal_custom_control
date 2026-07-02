#!/usr/bin/env python3
"""Fit a sparse beam-style compliance model from measured compliance matrices.

The simplified model keeps only these nonzero entries, using one-indexed
matrix names:

    c11 = a11 L
    c22 = a22 L^3
    c33 = a33 L^3
    c44 = a44 L
    c55 = a55
    c66 = a66
    c35 = c53 = a35 L^2
    c26 = c62 = a26 L^2

All other entries are set to zero. The length-scaled terms are fit with a
least-squares through-origin coefficient. The c55 and c66 terms are treated as
length-independent and are set to the average measured value.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

import numpy as np
import matplotlib.pyplot as plt

from src.pipeline_paths import COMPLIANCE_MATRIX_DIR, DATA_DIR, length_from_stem, trial_stem


DEFAULT_OUTPUT_DIR = DATA_DIR / "simplified_compliance_model"
COMPONENT_NAMES = ("ux", "uy", "uz", "theta_x", "theta_y", "theta_z")


@dataclass(frozen=True)
class MatrixSample:
    path: Path
    length_m: float
    matrix: np.ndarray


@dataclass(frozen=True)
class ModelTerm:
    name: str
    entries: tuple[tuple[int, int], ...]
    power: int
    fit_mode: str


MODEL_TERMS = (
    ModelTerm("a11", ((0, 0),), 1, "least_squares"),
    ModelTerm("a22", ((1, 1),), 3, "least_squares"),
    ModelTerm("a33", ((2, 2),), 3, "least_squares"),
    ModelTerm("a44", ((3, 3),), 1, "least_squares"),
    ModelTerm("a55", ((4, 4),), 0, "mean"),
    ModelTerm("a66", ((5, 5),), 0, "mean"),
    ModelTerm("a35", ((2, 4), (4, 2)), 2, "least_squares"),
    ModelTerm("a26", ((1, 5), (5, 1)), 2, "least_squares"),
)


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def sample_length(path: Path) -> float:
    suffix = "_compliance_matrix"
    stem = path.stem
    if stem.endswith(suffix):
        stem = stem[: -len(suffix)]
    return length_from_stem(stem)


def load_samples(matrix_dir: Path, pattern: str) -> list[MatrixSample]:
    paths = sorted(matrix_dir.glob(pattern), key=sample_length)
    if not paths:
        raise FileNotFoundError(f"no compliance matrices matching {matrix_dir / pattern}")
    return [MatrixSample(path=path, length_m=sample_length(path), matrix=load_compliance(path)) for path in paths]


def term_observations(samples: list[MatrixSample], term: ModelTerm) -> tuple[np.ndarray, np.ndarray]:
    x_values = []
    y_values = []
    for sample in samples:
        length_factor = sample.length_m**term.power
        for row, col in term.entries:
            x_values.append(length_factor)
            y_values.append(sample.matrix[row, col])
    return np.asarray(x_values, dtype=float), np.asarray(y_values, dtype=float)


def fit_coefficient(samples: list[MatrixSample], term: ModelTerm) -> float:
    x_values, y_values = term_observations(samples, term)
    if term.fit_mode == "mean":
        return float(np.mean(y_values))
    if term.fit_mode == "least_squares":
        denominator = float(x_values @ x_values)
        if denominator <= 0.0:
            raise ValueError(f"cannot fit {term.name}: zero length-factor denominator")
        return float((x_values @ y_values) / denominator)
    raise ValueError(f"unknown fit mode for {term.name}: {term.fit_mode}")


def fit_model(samples: list[MatrixSample]) -> dict[str, float]:
    return {term.name: fit_coefficient(samples, term) for term in MODEL_TERMS}


def reconstruct(coefficients: dict[str, float], length_m: float) -> np.ndarray:
    matrix = np.zeros((6, 6), dtype=float)
    for term in MODEL_TERMS:
        value = coefficients[term.name] * length_m**term.power
        for row, col in term.entries:
            matrix[row, col] = value
    return matrix


def power_template() -> np.ndarray:
    template = np.full((6, 6), "", dtype=object)
    for term in MODEL_TERMS:
        value = "a" if term.power == 0 else f"aL^{term.power}" if term.power > 1 else "aL"
        for row, col in term.entries:
            template[row, col] = value
    return template


def coefficient_matrix(coefficients: dict[str, float]) -> np.ndarray:
    matrix = np.zeros((6, 6), dtype=float)
    for term in MODEL_TERMS:
        for row, col in term.entries:
            matrix[row, col] = coefficients[term.name]
    return matrix


def save_string_matrix(path: Path, matrix: np.ndarray) -> None:
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(COMPONENT_NAMES)
        writer.writerows(matrix)


def save_outputs(output_dir: Path, samples: list[MatrixSample], coefficients: dict[str, float]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    header = ",".join(COMPONENT_NAMES)

    coefficients_csv = output_dir / "simplified_compliance_coefficients.csv"
    with coefficients_csv.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("coefficient", "entries_1_indexed", "length_power", "fit_mode", "value"))
        for term in MODEL_TERMS:
            entries = ";".join(f"c{row + 1}{col + 1}" for row, col in term.entries)
            writer.writerow((term.name, entries, term.power, term.fit_mode, f"{coefficients[term.name]:.18e}"))
    print(f"saved coefficients: {coefficients_csv}")

    coefficient_matrix_path = output_dir / "coefficient_matrix.csv"
    np.savetxt(coefficient_matrix_path, coefficient_matrix(coefficients), delimiter=",", header=header, comments="")
    print(f"saved coefficient matrix: {coefficient_matrix_path}")

    template_path = output_dir / "length_power_template.csv"
    save_string_matrix(template_path, power_template())
    print(f"saved length-power template: {template_path}")

    npz_path = output_dir / "simplified_compliance_model.npz"
    np.savez(
        npz_path,
        component_names=np.array(COMPONENT_NAMES),
        term_names=np.array([term.name for term in MODEL_TERMS]),
        powers=np.array([term.power for term in MODEL_TERMS], dtype=int),
        coefficients=np.array([coefficients[term.name] for term in MODEL_TERMS], dtype=float),
    )
    print(f"saved model archive: {npz_path}")

    predicted_dir = output_dir / "predicted_matrices_at_measured_lengths"
    predicted_dir.mkdir(parents=True, exist_ok=True)
    for sample in samples:
        predicted = reconstruct(coefficients, sample.length_m)
        path = predicted_dir / f"{trial_stem(sample.length_m)}_simplified_compliance_matrix.csv"
        np.savetxt(path, predicted, delimiter=",", header=header, comments="")
    print(f"saved simplified matrices: {predicted_dir}")


def plot_fits(output_dir: Path, samples: list[MatrixSample], coefficients: dict[str, float]) -> None:
    lengths = np.array([sample.length_m for sample in samples], dtype=float)
    length_grid = np.linspace(float(np.min(lengths)), float(np.max(lengths)), 300)

    cols = 4
    rows = math.ceil(len(MODEL_TERMS) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(4.1 * cols, 3.25 * rows), constrained_layout=True)
    flat_axes = np.asarray(axes).ravel()

    for ax, term in zip(flat_axes, MODEL_TERMS, strict=False):
        for entry_index, (row, col) in enumerate(term.entries):
            values = np.array([sample.matrix[row, col] for sample in samples], dtype=float)
            label = f"c{row + 1}{col + 1}"
            marker = "o" if entry_index == 0 else "x"
            ax.scatter(lengths, values, label=label, marker=marker)

        fit_values = coefficients[term.name] * length_grid**term.power
        length_scale = "1" if term.power == 0 else f"L^{term.power}"
        ax.plot(length_grid, fit_values, color="black", linewidth=1.6, label=f"{term.name} {length_scale}")
        ax.set_title(term.name)
        ax.set_xlabel("Boom length L (m)")
        ax.set_ylabel("Compliance")
        ax.grid(True, alpha=0.28)
        ax.legend(fontsize=8)

    for ax in flat_axes[len(MODEL_TERMS) :]:
        ax.axis("off")

    fig.suptitle("Simplified Boom Compliance Fits")
    plot_path = output_dir / "simplified_compliance_fit_scatter.png"
    fig.savefig(plot_path, dpi=200)
    plt.close(fig)
    print(f"saved fit plot: {plot_path}")


def print_summary(samples: list[MatrixSample], coefficients: dict[str, float]) -> None:
    print("simplified compliance model:")
    for term in MODEL_TERMS:
        entries = ", ".join(f"c{row + 1}{col + 1}" for row, col in term.entries)
        length_scale = "1" if term.power == 0 else f"L^{term.power}"
        print(f"  {entries}: {term.name} * {length_scale} = {coefficients[term.name]:.6e} * {length_scale}")

    print("residuals at measured lengths:")
    for sample in samples:
        predicted = reconstruct(coefficients, sample.length_m)
        residual = predicted - sample.matrix
        retained_mask = predicted != 0.0
        retained_residual = residual[retained_mask]
        sparse_error = residual.copy()
        sparse_error[retained_mask] = 0.0
        print(
            f"  {sample.length_m:g} m: "
            f"full_fro={np.linalg.norm(residual, ord='fro'):.6e}, "
            f"retained_max_abs={np.max(np.abs(retained_residual)):.6e}, "
            f"zeroed_fro={np.linalg.norm(sparse_error, ord='fro'):.6e}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--matrix-dir", type=Path, default=COMPLIANCE_MATRIX_DIR)
    parser.add_argument("--pattern", default="bota_*_compliance_matrix.csv")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()

    samples = load_samples(args.matrix_dir.expanduser(), args.pattern)
    print("loaded matrices:")
    for sample in samples:
        print(f"  {sample.length_m:g} m: {sample.path}")

    coefficients = fit_model(samples)
    print_summary(samples, coefficients)
    output_dir = args.output_dir.expanduser()
    save_outputs(output_dir, samples, coefficients)
    plot_fits(output_dir, samples, coefficients)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
