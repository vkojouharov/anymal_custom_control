#!/usr/bin/env python3
"""Plot compliance trends versus boom length for constrained 5N fits."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
OUTPUT = Path(__file__).resolve().with_name("compliance_trends_5N_constrained.png")
SHOW_PLOT = True
BENDING_FIT_MODEL = "cubic_origin"
TORSION_FIT_MODEL = "linear_origin"

DATASETS = [
    "data_1m_5N",
    "data_1p25m_5N",
    "data_1p5m_5N",
    "data_1p75m_5N",
    "data_2m_5N",
    "data_2p25m_5N",
]


def length_from_dataset(dataset: str) -> float:
    length_text = dataset.removeprefix("data_").split("m_", maxsplit=1)[0]
    return float(length_text.replace("p", "."))


def load_compliance(dataset: str) -> np.ndarray:
    path = EXPERIMENT_DIR / f"{dataset}_compliance_constrained.csv"
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def fit_through_origin(x: np.ndarray, y: np.ndarray, basis: np.ndarray) -> float:
    return float(np.dot(basis, y) / np.dot(basis, basis))


def fit_polynomial(x: np.ndarray, y: np.ndarray, powers: list[int]) -> np.ndarray:
    design = np.column_stack([x**power for power in powers])
    return np.linalg.lstsq(design, y, rcond=None)[0]


def evaluate_polynomial(x: np.ndarray, coefficients: np.ndarray, powers: list[int]) -> np.ndarray:
    values = np.zeros_like(x, dtype=float)
    for coefficient, power in zip(coefficients, powers):
        values += coefficient * x**power
    return values


def fit_model(x: np.ndarray, y: np.ndarray, model: str) -> tuple[np.ndarray, str, str]:
    model_powers = {
        "quadratic_origin": [2],
        "quadratic_offset": [2, 0],
        "quadratic_full": [2, 1, 0],
        "cubic_origin": [3],
        "cubic_offset": [3, 0],
        "cubic_timoshenko": [3, 1, 0],
        "linear_origin": [1],
        "linear_offset": [1, 0],
    }
    if model not in model_powers:
        raise ValueError(f"unknown fit model {model!r}; choose one of {sorted(model_powers)}")

    powers = model_powers[model]
    coefficients = fit_polynomial(x, y, powers)
    terms = []
    for coefficient, power in zip(coefficients, powers):
        if power == 0:
            terms.append(f"{coefficient:.3g}")
        elif power == 1:
            terms.append(f"{coefficient:.3g}L")
        else:
            terms.append(rf"{coefficient:.3g}L^{power}")
    return coefficients, " + ".join(terms), model


def main() -> int:
    lengths = np.array([length_from_dataset(dataset) for dataset in DATASETS])
    matrices = [load_compliance(dataset) for dataset in DATASETS]

    y_bending = 100.0 * np.array([matrix[1, 1] for matrix in matrices])
    z_bending = 100.0 * np.array([matrix[2, 2] for matrix in matrices])
    x_torsion = np.array([matrix[3, 3] for matrix in matrices])

    x_fit = np.linspace(float(np.min(lengths)), float(np.max(lengths)), 300)
    y_bending_coeffs, y_bending_label, _ = fit_model(lengths, y_bending, BENDING_FIT_MODEL)
    z_bending_coeffs, z_bending_label, _ = fit_model(lengths, z_bending, BENDING_FIT_MODEL)
    x_torsion_coeffs, x_torsion_label, _ = fit_model(lengths, x_torsion, TORSION_FIT_MODEL)
    bending_powers = {
        "quadratic_origin": [2],
        "quadratic_offset": [2, 0],
        "quadratic_full": [2, 1, 0],
        "cubic_origin": [3],
        "cubic_offset": [3, 0],
        "cubic_timoshenko": [3, 1, 0],
    }[BENDING_FIT_MODEL]
    torsion_powers = {
        "linear_origin": [1],
        "linear_offset": [1, 0],
    }[TORSION_FIT_MODEL]

    green = "#2f7d32"
    blue = "#2459a7"
    red = "#b83232"

    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.labelsize": 12,
            "legend.fontsize": 10,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
        }
    )
    fig, ax = plt.subplots(figsize=(8.2, 5.4))

    ax.scatter(lengths, y_bending, color=green, s=54, label=r"y-bending $100 C_{22}$", zorder=3)
    ax.plot(
        x_fit,
        evaluate_polynomial(x_fit, y_bending_coeffs, bending_powers),
        color=green,
        linestyle="--",
        linewidth=2.0,
        label=rf"y-bending fit ${y_bending_label}$",
    )

    ax.scatter(lengths, z_bending, color=blue, s=54, label=r"z-bending $100 C_{33}$", zorder=3)
    ax.plot(
        x_fit,
        evaluate_polynomial(x_fit, z_bending_coeffs, bending_powers),
        color=blue,
        linestyle="--",
        linewidth=2.0,
        label=rf"z-bending fit ${z_bending_label}$",
    )

    ax.scatter(lengths, x_torsion, color=red, s=54, label=r"x-torsion $C_{44}$", zorder=3)
    ax.plot(
        x_fit,
        evaluate_polynomial(x_fit, x_torsion_coeffs, torsion_powers),
        color=red,
        linestyle="--",
        linewidth=2.0,
        label=rf"x-torsion fit ${x_torsion_label}$",
    )

    ax.set_title("Constrained Compliance Trends at 5N")
    ax.set_xlabel("Boom length L [m]")
    ax.set_ylabel("Compliance value")
    ax.set_xticks(lengths)
    ax.grid(True, alpha=0.30)
    ax.legend(loc="best", frameon=True)

    fig.tight_layout()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT, dpi=180)
    print(f"saved: {OUTPUT}")
    print(f"bending fit model: {BENDING_FIT_MODEL}")
    print(f"torsion fit model: {TORSION_FIT_MODEL}")
    print(f"y-bending fit: y = {y_bending_label}")
    print(f"z-bending fit: y = {z_bending_label}")
    print(f"x-torsion fit: y = {x_torsion_label}")
    if SHOW_PLOT:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
