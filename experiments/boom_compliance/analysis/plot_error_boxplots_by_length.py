#!/usr/bin/env python3
"""Compare constrained compliance fit errors across 5N boom lengths."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
OUTPUT = Path(__file__).resolve().with_name("compliance_error_boxplots_by_length_5N_constrained.png")
X_LIMITS = (0.0, 0.5)

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


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def weighted_errors(dataset: str) -> np.ndarray:
    length_m = length_from_dataset(dataset)
    data = np.loadtxt(EXPERIMENT_DIR / f"{dataset}_shear_center.csv", delimiter=",", skiprows=1)
    compliance = load_compliance(EXPERIMENT_DIR / f"{dataset}_compliance_constrained.csv")

    w = data[:, 0:6].T
    x = data[:, 6:12].T
    sqrt_h = np.diag([1.0, 1.0, 1.0, length_m, length_m, length_m])
    weighted_residual = sqrt_h @ (x - compliance @ w)
    return np.linalg.norm(weighted_residual, axis=0)


def main() -> int:
    lengths = [length_from_dataset(dataset) for dataset in DATASETS]
    errors = [weighted_errors(dataset) for dataset in DATASETS]

    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.labelsize": 12,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
        }
    )
    fig, ax = plt.subplots(figsize=(8.2, 4.6))

    positions = np.arange(len(DATASETS), 0, -1)
    box = ax.boxplot(
        errors,
        vert=False,
        positions=positions,
        widths=0.42,
        patch_artist=True,
        showmeans=True,
        boxprops={"facecolor": "#d7e8f5", "edgecolor": "#333333", "linewidth": 1.2},
        whiskerprops={"color": "#333333", "linewidth": 1.1},
        capprops={"color": "#333333", "linewidth": 1.1},
        medianprops={"color": "#f28e2b", "linewidth": 1.8},
        meanprops={"marker": "^", "markerfacecolor": "#2f7d32", "markeredgecolor": "#2f7d32", "markersize": 6},
        flierprops={"marker": "o", "markerfacecolor": "#777777", "markeredgecolor": "#777777", "alpha": 0.45, "markersize": 4},
    )
    for patch in box["boxes"]:
        patch.set_alpha(0.95)

    ax.set_xlim(*X_LIMITS)
    ax.set_yticks(positions)
    ax.set_yticklabels([f"{length:g} m" for length in lengths])
    ax.set_xlabel(r"weighted residual norm $\|H^{1/2}(X - CW)\|_2$")
    ax.set_ylabel("Boom length")
    ax.set_title("Constrained Compliance Fit Error by Length")
    ax.grid(True, axis="x", alpha=0.30)
    ax.grid(True, axis="y", alpha=0.12)

    fig.tight_layout()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUTPUT, dpi=180)

    print(f"saved: {OUTPUT}")
    for length, values in zip(lengths, errors):
        print(
            f"{length:g} m: min/median/mean/max = "
            f"{np.min(values):.6e} / {np.median(values):.6e} / "
            f"{np.mean(values):.6e} / {np.max(values):.6e}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
