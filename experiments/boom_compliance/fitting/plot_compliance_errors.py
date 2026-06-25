#!/usr/bin/env python3
"""Plot weighted compliance-fit residuals for converted boom data."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_DATA_CSV = EXPERIMENT_DIR / "data_2p25m_5N_shear_center.csv"
DEFAULT_COMPLIANCE_CSV = EXPERIMENT_DIR / "data_2p25m_5N_compliance_constrained.csv"
DEFAULT_OUTPUT = Path(__file__).resolve().parent / "data_2p25m_5N_error_boxplot_constrained.png"
DEFAULT_BOOM_LENGTH_M = 2.25


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA_CSV)
    parser.add_argument("--compliance", type=Path, default=DEFAULT_COMPLIANCE_CSV)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--boom-length-m", type=float, default=DEFAULT_BOOM_LENGTH_M)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    data = np.loadtxt(args.data, delimiter=",", skiprows=1)
    w = data[:, 0:6].T
    x = data[:, 6:12].T
    c = load_compliance(args.compliance)

    sqrt_h = np.diag([1.0, 1.0, 1.0, args.boom_length_m, args.boom_length_m, args.boom_length_m])
    residual = x - c @ w
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
        label="count",
    )
    hist_ax.axvline(np.median(weighted_error_norm), color="#f28e2b", linewidth=2.0, label="median")
    hist_ax.axvline(np.mean(weighted_error_norm), color="#59a14f", linewidth=2.0, linestyle="--", label="mean")
    hist_ax.set_ylabel("count")
    hist_ax.set_title("Compliance Fit Error Distribution")
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
    box_ax.set_yticks([])
    box_ax.set_xlabel(r"weighted residual norm $\|H^{1/2}(X - CW)\|_2$")
    box_ax.grid(True, axis="x", alpha=0.30)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, bbox_inches="tight")

    print(f"samples: {weighted_error_norm.size}")
    print(f"total fitting cost: {float(np.sum(per_sample_cost)):.6e}")
    print(f"weighted error norm min/median/mean/max: "
          f"{float(np.min(weighted_error_norm)):.6e} / "
          f"{float(np.median(weighted_error_norm)):.6e} / "
          f"{float(np.mean(weighted_error_norm)):.6e} / "
          f"{float(np.max(weighted_error_norm)):.6e}")
    print(f"saved: {args.output}")

    if args.show:
        plt.show()
    else:
        plt.close(fig)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
