#!/usr/bin/env python3
"""Plot weighted compliance-fit residuals for converted boom data."""

from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
FITTING_DIR = Path(__file__).resolve().parent
DEFAULT_X_LIMITS = (0.0, 0.5)

# Editor-friendly batch mode. For one dataset, use a one-element list.
DATASETS: list[str] = [
    "data_1m_5N",
    "data_1p25m_5N",
    "data_1p5m_5N",
    "data_1p5m_7p5N",
    "data_1p5m_10N",
    "data_1p75m_5N",
    "data_2m_5N",
    "data_2p25m_5N",
]


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def infer_boom_length_m(dataset_name: str) -> float:
    match = re.match(r"^data_(?P<length>[0-9]+(?:p[0-9]+)?)m_", dataset_name)
    if match is None:
        raise ValueError(f"cannot infer boom length from dataset name: {dataset_name}")
    length_text = match.group("length")
    return float(length_text.replace("p", "."))


def plot_one(
    *,
    data_csv: Path,
    compliance_csv: Path,
    output: Path,
    boom_length_m: float,
    x_limits: tuple[float, float] = DEFAULT_X_LIMITS,
    show: bool = False,
) -> None:
    data = np.loadtxt(data_csv, delimiter=",", skiprows=1)
    w = data[:, 0:6].T
    x = data[:, 6:12].T
    c = load_compliance(compliance_csv)

    sqrt_h = np.diag([1.0, 1.0, 1.0, boom_length_m, boom_length_m, boom_length_m])
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
    box_ax.set_xlim(*x_limits)
    box_ax.set_yticks([])
    box_ax.set_xlabel(r"weighted residual norm $\|H^{1/2}(X - CW)\|_2$")
    box_ax.grid(True, axis="x", alpha=0.30)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180, bbox_inches="tight")

    print(f"data: {data_csv}")
    print(f"compliance: {compliance_csv}")
    print(f"boom length: {boom_length_m:g} m")
    print(f"samples: {weighted_error_norm.size}")
    print(f"total fitting cost: {float(np.sum(per_sample_cost)):.6e}")
    print(f"weighted error norm min/median/mean/max: "
          f"{float(np.min(weighted_error_norm)):.6e} / "
          f"{float(np.median(weighted_error_norm)):.6e} / "
          f"{float(np.mean(weighted_error_norm)):.6e} / "
          f"{float(np.max(weighted_error_norm)):.6e}")
    print(f"saved: {output}")

    if show:
        plt.show()
    else:
        plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--datasets",
        nargs="+",
        default=None,
        help=(
            "Dataset stems such as data_2p25m_5N. For each stem, reads "
            "<stem>_shear_center.csv and <stem>_compliance_constrained.csv, "
            "then writes fitting/<stem>_error_boxplot_constrained.png."
        ),
    )
    parser.add_argument("--x-min", type=float, default=DEFAULT_X_LIMITS[0])
    parser.add_argument("--x-max", type=float, default=DEFAULT_X_LIMITS[1])
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    x_limits = (args.x_min, args.x_max)
    datasets = args.datasets if args.datasets is not None else DATASETS
    if not datasets:
        raise ValueError("DATASETS is empty. Add at least one dataset stem, e.g. ['data_1p5m_5N'].")

    for dataset_name in datasets:
        plot_one(
            data_csv=EXPERIMENT_DIR / f"{dataset_name}_shear_center.csv",
            compliance_csv=EXPERIMENT_DIR / f"{dataset_name}_compliance_constrained.csv",
            output=FITTING_DIR / f"{dataset_name}_error_boxplot_constrained.png",
            boom_length_m=infer_boom_length_m(dataset_name),
            x_limits=x_limits,
            show=args.show,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
