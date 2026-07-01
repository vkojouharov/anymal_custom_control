#!/usr/bin/env python3
"""Scatter plot y-bending, z-bending, and x-torsion values versus boom length."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# Fill these in with your data. All four lists must have the same length.
LENGTHS_M = [
    0.5,
    0.75,
    1.0,
    1.25,
    1.5,
    1.75,
    2.0,
    2.25,
    2.5,
]

Y_BEND_VALUES = [
    # value for each length above
    0.000644,
    0.001371,
    0.003039,
    0.004734,
    0.008766,
    0.011099,
    0.017664,
    0.024361,
    0.030415
]

Z_BEND_VALUES = [
    # value for each length above
    0.000488,
    0.000752,
    0.001753,
    0.003354,
    0.004562,
    0.007205,
    0.011017,
    0.017276,
    0.025367
]

X_TORSION_VALUES = [
    # value for each length above
    0.619,
    1.369,
    2.228,
    3.043,
    3.739,
    4.712,
    5.568,
    6.265,
    7.314
]


def validate_inputs(lengths: list[float], *series: list[float]) -> None:
    expected_count = len(lengths)
    if expected_count == 0:
        raise ValueError("add at least one length/value row before plotting")

    for values in series:
        if len(values) != expected_count:
            raise ValueError(
                "LENGTHS_M, Y_BEND_VALUES, Z_BEND_VALUES, and X_TORSION_VALUES "
                f"must all have the same length; got {expected_count}, {len(values)}"
            )


def fit_power_law(lengths: np.ndarray, values: np.ndarray, power: int, through_origin: bool) -> tuple[float, float]:
    x = lengths**power
    if through_origin:
        slope = float((x @ values) / (x @ x))
        intercept = 0.0
    else:
        slope, intercept = np.polyfit(x, values, deg=1)
        slope = float(slope)
        intercept = float(intercept)
    return slope, intercept


def fitted_values(lengths: np.ndarray, slope: float, intercept: float, power: int) -> np.ndarray:
    return slope * lengths**power + intercept


def print_fit(label: str, slope: float, intercept: float, power: int) -> None:
    if intercept == 0.0:
        print(f"{label}: value = {slope:.6e} * L^{power}")
    else:
        sign = "+" if intercept >= 0.0 else "-"
        print(f"{label}: value = {slope:.6e} * L^{power} {sign} {abs(intercept):.6e}")


def plot_values(save_path: Path | None, through_origin: bool) -> None:
    validate_inputs(LENGTHS_M, Y_BEND_VALUES, Z_BEND_VALUES, X_TORSION_VALUES)

    lengths = np.asarray(LENGTHS_M, dtype=float)
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 8.0), sharex=True)
    fig.suptitle("Compliance Terms vs Boom Length")

    plots = (
        ("Y Bend", Y_BEND_VALUES, 3, "tab:blue"),
        ("Z Bend", Z_BEND_VALUES, 3, "tab:green"),
        ("X Torsion", X_TORSION_VALUES, 1, "tab:red"),
    )

    fit_lengths = np.linspace(float(np.min(lengths)), float(np.max(lengths)), 200)
    for ax, (label, values, power, color) in zip(axes, plots, strict=True):
        values_array = np.asarray(values, dtype=float)
        slope, intercept = fit_power_law(lengths, values_array, power, through_origin)
        print_fit(label, slope, intercept, power)

        ax.scatter(lengths, values_array, color=color, s=44)
        ax.plot(
            fit_lengths,
            fitted_values(fit_lengths, slope, intercept, power),
            color=color,
            alpha=0.85,
            linewidth=1.8,
            label=f"fit: a L^{power}" + (" + b" if not through_origin else ""),
        )
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    axes[-1].set_xlabel("Length (m)")
    fig.tight_layout()

    if save_path is not None:
        save_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, dpi=200)
        print(f"saved plot to {save_path}")
    else:
        plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--save", type=Path, default=None, help="Save plot to this path instead of opening a window")
    parser.add_argument(
        "--through-origin",
        action="store_true",
        help="Fit pure beam models: y/z bending = a L^3 and x torsion = a L",
    )
    args = parser.parse_args()

    plot_values(args.save, args.through_origin)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
