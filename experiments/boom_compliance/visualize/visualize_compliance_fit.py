#!/usr/bin/env python3
"""Visualize compliance fit quality by applied force direction."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.cm import ScalarMappable
from matplotlib.colors import LinearSegmentedColormap, Normalize


EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
FITTING_DIR = EXPERIMENT_DIR / "fitting"
if str(FITTING_DIR) not in sys.path:
    sys.path.insert(0, str(FITTING_DIR))

from plot_compliance_errors import load_compliance


DEFAULT_DATA_CSV = EXPERIMENT_DIR / "data_1p5m_7p5N_shear_center.csv"
DEFAULT_COMPLIANCE_CSV = EXPERIMENT_DIR / "data_1p5m_7p5N_compliance.csv"
DEFAULT_BOOM_LENGTH_M = 1.5


def set_equal_bounds(ax, radius: float) -> None:
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_zlim(-radius, radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA_CSV)
    parser.add_argument("--compliance", type=Path, default=DEFAULT_COMPLIANCE_CSV)
    parser.add_argument("--boom-length-m", type=float, default=DEFAULT_BOOM_LENGTH_M)
    parser.add_argument("--arrow-length", type=float, default=1.0)
    args = parser.parse_args()

    data = np.loadtxt(args.data, delimiter=",", skiprows=1)
    w = data[:, 0:6].T
    x = data[:, 6:12].T
    c = load_compliance(args.compliance)

    sqrt_h = np.diag([1.0, 1.0, 1.0, args.boom_length_m, args.boom_length_m, args.boom_length_m])
    weighted_residual = sqrt_h @ (x - c @ w)
    errors = np.linalg.norm(weighted_residual, axis=0)

    forces = w[:3, :]
    force_norms = np.linalg.norm(forces, axis=0)
    directions = forces / np.maximum(force_norms, 1e-12)

    best = float(np.min(errors))
    worst = float(np.max(errors))
    cmap = LinearSegmentedColormap.from_list("fit_green_to_red", ["#159447", "#f3df4e", "#e67e22", "#c92525"])
    norm = Normalize(vmin=best, vmax=worst if worst > best else best + 1e-12)

    fig = plt.figure("Compliance Fit By Force Direction", figsize=(9.5, 8.0))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title("Compliance Fit Error by Force Direction")
    ax.set_xlabel("Fx direction")
    ax.set_ylabel("Fy direction")
    ax.set_zlabel("Fz direction")

    origin = np.zeros(3)
    for row_number, (direction, error) in enumerate(zip(directions.T, errors), start=1):
        end = args.arrow_length * direction
        color = cmap(norm(error))
        ax.quiver(
            origin[0],
            origin[1],
            origin[2],
            end[0],
            end[1],
            end[2],
            color=color,
            linewidth=2.4,
            arrow_length_ratio=0.12,
            normalize=False,
        )
        ax.text(end[0] * 1.05, end[1] * 1.05, end[2] * 1.05, str(row_number), color=color, fontsize=8)

    set_equal_bounds(ax, args.arrow_length * 1.25)
    try:
        ax.view_init(elev=18, azim=38)
    except TypeError:
        pass

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    colorbar = fig.colorbar(sm, ax=ax, shrink=0.72, pad=0.10)
    colorbar.set_label(r"weighted fit error $\|H^{1/2}(X - CW)\|_2$")

    print(f"data: {args.data}")
    print(f"compliance: {args.compliance}")
    print(f"rows plotted: {errors.size}")
    print(f"error min/median/mean/max: "
          f"{best:.6e} / {float(np.median(errors)):.6e} / "
          f"{float(np.mean(errors)):.6e} / {worst:.6e}")
    print("green = lower error, red = higher error")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
