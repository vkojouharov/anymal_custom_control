#!/usr/bin/env python3

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap


DISK_RADIUS = 2.0
THETA_MIN_DEG = -45.0
THETA_MAX_DEG = 45.0
DEFAULT_GRID_SAMPLES = 300
SMOOTHING_EPS = 0.05

HEATMAP_CMAP = LinearSegmentedColormap.from_list(
    "penalty_model_heatmap",
    ["#2ca25f", "#ffff99", "#fdae61", "#d7191c"],
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize a wedge-defined penalty model inside a circular disk."
    )
    parser.add_argument(
        "--grid-samples",
        type=int,
        default=DEFAULT_GRID_SAMPLES,
        help="Number of samples per axis for the XY grid.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional path to save the figure instead of showing it.",
    )
    return parser.parse_args()


def compute_penalty(radius, grid_samples):
    axis = np.linspace(-radius, radius, grid_samples)
    points_x, points_y = np.meshgrid(axis, axis, indexing="xy")

    inside_disk = points_x**2 + points_y**2 <= radius**2
    inside_wedge = (points_x >= 0.0) & (np.abs(points_y) <= points_x)
    valid_mask = inside_disk & inside_wedge

    penalty = (-points_x + np.sqrt(points_y**2 + SMOOTHING_EPS**2)) / np.sqrt(2.0)

    masked_penalty = np.ma.array(penalty, mask=~valid_mask)
    return points_x, points_y, masked_penalty


def plot_penalty(points_x, points_y, penalty, radius):
    fig, ax = plt.subplots(figsize=(7, 7))

    mesh = ax.pcolormesh(
        points_x,
        points_y,
        penalty,
        shading="auto",
        cmap=HEATMAP_CMAP,
    )
    fig.colorbar(
        mesh,
        ax=ax,
        label=rf"$(-x + \sqrt{{y^2 + {SMOOTHING_EPS:.2f}^2}}) / \sqrt{{2}}$",
    )

    boundary = plt.Circle((0.0, 0.0), radius, fill=False, color="black", linewidth=2)
    ax.add_patch(boundary)

    ray_extent = radius / np.sqrt(2.0)
    ax.plot(
        [0.0, ray_extent],
        [0.0, ray_extent],
        linestyle="--",
        color="black",
        linewidth=1.5,
    )
    ax.plot(
        [0.0, ray_extent],
        [0.0, -ray_extent],
        linestyle="--",
        color="black",
        linewidth=1.5,
    )

    ax.set_aspect("equal")
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(
        r"Penalty inside $\theta \in [-45^\circ, 45^\circ]$: "
        + rf"$(-x + \sqrt{{y^2 + {SMOOTHING_EPS:.2f}^2}}) / \sqrt{{2}}$"
    )
    ax.set_facecolor(fig.get_facecolor())
    fig.tight_layout()
    return fig


def main():
    args = parse_args()
    points_x, points_y, penalty = compute_penalty(DISK_RADIUS, args.grid_samples)
    fig = plot_penalty(points_x, points_y, penalty, DISK_RADIUS)

    if args.output:
        fig.savefig(args.output, dpi=200, bbox_inches="tight")
        return

    plt.show()


if __name__ == "__main__":
    main()
