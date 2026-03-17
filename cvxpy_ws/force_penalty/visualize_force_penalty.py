#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable


FORCE_VECTOR = np.array([0.0, 0.0, 1.0], dtype=float)
FORCE_APPLICATION_POINT = np.array([0.0, 0.0, 1.0], dtype=float)
DISK_CENTER = np.array([0.0, 0.0, 0.0], dtype=float)
DISK_Z = 0.0
DISK_RADIUS = 2.0
DEFAULT_RADIAL_SAMPLES = 100
DEFAULT_ANGULAR_SAMPLES = 180
DEFAULT_SLICE_SAMPLES = 400

HEATMAP_CMAP = LinearSegmentedColormap.from_list(
    "force_penalty_heatmap",
    ["#2ca25f", "#ffff99", "#fdae61", "#d7191c"],
)
OUTPUT_DIR = Path(__file__).resolve().parent


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize the sin(theta) force-alignment penalty over an XY disk."
    )
    parser.add_argument(
        "--radial-samples",
        type=int,
        default=DEFAULT_RADIAL_SAMPLES,
        help="Number of radial samples.",
    )
    parser.add_argument(
        "--angular-samples",
        type=int,
        default=DEFAULT_ANGULAR_SAMPLES,
        help="Number of angular samples.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional path to save the figure instead of showing it.",
    )
    parser.add_argument(
        "--slice-samples",
        type=int,
        default=DEFAULT_SLICE_SAMPLES,
        help="Number of samples for the y=0 slice figure.",
    )
    args = parser.parse_args()
    if args.radial_samples < 2:
        parser.error("--radial-samples must be at least 2.")
    if args.angular_samples < 3:
        parser.error("--angular-samples must be at least 3.")
    if args.slice_samples < 2:
        parser.error("--slice-samples must be at least 2.")
    return args


def compute_force_penalty(radius, disk_z, radial_samples, angular_samples):
    r = np.linspace(0.0, radius, radial_samples)
    theta = np.linspace(0.0, 2.0 * np.pi, angular_samples, endpoint=False)
    rr, tt = np.meshgrid(r, theta, indexing="xy")

    points_x = rr * np.cos(tt)
    points_y = rr * np.sin(tt)
    points_z = np.full_like(points_x, disk_z)

    sampled_points = np.stack((points_x, points_y, points_z), axis=-1)

    lever_arm = FORCE_APPLICATION_POINT - sampled_points
    distance_to_task = np.linalg.norm(lever_arm, axis=-1)
    force_norm = np.linalg.norm(FORCE_VECTOR)
    cross_norm = np.linalg.norm(np.cross(lever_arm, FORCE_VECTOR), axis=-1)
    alignment_penalty = cross_norm / (distance_to_task * force_norm)

    return points_x, points_y, alignment_penalty


def find_global_minimum(points_x, points_y, penalty):
    flat_x = points_x.ravel()
    flat_y = points_y.ravel()
    flat_penalty = penalty.ravel()
    min_value = np.min(flat_penalty)
    candidate_mask = np.isclose(flat_penalty, min_value, rtol=1.0e-9, atol=1.0e-12)
    candidate_indices = np.flatnonzero(candidate_mask)
    tie_break_order = np.lexsort(
        (
            np.abs(flat_y[candidate_indices]),
            flat_y[candidate_indices],
            flat_x[candidate_indices],
        )
    )
    min_index = candidate_indices[tie_break_order[0]]
    return (
        flat_x[min_index],
        flat_y[min_index],
        flat_penalty[min_index],
    )


def fit_minimum_centered_surrogate(points_x, points_y, penalty, min_point, min_value):
    distances = np.sqrt((points_x - min_point[0]) ** 2 + (points_y - min_point[1]) ** 2)
    centered_penalty = penalty - min_value
    denominator = np.sum(distances**2)
    if denominator <= 0.0:
        scale = 0.0
    else:
        scale = max(np.sum(distances * centered_penalty) / denominator, 0.0)
    surrogate = min_value + scale * distances
    return scale, surrogate


def add_disk_overlays(ax, radius):
    boundary = plt.Circle(
        (DISK_CENTER[0], DISK_CENTER[1]), radius, fill=False, color="white", linewidth=2
    )
    ax.add_patch(boundary)

    force_norm = np.linalg.norm(FORCE_VECTOR[:2])
    if force_norm > 0.0:
        direction = FORCE_VECTOR[:2] / force_norm
        arrow_scale = 0.35 * radius
        ax.arrow(
            FORCE_APPLICATION_POINT[0],
            FORCE_APPLICATION_POINT[1],
            direction[0] * arrow_scale,
            direction[1] * arrow_scale,
            width=0.015 * radius,
            head_width=0.08 * radius,
            head_length=0.12 * radius,
            length_includes_head=True,
            color="black",
        )

    ax.scatter(
        [FORCE_APPLICATION_POINT[0]],
        [FORCE_APPLICATION_POINT[1]],
        color="black",
        s=35,
        zorder=3,
    )

    ax.set_aspect("equal")
    ax.set_box_aspect(1)
    ax.set_xlim(-radius, radius)
    ax.set_ylim(-radius, radius)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_facecolor(ax.figure.get_facecolor())


def compute_alignment_penalty_slice(radius, slice_samples):
    x_values = np.linspace(-radius, radius, slice_samples)
    sampled_points = np.column_stack(
        (x_values, np.zeros_like(x_values), np.full_like(x_values, DISK_Z))
    )

    lever_arm = FORCE_APPLICATION_POINT - sampled_points
    distance_to_task = np.linalg.norm(lever_arm, axis=-1)
    force_norm = np.linalg.norm(FORCE_VECTOR)
    cross_norm = np.linalg.norm(np.cross(lever_arm, FORCE_VECTOR), axis=-1)
    alignment_penalty = cross_norm / (distance_to_task * force_norm)

    return x_values, alignment_penalty


def compute_surrogate_slice(x_values, min_point, min_value, scale):
    distances = np.sqrt((x_values - min_point[0]) ** 2 + min_point[1] ** 2)
    return min_value + scale * distances


def add_colored_slice(ax, x_values, alignment_penalty, norm, min_point=None, min_value=None):
    points = np.column_stack((x_values, alignment_penalty))
    segments = np.stack((points[:-1], points[1:]), axis=1)
    segment_values = 0.5 * (alignment_penalty[:-1] + alignment_penalty[1:])
    colored_curve = LineCollection(
        segments,
        cmap=HEATMAP_CMAP,
        norm=norm,
        linewidths=3,
    )
    colored_curve.set_array(segment_values)
    ax.add_collection(colored_curve)

    ax.set_xlim(x_values[0], x_values[-1])
    y_padding = 0.05 * max(np.ptp(alignment_penalty), 1.0)
    ax.set_ylim(
        np.min(alignment_penalty) - y_padding,
        np.max(alignment_penalty) + y_padding,
    )
    ax.set_ylabel(r"$\sin(\theta)$")
    ax.set_title(r"Alignment penalty slice at $y = 0$")
    ax.grid(True, alpha=0.25)

    if min_point is not None and min_value is not None and np.isclose(min_point[1], 0.0):
        ax.scatter(
            [min_point[0]],
            [min_value],
            color="black",
            marker="*",
            s=300,
            zorder=4,
        )

    return colored_curve


def plot_alignment_penalty(
    points_x,
    points_y,
    alignment_penalty,
    slice_x,
    slice_penalty,
    radius,
    figure_title,
    slice_title,
    min_point,
    min_value,
):
    vmin = np.min([np.min(alignment_penalty), np.min(slice_penalty)])
    vmax = np.max([np.max(alignment_penalty), np.max(slice_penalty)])
    if np.isclose(vmin, vmax):
        vmax = vmin + 1.0e-6
    norm = Normalize(vmin=vmin, vmax=vmax)
    levels = np.linspace(vmin, vmax, 32)

    fig = plt.figure(figsize=(8.2, 9.6), constrained_layout=True)
    grid = fig.add_gridspec(
        nrows=2,
        ncols=2,
        height_ratios=[1.0, 4.6],
        width_ratios=[1.0, 0.06],
    )
    slice_ax = fig.add_subplot(grid[0, 0])
    disk_ax = fig.add_subplot(grid[1, 0], sharex=slice_ax)
    colorbar_ax = fig.add_subplot(grid[:, 1])

    add_colored_slice(slice_ax, slice_x, slice_penalty, norm, min_point, min_value)
    slice_ax.set_title(slice_title)
    slice_ax.tick_params(axis="x", labelbottom=False)

    disk_ax.tricontourf(
        points_x.ravel(),
        points_y.ravel(),
        alignment_penalty.ravel(),
        levels=levels,
        cmap=HEATMAP_CMAP,
        norm=norm,
    )
    add_disk_overlays(disk_ax, radius)
    disk_ax.scatter(
        [min_point[0]],
        [min_point[1]],
        color="black",
        marker="*",
        s=340,
        zorder=4,
    )
    disk_ax.set_title(figure_title)
    disk_ax.set_xlabel("x [m]")

    colorbar = fig.colorbar(
        ScalarMappable(norm=norm, cmap=HEATMAP_CMAP),
        cax=colorbar_ax,
        label=r"$\sin(\theta)$",
    )
    colorbar.set_ticks(np.linspace(vmin, vmax, 5))

    return fig


def format_force_component(value):
    formatted = f"{value:.3f}".rstrip("0").rstrip(".")
    if formatted in {"", "-0"}:
        formatted = "0"
    return formatted.replace("-", "m").replace(".", "p")


def append_stem_suffix(path, suffix):
    return path.with_name(f"{path.stem}{suffix}{path.suffix}")


def build_default_output_path(suffix=""):
    force_suffix = "_".join(
        f"{axis}{format_force_component(value)}"
        for axis, value in zip(("fx", "fy", "fz"), FORCE_VECTOR)
    )
    return OUTPUT_DIR / f"force_penalty_{force_suffix}{suffix}.png"


def register_close_save(fig, output_path):
    saved = False

    def save_on_close(event):
        nonlocal saved
        if saved:
            return
        fig.savefig(output_path, dpi=200, bbox_inches="tight")
        saved = True
        print(f"Saved figure to {output_path}")

    fig.canvas.mpl_connect("close_event", save_on_close)
    return output_path


def main():
    args = parse_args()
    points_x, points_y, alignment_penalty = compute_force_penalty(
        DISK_RADIUS, DISK_Z, args.radial_samples, args.angular_samples
    )
    slice_x, slice_penalty = compute_alignment_penalty_slice(
        DISK_RADIUS, args.slice_samples
    )
    min_x, min_y, min_value = find_global_minimum(points_x, points_y, alignment_penalty)
    min_point = np.array([min_x, min_y], dtype=float)
    surrogate_scale, linearized_penalty = fit_minimum_centered_surrogate(
        points_x, points_y, alignment_penalty, min_point, min_value
    )
    linearized_slice = compute_surrogate_slice(
        slice_x, min_point, min_value, surrogate_scale
    )

    fig = plot_alignment_penalty(
        points_x,
        points_y,
        alignment_penalty,
        slice_x,
        slice_penalty,
        DISK_RADIUS,
        r"Alignment penalty $\sin(\theta)$",
        r"Alignment penalty slice at $y = 0$",
        min_point,
        min_value,
    )
    linearized_fig = plot_alignment_penalty(
        points_x,
        points_y,
        linearized_penalty,
        slice_x,
        linearized_slice,
        DISK_RADIUS,
        r"Minimum-centered approximation of $\sin(\theta)$",
        r"Approximate penalty slice at $y = 0$",
        min_point,
        min_value,
    )

    if args.output:
        output_path = Path(args.output)
        fig.savefig(output_path, dpi=200, bbox_inches="tight")
        linearized_fig.savefig(
            append_stem_suffix(output_path, "_linearized"),
            dpi=200,
            bbox_inches="tight",
        )
        return

    register_close_save(fig, build_default_output_path())
    register_close_save(linearized_fig, build_default_output_path("_linearized"))
    plt.show()


if __name__ == "__main__":
    main()
