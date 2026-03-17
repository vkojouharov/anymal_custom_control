#!/usr/bin/env python3

import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.colors import Normalize


FORCE_VECTOR = np.array([1.0, 0.0, 0.0], dtype=float)
FORCE_APPLICATION_POINT = np.array([0.0, 0.0, 1.0], dtype=float)
DISK_CENTER = np.array([0.0, 0.0, 0.0], dtype=float)
DISK_Z = 0.0
DISK_RADIUS = 2.0
DEFAULT_RADIAL_SAMPLES = 100
DEFAULT_ANGULAR_SAMPLES = 180
DEFAULT_ALIGNMENT_WEIGHT = 1.0
DEFAULT_DISTANCE_WEIGHT = 0.05
DEFAULT_DISTANCE_DISPLAY_SCALE = 0.5

SURROGATE_SLOW_DECAY_RATES = np.linspace(0.05, 1.5, 30)
SURROGATE_FAST_DECAY_RATES = np.linspace(0.5, 12.0, 80)
SURROGATE_NEAR_FIELD_WEIGHT_DECAY = 1.5
SURROGATE_TAPER_PREFERENCE = 2.0e-3

HEATMAP_CMAP = LinearSegmentedColormap.from_list(
    "force_penalty_heatmap",
    ["#2ca25f", "#ffff99", "#fdae61", "#d7191c"],
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize sin(theta), 0.5|x|_2, and a weighted mixed cost over an XY disk."
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
        "--alignment-weight",
        type=float,
        default=DEFAULT_ALIGNMENT_WEIGHT,
        help="Weight multiplying sin(theta) in the mixed cost plot.",
    )
    parser.add_argument(
        "--distance-weight",
        type=float,
        default=DEFAULT_DISTANCE_WEIGHT,
        help="Weight multiplying |x|_2 in the mixed cost plot.",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="Optional path to save the figure instead of showing it.",
    )
    args = parser.parse_args()
    if args.radial_samples < 2:
        parser.error("--radial-samples must be at least 2.")
    if args.angular_samples < 3:
        parser.error("--angular-samples must be at least 3.")
    return args


def compute_force_penalty(radius, disk_z, radial_samples, angular_samples):
    r = np.linspace(0.0, radius, radial_samples)
    theta = np.linspace(0.0, 2.0 * np.pi, angular_samples, endpoint=False)
    rr, tt = np.meshgrid(r, theta, indexing="xy")

    points_x = rr * np.cos(tt)
    points_y = rr * np.sin(tt)
    points_z = np.full_like(points_x, disk_z)

    sampled_points = np.stack((points_x, points_y, points_z), axis=-1)

    # x points from the sampled disk location to the force application point.
    lever_arm = FORCE_APPLICATION_POINT - sampled_points
    distance_to_task = np.linalg.norm(lever_arm, axis=-1)
    force_norm = np.linalg.norm(FORCE_VECTOR)
    cross_norm = np.linalg.norm(np.cross(lever_arm, FORCE_VECTOR), axis=-1)
    alignment_penalty = cross_norm / (distance_to_task * force_norm)

    return points_x, points_y, distance_to_task, alignment_penalty


def fit_exponential_surrogate(points_x, points_y, penalty):
    mask = points_x.ravel() >= 0.0
    x_fit = points_x.ravel()[mask]
    y_fit = points_y.ravel()[mask]
    penalty_fit = penalty.ravel()[mask]
    sample_weights = np.exp(-SURROGATE_NEAR_FIELD_WEIGHT_DECAY * x_fit)
    sample_weights /= np.mean(sample_weights)
    sqrt_weights = np.sqrt(sample_weights)

    best_result = None
    for slow_decay_rate in SURROGATE_SLOW_DECAY_RATES:
        for fast_decay_rate in SURROGATE_FAST_DECAY_RATES:
            if fast_decay_rate <= slow_decay_rate:
                continue

            design_matrix = np.column_stack(
                (
                    np.ones_like(x_fit),
                    y_fit**2,
                    np.exp(-slow_decay_rate * x_fit),
                    np.exp(-fast_decay_rate * x_fit),
                )
            )
            coefficients, _, _, _ = np.linalg.lstsq(
                design_matrix * sqrt_weights[:, None],
                penalty_fit * sqrt_weights,
                rcond=None,
            )
            offset, beta, slow_amplitude, fast_amplitude = coefficients
            if beta < 0.0 or slow_amplitude < 0.0 or fast_amplitude < 0.0:
                continue

            residual = design_matrix @ coefficients - penalty_fit
            weighted_mse = np.mean(sample_weights * residual**2)
            initial_taper = (
                slow_amplitude * slow_decay_rate
                + fast_amplitude * fast_decay_rate
            )
            score = weighted_mse - SURROGATE_TAPER_PREFERENCE * initial_taper
            if best_result is None or score < best_result["score"]:
                best_result = {
                    "offset": offset,
                    "beta": beta,
                    "slow_amplitude": slow_amplitude,
                    "fast_amplitude": fast_amplitude,
                    "slow_decay_rate": slow_decay_rate,
                    "fast_decay_rate": fast_decay_rate,
                    "weighted_mse": weighted_mse,
                    "initial_taper": initial_taper,
                    "score": score,
                }

    if best_result is None:
        raise RuntimeError("Failed to fit an exponential surrogate with nonnegative parameters.")

    surrogate = np.full_like(penalty, np.nan)
    surrogate_mask = points_x >= 0.0
    surrogate[surrogate_mask] = (
        best_result["offset"]
        + best_result["beta"] * points_y[surrogate_mask] ** 2
        + best_result["slow_amplitude"]
        * np.exp(-best_result["slow_decay_rate"] * points_x[surrogate_mask])
        + best_result["fast_amplitude"]
        * np.exp(-best_result["fast_decay_rate"] * points_x[surrogate_mask])
    )
    return best_result, surrogate


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


def plot_three_panel_figure(
    points_x,
    points_y,
    alignment_penalty,
    distance_to_task,
    alignment_weight,
    distance_weight,
    radius,
):
    mixed_cost = alignment_weight * alignment_penalty + distance_weight * distance_to_task
    scaled_distance_to_task = DEFAULT_DISTANCE_DISPLAY_SCALE * distance_to_task
    fig, axes = plt.subplots(1, 3, figsize=(18, 6), constrained_layout=True)

    displayed_values = (
        alignment_penalty,
        scaled_distance_to_task,
        mixed_cost,
    )
    shared_vmin = min(np.min(values) for values in displayed_values)
    shared_vmax = max(np.max(values) for values in displayed_values)
    shared_norm = Normalize(vmin=shared_vmin, vmax=shared_vmax)
    shared_levels = np.linspace(shared_vmin, shared_vmax, 32)

    panels = (
        (alignment_penalty, r"$\sin(\theta)$", r"Alignment penalty $\sin(\theta)$"),
        (
            scaled_distance_to_task,
            rf"${DEFAULT_DISTANCE_DISPLAY_SCALE:.1f}|x|_2$",
            rf"Distance term ${DEFAULT_DISTANCE_DISPLAY_SCALE:.1f}|x|_2$",
        ),
        (
            mixed_cost,
            rf"${alignment_weight:.1f}\sin(\theta) + {distance_weight:.2f}|x|_2$",
            rf"Mixed cost ${alignment_weight:.1f}\sin(\theta) + {distance_weight:.2f}|x|_2$",
        ),
    )

    for ax, (values, colorbar_label, title) in zip(axes, panels):
        contour = ax.tricontourf(
            points_x.ravel(),
            points_y.ravel(),
            values.ravel(),
            levels=shared_levels,
            cmap=HEATMAP_CMAP,
            norm=shared_norm,
        )
        add_disk_overlays(ax, radius)
        ax.set_title(title)

    fig.colorbar(
        contour,
        ax=axes,
        label=(
            r"Shared scale: "
            + r"$\sin(\theta)$, "
            + rf"${DEFAULT_DISTANCE_DISPLAY_SCALE:.1f}|x|_2$, "
            + rf"${alignment_weight:.1f}\sin(\theta) + {distance_weight:.2f}|x|_2$"
        ),
    )

    return fig


def main():
    args = parse_args()
    points_x, points_y, distance_to_task, alignment_penalty = compute_force_penalty(
        DISK_RADIUS, DISK_Z, args.radial_samples, args.angular_samples
    )

    # Keep the smoother exponential surrogate available for future comparisons.
    _surrogate_params, _surrogate_penalty = fit_exponential_surrogate(
        points_x, points_y, alignment_penalty
    )

    fig = plot_three_panel_figure(
        points_x,
        points_y,
        alignment_penalty,
        distance_to_task,
        args.alignment_weight,
        args.distance_weight,
        DISK_RADIUS,
    )

    if args.output:
        fig.savefig(args.output, dpi=200, bbox_inches="tight")
        return

    plt.show()


if __name__ == "__main__":
    main()
