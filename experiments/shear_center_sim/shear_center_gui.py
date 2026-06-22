#!/usr/bin/env python3
"""Interactive thin-wall shear-center visualization for a circular tape spring."""

from __future__ import annotations

import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Wedge
from matplotlib.widgets import Button, Slider


MIDLINE_DIAMETER_MM = 50.0
MIDLINE_RADIUS_MM = MIDLINE_DIAMETER_MM / 2.0
NOMINAL_THICKNESS_MM = 1.0
ALPHA_MIN_DEG = 90.0
ALPHA_MAX_DEG = 360.0
ALPHA_INITIAL_DEG = 180.0
CLOSED_TOL_DEG = 1e-9


@dataclass(frozen=True)
class SectionResult:
    alpha_deg: float
    centroid_mm: np.ndarray
    shear_center_mm: np.ndarray
    theta_start_deg: float
    theta_end_deg: float
    is_closed: bool


def section_result(alpha_deg: float) -> SectionResult:
    """Return thin-wall centroid and shear-center positions for the arc section.

    Coordinates are in mm relative to the circle center. The partial section is
    oriented with its free opening upward, so open-section markers lie on the
    negative Y symmetry axis.
    """
    alpha = float(np.clip(alpha_deg, ALPHA_MIN_DEG, ALPHA_MAX_DEG))
    is_closed = math.isclose(alpha, ALPHA_MAX_DEG, abs_tol=CLOSED_TOL_DEG)

    if is_closed:
        return SectionResult(
            alpha_deg=alpha,
            centroid_mm=np.zeros(2),
            shear_center_mm=np.zeros(2),
            theta_start_deg=0.0,
            theta_end_deg=360.0,
            is_closed=True,
        )

    beta = math.radians(alpha / 2.0)
    centroid_y = -MIDLINE_RADIUS_MM * math.sin(beta) / beta
    denominator = beta - math.sin(beta) * math.cos(beta)
    shear_center_y = (
        -2.0
        * MIDLINE_RADIUS_MM
        * (math.sin(beta) - beta * math.cos(beta))
        / denominator
    )

    beta_deg = alpha / 2.0
    return SectionResult(
        alpha_deg=alpha,
        centroid_mm=np.array([0.0, centroid_y]),
        shear_center_mm=np.array([0.0, shear_center_y]),
        theta_start_deg=-90.0 - beta_deg,
        theta_end_deg=-90.0 + beta_deg,
        is_closed=False,
    )


def polar_point(radius_mm: float, theta_deg: float) -> np.ndarray:
    theta = math.radians(theta_deg)
    return np.array([radius_mm * math.cos(theta), radius_mm * math.sin(theta)])


def coordinate_label(name: str, point_mm: np.ndarray) -> str:
    return f"{name} y = {point_mm[1]:+.2f} mm"


def build_plot() -> None:
    fig, ax = plt.subplots(figsize=(8.5, 8.0))
    fig.canvas.manager.set_window_title("Tape Spring Shear Center")
    plt.subplots_adjust(left=0.10, right=0.96, top=0.90, bottom=0.22)

    result = section_result(ALPHA_INITIAL_DEG)
    wall_patch = make_wall_patch(result)
    ax.add_patch(wall_patch)

    center_marker, = ax.plot(
        [0.0],
        [0.0],
        marker="+",
        color="0.45",
        markersize=14,
        markeredgewidth=2.0,
        linestyle="None",
    )
    centroid_marker, = ax.plot(
        [result.centroid_mm[0]],
        [result.centroid_mm[1]],
        marker="o",
        color="#1f77b4",
        markerfacecolor="white",
        markersize=8,
        markeredgewidth=2.0,
        linestyle="None",
    )
    shear_marker, = ax.plot(
        [result.shear_center_mm[0]],
        [result.shear_center_mm[1]],
        marker="D",
        color="#d62728",
        markerfacecolor="white",
        markersize=8,
        markeredgewidth=2.0,
        linestyle="None",
    )
    edge_lines = [
        ax.plot([], [], color="0.55", linestyle="--", linewidth=1.0)[0],
        ax.plot([], [], color="0.55", linestyle="--", linewidth=1.0)[0],
    ]
    update_edge_lines(edge_lines, result)

    readout = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        va="top",
        ha="left",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "0.80"},
    )
    centroid_annotation = ax.annotate(
        "",
        xy=result.centroid_mm,
        xytext=(10, 8),
        textcoords="offset points",
        ha="left",
        va="center",
        color="#1f77b4",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "#1f77b4"},
    )
    shear_annotation = ax.annotate(
        "",
        xy=result.shear_center_mm,
        xytext=(10, -8),
        textcoords="offset points",
        ha="left",
        va="center",
        color="#d62728",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "#d62728"},
    )
    update_point_annotations(centroid_annotation, shear_annotation, result)

    ax.set_title("Thin-Walled Circular Tape Spring Cross Section")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="0.90", linewidth=0.8)
    set_plot_limits(ax)

    slider_ax = fig.add_axes([0.15, 0.105, 0.68, 0.035])
    alpha_slider = Slider(
        ax=slider_ax,
        label="alpha",
        valmin=ALPHA_MIN_DEG,
        valmax=ALPHA_MAX_DEG,
        valinit=ALPHA_INITIAL_DEG,
        valstep=1.0,
        valfmt="%0.0f deg",
    )

    reset_ax = fig.add_axes([0.85, 0.095, 0.10, 0.055])
    reset_button = Button(reset_ax, "Reset")

    artists = {
        "wall_patch": wall_patch,
        "centroid_marker": centroid_marker,
        "shear_marker": shear_marker,
        "edge_lines": edge_lines,
        "readout": readout,
        "centroid_annotation": centroid_annotation,
        "shear_annotation": shear_annotation,
    }

    def redraw(alpha_deg: float) -> None:
        nonlocal wall_patch
        updated = section_result(alpha_deg)

        wall_patch.remove()
        wall_patch = make_wall_patch(updated)
        ax.add_patch(wall_patch)
        artists["wall_patch"] = wall_patch

        centroid_marker.set_data([updated.centroid_mm[0]], [updated.centroid_mm[1]])
        shear_marker.set_data([updated.shear_center_mm[0]], [updated.shear_center_mm[1]])
        update_edge_lines(edge_lines, updated)
        update_readout(readout, updated)
        update_point_annotations(centroid_annotation, shear_annotation, updated)
        fig.canvas.draw_idle()

    alpha_slider.on_changed(redraw)
    reset_button.on_clicked(lambda _event: alpha_slider.reset())
    update_readout(readout, result)

    # Keep a reference to widgets so they are not garbage-collected.
    fig._shear_center_widgets = (alpha_slider, reset_button, artists, center_marker)  # type: ignore[attr-defined]

    plt.show()


def make_wall_patch(result: SectionResult) -> Wedge:
    return Wedge(
        center=(0.0, 0.0),
        r=MIDLINE_RADIUS_MM + NOMINAL_THICKNESS_MM / 2.0,
        theta1=result.theta_start_deg,
        theta2=result.theta_end_deg,
        width=NOMINAL_THICKNESS_MM,
        facecolor="#c6d8ef",
        edgecolor="#2f5f8f",
        linewidth=2.0,
        alpha=0.90,
        zorder=1,
    )


def update_edge_lines(edge_lines: list[plt.Line2D], result: SectionResult) -> None:
    if result.is_closed:
        for line in edge_lines:
            line.set_data([], [])
        return

    for line, theta_deg in zip(edge_lines, (result.theta_start_deg, result.theta_end_deg)):
        point = polar_point(MIDLINE_RADIUS_MM, theta_deg)
        line.set_data([0.0, point[0]], [0.0, point[1]])


def update_readout(readout, result: SectionResult) -> None:
    closed_note = "closed tube" if result.is_closed else "open section"
    readout.set_text(f"alpha: {result.alpha_deg:.0f} deg ({closed_note})")


def update_point_annotations(centroid_annotation, shear_annotation, result: SectionResult) -> None:
    centroid_annotation.xy = result.centroid_mm
    centroid_annotation.set_text(coordinate_label("centroid", result.centroid_mm))

    shear_annotation.xy = result.shear_center_mm
    shear_annotation.set_text(coordinate_label("shear center", result.shear_center_mm))


def set_plot_limits(ax) -> None:
    radius = MIDLINE_RADIUS_MM + NOMINAL_THICKNESS_MM
    shear_extent = 2.15 * MIDLINE_RADIUS_MM
    ax.set_xlim(-1.35 * radius, 1.35 * radius)
    ax.set_ylim(-shear_extent, 1.35 * radius)


def main() -> int:
    build_plot()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
