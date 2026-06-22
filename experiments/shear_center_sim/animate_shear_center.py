#!/usr/bin/env python3
"""Export a shear-center sweep animation to MP4."""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation

from shear_center_gui import (
    ALPHA_MAX_DEG,
    ALPHA_MIN_DEG,
    coordinate_label,
    make_wall_patch,
    section_result,
    set_plot_limits,
    update_edge_lines,
    update_point_annotations,
    update_readout,
)


FPS = 30
DURATION_SECONDS = 5.0
OUTPUT_NAME = "shear_center_sweep.mp4"


def alpha_frames() -> np.ndarray:
    frame_count = int(FPS * DURATION_SECONDS)
    half_count = frame_count // 2
    down = np.linspace(ALPHA_MAX_DEG, ALPHA_MIN_DEG, half_count, endpoint=True)
    up = np.linspace(ALPHA_MIN_DEG, ALPHA_MAX_DEG, frame_count - half_count + 1, endpoint=True)[1:]
    return np.concatenate([down, up])


def build_animation() -> tuple[FuncAnimation, plt.Figure]:
    fig, ax = plt.subplots(figsize=(8.5, 8.0))
    fig.subplots_adjust(left=0.10, right=0.96, top=0.90, bottom=0.10)

    result = section_result(ALPHA_MAX_DEG)
    wall_patch = make_wall_patch(result)
    ax.add_patch(wall_patch)

    ax.plot(
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
        coordinate_label("centroid", result.centroid_mm),
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
        coordinate_label("shear center", result.shear_center_mm),
        xy=result.shear_center_mm,
        xytext=(10, -8),
        textcoords="offset points",
        ha="left",
        va="center",
        color="#d62728",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "#d62728"},
    )
    update_readout(readout, result)

    ax.set_title("Thin-Walled Circular Tape Spring Cross Section")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="0.90", linewidth=0.8)
    set_plot_limits(ax)

    def draw(alpha_deg: float):
        nonlocal wall_patch
        updated = section_result(float(alpha_deg))

        wall_patch.remove()
        wall_patch = make_wall_patch(updated)
        ax.add_patch(wall_patch)

        centroid_marker.set_data([updated.centroid_mm[0]], [updated.centroid_mm[1]])
        shear_marker.set_data([updated.shear_center_mm[0]], [updated.shear_center_mm[1]])
        update_edge_lines(edge_lines, updated)
        update_readout(readout, updated)
        update_point_annotations(centroid_annotation, shear_annotation, updated)

        return (
            wall_patch,
            centroid_marker,
            shear_marker,
            *edge_lines,
            readout,
            centroid_annotation,
            shear_annotation,
        )

    animation = FuncAnimation(
        fig,
        draw,
        frames=alpha_frames(),
        interval=1000.0 / FPS,
        blit=False,
        repeat=False,
    )
    return animation, fig


def main() -> int:
    output_path = Path(__file__).resolve().with_name(OUTPUT_NAME)
    animation, fig = build_animation()
    writer = FFMpegWriter(fps=FPS, metadata={"title": "Tape Spring Shear Center Sweep"}, bitrate=2400)
    animation.save(output_path, writer=writer, dpi=150)
    plt.close(fig)
    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
