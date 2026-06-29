#!/usr/bin/env python3
"""Render an MPC rollout animation from an egocentric servo run CSV."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Optional

from visualize_trajectory import parse_json_matrix, read_metadata, read_rows, resolve_run, value


TAG_BLUE = (14.0 / 255.0, 158.0 / 255.0, 213.0 / 255.0)
ODOM_ORANGE = (230.0 / 255.0, 126.0 / 255.0, 34.0 / 255.0)
MPC_GREEN = (46.0 / 255.0, 160.0 / 255.0, 67.0 / 255.0)
ROBOT_RED = (255.0 / 255.0, 82.0 / 255.0, 74.0 / 255.0)


def main() -> int:
    parser = argparse.ArgumentParser(description="Render an MPC rollout animation from trajectory.csv")
    parser.add_argument(
        "run",
        nargs="?",
        default=None,
        help="Run directory, trajectory.csv path, or run root. Defaults to latest run beside this script.",
    )
    parser.add_argument("--output", default=None, help="Output MP4 path. Defaults to <run>/mpc_rollout.mp4")
    parser.add_argument("--fps", type=float, default=10.0, help="Saved video FPS")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--tail-count", type=int, default=25, help="Number of old MPC horizons to show faded")
    parser.add_argument("--no-odom", action="store_true", help="Do not draw legged-odometry path")
    args = parser.parse_args()

    run_dir, csv_path = resolve_run(args.run)
    rows = read_rows(csv_path)
    rows = [row for row in rows if value(row, "mpc_x_tag_m") is not None]
    if not rows:
        raise SystemExit(f"No MPC rows found in {csv_path}")
    metadata = read_metadata(run_dir)
    output = Path(args.output).expanduser() if args.output else run_dir / "mpc_rollout.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)
    frame_row_indices = timeline_row_indices(rows, args.fps)

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FFMpegWriter
    except ImportError as exc:
        raise SystemExit("matplotlib is required: python3 -m pip install matplotlib") from exc

    try:
        from matplotlib import animation

        if not animation.writers.is_available("ffmpeg"):
            raise SystemExit("Matplotlib ffmpeg writer is not available; install ffmpeg to save MP4.")
    except Exception as exc:
        if isinstance(exc, SystemExit):
            raise
        raise SystemExit(f"Failed to check ffmpeg availability: {exc}") from exc

    limits = compute_plot_limits(rows)
    fig, ax = plt.subplots(figsize=(9.5, 6.0))

    # Force final canvas to even pixel dimensions for H.264
    dpi = args.dpi
    w_px = int(round(fig.get_figwidth() * dpi))
    h_px = int(round(fig.get_figheight() * dpi))

    w_px += w_px % 2
    h_px += h_px % 2

    fig.set_size_inches(w_px / dpi, h_px / dpi)

    writer = FFMpegWriter(fps=args.fps, metadata={"title": f"MPC rollout {run_dir.name}"})
    with writer.saving(fig, str(output), args.dpi):
        for video_frame_index, row_index in enumerate(frame_row_indices):
            draw_frame(
                ax=ax,
                rows=rows,
                row_index=row_index,
                video_frame_index=video_frame_index,
                video_frame_count=len(frame_row_indices),
                metadata=metadata,
                limits=limits,
                tail_count=args.tail_count,
                show_odom=not args.no_odom,
            )
            writer.grab_frame()
    plt.close(fig)
    print(f"saved {output}")
    return 0


def draw_frame(
    *,
    ax,
    rows: list[dict[str, object]],
    row_index: int,
    video_frame_index: int,
    video_frame_count: int,
    metadata: dict,
    limits: tuple[tuple[float, float], tuple[float, float]],
    tail_count: int,
    show_odom: bool,
) -> None:
    ax.clear()
    row = rows[row_index]
    draw_tag(ax, metadata)
    draw_goal(ax, rows)
    draw_old_horizons(ax, rows, row_index, tail_count)
    draw_current_horizon(ax, row)
    draw_executed_path(ax, rows[: row_index + 1], "mpc_x_tag_m", "mpc_y_tag_m", "AprilTag localization", TAG_BLUE)
    if show_odom:
        draw_executed_path(ax, rows[: row_index + 1], "odom_mpc_x_tag_m", "odom_mpc_y_tag_m", "Legged odometry", ODOM_ORANGE)
    draw_robot(ax, row)
    ax.axhline(0.0, color="#d0d0d0", linewidth=0.8)
    ax.axvline(0.0, color="#d0d0d0", linewidth=0.8)
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.4, alpha=0.35)
    ax.set_xlabel("MPC tag-frame +X [m]")
    ax.set_ylabel("MPC tag-frame +Y [m]")
    ax.set_title(status_title(row, row_index, len(rows), video_frame_index, video_frame_count))
    ax.legend(loc="upper left", fontsize=8)


def draw_tag(ax, metadata: dict) -> None:
    from matplotlib.patches import Rectangle

    tag_size = metadata.get("apriltag_tag_length_m") or 0.20066
    tag_height = float(tag_size)
    tag_thickness = max(0.025, tag_height * 0.08)
    rect = Rectangle(
        (-tag_thickness / 2.0, -tag_height / 2.0),
        tag_thickness,
        tag_height,
        facecolor=TAG_BLUE,
        edgecolor="black",
        linewidth=1.0,
        alpha=0.55,
        label="AprilTag",
        zorder=2,
    )
    ax.add_patch(rect)
    ax.arrow(0.0, 0.0, 0.20, 0.0, head_width=0.035, head_length=0.04, color=TAG_BLUE, length_includes_head=True)


def draw_goal(ax, rows: list[dict[str, object]]) -> None:
    for row in rows:
        x = value(row, "mpc_x_tag_m")
        error = value(row, "range_error_m")
        if x is not None and error is not None:
            ax.scatter([x - error], [0.0], marker="x", color=MPC_GREEN, s=75, linewidths=2.0, label="goal", zorder=4)
            return


def draw_old_horizons(ax, rows: list[dict[str, object]], frame_index: int, tail_count: int) -> None:
    start = max(0, frame_index - max(0, tail_count))
    for old_row in rows[start:frame_index]:
        states = parse_json_matrix(old_row.get("mpc_predicted_states_json"))
        xs, ys = horizon_xy(states)
        if len(xs) >= 2:
            ax.plot(xs, ys, linestyle="--", color=ODOM_ORANGE, alpha=0.18, linewidth=1.0, zorder=1)


def draw_current_horizon(ax, row: dict[str, object]) -> None:
    states = parse_json_matrix(row.get("mpc_predicted_states_json"))
    xs, ys = horizon_xy(states)
    if len(xs) >= 2:
        ax.plot(xs, ys, linestyle="-", color=ODOM_ORANGE, alpha=0.95, linewidth=2.1, label="MPC horizon", zorder=3)
        ax.scatter([xs[-1]], [ys[-1]], marker="s", color=ODOM_ORANGE, s=28, zorder=4)


def draw_executed_path(ax, rows: list[dict[str, object]], x_key: str, y_key: str, label: str, color) -> None:
    points = [(value(row, x_key), value(row, y_key)) for row in rows]
    valid = [(x, y) for x, y in points if x is not None and y is not None]
    if not valid:
        return
    xs, ys = zip(*valid)
    ax.plot(xs, ys, color=color, linewidth=1.8, label=label, zorder=5)
    ax.scatter([xs[0]], [ys[0]], color=color, marker="o", s=32, zorder=6)
    ax.scatter([xs[-1]], [ys[-1]], color=color, marker=".", s=40, zorder=6)


def draw_robot(ax, row: dict[str, object]) -> None:
    from matplotlib.patches import Polygon

    x = value(row, "mpc_x_tag_m")
    y = value(row, "mpc_y_tag_m")
    theta = value(row, "mpc_theta_tag_rad")
    if x is None or y is None or theta is None:
        return
    length = 0.55
    width = 0.34
    corners = np_array(
        [
            [length / 2.0, width / 2.0],
            [length / 2.0, -width / 2.0],
            [-length / 2.0, -width / 2.0],
            [-length / 2.0, width / 2.0],
        ]
    )
    c = math.cos(theta)
    s = math.sin(theta)
    rot = np_array([[c, -s], [s, c]])
    world = corners @ rot.T + np_array([x, y])
    ax.add_patch(Polygon(world, closed=True, facecolor=ROBOT_RED, edgecolor="black", alpha=0.85, zorder=7, label="robot"))
    ax.arrow(x, y, -0.25 * c, -0.25 * s, head_width=0.04, head_length=0.05, color="black", length_includes_head=True, zorder=8)


def status_title(
    row: dict[str, object],
    row_index: int,
    row_count: int,
    video_frame_index: int,
    video_frame_count: int,
) -> str:
    t = value(row, "t_rel_sec")
    state = row.get("state") or "-"
    source = row.get("command_source") or "-"
    solver = row.get("solver_status") or "-"
    heading = value(row, "cmd_heading")
    lateral = value(row, "cmd_lateral")
    turning = value(row, "cmd_turning")
    err_r = value(row, "range_error_m")
    err_y = value(row, "lateral_error_m")
    err_th = value(row, "yaw_error_rad")
    return (
        f"frame {video_frame_index + 1}/{video_frame_count} | row {row_index + 1}/{row_count} | t={fmt(t)}s | {state} | {source} | {solver}\n"
        f"err=[{fmt(err_r)}, {fmt(err_y)}, {fmt(err_th)}] | cmd=[{fmt(heading)}, {fmt(lateral)}, {fmt(turning)}]"
    )


def timeline_row_indices(rows: list[dict[str, object]], fps: float) -> list[int]:
    if len(rows) <= 1 or fps <= 0.0:
        return list(range(len(rows)))
    times = [value(row, "t_rel_sec") for row in rows]
    if any(t is None for t in times):
        return list(range(len(rows)))
    end_time = float(times[-1])
    frame_period = 1.0 / fps
    frame_count = max(1, int(math.ceil(end_time / frame_period)) + 1)
    indices: list[int] = []
    row_index = 0
    for frame in range(frame_count):
        t = min(frame * frame_period, end_time)
        while row_index + 1 < len(rows) and float(times[row_index + 1]) <= t:
            row_index += 1
        indices.append(row_index)
    if indices[-1] != len(rows) - 1:
        indices.append(len(rows) - 1)
    return indices


def compute_plot_limits(rows: list[dict[str, object]]) -> tuple[tuple[float, float], tuple[float, float]]:
    xs: list[float] = [0.0]
    ys: list[float] = [0.0]
    for row in rows:
        for x_key, y_key in [
            ("mpc_x_tag_m", "mpc_y_tag_m"),
            ("odom_mpc_x_tag_m", "odom_mpc_y_tag_m"),
        ]:
            x = value(row, x_key)
            y = value(row, y_key)
            if x is not None and y is not None:
                xs.append(x)
                ys.append(y)
        for state in parse_json_matrix(row.get("mpc_predicted_states_json")):
            if len(state) >= 2:
                xs.append(state[0])
                ys.append(state[1])
        x = value(row, "mpc_x_tag_m")
        error = value(row, "range_error_m")
        if x is not None and error is not None:
            xs.append(x - error)
            ys.append(0.0)
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_pad = max(0.35, 0.12 * max(1e-6, x_max - x_min))
    y_pad = max(0.35, 0.12 * max(1e-6, y_max - y_min))
    return (x_min - x_pad, x_max + x_pad), (y_min - y_pad, y_max + y_pad)


def horizon_xy(states: list[list[float]]) -> tuple[list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    for state in states:
        if len(state) < 2:
            continue
        xs.append(state[0])
        ys.append(state[1])
    return xs, ys


def np_array(values):
    import numpy as np

    return np.asarray(values, dtype=float)


def fmt(value_obj: Optional[float]) -> str:
    if value_obj is None:
        return "-"
    return f"{value_obj:.3f}"


if __name__ == "__main__":
    raise SystemExit(main())
