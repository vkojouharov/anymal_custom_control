#!/usr/bin/env python3
"""Visualize synchronized ANYmal egocentric servo trajectory logs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable, Optional


DISPLAY_Y_SIGN = -1.0


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot an egocentric servo trajectory run")
    parser.add_argument(
        "run",
        nargs="?",
        default=None,
        help="Run directory, trajectory.csv path, or run root. Defaults to latest run beside this script.",
    )
    parser.add_argument("--save", default=None, help="Optional output image path")
    parser.add_argument("--no-show", action="store_true", help="Do not open an interactive plot window")
    args = parser.parse_args()

    run_dir, csv_path = resolve_run(args.run)
    rows = read_rows(csv_path)
    if not rows:
        raise SystemExit(f"No trajectory rows found in {csv_path}")

    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required: python3 -m pip install matplotlib") from exc

    metadata = read_metadata(run_dir)
    title = run_dir.name
    if metadata and metadata.get("apriltag_tag_length_m"):
        title += f" | tag {metadata['apriltag_tag_length_m']:.5f} m"

    fig = plt.figure(figsize=(14, 10))
    fig.suptitle(title)

    ax_3d = fig.add_subplot(2, 2, 1, projection="3d")
    plot_tag_xyz(ax_3d, rows)

    ax_top = fig.add_subplot(2, 2, 2)
    plot_topdown(ax_top, rows, metadata)

    ax_errors = fig.add_subplot(2, 2, 3)
    plot_errors(ax_errors, rows)

    ax_cmd = fig.add_subplot(2, 2, 4)
    plot_commands(ax_cmd, rows)

    fig.tight_layout()
    default_out = run_dir / "trajectory_plot.png"
    save_figure(fig, default_out)
    if args.save:
        out = Path(args.save).expanduser()
        if not out.is_absolute():
            out = (Path.cwd() / out).resolve()
        if out != default_out.resolve():
            save_figure(fig, out)
    if not args.no_show:
        plt.show()
    return 0


def save_figure(fig, out: Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=160)
    print(f"saved {out}")


def resolve_run(arg: Optional[str]) -> tuple[Path, Path]:
    root = Path(__file__).resolve().parent
    path = Path(arg).expanduser() if arg else root
    if not path.is_absolute():
        cwd_path = (Path.cwd() / path).resolve()
        script_path = (root / path).resolve()
        path = cwd_path if cwd_path.exists() else script_path

    if path.is_file():
        if path.name != "trajectory.csv":
            raise SystemExit(f"Expected trajectory.csv, got {path}")
        return path.parent, path

    if (path / "trajectory.csv").is_file():
        return path, path / "trajectory.csv"

    candidates = sorted(
        (item for item in path.glob("egocentric_servo_*/trajectory.csv") if item.is_file()),
        key=lambda item: item.stat().st_mtime,
    )
    if not candidates:
        raise SystemExit(f"No egocentric_servo_*/trajectory.csv runs found under {path}")
    csv_path = candidates[-1]
    return csv_path.parent, csv_path


def read_rows(csv_path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            rows.append({key: parse_cell(value) for key, value in row.items()})
    if rows:
        t0 = value(rows[0], "stamp_sec") or 0.0
        for row in rows:
            stamp = value(row, "stamp_sec")
            row["t_rel_sec"] = stamp - t0 if stamp is not None else None
    return rows


def read_metadata(run_dir: Path) -> dict:
    path = run_dir / "metadata.json"
    if not path.is_file():
        raise SystemExit(f"Missing metadata.json in {run_dir}")
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Failed to read {path}: {exc}") from exc


def parse_cell(value: Optional[str]) -> object:
    if value in {None, "", "None"}:
        return None
    if value == "True":
        return True
    if value == "False":
        return False
    try:
        return float(value)
    except (TypeError, ValueError):
        return value


def series(rows: Iterable[dict[str, object]], key: str) -> list[Optional[float]]:
    return [value(row, key) for row in rows]


def value(row: dict[str, object], key: str) -> Optional[float]:
    raw = row.get(key)
    return raw if isinstance(raw, float) else None


def valid_xyz(rows: list[dict[str, object]], keys: tuple[str, str, str]) -> tuple[list[float], list[float], list[float]]:
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    for row in rows:
        x = value(row, keys[0])
        y = value(row, keys[1])
        z = value(row, keys[2])
        if x is None or y is None or z is None:
            continue
        xs.append(x)
        ys.append(y)
        zs.append(z)
    return xs, ys, zs


def plot_tag_xyz(ax, rows: list[dict[str, object]]) -> None:
    x, y, z = valid_xyz(rows, ("tag_x_camera_m", "tag_y_camera_m", "tag_z_camera_m"))
    if x:
        ax.plot(x, y, z, label="AprilTag camera XYZ", color="#1f77b4")
        ax.scatter([x[0]], [y[0]], [z[0]], color="#2ca02c", label="start")
        ax.scatter([x[-1]], [y[-1]], [z[-1]], color="#d62728", label="end")
        set_equal_3d_axes(ax, x, y, z)
    ax.set_xlabel("camera X right (m)")
    ax.set_ylabel("camera Y down (m)")
    ax.set_zlabel("camera Z forward (m)")
    set_camera_y_vertical_view(ax)
    ax.set_title("AprilTag 3D Camera-Frame Trajectory")
    ax.legend(loc="best")


def set_equal_3d_axes(ax, xs: list[float], ys: list[float], zs: list[float]) -> None:
    x_mid, x_radius = midpoint_radius(xs)
    y_mid, y_radius = midpoint_radius(ys)
    z_mid, z_radius = midpoint_radius(zs)
    radius = max(x_radius, y_radius, z_radius, 0.05)
    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(z_mid - radius, z_mid + radius)
    try:
        ax.set_box_aspect((1.0, 1.0, 1.0))
    except AttributeError:
        pass


def set_camera_y_vertical_view(ax) -> None:
    try:
        ax.view_init(elev=18.0, azim=-70.0, vertical_axis="y")
    except TypeError:
        ax.view_init(elev=18.0, azim=-70.0)


def midpoint_radius(values: list[float]) -> tuple[float, float]:
    low = min(values)
    high = max(values)
    return (low + high) / 2.0, (high - low) / 2.0


def plot_topdown(ax, rows: list[dict[str, object]], metadata: dict) -> None:
    tag_x = series(rows, "mpc_x_tag_m")
    tag_y = series(rows, "mpc_y_tag_m")
    odom_x = series(rows, "odom_mpc_x_tag_m")
    odom_y = series(rows, "odom_mpc_y_tag_m")
    if not any(x is not None for x in tag_x) and not any(x is not None for x in odom_x):
        raise SystemExit("trajectory.csv does not contain MPC trajectory fields")

    plot_mpc_horizons(ax, rows)
    odom_x_aligned, odom_y_aligned = final_align_xy(tag_x, tag_y, odom_x, odom_y)
    tag_y_display = display_y_series(tag_y)
    odom_y_display = display_y_series(odom_y_aligned)
    plot_xy(ax, tag_x, tag_y_display, "AprilTag pose-derived motion", "#1f77b4")
    plot_xy(ax, odom_x_aligned, odom_y_display, "legged odometry (final-aligned)", "#ff7f0e")
    plot_initial_tag(ax, metadata)
    plot_goal(ax, rows)
    ax.axhline(0.0, color="#cccccc", linewidth=0.8)
    ax.axvline(0.0, color="#cccccc", linewidth=0.8)
    fit_equal_2d_axes(ax, tag_x, tag_y_display, odom_x_aligned, odom_y_display, rows)
    ax.set_xlabel("MPC tag-frame +X (m)")
    ax.set_ylabel("display +Y (mirrored MPC tag-frame Y, m)")
    ax.set_title("Top-Down MPC Trajectories (Final-Aligned Odom)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.25)


def plot_initial_tag(ax, metadata: dict) -> None:
    from matplotlib.patches import Rectangle

    tag_size = metadata.get("apriltag_tag_length_m")
    if not isinstance(tag_size, (float, int)):
        raise SystemExit("metadata.json must contain apriltag_tag_length_m")
    cx, cy = 0.0, 0.0
    tag_height = float(tag_size)
    tag_thickness = max(0.025, tag_height * 0.08)
    rect = Rectangle(
        (cx - tag_thickness / 2.0, cy - tag_height / 2.0),
        tag_thickness,
        tag_height,
        facecolor="#2ca02c",
        edgecolor="#1b7f1b",
        linewidth=1.2,
        alpha=0.35,
        label="initial tag face",
    )
    ax.add_patch(rect)
    ax.arrow(cx, cy, 0.18, 0.0, head_width=0.035, head_length=0.04, color="#2ca02c", length_includes_head=True)


def plot_goal(ax, rows: list[dict[str, object]]) -> None:
    goal = goal_xy(rows)
    if goal is None:
        return
    goal_x, goal_y = goal
    ax.scatter([goal_x], [goal_y], marker="x", color="#2ca02c", s=70, linewidths=2.0, label="MPC goal")


def goal_xy(rows: list[dict[str, object]]) -> Optional[tuple[float, float]]:
    for row in rows:
        x = value(row, "mpc_x_tag_m")
        error = value(row, "range_error_m")
        if x is not None and error is not None:
            return x - error, 0.0
    return None


def fit_equal_2d_axes(
    ax,
    tag_x: list[Optional[float]],
    tag_y: list[Optional[float]],
    odom_x: list[Optional[float]],
    odom_y: list[Optional[float]],
    rows: list[dict[str, object]],
) -> None:
    xs: list[float] = [0.0]
    ys: list[float] = [0.0]
    collect_valid_xy(xs, ys, tag_x, tag_y)
    collect_valid_xy(xs, ys, odom_x, odom_y)
    goal = goal_xy(rows)
    if goal is not None:
        xs.append(goal[0])
        ys.append(goal[1])
    for states in (parse_json_matrix(row.get("mpc_predicted_states_json")) for row in rows):
        for state in states:
            if len(state) >= 2:
                xs.append(state[0])
                ys.append(display_y(state[1]))
    x_mid, x_radius = midpoint_radius(xs)
    y_mid, y_radius = midpoint_radius(ys)
    radius = max(x_radius, y_radius, 0.1)
    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_aspect("equal", adjustable="box")


def collect_valid_xy(
    out_x: list[float],
    out_y: list[float],
    xs: list[Optional[float]],
    ys: list[Optional[float]],
) -> None:
    for x, y in zip(xs, ys):
        if x is None or y is None:
            continue
        out_x.append(x)
        out_y.append(y)


def plot_mpc_horizons(ax, rows: list[dict[str, object]]) -> None:
    horizon_rows = [
        (index, parse_json_matrix(row.get("mpc_predicted_states_json")))
        for index, row in enumerate(rows)
    ]
    horizon_rows = [(index, states) for index, states in horizon_rows if states]
    if not horizon_rows:
        return
    recent = horizon_rows[-50:]
    for index, states in recent[:-1]:
        xs = [state[0] for state in states if len(state) >= 2]
        ys = [display_y(state[1]) for state in states if len(state) >= 2]
        if len(xs) >= 2:
            ax.plot(xs, ys, linestyle="--", color="#e67e22", alpha=0.18, linewidth=1.0)
    _, latest = recent[-1]
    xs = [state[0] for state in latest if len(state) >= 2]
    ys = [display_y(state[1]) for state in latest if len(state) >= 2]
    if len(xs) >= 2:
        ax.plot(xs, ys, linestyle="-", color="#e67e22", alpha=0.9, linewidth=2.0, label="latest MPC horizon")


def parse_json_matrix(value: object) -> list[list[float]]:
    if not isinstance(value, str) or not value:
        return []
    try:
        payload = json.loads(value)
    except json.JSONDecodeError:
        return []
    if not isinstance(payload, list):
        return []
    rows: list[list[float]] = []
    for row in payload:
        if not isinstance(row, list):
            continue
        try:
            values = [float(item) for item in row]
        except (TypeError, ValueError):
            continue
        rows.append(values)
    return rows


def plot_xy(ax, xs: list[Optional[float]], ys: list[Optional[float]], label: str, color: str) -> None:
    valid = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if not valid:
        return
    vx, vy = zip(*valid)
    ax.plot(vx, vy, label=label, color=color)
    ax.scatter([vx[0]], [vy[0]], color=color, marker="o", s=35)
    ax.scatter([vx[-1]], [vy[-1]], color=color, marker="x", s=45)


def final_align_xy(
    reference_x: list[Optional[float]],
    reference_y: list[Optional[float]],
    moving_x: list[Optional[float]],
    moving_y: list[Optional[float]],
) -> tuple[list[Optional[float]], list[Optional[float]]]:
    offset = final_alignment_offset(reference_x, reference_y, moving_x, moving_y)
    if offset is None:
        return moving_x, moving_y
    dx, dy = offset
    return shift_series(moving_x, dx), shift_series(moving_y, dy)


def final_alignment_offset(
    reference_x: list[Optional[float]],
    reference_y: list[Optional[float]],
    moving_x: list[Optional[float]],
    moving_y: list[Optional[float]],
) -> Optional[tuple[float, float]]:
    for rx, ry, mx, my in reversed(list(zip(reference_x, reference_y, moving_x, moving_y))):
        if rx is None or ry is None or mx is None or my is None:
            continue
        return rx - mx, ry - my
    return None


def shift_series(values: list[Optional[float]], offset: float) -> list[Optional[float]]:
    return [None if value is None else value + offset for value in values]


def display_y(y: float) -> float:
    return DISPLAY_Y_SIGN * float(y)


def display_optional_y(y: Optional[float]) -> Optional[float]:
    return None if y is None else display_y(y)


def display_y_series(values: list[Optional[float]]) -> list[Optional[float]]:
    return [display_optional_y(value) for value in values]


def display_theta(theta: float) -> float:
    return -float(theta) if DISPLAY_Y_SIGN < 0.0 else float(theta)


def subtract_series(left: list[Optional[float]], right: list[Optional[float]]) -> list[Optional[float]]:
    return [
        None if left_value is None or right_value is None else left_value - right_value
        for left_value, right_value in zip(left, right)
    ]


def plot_errors(ax, rows: list[dict[str, object]]) -> None:
    t = series(rows, "t_rel_sec")
    for key, label in [
        ("range_error_m", "range error m"),
        ("lateral_error_m", "lateral error m"),
        ("yaw_error_rad", "yaw error rad"),
    ]:
        plot_time(ax, t, series(rows, key), label)
    tag_x = series(rows, "mpc_x_tag_m")
    tag_y = series(rows, "mpc_y_tag_m")
    odom_x = series(rows, "odom_mpc_x_tag_m")
    odom_y = series(rows, "odom_mpc_y_tag_m")
    odom_x_aligned, odom_y_aligned = final_align_xy(tag_x, tag_y, odom_x, odom_y)
    plot_time(ax, t, subtract_series(tag_x, odom_x_aligned), "tag-odom x drift m (final-aligned)")
    plot_time(
        ax,
        t,
        subtract_series(display_y_series(tag_y), display_y_series(odom_y_aligned)),
        "tag-odom display y drift m (final-aligned)",
    )
    ax.axhline(0.0, color="#cccccc", linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_title("Servo Errors")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.25)


def plot_commands(ax, rows: list[dict[str, object]]) -> None:
    t = series(rows, "t_rel_sec")
    for key, label in [
        ("cmd_heading", "heading"),
        ("cmd_lateral", "lateral"),
        ("cmd_turning", "turning"),
    ]:
        plot_time(ax, t, series(rows, key), label)
    ax.axhline(0.0, color="#cccccc", linewidth=0.8)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("normalized command")
    ax.set_title("Base Commands")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.25)


def plot_time(ax, t: list[Optional[float]], y: list[Optional[float]], label: str) -> None:
    valid = [(tx, yy) for tx, yy in zip(t, y) if tx is not None and yy is not None]
    if not valid:
        return
    vx, vy = zip(*valid)
    ax.plot(vx, vy, label=label)


if __name__ == "__main__":
    raise SystemExit(main())
