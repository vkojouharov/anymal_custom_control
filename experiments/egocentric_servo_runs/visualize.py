#!/usr/bin/env python3
"""Visualize synchronized ANYmal egocentric servo trajectory logs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Iterable, Optional


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
    if args.save:
        out = Path(args.save)
        out.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(out, dpi=160)
        print(f"saved {out}")
    if not args.no_show:
        plt.show()
    return 0


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
    ax.set_xlabel("camera X right (m)")
    ax.set_ylabel("camera Y down (m)")
    ax.set_zlabel("camera Z forward (m)")
    ax.set_title("AprilTag 3D Camera-Frame Trajectory")
    ax.legend(loc="best")


def plot_topdown(ax, rows: list[dict[str, object]], metadata: dict) -> None:
    tag_x = series(rows, "tag_aligned_rel_x_m")
    tag_y = series(rows, "tag_aligned_rel_y_m")
    odom_x = series(rows, "odom_tag_aligned_rel_x_m")
    odom_y = series(rows, "odom_tag_aligned_rel_y_m")
    if not any(x is not None for x in tag_x) and not any(x is not None for x in odom_x):
        raise SystemExit("trajectory.csv does not contain the new tag-aligned trajectory fields")

    plot_xy(ax, tag_x, tag_y, "AprilTag pose-derived motion", "#1f77b4")
    plot_xy(ax, odom_x, odom_y, "legged odometry", "#ff7f0e")
    plot_initial_tag(ax, metadata)
    ax.axhline(0.0, color="#cccccc", linewidth=0.8)
    ax.axvline(0.0, color="#cccccc", linewidth=0.8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("start tag-frame +X into tag face (m)")
    ax.set_ylabel("start tag-frame +Y tag-left (m)")
    ax.set_title("Top-Down Tag-Aligned Trajectories")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.25)


def plot_initial_tag(ax, metadata: dict) -> None:
    tag = metadata.get("origin", {}).get("tag") if isinstance(metadata, dict) else None
    if not isinstance(tag, dict):
        raise SystemExit("metadata.json must contain origin.tag")
    center = tag.get("tag_aligned_center_m")
    if not _is_vector2(center):
        raise SystemExit("metadata.json must contain origin.tag.tag_aligned_center_m")
    tag_size = tag.get("tag_size_m") or metadata.get("apriltag_tag_length_m")
    if not isinstance(tag_size, (float, int)):
        raise SystemExit("metadata.json must contain origin.tag.tag_size_m or apriltag_tag_length_m")

    from matplotlib.patches import Rectangle

    cx, cy = float(center[0]), float(center[1])
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
    ax.arrow(cx, cy, -0.18, 0.0, head_width=0.035, head_length=0.04, color="#2ca02c", length_includes_head=True)


def _is_vector2(value: object) -> bool:
    return isinstance(value, list) and len(value) == 2 and all(isinstance(item, (float, int)) for item in value)


def plot_xy(ax, xs: list[Optional[float]], ys: list[Optional[float]], label: str, color: str) -> None:
    valid = [(x, y) for x, y in zip(xs, ys) if x is not None and y is not None]
    if not valid:
        return
    vx, vy = zip(*valid)
    ax.plot(vx, vy, label=label, color=color)
    ax.scatter([vx[0]], [vy[0]], color=color, marker="o", s=35)
    ax.scatter([vx[-1]], [vy[-1]], color=color, marker="x", s=45)


def plot_errors(ax, rows: list[dict[str, object]]) -> None:
    t = series(rows, "t_rel_sec")
    for key, label in [
        ("range_error_m", "range error m"),
        ("lateral_error_m", "lateral error m"),
        ("yaw_error_rad", "yaw error rad"),
    ]:
        plot_time(ax, t, series(rows, key), label)
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
