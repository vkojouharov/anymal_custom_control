#!/usr/bin/env python3
"""Animate all ten July 23 hardware runs converging on the same target.

All runs start together at 10 Hz.  A run that finishes early remains parked
at its final measured pose until the slowest run converges.  The animation
then holds briefly on the completed field.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
MPC_HZ = 10.0
GOAL = np.array([1.0, 0.0])
FINAL_HOLD_SECONDS = 1.5

WARM_COLORS = (
    "#7f1d1d",
    "#991b1b",
    "#b91c1c",
    "#dc2626",
    "#ef4444",
    "#f05a47",
    "#f97316",
    "#fb8c24",
    "#f59e0b",
    "#eab308",
)

TAG_BLUE = "#0e9ed5"
GOAL_GREEN = "#16a34a"
BACKGROUND = "#ffffff"
GRID = "#d6cfc8"

FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
LOG_ROW = re.compile(
    rf"^\s*state=\[\s*({FLOAT})\s+({FLOAT})\s+({FLOAT})\s*\]"
    rf"\s+u0=\[\s*({FLOAT})\s+({FLOAT})\s+({FLOAT})\s*\]"
    rf"\s+state_cost=({FLOAT})"
    rf"\s+solve={FLOAT}ms\s*$"
)


@dataclass(frozen=True)
class RunData:
    name: str
    states: np.ndarray
    linear_controls: np.ndarray
    state_costs: np.ndarray
    color: str

    @property
    def convergence_time(self) -> float:
        return (len(self.states) - 1) / MPC_HZ


def run_number(path: Path) -> int:
    match = re.fullmatch(r"run(\d+)", path.stem)
    if match is None:
        raise ValueError(f"Expected a run<number>.txt filename, got {path.name}")
    return int(match.group(1))


def default_inputs() -> list[Path]:
    paths = sorted(SCRIPT_DIR.glob("run*.txt"), key=run_number)
    if len(paths) != 10:
        raise ValueError(f"Expected 10 run logs beside this script, found {len(paths)}")
    return paths


def parse_run(path: Path, color: str) -> RunData:
    states: list[list[float]] = []
    linear_controls: list[list[float]] = []
    state_costs: list[float] = []
    malformed: list[tuple[int, str]] = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        match = LOG_ROW.match(line)
        if match is not None:
            values = [float(value) for value in match.groups()]
            states.append(values[:3])
            linear_controls.append(values[3:5])
            state_costs.append(values[6])
        elif line.startswith("state="):
            malformed.append((line_number, raw_line))
    if malformed:
        details = "\n".join(f"  line {number}: {line}" for number, line in malformed)
        raise ValueError(f"Malformed MPC rows in {path}:\n{details}")
    if not states:
        raise ValueError(f"No MPC state rows found in {path}")
    return RunData(
        name=path.stem,
        states=np.asarray(states, dtype=float),
        linear_controls=np.asarray(linear_controls, dtype=float),
        state_costs=np.asarray(state_costs, dtype=float),
        color=color,
    )


def display_xy(states: np.ndarray) -> np.ndarray:
    """Convert solver tag-frame positions to the established replay view."""
    return -states[:, :2]


def plot_limits(runs: list[RunData]) -> tuple[tuple[float, float], tuple[float, float]]:
    points = np.vstack(
        [
            *(display_xy(run.states) for run in runs),
            np.array([[0.0, 0.0], [-GOAL[0], -GOAL[1]]]),
        ]
    )
    x_min, y_min = np.min(points, axis=0)
    x_max, y_max = np.max(points, axis=0)
    return (
        (float(x_min - 0.55), float(x_max + 0.45)),
        (float(y_min - 0.55), float(y_max + 0.55)),
    )


def draw_robot(ax, state: np.ndarray, color: str, label: str, completed: bool) -> None:
    from matplotlib import patheffects
    from matplotlib.patches import Polygon

    # Hardware theta is the negative of geometric body heading; the camera
    # faces body -X, matching the convention used by the individual replays.
    x, y = -state[:2]
    heading = math.pi - state[2]
    length, width = 0.38, 0.24
    body = np.asarray(
        [
            [0.62 * length, 0.0],
            [0.36 * length, -0.50 * width],
            [-0.50 * length, -0.50 * width],
            [-0.50 * length, 0.50 * width],
            [0.36 * length, 0.50 * width],
        ]
    )
    c, s = math.cos(heading), math.sin(heading)
    rotation = np.array([[c, -s], [s, c]])
    vertices = body @ rotation.T + np.array([x, y])

    if completed:
        ax.scatter(
            [x],
            [y],
            s=310,
            facecolor=color,
            edgecolor="none",
            alpha=0.13,
            zorder=7,
        )
    ax.add_patch(
        Polygon(
            vertices,
            closed=True,
            facecolor=color,
            edgecolor="white",
            linewidth=1.15,
            alpha=0.88,
            zorder=9,
        )
    )
    text = ax.text(
        x,
        y,
        label.removeprefix("run"),
        ha="center",
        va="center",
        color="white",
        fontsize=7,
        fontweight="bold",
        zorder=10,
    )
    text.set_path_effects([patheffects.withStroke(linewidth=1.0, foreground="#532014")])


def draw_tag_and_goal(ax, pulse: float) -> None:
    from matplotlib.patches import Rectangle

    ax.add_patch(
        Rectangle(
            (-0.04, -0.25),
            0.08,
            0.50,
            facecolor=TAG_BLUE,
            edgecolor="#073b4c",
            linewidth=1.0,
            zorder=5,
        )
    )
    ax.text(0.10, 0.0, "AprilTag", color="#07516b", va="center", fontsize=8, fontweight="bold")

    gx, gy = -GOAL
    ax.scatter(
        [gx],
        [gy],
        s=430 + 90 * pulse,
        facecolor=GOAL_GREEN,
        edgecolor="none",
        alpha=0.10,
        zorder=4,
    )
    ax.scatter(
        [gx],
        [gy],
        s=145,
        facecolor="white",
        edgecolor=GOAL_GREEN,
        linewidth=2.4,
        zorder=6,
    )
    ax.scatter([gx], [gy], marker="x", s=72, color=GOAL_GREEN, linewidth=2.4, zorder=7)
    ax.text(gx, gy - 0.28, "TARGET", color="#137333", ha="center", fontsize=8, fontweight="bold")


def draw_status_panel(status_ax, runs: list[RunData], data_frame: int, all_done: bool) -> None:
    from matplotlib.patches import FancyBboxPatch

    status_ax.clear()
    status_ax.set_facecolor(BACKGROUND)
    status_ax.set_xlim(0.0, 1.0)
    status_ax.set_ylim(0.0, 1.0)
    status_ax.axis("off")
    status_ax.text(
        0.04,
        0.96,
        "CONVERGENCE BOARD",
        color="#4a2922",
        fontsize=11,
        fontweight="bold",
        va="top",
    )

    for row, run in enumerate(runs):
        y = 0.875 - row * 0.071
        completed = data_frame >= len(run.states) - 1
        current = min(data_frame, len(run.states) - 1)
        status_ax.plot([0.05, 0.19], [y, y], color=run.color, linewidth=5, solid_capstyle="round")
        status_ax.text(0.24, y, run.name, va="center", fontsize=9, color="#4a2922", fontweight="bold")
        if completed:
            status_ax.text(
                0.94,
                y,
                f"✓ {run.convergence_time:.1f}s",
                va="center",
                ha="right",
                fontsize=9,
                color=GOAL_GREEN,
                fontweight="bold",
            )
        else:
            status_ax.text(
                0.94,
                y,
                f"cost {run.state_costs[current]:.1f}",
                va="center",
                ha="right",
                fontsize=8,
                color="#765047",
            )

    completed_count = sum(data_frame >= len(run.states) - 1 for run in runs)
    status_ax.text(
        0.05,
        0.115,
        f"{completed_count} / {len(runs)} converged",
        color=GOAL_GREEN if all_done else "#765047",
        fontsize=10,
        fontweight="bold",
    )
    status_ax.add_patch(
        FancyBboxPatch(
            (0.05, 0.060),
            0.89,
            0.025,
            boxstyle="round,pad=0.004,rounding_size=0.012",
            facecolor="#eadfd8",
            edgecolor="none",
        )
    )
    status_ax.add_patch(
        FancyBboxPatch(
            (0.05, 0.060),
            0.89 * completed_count / len(runs),
            0.025,
            boxstyle="round,pad=0.004,rounding_size=0.012",
            facecolor=GOAL_GREEN,
            edgecolor="none",
        )
    )


def draw_frame(
    field_ax,
    status_ax,
    runs: list[RunData],
    frame: int,
    data_frame_count: int,
    limits: tuple[tuple[float, float], tuple[float, float]],
) -> None:
    field_ax.clear()
    field_ax.set_facecolor(BACKGROUND)
    data_frame = min(frame, data_frame_count - 1)
    time_s = data_frame / MPC_HZ
    all_done = data_frame == data_frame_count - 1
    pulse = 0.5 + 0.5 * math.sin(2.0 * math.pi * frame / MPC_HZ)

    for run in runs:
        current = min(data_frame, len(run.states) - 1)
        path = display_xy(run.states[: current + 1])
        start = display_xy(run.states[:1])[0]
        field_ax.scatter(
            [start[0]],
            [start[1]],
            s=24,
            facecolor=BACKGROUND,
            edgecolor=run.color,
            linewidth=1.2,
            alpha=0.85,
            zorder=2,
        )
        field_ax.plot(
            path[:, 0],
            path[:, 1],
            color=run.color,
            linewidth=2.0,
            alpha=0.74,
            zorder=3,
        )
        # Position is displayed as negative tag-frame position, so negate the
        # logged tag-frame planar control to express it in the same view.
        control = -run.linear_controls[current]
        field_ax.quiver(
            path[-1, 0],
            path[-1, 1],
            control[0],
            control[1],
            angles="xy",
            scale_units="xy",
            scale=1.0,
            color=run.color,
            width=0.006,
            headwidth=4.2,
            headlength=5.2,
            headaxislength=4.5,
            edgecolor="white",
            linewidth=0.35,
            alpha=0.95,
            zorder=8,
        )
        draw_robot(
            field_ax,
            run.states[current],
            run.color,
            run.name,
            completed=data_frame >= len(run.states) - 1,
        )

    draw_tag_and_goal(field_ax, pulse)
    field_ax.axhline(0.0, color=GRID, linewidth=0.8, zorder=0)
    field_ax.axvline(0.0, color=GRID, linewidth=0.8, zorder=0)
    field_ax.set_xlim(*limits[0])
    field_ax.set_ylim(*limits[1])
    field_ax.set_aspect("equal", adjustable="box")
    field_ax.grid(True, color=GRID, linewidth=0.45, alpha=0.55)
    field_ax.set_xlabel("display x [m] (-tag-frame x)", color="#5f4943")
    field_ax.set_ylabel("display y [m] (-tag-frame y)", color="#5f4943")
    for spine in field_ax.spines.values():
        spine.set_color("#bfaea7")

    converged = sum(data_frame >= len(run.states) - 1 for run in runs)
    if all_done:
        subtitle = f"All 10 runs converged  •  slowest arrival {time_s:.1f} s"
    else:
        subtitle = f"t = {time_s:04.1f} s  •  {converged}/10 parked at the target"
    field_ax.set_title(
        f"mpc convergence across workspace\n{subtitle}",
        color=GOAL_GREEN if all_done else "#6b2d20",
        fontsize=15,
        fontweight="bold",
        pad=12,
    )
    draw_status_panel(status_ax, runs, data_frame, all_done)


def render_video(runs: list[RunData], output: Path, fps: float, dpi: int) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "anymal-mpc-matplotlib"))
    os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "anymal-mpc-xdg-cache"))
    try:
        import matplotlib.pyplot as plt
        from matplotlib import animation
        from matplotlib.animation import FFMpegWriter
    except ImportError as exc:
        raise RuntimeError("matplotlib is required to render the convergence video") from exc
    if not animation.writers.is_available("ffmpeg"):
        raise RuntimeError("ffmpeg is required to render the MP4 video")

    data_frame_count = max(len(run.states) for run in runs)
    hold_frames = max(1, int(round(FINAL_HOLD_SECONDS * fps)))
    total_frames = data_frame_count + hold_frames
    limits = plot_limits(runs)

    output.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(12.0, 7.2), facecolor=BACKGROUND)
    grid = fig.add_gridspec(1, 2, width_ratios=(4.4, 1.25), wspace=0.05)
    field_ax = fig.add_subplot(grid[0, 0])
    status_ax = fig.add_subplot(grid[0, 1])
    writer = FFMpegWriter(
        fps=fps,
        metadata={"title": "All July 23 hardware MPC runs converging"},
        codec="h264",
    )
    with writer.saving(fig, str(output), dpi):
        for frame in range(total_frames):
            draw_frame(field_ax, status_ax, runs, frame, data_frame_count, limits)
            writer.grab_frame()
    plt.close(fig)

    print(
        f"Wrote {total_frames} frames ({data_frame_count} data + {hold_frames} final hold) "
        f"at {fps:g} FPS to {output}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "logs",
        nargs="*",
        type=Path,
        help="Ten run<number>.txt logs; defaults to run1.txt through run10.txt",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=SCRIPT_DIR / "all_runs_convergence.mp4",
        help="Output MP4 path",
    )
    parser.add_argument("--fps", type=float, default=MPC_HZ, help="Output video frame rate")
    parser.add_argument("--dpi", type=int, default=140, help="Video resolution scale")
    args = parser.parse_args()

    inputs = args.logs or default_inputs()
    if len(inputs) != len(WARM_COLORS):
        parser.error(f"expected exactly {len(WARM_COLORS)} input logs, got {len(inputs)}")
    if args.fps <= 0.0:
        parser.error("--fps must be positive")
    inputs = sorted((path.expanduser().resolve() for path in inputs), key=run_number)
    runs = [parse_run(path, WARM_COLORS[index]) for index, path in enumerate(inputs)]
    render_video(runs, args.output.expanduser().resolve(), args.fps, args.dpi)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
