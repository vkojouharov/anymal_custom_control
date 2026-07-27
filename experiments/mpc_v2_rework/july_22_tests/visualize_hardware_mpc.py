#!/usr/bin/env python3
"""Convert barebones hardware MPC logs to CSV and render MPC replay videos.

The replay solves the same optimization used by
``run_mpc_barebones_test.py`` at every logged measured state.  Each animation
frame represents one MPC update and is written at the hardware MPC rate of
5 Hz.  The logged command from the preceding row is used as ``u_prev``.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent

MPC_HZ = 5.0
DT = 1.0 / MPC_HZ
HORIZON = 10
GOAL = np.array([1.0, 0.0, 0.0])
P_XY = 50.0
P_THETA = 10.0
R = np.diag([0.1, 0.1, 0.05])
S = np.diag([2.0, 2.0, 0.2])
U_MAX = np.array([0.35, 0.35, 0.2])
DU_MAX = np.array([10.0, 10.0, 10.0])
ALPHA_FOV = math.radians(30.0)

TAG_BLUE = (14.0 / 255.0, 158.0 / 255.0, 213.0 / 255.0)
MPC_ORANGE = (230.0 / 255.0, 126.0 / 255.0, 34.0 / 255.0)
ROBOT_RED = (255.0 / 255.0, 82.0 / 255.0, 74.0 / 255.0)
MEASURED_BLACK = (32.0 / 255.0, 32.0 / 255.0, 32.0 / 255.0)

FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
LOG_ROW = re.compile(
    rf"^\s*state=\[\s*({FLOAT})\s+({FLOAT})\s+({FLOAT})\s*\]"
    rf"\s+u0=\[\s*({FLOAT})\s+({FLOAT})\s+({FLOAT})\s*\]"
    rf"\s+solve=({FLOAT})ms\s*$"
)


@dataclass(frozen=True)
class LogSample:
    step: int
    state: np.ndarray
    command: np.ndarray
    solve_time_ms: float


@dataclass(frozen=True)
class ReplaySolve:
    predicted_states: np.ndarray
    predicted_controls: np.ndarray
    command: np.ndarray
    status: str


def parse_log(path: Path) -> list[LogSample]:
    """Parse all state/command/solve rows and reject malformed data rows."""
    samples: list[LogSample] = []
    malformed: list[tuple[int, str]] = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
        line = raw_line.strip()
        match = LOG_ROW.match(line)
        if match is not None:
            values = [float(value) for value in match.groups()]
            samples.append(
                LogSample(
                    step=len(samples),
                    state=np.asarray(values[:3], dtype=float),
                    command=np.asarray(values[3:6], dtype=float),
                    solve_time_ms=values[6],
                )
            )
        elif line.startswith("state="):
            malformed.append((line_number, raw_line))
    if malformed:
        details = "\n".join(f"  line {number}: {line}" for number, line in malformed)
        raise ValueError(f"Malformed MPC rows in {path}:\n{details}")
    if not samples:
        raise ValueError(f"No MPC state rows found in {path}")
    return samples


def write_csv(samples: list[LogSample], output: Path) -> None:
    """Write normalized state, world-frame command, and timing columns."""
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "step",
                "time_s",
                "state_x_m",
                "state_y_m",
                "state_theta_rad",
                "command_x_mps",
                "command_y_mps",
                "command_theta_radps",
                "solve_time_ms",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    sample.step,
                    f"{sample.step * DT:.6f}",
                    *(f"{value:.9f}" for value in sample.state),
                    *(f"{value:.9f}" for value in sample.command),
                    f"{sample.solve_time_ms:.6f}",
                ]
            )


class HardwareMpcReplaySolver:
    """Exact offline form of the MPC in run_mpc_barebones_test.py."""

    def __init__(self) -> None:
        try:
            import cvxpy as cp
        except ImportError as exc:
            raise RuntimeError("cvxpy is required to reconstruct the MPC horizons") from exc

        self.cp = cp
        self.x = cp.Variable((HORIZON + 1, 3))
        self.u = cp.Variable((HORIZON, 3))
        self.x0 = cp.Parameter(3)
        self.u_prev = cp.Parameter(3)
        self.terminal_factor = cp.Parameter((3, 3))
        self.terminal_target = cp.Parameter(3)
        self.bearing_gradient = cp.Parameter(3)
        self.bearing_offset = cp.Parameter()
        self.weighted_gradient = cp.Parameter(3)
        self.weighted_offset = cp.Parameter()

        constraints = [self.x[0] == self.x0]
        for k in range(HORIZON):
            constraints += [
                self.x[k + 1] == self.x[k] + DT * self.u[k],
                -U_MAX <= self.u[k],
                self.u[k] <= U_MAX,
            ]
        constraints += [
            -DU_MAX <= self.u[0] - self.u_prev,
            self.u[0] - self.u_prev <= DU_MAX,
        ]
        for k in range(1, HORIZON):
            constraints += [
                -DU_MAX <= self.u[k] - self.u[k - 1],
                self.u[k] - self.u[k - 1] <= DU_MAX,
            ]

        beta = [self.bearing_offset + self.bearing_gradient @ self.x[k] for k in range(HORIZON + 1)]
        constraints += [value <= ALPHA_FOV for value in beta]
        constraints += [value >= -ALPHA_FOV for value in beta]

        cost = cp.sum_squares(self.terminal_factor @ self.x[HORIZON] - self.terminal_target)
        cost += cp.sum_squares(
            cp.hstack(
                [
                    self.weighted_offset + self.weighted_gradient @ self.x[k]
                    for k in range(1, HORIZON + 1)
                ]
            )
        )
        r_factor = np.sqrt(R)
        s_factor = np.sqrt(S)
        for k in range(HORIZON):
            cost += cp.sum_squares(r_factor @ self.u[k])
        cost += cp.sum_squares(s_factor @ (self.u[0] - self.u_prev))
        for k in range(1, HORIZON):
            cost += cp.sum_squares(s_factor @ (self.u[k] - self.u[k - 1]))

        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        if not self.problem.is_dcp(dpp=True):
            raise RuntimeError("Reconstructed MPC is not DPP compliant")

    def solve(self, state: np.ndarray, previous_command: np.ndarray) -> ReplaySolve:
        distance = max(float(np.hypot(state[0], state[1])), 1e-6)
        gradient = np.array([-state[1] / distance**2, state[0] / distance**2, 1.0])
        beta0 = wrap(math.atan2(state[1], state[0]) + state[2])
        offset = beta0 - gradient @ state
        factor = np.diag(np.sqrt([P_XY, P_XY, P_THETA / distance**2]))

        self.x0.value = state
        self.u_prev.value = previous_command
        self.terminal_factor.value = factor
        self.terminal_target.value = factor @ GOAL
        self.bearing_gradient.value = gradient
        self.bearing_offset.value = offset
        self.weighted_gradient.value = math.sqrt(distance) * gradient
        self.weighted_offset.value = math.sqrt(distance) * offset

        try:
            self.problem.solve(solver=self.cp.OSQP, warm_start=True, verbose=False)
        except self.cp.SolverError:
            self.problem.solve(solver=self.cp.CLARABEL, warm_start=True, verbose=False)
        status = str(self.problem.status)
        if status not in (str(self.cp.OPTIMAL), str(self.cp.OPTIMAL_INACCURATE)):
            raise RuntimeError(f"MPC solve failed at state {state}: {status}")
        if self.x.value is None or self.u.value is None:
            raise RuntimeError(f"MPC returned no trajectory at state {state}: {status}")
        predicted_states = np.asarray(self.x.value, dtype=float).copy()
        predicted_controls = np.asarray(self.u.value, dtype=float).copy()
        return ReplaySolve(
            predicted_states=predicted_states,
            predicted_controls=predicted_controls,
            command=predicted_controls[0].copy(),
            status=status,
        )


def resolve_horizons(samples: list[LogSample]) -> list[ReplaySolve]:
    solver = HardwareMpcReplaySolver()
    solutions: list[ReplaySolve] = []
    for index, sample in enumerate(samples):
        previous_command = np.zeros(3) if index == 0 else samples[index - 1].command
        solutions.append(solver.solve(sample.state, previous_command))
    return solutions


def draw_oriented_rectangle(ax, pose: np.ndarray, length: float, width: float, color, label=None) -> None:
    from matplotlib.patches import Polygon

    px, py, theta = pose
    corners = np.asarray(
        [
            [length / 2.0, width / 2.0],
            [length / 2.0, -width / 2.0],
            [-length / 2.0, -width / 2.0],
            [-length / 2.0, width / 2.0],
        ]
    )
    c, s = math.cos(theta), math.sin(theta)
    rotation = np.array([[c, -s], [s, c]])
    world = corners @ rotation.T + np.array([px, py])
    ax.add_patch(
        Polygon(
            world,
            closed=True,
            facecolor=color,
            edgecolor="black",
            linewidth=1.0,
            alpha=0.88,
            label=label,
            zorder=7,
        )
    )


def draw_fov_cone(ax, state: np.ndarray, max_range: float) -> None:
    from matplotlib.patches import Polygon

    x, y, theta = state
    # Hardware theta is the negative of the geometric body heading. The camera
    # faces body -X, so its optical-axis angle in the solver frame is pi-theta.
    center = math.pi - theta
    angles = np.linspace(center - ALPHA_FOV, center + ALPHA_FOV, 50)
    arc = np.column_stack([x + max_range * np.cos(angles), y + max_range * np.sin(angles)])
    vertices = -np.vstack(([x, y], arc))
    ax.add_patch(
        Polygon(
            vertices,
            closed=True,
            facecolor=TAG_BLUE,
            edgecolor=TAG_BLUE,
            linewidth=0.8,
            alpha=0.18,
            zorder=0,
        )
    )


def plot_limits(samples: list[LogSample], solutions: list[ReplaySolve]) -> tuple[tuple[float, float], tuple[float, float]]:
    states = np.asarray([sample.state for sample in samples])
    predicted = np.vstack([solution.predicted_states for solution in solutions])
    xs = -np.concatenate(([0.0, GOAL[0]], states[:, 0], predicted[:, 0]))
    ys = -np.concatenate(([0.0, GOAL[1]], states[:, 1], predicted[:, 1]))
    x_min, x_max = float(np.min(xs)), float(np.max(xs))
    y_min, y_max = float(np.min(ys)), float(np.max(ys))
    x_pad = max(0.35, 0.08 * max(x_max - x_min, 1.0))
    y_pad = max(0.35, 0.08 * max(y_max - y_min, 1.0))
    return (x_min - x_pad, x_max + x_pad), (y_min - y_pad, y_max + y_pad)


def choose_legend_corner(
    samples: list[LogSample],
    solutions: list[ReplaySolve],
    limits: tuple[tuple[float, float], tuple[float, float]],
) -> str:
    """Choose one low-occupancy corner and keep it fixed for the whole run."""
    measured = np.asarray([sample.state[:2] for sample in samples])
    predicted = np.vstack([solution.predicted_states[:, :2] for solution in solutions])
    anchors = np.asarray([[0.0, 0.0], GOAL[:2]])
    points = -np.vstack([measured, predicted, anchors])
    x_normalized = (points[:, 0] - limits[0][0]) / (limits[0][1] - limits[0][0])
    y_normalized = (points[:, 1] - limits[1][0]) / (limits[1][1] - limits[1][0])
    candidates = [
        ("upper right", x_normalized > 0.58, y_normalized > 0.70),
        ("upper left", x_normalized < 0.42, y_normalized > 0.70),
        ("lower right", x_normalized > 0.58, y_normalized < 0.30),
        ("lower left", x_normalized < 0.42, y_normalized < 0.30),
    ]
    return min(candidates, key=lambda candidate: int(np.count_nonzero(candidate[1] & candidate[2])))[0]


def draw_frame(
    ax,
    samples: list[LogSample],
    solutions: list[ReplaySolve],
    frame: int,
    limits: tuple[tuple[float, float], tuple[float, float]],
    run_name: str,
    tail_count: int,
    legend_loc: str,
) -> None:
    ax.clear()
    sample = samples[frame]
    solution = solutions[frame]

    start = max(0, frame - tail_count)
    for old_index in range(start, frame):
        tail_start = frame - old_index
        old_prediction = solutions[old_index].predicted_states
        if tail_start >= len(old_prediction):
            continue
        tail = old_prediction[tail_start:]
        ax.plot(-tail[:, 0], -tail[:, 1], "--", color=MPC_ORANGE, alpha=0.35, linewidth=1.0, zorder=1)

    prediction = solution.predicted_states
    ax.plot(
        -prediction[:, 0],
        -prediction[:, 1],
        color=MPC_ORANGE,
        linewidth=2.0,
        marker=".",
        markersize=3.0,
        label="re-solved MPC horizon",
        zorder=3,
    )
    ax.scatter([-prediction[-1, 0]], [-prediction[-1, 1]], marker="s", color=MPC_ORANGE, s=28, zorder=4)

    measured = np.asarray([value.state for value in samples[: frame + 1]])
    ax.plot(
        -measured[:, 0],
        -measured[:, 1],
        color=MEASURED_BLACK,
        linewidth=1.8,
        marker="o",
        markersize=2.5,
        label="measured hardware path",
        zorder=5,
    )

    max_range = max(1.5, float(np.hypot(sample.state[0], sample.state[1])) * 1.15)
    draw_fov_cone(ax, sample.state, max_range=max_range)
    draw_oriented_rectangle(ax, np.array([0.0, 0.0, 0.0]), 0.08, 0.50, TAG_BLUE, label="AprilTag")
    draw_oriented_rectangle(
        ax,
        np.array([-sample.state[0], -sample.state[1], math.pi - sample.state[2]]),
        0.70,
        0.40,
        ROBOT_RED,
        label="ANYmal",
    )
    ax.scatter([-GOAL[0]], [-GOAL[1]], marker="x", color="green", s=70, linewidths=2.0, label="goal", zorder=8)
    ax.axhline(0.0, color="0.82", linewidth=0.7, zorder=-1)
    ax.axvline(0.0, color="0.82", linewidth=0.7, zorder=-1)

    ax.set_title(
        f"{run_name} | state = [{sample.state[0]:.3f}, {sample.state[1]:.3f}, {sample.state[2]:.3f}] | "
        f"control = [{sample.command[0]:.3f}, {sample.command[1]:.3f}, {sample.command[2]:.3f}]"
    )
    ax.set_xlabel("display x [m] (-tag-frame x)")
    ax.set_ylabel("display y [m] (-tag-frame y)")
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.4, alpha=0.35)
    ax.legend(loc=legend_loc, fontsize=8)


def render_video(
    samples: list[LogSample],
    solutions: list[ReplaySolve],
    output: Path,
    run_name: str,
    fps: float,
    dpi: int,
    tail_count: int,
) -> None:
    os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "anymal-mpc-matplotlib"))
    os.environ.setdefault("XDG_CACHE_HOME", str(Path(tempfile.gettempdir()) / "anymal-mpc-xdg-cache"))
    try:
        import matplotlib.pyplot as plt
        from matplotlib import animation
        from matplotlib.animation import FFMpegWriter
    except ImportError as exc:
        raise RuntimeError("matplotlib is required to render MPC videos") from exc
    if not animation.writers.is_available("ffmpeg"):
        raise RuntimeError("ffmpeg is required to render MP4 videos")

    output.parent.mkdir(parents=True, exist_ok=True)
    limits = plot_limits(samples, solutions)
    legend_loc = choose_legend_corner(samples, solutions, limits)
    fig, ax = plt.subplots(figsize=(9.5, 6.0))
    writer = FFMpegWriter(fps=fps, metadata={"title": f"Hardware MPC replay: {run_name}"})
    with writer.saving(fig, str(output), dpi):
        for frame in range(len(samples)):
            draw_frame(ax, samples, solutions, frame, limits, run_name, tail_count, legend_loc)
            writer.grab_frame()
    plt.close(fig)


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def default_inputs() -> list[Path]:
    return sorted(SCRIPT_DIR.glob("run*.txt"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path, help="Input terminal logs; defaults to run*.txt beside this script")
    parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR, help="Directory for CSV and MP4 outputs")
    parser.add_argument("--fps", type=float, default=MPC_HZ, help="Video frames per second (default: hardware MPC rate)")
    parser.add_argument("--dpi", type=int, default=140, help="Video resolution scale")
    parser.add_argument("--tail-count", type=int, default=HORIZON, help="Number of prior horizon traces to retain")
    parser.add_argument("--csv-only", action="store_true", help="Only convert logs; do not solve or render videos")
    args = parser.parse_args()

    inputs = args.logs or default_inputs()
    if not inputs:
        parser.error("no input logs found")
    if args.fps <= 0.0:
        parser.error("--fps must be positive")

    for input_path in inputs:
        path = input_path.expanduser().resolve()
        samples = parse_log(path)
        csv_output = args.output_dir / f"{path.stem}.csv"
        write_csv(samples, csv_output)
        print(f"{path.name}: wrote {len(samples)} samples to {csv_output}")
        if args.csv_only:
            continue

        solutions = resolve_horizons(samples)
        errors = np.asarray([solution.command - sample.command for solution, sample in zip(solutions, samples)])
        rms_error = float(np.sqrt(np.mean(errors**2)))
        max_error = float(np.max(np.abs(errors)))
        video_output = args.output_dir / f"{path.stem}_mpc_replay.mp4"
        render_video(samples, solutions, video_output, path.stem, args.fps, args.dpi, args.tail_count)
        print(
            f"{path.name}: wrote {len(samples)} frames at {args.fps:g} Hz to {video_output} "
            f"(command reconstruction RMS={rms_error:.6f}, max={max_error:.6f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
