#!/usr/bin/env python3
"""Convert July 23 hardware MPC logs to CSV and render replay videos.

This is the July 23 adaptation of the July 22 replay tool.  It uses the
controller settings from that test day (10 Hz, a 30-step horizon, updated
state/heading weights and velocity limits) and records the newly logged
``state_cost`` value in the CSV and video title.

Each animation frame represents one MPC update, so the default output frame
rate is 10 FPS.  The logged command from the preceding row is used as
``u_prev`` while reconstructing each MPC horizon.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
JULY_22_SCRIPT = SCRIPT_DIR.parent / "july_22_tests" / "visualize_hardware_mpc.py"


def load_replay_module():
    """Load the shared July 22 renderer without requiring a Python package."""
    spec = importlib.util.spec_from_file_location("july_22_hardware_mpc_replay", JULY_22_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load the replay implementation from {JULY_22_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


replay = load_replay_module()

# Settings used by run_mpc_barebones_test.py for the July 23 tests.
replay.SCRIPT_DIR = SCRIPT_DIR
replay.MPC_HZ = 10.0
replay.DT = 1.0 / replay.MPC_HZ
replay.HORIZON = 30
replay.P_XY = 50.0
replay.P_THETA = 25.0
# The run/video logs were captured immediately before commit c21a6e3 raised
# the yaw limit from 0.25 to 0.5.  src_hallway_run was recorded afterward.
DEFAULT_U_MAX = np.array([0.5, 0.5, 0.25])
POST_UPDATE_U_MAX = np.array([0.5, 0.5, 0.5])
replay.U_MAX = DEFAULT_U_MAX.copy()
replay.DU_MAX = np.array([10.0, 10.0, 10.0])

FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
LOG_ROW = re.compile(
    rf"^\s*state=\[\s*({FLOAT})\s+({FLOAT})\s+({FLOAT})\s*\]"
    rf"\s+u0=\[\s*({FLOAT})\s+({FLOAT})\s+({FLOAT})\s*\]"
    rf"\s+state_cost=({FLOAT})"
    rf"\s+solve=({FLOAT})ms\s*$"
)


@dataclass(frozen=True)
class LogSample:
    step: int
    state: np.ndarray
    command: np.ndarray
    state_cost: float
    solve_time_ms: float


def parse_log(path: Path) -> list[LogSample]:
    """Parse state, command, state cost, and solve-time rows."""
    replay.U_MAX = (
        POST_UPDATE_U_MAX.copy()
        if path.stem == "src_hallway_run"
        else DEFAULT_U_MAX.copy()
    )
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
                    state_cost=values[6],
                    solve_time_ms=values[7],
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
    """Write normalized state, command, state-cost, and timing columns."""
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
                "state_cost",
                "solve_time_ms",
            ]
        )
        for sample in samples:
            writer.writerow(
                [
                    sample.step,
                    f"{sample.step * replay.DT:.6f}",
                    *(f"{value:.9f}" for value in sample.state),
                    *(f"{value:.9f}" for value in sample.command),
                    f"{sample.state_cost:.6f}",
                    f"{sample.solve_time_ms:.6f}",
                ]
            )


def default_inputs() -> list[Path]:
    """Return every July 23 MPC data log, excluding the free-form notes."""
    inputs: set[Path] = set()
    for pattern in ("run*.txt", "video*.txt", "src_hallway_run.txt"):
        inputs.update(SCRIPT_DIR.glob(pattern))
    return sorted(inputs)


base_draw_frame = replay.draw_frame


def draw_frame(
    ax,
    samples,
    solutions,
    frame,
    limits,
    run_name,
    tail_count,
    legend_loc,
) -> None:
    """Use the established replay drawing and add the logged state cost."""
    base_draw_frame(
        ax,
        samples,
        solutions,
        frame,
        limits,
        run_name,
        tail_count,
        legend_loc,
    )
    ax.set_title(f"{ax.get_title()} | state cost = {samples[frame].state_cost:.3f}")


# The July 22 module owns the CLI and renderer.  Replace its experiment-specific
# hooks/settings while retaining one implementation of the plotting machinery.
replay.__doc__ = __doc__
replay.parse_log = parse_log
replay.write_csv = write_csv
replay.default_inputs = default_inputs
replay.draw_frame = draw_frame


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "logs",
        nargs="*",
        type=Path,
        help="Input terminal logs; defaults to all July 23 MPC data logs",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=SCRIPT_DIR,
        help="Directory for CSV and MP4 outputs",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=replay.MPC_HZ,
        help="Video frames per second (default: hardware MPC rate)",
    )
    parser.add_argument("--dpi", type=int, default=140, help="Video resolution scale")
    parser.add_argument(
        "--tail-count",
        type=int,
        default=replay.HORIZON,
        help="Number of prior horizon traces to retain",
    )
    parser.add_argument("--csv-only", action="store_true", help="Only convert logs; do not render videos")
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

        solutions = replay.resolve_horizons(samples)
        errors = np.asarray(
            [solution.command - sample.command for solution, sample in zip(solutions, samples)]
        )
        rms_error = float(np.sqrt(np.mean(errors**2)))
        max_error = float(np.max(np.abs(errors)))
        video_output = args.output_dir / f"{path.stem}_mpc_replay.mp4"
        replay.render_video(
            samples,
            solutions,
            video_output,
            path.stem,
            args.fps,
            args.dpi,
            args.tail_count,
        )
        print(
            f"{path.name}: wrote {len(samples)} frames at {args.fps:g} Hz to {video_output} "
            f"(command reconstruction RMS={rms_error:.6f}, max={max_error:.6f})"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
