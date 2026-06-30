#!/usr/bin/env python3
"""Render synchronized POV video and MPC rollout from an egocentric servo run."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
from pathlib import Path
from typing import Optional

from visualize_MPC_planning import (
    ODOM_PURPLE,
    TAG_BLUE,
    draw_current_horizon,
    draw_executed_path,
    draw_goal,
    draw_old_horizons,
    draw_robot,
    draw_tag,
    status_title,
)
from visualize_trajectory import parse_json_matrix, read_metadata, read_rows, resolve_run, value


def main() -> int:
    parser = argparse.ArgumentParser(description="Render synced RGB POV and MPC rollout video")
    parser.add_argument(
        "run",
        nargs="?",
        default=None,
        help="Run directory, trajectory.csv path, or run root. Defaults to latest run beside this script.",
    )
    parser.add_argument("--output", default=None, help="Output MP4 path. Defaults to <run>/synced_mpc_pov.mp4")
    parser.add_argument("--fps", type=float, default=30.0, help="Saved video FPS")
    parser.add_argument("--dpi", type=int, default=130)
    parser.add_argument("--tail-count", type=int, default=25, help="Number of old MPC horizons to show faded")
    parser.add_argument("--duration", type=float, default=None, help="Optional max duration in seconds for quick renders")
    parser.add_argument("--start-offset", type=float, default=0.0, help="Seconds after synchronized start to begin rendering")
    parser.add_argument("--no-odom", action="store_true", help="Do not draw legged-odometry path")
    args = parser.parse_args()

    run_dir, csv_path = resolve_run(args.run)
    rows = [row for row in read_rows(csv_path) if value(row, "mpc_x_tag_m") is not None and value(row, "stamp_sec") is not None]
    if not rows:
        raise SystemExit(f"No MPC rows found in {csv_path}")
    metadata = read_metadata(run_dir)
    video_path, video_index_path = resolve_video_paths(run_dir, metadata)
    video_frames = read_video_frame_index(video_index_path)
    if not video_frames:
        raise SystemExit(f"No video frame timestamps found in {video_index_path}")

    output = Path(args.output).expanduser() if args.output else run_dir / "synced_mpc_pov.mp4"
    output.parent.mkdir(parents=True, exist_ok=True)

    start_sec, end_sec = synchronized_time_window(rows, video_frames, args.start_offset, args.duration)
    if end_sec <= start_sec:
        raise SystemExit("No overlapping timestamp range between trajectory.csv and video_frames.csv")
    frame_times = output_frame_times(start_sec, end_sec, args.fps)
    mpc_indices = timeline_indices_by_stamp([float(value(row, "stamp_sec")) for row in rows], frame_times)
    video_indices = timeline_indices_by_stamp([stamp for _, stamp in video_frames], frame_times)

    try:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FFMpegWriter
    except ImportError as exc:
        raise SystemExit("matplotlib is required to render synced video") from exc

    try:
        from matplotlib import animation

        if not animation.writers.is_available("ffmpeg"):
            raise SystemExit("Matplotlib ffmpeg writer is not available; install ffmpeg to save MP4.")
    except Exception as exc:
        if isinstance(exc, SystemExit):
            raise
        raise SystemExit(f"Failed to check ffmpeg availability: {exc}") from exc

    limits = compute_wide_equal_limits(rows)
    fig, (ax_video, ax_mpc) = plt.subplots(
        2,
        1,
        figsize=(12.5, 8.0),
        gridspec_kw={"height_ratios": [3.2, 1.0]},
    )
    set_even_canvas(fig, args.dpi)

    reader = open_video_reader(video_path)
    writer = FFMpegWriter(fps=args.fps, metadata={"title": f"Synced MPC POV {run_dir.name}"})
    try:
        with writer.saving(fig, str(output), args.dpi):
            for frame_number, (stamp_sec, row_index, video_index) in enumerate(zip(frame_times, mpc_indices, video_indices)):
                rgb_frame = reader.read(video_frames[video_index][0])
                draw_synced_frame(
                    ax_video=ax_video,
                    ax_mpc=ax_mpc,
                    rgb_frame=rgb_frame,
                    rows=rows,
                    row_index=row_index,
                    video_index=video_frames[video_index][0],
                    stamp_sec=stamp_sec,
                    start_sec=start_sec,
                    frame_number=frame_number,
                    frame_count=len(frame_times),
                    metadata=metadata,
                    limits=limits,
                    tail_count=args.tail_count,
                    show_odom=not args.no_odom,
                )
                writer.grab_frame()
    finally:
        reader.close()
        plt.close(fig)

    print(f"saved {output}")
    return 0


def resolve_video_paths(run_dir: Path, metadata: dict) -> tuple[Path, Path]:
    video_path = run_dir / "trajectory_rgb.mp4"
    frame_index_path = run_dir / "video_frames.csv"
    if video_path.is_file() and frame_index_path.is_file():
        return video_path, frame_index_path

    video_meta = metadata.get("video") if isinstance(metadata.get("video"), dict) else {}
    meta_video_path = Path(video_meta.get("path", "")).expanduser()
    meta_index_path = Path(video_meta.get("frame_index_path", "")).expanduser()
    if meta_video_path.is_file() and meta_index_path.is_file():
        return meta_video_path, meta_index_path
    raise SystemExit(f"Missing trajectory_rgb.mp4 or video_frames.csv in {run_dir}")


def read_video_frame_index(path: Path) -> list[tuple[int, float]]:
    frames: list[tuple[int, float]] = []
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            try:
                frames.append((int(row["frame_index"]), float(row["stamp_sec"])))
            except (KeyError, TypeError, ValueError):
                continue
    return frames


def open_video_reader(path: Path):
    try:
        import imageio.v2 as imageio

        return ImageioVideoReader(imageio.get_reader(path))
    except Exception:
        pass
    try:
        import cv2

        return OpenCvVideoReader(path, cv2)
    except Exception:
        pass
    return FfmpegPipeVideoReader(path)


class ImageioVideoReader:
    def __init__(self, reader) -> None:
        self._reader = reader

    def read(self, frame_index: int):
        return self._reader.get_data(frame_index)

    def close(self) -> None:
        self._reader.close()


class OpenCvVideoReader:
    def __init__(self, path: Path, cv2_module) -> None:
        self._cv2 = cv2_module
        self._capture = self._cv2.VideoCapture(str(path))
        if not self._capture.isOpened():
            raise RuntimeError(f"OpenCV could not open {path}")

    def read(self, frame_index: int):
        self._capture.set(self._cv2.CAP_PROP_POS_FRAMES, int(frame_index))
        ok, frame_bgr = self._capture.read()
        if not ok:
            raise RuntimeError(f"OpenCV could not read video frame {frame_index}")
        return self._cv2.cvtColor(frame_bgr, self._cv2.COLOR_BGR2RGB)

    def close(self) -> None:
        self._capture.release()


class FfmpegPipeVideoReader:
    def __init__(self, path: Path) -> None:
        self._path = path
        self._width, self._height = probe_video_size(path)
        self._frame_size = self._width * self._height * 3
        self._process = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-i",
                str(path),
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        self._current_index = -1
        self._current_frame = None

    def read(self, frame_index: int):
        if frame_index < self._current_index:
            raise RuntimeError("ffmpeg pipe reader only supports monotonically increasing frame access")
        while self._current_index < frame_index:
            frame = self._read_next()
            self._current_index += 1
            self._current_frame = frame
        return self._current_frame

    def close(self) -> None:
        if self._process.stdout:
            self._process.stdout.close()
        self._process.terminate()
        try:
            self._process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            self._process.kill()

    def _read_next(self):
        import numpy as np

        if self._process.stdout is None:
            raise RuntimeError("ffmpeg stdout pipe is not available")
        raw = self._process.stdout.read(self._frame_size)
        if len(raw) != self._frame_size:
            raise RuntimeError(f"ffmpeg could not read frame {self._current_index + 1} from {self._path}")
        return np.frombuffer(raw, dtype=np.uint8).reshape((self._height, self._width, 3))


def probe_video_size(path: Path) -> tuple[int, int]:
    result = subprocess.run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height",
            "-of",
            "csv=p=0:s=x",
            str(path),
        ],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    width_str, height_str = result.stdout.strip().split("x", 1)
    return int(width_str), int(height_str)


def synchronized_time_window(
    rows: list[dict[str, object]],
    video_frames: list[tuple[int, float]],
    start_offset_sec: float,
    duration_sec: Optional[float],
) -> tuple[float, float]:
    row_start = float(value(rows[0], "stamp_sec"))
    row_end = float(value(rows[-1], "stamp_sec"))
    video_start = video_frames[0][1]
    video_end = video_frames[-1][1]
    start = max(row_start, video_start) + max(0.0, float(start_offset_sec))
    end = min(row_end, video_end)
    if duration_sec is not None:
        end = min(end, start + max(0.0, float(duration_sec)))
    return start, end


def output_frame_times(start_sec: float, end_sec: float, fps: float) -> list[float]:
    if fps <= 0.0:
        raise SystemExit("--fps must be positive")
    period = 1.0 / fps
    count = max(1, int(math.floor((end_sec - start_sec) / period)) + 1)
    times = [start_sec + index * period for index in range(count)]
    if times[-1] < end_sec:
        times.append(end_sec)
    return times


def timeline_indices_by_stamp(stamps: list[float], frame_times: list[float]) -> list[int]:
    indices: list[int] = []
    index = 0
    for stamp in frame_times:
        while index + 1 < len(stamps) and stamps[index + 1] <= stamp:
            index += 1
        indices.append(index)
    return indices


def draw_synced_frame(
    *,
    ax_video,
    ax_mpc,
    rgb_frame,
    rows: list[dict[str, object]],
    row_index: int,
    video_index: int,
    stamp_sec: float,
    start_sec: float,
    frame_number: int,
    frame_count: int,
    metadata: dict,
    limits: tuple[tuple[float, float], tuple[float, float]],
    tail_count: int,
    show_odom: bool,
) -> None:
    row = rows[row_index]

    ax_video.clear()
    ax_video.imshow(rgb_frame)
    ax_video.axis("off")
    ax_video.set_title(f"POV RGB | t={stamp_sec - start_sec:.2f}s | video frame {video_index}")

    ax_mpc.clear()
    draw_tag(ax_mpc, metadata)
    draw_goal(ax_mpc, rows)
    draw_old_horizons(ax_mpc, rows, row_index, tail_count)
    draw_current_horizon(ax_mpc, row)
    draw_executed_path(ax_mpc, rows[: row_index + 1], "mpc_x_tag_m", "mpc_y_tag_m", "AprilTag localization", TAG_BLUE)
    if show_odom:
        draw_executed_path(ax_mpc, rows[: row_index + 1], "odom_mpc_x_tag_m", "odom_mpc_y_tag_m", "Legged odometry", ODOM_PURPLE)
    draw_robot(ax_mpc, row)
    ax_mpc.axhline(0.0, color="#d0d0d0", linewidth=0.8)
    ax_mpc.axvline(0.0, color="#d0d0d0", linewidth=0.8)
    ax_mpc.set_xlim(*limits[0])
    ax_mpc.set_ylim(*limits[1])
    ax_mpc.set_aspect("equal", adjustable="box")
    ax_mpc.grid(True, linewidth=0.4, alpha=0.35)
    ax_mpc.set_xlabel("MPC tag-frame +X [m]")
    ax_mpc.set_ylabel("MPC tag-frame +Y [m]")
    ax_mpc.set_title(status_title(row, row_index, len(rows), frame_number, frame_count))
    ax_mpc.legend(loc="upper left", fontsize=7)


def compute_wide_equal_limits(rows: list[dict[str, object]]) -> tuple[tuple[float, float], tuple[float, float]]:
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
    y_pad = max(0.20, 0.18 * max(1e-6, y_max - y_min))
    return (x_min - x_pad, x_max + x_pad), (y_min - y_pad, y_max + y_pad)


def set_even_canvas(fig, dpi: int) -> None:
    width_px = int(round(fig.get_figwidth() * dpi))
    height_px = int(round(fig.get_figheight() * dpi))
    width_px += width_px % 2
    height_px += height_px % 2
    fig.set_size_inches(width_px / dpi, height_px / dpi)


if __name__ == "__main__":
    raise SystemExit(main())
