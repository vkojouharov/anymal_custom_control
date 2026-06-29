"""Trajectory recording for MPC egocentric visual-servo runs."""

from __future__ import annotations

import csv
import json
import math
import shutil
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2

from .constants import APRILTAG_TAG_LENGTH_M
from .messages import OdomPose, TagPose
from .mpc import MpcCommand, MpcState


@dataclass(frozen=True)
class RecorderOrigins:
    odom: Optional[OdomPose]
    tag: Optional[TagPose]
    mpc_state: Optional[MpcState]


class TrajectoryRecorder:
    def __init__(
        self,
        record_root: str | Path,
        archive_root: str | Path | None = None,
        *,
        record_video: bool = True,
        video_fps: float = 30.0,
    ) -> None:
        self._record_root = Path(record_root)
        self._archive_root = Path(archive_root) if archive_root else None
        self._record_video = bool(record_video)
        self._video_fps = float(video_fps)
        self._run_dir: Optional[Path] = None
        self._archive_dir: Optional[Path] = None
        self._archive_error: Optional[str] = None
        self._csv_handle = None
        self._writer: Optional[csv.DictWriter] = None
        self._origins = RecorderOrigins(odom=None, tag=None, mpc_state=None)
        self._sample_count = 0
        self._metadata: Optional[dict] = None
        self._video_lock = threading.Lock()
        self._video_writer = None
        self._video_path: Optional[Path] = None
        self._video_index_path: Optional[Path] = None
        self._video_index_handle = None
        self._video_index_writer: Optional[csv.DictWriter] = None
        self._video_size: Optional[tuple[int, int]] = None
        self._video_frame_count = 0
        self._video_error: Optional[str] = None
        self._last_video_stamp_sec: Optional[float] = None

    @property
    def active(self) -> bool:
        return self._writer is not None

    @property
    def run_dir(self) -> Optional[Path]:
        return self._run_dir

    @property
    def archive_dir(self) -> Optional[Path]:
        return self._archive_dir

    @property
    def archive_error(self) -> Optional[str]:
        return self._archive_error

    @property
    def sample_count(self) -> int:
        return self._sample_count

    def video_status(self) -> dict[str, object]:
        with self._video_lock:
            return self._video_status_locked()

    def start(self) -> Path:
        self.stop()
        timestamp = time.strftime("%m%d%y_%H%M%S")
        self._run_dir = self._next_run_dir(self._record_root, timestamp)
        self._run_dir.mkdir(parents=True, exist_ok=False)
        self._archive_dir = None
        self._archive_error = None
        self._reset_video_state()
        self._origins = RecorderOrigins(odom=None, tag=None, mpc_state=None)
        self._metadata = {
            "created_time_sec": time.time(),
            "record_root": str(self._record_root),
            "archive_root": str(self._archive_root) if self._archive_root else None,
            "apriltag_tag_length_m": APRILTAG_TAG_LENGTH_M,
            "origin": {
                "odom": None,
                "tag": None,
                "mpc_state": None,
            },
            "video": self.video_status(),
        }
        self._write_metadata()
        self._csv_handle = (self._run_dir / "trajectory.csv").open("w", newline="")
        self._writer = csv.DictWriter(self._csv_handle, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._sample_count = 0
        return self._run_dir

    def set_origin(self, *, odom: Optional[OdomPose], tag: Optional[TagPose], mpc_state: Optional[MpcState]) -> None:
        self._origins = RecorderOrigins(odom=odom, tag=tag, mpc_state=mpc_state)
        if self._metadata is None:
            return
        self._metadata["origin"] = {
            "odom": _odom_dict(odom),
            "tag": _tag_dict(tag),
            "mpc_state": _mpc_state_dict(mpc_state),
        }
        self._write_metadata()

    def stop(self) -> Optional[Path]:
        was_active = self._writer is not None
        if self._csv_handle is not None:
            self._csv_handle.flush()
            self._csv_handle.close()
        self._csv_handle = None
        self._writer = None
        self._close_video_writer()
        if was_active:
            self._update_video_metadata()
            self._archive_completed_run()
        return self._archive_dir

    @staticmethod
    def _next_run_dir(record_root: Path, timestamp: str) -> Path:
        base = record_root / f"egocentric_servo_{timestamp}"
        if not base.exists():
            return base
        for index in range(2, 1000):
            candidate = record_root / f"egocentric_servo_{timestamp}_{index:02d}"
            if not candidate.exists():
                return candidate
        raise RuntimeError(f"No available run directory for timestamp {timestamp}")

    def _archive_completed_run(self) -> None:
        self._archive_dir = None
        self._archive_error = None
        if self._run_dir is None or self._archive_root is None:
            return
        try:
            if self._run_dir.resolve().parent == self._archive_root.resolve():
                self._archive_dir = self._run_dir
                return
        except OSError:
            pass
        try:
            self._archive_root.mkdir(parents=True, exist_ok=True)
            destination = self._archive_root / self._run_dir.name
            if destination.exists():
                destination = self._next_run_dir(self._archive_root, self._run_dir.name.removeprefix("egocentric_servo_"))
            shutil.copytree(self._run_dir, destination)
            self._archive_dir = destination
        except OSError as exc:
            self._archive_error = str(exc)

    def write_video_frame(self, *, stamp_sec: float, frame_bgr) -> Optional[str]:
        if not self._record_video or self._writer is None or self._run_dir is None:
            return None
        with self._video_lock:
            if self._video_error is not None:
                return None
            if self._last_video_stamp_sec is not None and self._video_fps > 0.0:
                min_period_sec = 1.0 / self._video_fps
                if stamp_sec - self._last_video_stamp_sec < min_period_sec * 0.8:
                    return None
            try:
                if self._video_writer is None:
                    self._open_video_writer_locked(frame_bgr)
                if self._video_writer is None:
                    return None
                frame_to_write = frame_bgr
                if self._video_size is not None:
                    width, height = self._video_size
                    if frame_bgr.shape[1] != width or frame_bgr.shape[0] != height:
                        frame_to_write = cv2.resize(frame_bgr, (width, height), interpolation=cv2.INTER_AREA)
                self._video_writer.write(frame_to_write)
                if self._video_index_writer is not None:
                    self._video_index_writer.writerow({"frame_index": self._video_frame_count, "stamp_sec": stamp_sec})
                self._last_video_stamp_sec = stamp_sec
                self._video_frame_count += 1
                if self._video_frame_count % 30 == 0 and self._video_index_handle is not None:
                    self._video_index_handle.flush()
                return None
            except Exception as exc:
                self._video_error = str(exc)
                self._release_video_writer_locked()
                return self._video_error

    def _reset_video_state(self) -> None:
        with self._video_lock:
            self._release_video_writer_locked()
            self._video_path = (self._run_dir / "trajectory_rgb.mp4") if self._run_dir is not None else None
            self._video_index_path = (self._run_dir / "video_frames.csv") if self._run_dir is not None else None
            self._video_size = None
            self._video_frame_count = 0
            self._video_error = None
            self._last_video_stamp_sec = None

    def _open_video_writer_locked(self, frame_bgr) -> None:
        if self._run_dir is None or self._video_path is None:
            return
        if len(frame_bgr.shape) < 3 or frame_bgr.shape[2] != 3:
            raise ValueError("RGB video frame is not a 3-channel BGR image")
        height, width = frame_bgr.shape[:2]
        if width <= 0 or height <= 0:
            raise ValueError(f"Invalid RGB video frame size: {width}x{height}")
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter(str(self._video_path), fourcc, self._video_fps, (width, height))
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(f"OpenCV VideoWriter failed to open {self._video_path}")
        self._video_writer = writer
        self._video_size = (width, height)
        if self._video_index_path is not None:
            self._video_index_handle = self._video_index_path.open("w", newline="")
            self._video_index_writer = csv.DictWriter(self._video_index_handle, fieldnames=VIDEO_INDEX_FIELDNAMES)
            self._video_index_writer.writeheader()

    def _close_video_writer(self) -> None:
        with self._video_lock:
            self._release_video_writer_locked()

    def _release_video_writer_locked(self) -> None:
        if self._video_writer is not None:
            self._video_writer.release()
        self._video_writer = None
        if self._video_index_handle is not None:
            self._video_index_handle.flush()
            self._video_index_handle.close()
        self._video_index_handle = None
        self._video_index_writer = None

    def _video_status_locked(self) -> dict[str, object]:
        return {
            "enabled": self._record_video,
            "path": str(self._video_path) if self._video_path else None,
            "frame_index_path": str(self._video_index_path) if self._video_index_path else None,
            "fps": self._video_fps,
            "frame_count": self._video_frame_count,
            "error": self._video_error,
        }

    def _update_video_metadata(self) -> None:
        if self._metadata is None:
            return
        self._metadata["video"] = self.video_status()
        self._write_metadata()

    def _write_metadata(self) -> None:
        if self._run_dir is None or self._metadata is None:
            return
        (self._run_dir / "metadata.json").write_text(json.dumps(self._metadata, indent=2, sort_keys=True))

    def write_sample(
        self,
        *,
        stamp_sec: float,
        state: str,
        message: str,
        tag_fresh: bool,
        tag: Optional[TagPose],
        mpc_state: Optional[MpcState],
        odom: Optional[OdomPose],
        odom_mpc_state: Optional[MpcState],
        command: MpcCommand,
        predicted_states: list[list[float]],
        predicted_controls: list[list[float]],
        requested_mode: Optional[str],
    ) -> None:
        if self._writer is None:
            return
        drift = _state_delta(mpc_state, odom_mpc_state)
        row = {
            "stamp_sec": stamp_sec,
            "state": state,
            "message": message,
            "requested_mode": requested_mode,
            "tag_fresh": tag_fresh,
            "command_source": command.command_source,
            "open_loop_step": command.open_loop_step,
            "tag_id": tag.tag_id if tag else None,
            "tag_margin": tag.decision_margin if tag else None,
            "tag_x_camera_m": float(tag.position_camera_m[0]) if tag else None,
            "tag_y_camera_m": float(tag.position_camera_m[1]) if tag else None,
            "tag_z_camera_m": float(tag.position_camera_m[2]) if tag else None,
            "tag_range_m": tag.range_m if tag else None,
            "tag_bearing_rad": tag.bearing_rad if tag else None,
            **_tag_rotation_fields(tag),
            "mpc_x_tag_m": mpc_state.x if mpc_state else None,
            "mpc_y_tag_m": mpc_state.y if mpc_state else None,
            "mpc_theta_tag_rad": mpc_state.theta if mpc_state else None,
            "odom_x": odom.x if odom else None,
            "odom_y": odom.y if odom else None,
            "odom_yaw": odom.yaw if odom else None,
            "odom_mpc_x_tag_m": odom_mpc_state.x if odom_mpc_state else None,
            "odom_mpc_y_tag_m": odom_mpc_state.y if odom_mpc_state else None,
            "odom_mpc_theta_tag_rad": odom_mpc_state.theta if odom_mpc_state else None,
            "tag_minus_odom_x_m": drift[0],
            "tag_minus_odom_y_m": drift[1],
            "tag_minus_odom_theta_rad": drift[2],
            "cmd_vx_body_mps": command.vx_body_mps,
            "cmd_vy_body_mps": command.vy_body_mps,
            "cmd_omega_radps": command.omega_radps,
            "cmd_xdot_tag_mps": command.u_world[0],
            "cmd_ydot_tag_mps": command.u_world[1],
            "cmd_thetadot_radps": command.u_world[2],
            "cmd_heading": command.heading,
            "cmd_lateral": command.lateral,
            "cmd_turning": command.turning,
            "range_error_m": command.range_error_m,
            "lateral_error_m": command.lateral_error_m,
            "yaw_error_rad": command.yaw_error_rad,
            "solver_status": command.solver_status,
            "solve_time_ms": command.solve_time_ms,
            "target_reached": command.target_reached,
            "mpc_predicted_states_json": json.dumps(predicted_states, separators=(",", ":")),
            "mpc_predicted_controls_json": json.dumps(predicted_controls, separators=(",", ":")),
        }
        self._writer.writerow(row)
        self._sample_count += 1
        if self._sample_count % 10 == 0 and self._csv_handle is not None:
            self._csv_handle.flush()


VIDEO_INDEX_FIELDNAMES = ["frame_index", "stamp_sec"]


FIELDNAMES = [
    "stamp_sec",
    "state",
    "message",
    "requested_mode",
    "tag_fresh",
    "command_source",
    "open_loop_step",
    "tag_id",
    "tag_margin",
    "tag_x_camera_m",
    "tag_y_camera_m",
    "tag_z_camera_m",
    "tag_range_m",
    "tag_bearing_rad",
    "tag_rotation_camera_tag_00",
    "tag_rotation_camera_tag_01",
    "tag_rotation_camera_tag_02",
    "tag_rotation_camera_tag_10",
    "tag_rotation_camera_tag_11",
    "tag_rotation_camera_tag_12",
    "tag_rotation_camera_tag_20",
    "tag_rotation_camera_tag_21",
    "tag_rotation_camera_tag_22",
    "mpc_x_tag_m",
    "mpc_y_tag_m",
    "mpc_theta_tag_rad",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "odom_mpc_x_tag_m",
    "odom_mpc_y_tag_m",
    "odom_mpc_theta_tag_rad",
    "tag_minus_odom_x_m",
    "tag_minus_odom_y_m",
    "tag_minus_odom_theta_rad",
    "cmd_vx_body_mps",
    "cmd_vy_body_mps",
    "cmd_omega_radps",
    "cmd_xdot_tag_mps",
    "cmd_ydot_tag_mps",
    "cmd_thetadot_radps",
    "cmd_heading",
    "cmd_lateral",
    "cmd_turning",
    "range_error_m",
    "lateral_error_m",
    "yaw_error_rad",
    "solver_status",
    "solve_time_ms",
    "target_reached",
    "mpc_predicted_states_json",
    "mpc_predicted_controls_json",
]


def _state_delta(a: Optional[MpcState], b: Optional[MpcState]) -> tuple[Optional[float], Optional[float], Optional[float]]:
    if a is None or b is None:
        return None, None, None
    return a.x - b.x, a.y - b.y, _wrap_angle(a.theta - b.theta)


def _tag_dict(tag: Optional[TagPose]) -> Optional[dict]:
    if tag is None:
        return None
    return {
        "id": tag.tag_id,
        "stamp_sec": tag.stamp_sec,
        "position_camera_m": [float(item) for item in tag.position_camera_m],
        "rotation_camera_tag": _rotation_matrix_list(tag),
        "tag_size_m": tag.tag_size_m,
    }


def _tag_rotation_fields(tag: Optional[TagPose]) -> dict[str, Optional[float]]:
    fields: dict[str, Optional[float]] = {}
    for row in range(3):
        for col in range(3):
            key = f"tag_rotation_camera_tag_{row}{col}"
            fields[key] = float(tag.rotation_camera_tag[row, col]) if tag and tag.rotation_camera_tag is not None else None
    return fields


def _rotation_matrix_list(tag: TagPose) -> Optional[list[list[float]]]:
    if tag.rotation_camera_tag is None:
        return None
    return [[float(tag.rotation_camera_tag[row, col]) for col in range(3)] for row in range(3)]


def _odom_dict(odom: Optional[OdomPose]) -> Optional[dict]:
    if odom is None:
        return None
    return {"stamp_sec": odom.stamp_sec, "x": odom.x, "y": odom.y, "yaw": odom.yaw}


def _mpc_state_dict(state: Optional[MpcState]) -> Optional[dict]:
    if state is None:
        return None
    return {"x": state.x, "y": state.y, "theta": state.theta}


def _wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
