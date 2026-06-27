"""Trajectory recording for egocentric visual-servo runs."""

from __future__ import annotations

import csv
import json
import shutil
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2

from .constants import APRILTAG_TAG_LENGTH_M
from .messages import ImuQuat, OdomPose, TagPose, initial_tag_center_xy, odom_relative_xy, odom_tag_aligned_xy, tag_pose_tag_aligned_xy
from .policy import ServoCommand


@dataclass(frozen=True)
class RecorderOrigins:
    odom: Optional[OdomPose]
    tag: Optional[TagPose]
    imu: Optional[ImuQuat]


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
        self._origins: Optional[RecorderOrigins] = None
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

    def start(self, *, odom: Optional[OdomPose], tag: Optional[TagPose], imu: Optional[ImuQuat]) -> Path:
        self.stop()
        timestamp = time.strftime("%m%d%y_%H%M%S")
        self._run_dir = self._next_run_dir(self._record_root, timestamp)
        self._run_dir.mkdir(parents=True, exist_ok=False)
        self._archive_dir = None
        self._archive_error = None
        self._reset_video_state()
        self._origins = RecorderOrigins(odom=odom, tag=tag, imu=imu)
        self._metadata = {
            "created_time_sec": time.time(),
            "record_root": str(self._record_root),
            "archive_root": str(self._archive_root) if self._archive_root else None,
            "apriltag_tag_length_m": APRILTAG_TAG_LENGTH_M,
            "origin": {
                "odom": _odom_dict(odom),
                "tag": _tag_dict(tag),
                "imu": _imu_dict(imu),
            },
            "video": self.video_status(),
        }
        self._write_metadata()
        self._csv_handle = (self._run_dir / "trajectory.csv").open("w", newline="")
        self._writer = csv.DictWriter(self._csv_handle, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._sample_count = 0
        return self._run_dir

    def stop(self) -> Optional[Path]:
        was_active = self._writer is not None
        if self._csv_handle is not None:
            self._csv_handle.flush()
            self._csv_handle.close()
        self._csv_handle = None
        self._writer = None
        self._close_video_writer()
        self._origins = None
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
                    self._video_index_writer.writerow(
                        {
                            "frame_index": self._video_frame_count,
                            "stamp_sec": stamp_sec,
                        }
                    )
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
        tag: Optional[TagPose],
        odom: Optional[OdomPose],
        imu: Optional[ImuQuat],
        command: ServoCommand,
        requested_mode: Optional[str],
    ) -> None:
        if self._writer is None or self._origins is None:
            return

        odom_rel = (None, None, None)
        odom_tag_aligned_rel = (None, None)
        if odom is not None and self._origins.odom is not None:
            odom_rel = odom_relative_xy(odom, self._origins.odom)
            if self._origins.tag is not None:
                odom_tag_aligned_rel = odom_tag_aligned_xy(odom_rel[0], odom_rel[1], self._origins.tag) or (None, None)

        tag_aligned_rel = (None, None)
        if tag is not None and self._origins.tag is not None:
            tag_aligned_rel = tag_pose_tag_aligned_xy(tag, self._origins.tag) or (None, None)

        row = {
            "stamp_sec": stamp_sec,
            "state": state,
            "message": message,
            "requested_mode": requested_mode,
            "tag_id": tag.tag_id if tag else None,
            "tag_margin": tag.decision_margin if tag else None,
            "tag_x_camera_m": float(tag.position_camera_m[0]) if tag else None,
            "tag_y_camera_m": float(tag.position_camera_m[1]) if tag else None,
            "tag_z_camera_m": float(tag.position_camera_m[2]) if tag else None,
            "tag_range_m": tag.range_m if tag else None,
            "tag_bearing_rad": tag.bearing_rad if tag else None,
            "tag_face_yaw_error_rad": tag.face_yaw_error_rad if tag else None,
            "tag_face_normal_x_camera": tag.face_normal_camera[0] if tag and tag.face_normal_camera else None,
            "tag_face_normal_y_camera": tag.face_normal_camera[1] if tag and tag.face_normal_camera else None,
            "tag_face_normal_z_camera": tag.face_normal_camera[2] if tag and tag.face_normal_camera else None,
            **_tag_rotation_fields(tag),
            "tag_aligned_rel_x_m": tag_aligned_rel[0],
            "tag_aligned_rel_y_m": tag_aligned_rel[1],
            "odom_x": odom.x if odom else None,
            "odom_y": odom.y if odom else None,
            "odom_yaw": odom.yaw if odom else None,
            "odom_rel_x": odom_rel[0],
            "odom_rel_y": odom_rel[1],
            "odom_rel_yaw": odom_rel[2],
            "odom_tag_aligned_rel_x_m": odom_tag_aligned_rel[0],
            "odom_tag_aligned_rel_y_m": odom_tag_aligned_rel[1],
            "imu_x": imu.x if imu else None,
            "imu_y": imu.y if imu else None,
            "imu_z": imu.z if imu else None,
            "imu_w": imu.w if imu else None,
            "cmd_heading": command.heading,
            "cmd_lateral": command.lateral,
            "cmd_turning": command.turning,
            "range_error_m": command.range_error_m,
            "lateral_error_m": command.lateral_error_m,
            "yaw_error_rad": command.yaw_error_rad,
            "cmd_face_yaw_error_rad": command.face_yaw_error_rad,
            "cmd_face_blend": command.face_blend,
            "target_reached": command.target_reached,
        }
        self._writer.writerow(row)
        self._sample_count += 1
        if self._sample_count % 20 == 0 and self._csv_handle is not None:
            self._csv_handle.flush()


VIDEO_INDEX_FIELDNAMES = ["frame_index", "stamp_sec"]


FIELDNAMES = [
    "stamp_sec",
    "state",
    "message",
    "requested_mode",
    "tag_id",
    "tag_margin",
    "tag_x_camera_m",
    "tag_y_camera_m",
    "tag_z_camera_m",
    "tag_range_m",
    "tag_bearing_rad",
    "tag_face_yaw_error_rad",
    "tag_face_normal_x_camera",
    "tag_face_normal_y_camera",
    "tag_face_normal_z_camera",
    "tag_rotation_camera_tag_00",
    "tag_rotation_camera_tag_01",
    "tag_rotation_camera_tag_02",
    "tag_rotation_camera_tag_10",
    "tag_rotation_camera_tag_11",
    "tag_rotation_camera_tag_12",
    "tag_rotation_camera_tag_20",
    "tag_rotation_camera_tag_21",
    "tag_rotation_camera_tag_22",
    "tag_aligned_rel_x_m",
    "tag_aligned_rel_y_m",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "odom_rel_x",
    "odom_rel_y",
    "odom_rel_yaw",
    "odom_tag_aligned_rel_x_m",
    "odom_tag_aligned_rel_y_m",
    "imu_x",
    "imu_y",
    "imu_z",
    "imu_w",
    "cmd_heading",
    "cmd_lateral",
    "cmd_turning",
    "range_error_m",
    "lateral_error_m",
    "yaw_error_rad",
    "cmd_face_yaw_error_rad",
    "cmd_face_blend",
    "target_reached",
]


def _tag_dict(tag: Optional[TagPose]) -> Optional[dict]:
    if tag is None:
        return None
    tag_aligned_center = initial_tag_center_xy(tag)
    return {
        "id": tag.tag_id,
        "stamp_sec": tag.stamp_sec,
        "position_camera_m": [float(item) for item in tag.position_camera_m],
        "rotation_camera_tag": _rotation_matrix_list(tag),
        "face_normal_camera": list(tag.face_normal_camera) if tag.face_normal_camera else None,
        "tag_aligned_center_m": list(tag_aligned_center) if tag_aligned_center is not None else None,
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


def _imu_dict(imu: Optional[ImuQuat]) -> Optional[dict]:
    if imu is None:
        return None
    return {"stamp_sec": imu.stamp_sec, "x": imu.x, "y": imu.y, "z": imu.z, "w": imu.w}
