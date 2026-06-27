"""Trajectory recording for egocentric visual-servo runs."""

from __future__ import annotations

import csv
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from .constants import APRILTAG_TAG_LENGTH_M
from .messages import ImuQuat, OdomPose, TagPose, odom_relative_xy
from .policy import ServoCommand


@dataclass(frozen=True)
class RecorderOrigins:
    odom: Optional[OdomPose]
    tag: Optional[TagPose]
    imu: Optional[ImuQuat]


class TrajectoryRecorder:
    def __init__(self, record_root: str | Path) -> None:
        self._record_root = Path(record_root)
        self._run_dir: Optional[Path] = None
        self._csv_handle = None
        self._writer: Optional[csv.DictWriter] = None
        self._origins: Optional[RecorderOrigins] = None
        self._sample_count = 0

    @property
    def active(self) -> bool:
        return self._writer is not None

    @property
    def run_dir(self) -> Optional[Path]:
        return self._run_dir

    @property
    def sample_count(self) -> int:
        return self._sample_count

    def start(self, *, odom: Optional[OdomPose], tag: Optional[TagPose], imu: Optional[ImuQuat]) -> Path:
        self.stop()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self._run_dir = self._record_root / f"egocentric_servo_{timestamp}"
        self._run_dir.mkdir(parents=True, exist_ok=False)
        self._origins = RecorderOrigins(odom=odom, tag=tag, imu=imu)
        metadata = {
            "created_time_sec": time.time(),
            "apriltag_tag_length_m": APRILTAG_TAG_LENGTH_M,
            "origin": {
                "odom": _odom_dict(odom),
                "tag": _tag_dict(tag),
                "imu": _imu_dict(imu),
            },
        }
        (self._run_dir / "metadata.json").write_text(json.dumps(metadata, indent=2, sort_keys=True))
        self._csv_handle = (self._run_dir / "trajectory.csv").open("w", newline="")
        self._writer = csv.DictWriter(self._csv_handle, fieldnames=FIELDNAMES)
        self._writer.writeheader()
        self._sample_count = 0
        return self._run_dir

    def stop(self) -> None:
        if self._csv_handle is not None:
            self._csv_handle.flush()
            self._csv_handle.close()
        self._csv_handle = None
        self._writer = None
        self._origins = None

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
        if odom is not None and self._origins.odom is not None:
            odom_rel = odom_relative_xy(odom, self._origins.odom)

        tag_rel = (None, None, None)
        if tag is not None and self._origins.tag is not None:
            delta = tag.position_camera_m - self._origins.tag.position_camera_m
            tag_rel = (float(delta[0]), float(delta[1]), float(delta[2]))

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
            "tag_rel_x_m": tag_rel[0],
            "tag_rel_y_m": tag_rel[1],
            "tag_rel_z_m": tag_rel[2],
            "odom_x": odom.x if odom else None,
            "odom_y": odom.y if odom else None,
            "odom_yaw": odom.yaw if odom else None,
            "odom_rel_x": odom_rel[0],
            "odom_rel_y": odom_rel[1],
            "odom_rel_yaw": odom_rel[2],
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
            "target_reached": command.target_reached,
        }
        self._writer.writerow(row)
        self._sample_count += 1
        if self._sample_count % 20 == 0 and self._csv_handle is not None:
            self._csv_handle.flush()


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
    "tag_rel_x_m",
    "tag_rel_y_m",
    "tag_rel_z_m",
    "odom_x",
    "odom_y",
    "odom_yaw",
    "odom_rel_x",
    "odom_rel_y",
    "odom_rel_yaw",
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
    "target_reached",
]


def _tag_dict(tag: Optional[TagPose]) -> Optional[dict]:
    if tag is None:
        return None
    return {
        "id": tag.tag_id,
        "stamp_sec": tag.stamp_sec,
        "position_camera_m": [float(item) for item in tag.position_camera_m],
        "tag_size_m": tag.tag_size_m,
    }


def _odom_dict(odom: Optional[OdomPose]) -> Optional[dict]:
    if odom is None:
        return None
    return {"stamp_sec": odom.stamp_sec, "x": odom.x, "y": odom.y, "yaw": odom.yaw}


def _imu_dict(imu: Optional[ImuQuat]) -> Optional[dict]:
    if imu is None:
        return None
    return {"stamp_sec": imu.stamp_sec, "x": imu.x, "y": imu.y, "z": imu.z, "w": imu.w}
