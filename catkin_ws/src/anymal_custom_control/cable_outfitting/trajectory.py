from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from anymal_custom_control.control.giraf_arm_common import TASK_VELOCITY_LIMITS


POLICIES = frozenset(("pick", "place", "hook", "home"))
DEFAULT_DEPLOYMENT_TIMEOUT_S = 5.0


@dataclass(frozen=True)
class CameraConfig:
    mxid: str
    fps: float
    tag_size_m: float


@dataclass(frozen=True)
class HardwareConfig:
    navigation_camera: CameraConfig
    arm_camera: CameraConfig


@dataclass(frozen=True)
class TaskPoint:
    name: str
    navigation_tag_id: int
    navigation_goal: np.ndarray
    search_omega: float | None
    search_timeout_s: float | None
    deployment_twist: np.ndarray | None
    deployment_timeout_s: float | None
    manipulation_tag_id: int | None
    policy: str | None


@dataclass(frozen=True)
class Trajectory:
    name: str
    task_points: tuple[TaskPoint, ...]


def _read_yaml(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text())
    except OSError as exc:
        raise ValueError(f"cannot read {source}: {exc}") from exc
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {source}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"{source} must contain a YAML mapping")
    return data


def _mapping(value: object, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def _text(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a nonempty string")
    return value.strip()


def _keys(
    data: dict[str, Any],
    allowed: set[str],
    label: str,
    required: set[str] | None = None,
) -> None:
    unknown = sorted(set(data) - allowed)
    missing = sorted((allowed if required is None else required) - set(data))
    if unknown:
        raise ValueError(f"{label} has unknown keys: {', '.join(unknown)}")
    if missing:
        raise ValueError(f"{label} is missing keys: {', '.join(missing)}")


def _positive(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite and positive") from exc
    if not np.isfinite(number) or number <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return number


def _search_omega(value: object, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be finite, nonzero, and within [-1, 1]") from exc
    if not np.isfinite(number) or number == 0.0 or abs(number) > 1.0:
        raise ValueError(f"{label} must be finite, nonzero, and within [-1, 1]")
    return number


def _tag_id(value: object, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"{label} must be a nonnegative integer")
    return value


def _vector(value: object, length: int, label: str) -> np.ndarray:
    try:
        vector = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain {length} finite values") from exc
    if vector.shape != (length,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{label} must contain {length} finite values")
    return vector


def _camera(data: object, label: str) -> CameraConfig:
    item = _mapping(data, label)
    _keys(item, {"mxid", "fps", "tag_size_m"}, label)
    mxid = _text(item["mxid"], f"{label}.mxid")
    return CameraConfig(mxid, _positive(item["fps"], f"{label}.fps"), _positive(item["tag_size_m"], f"{label}.tag_size_m"))


def load_hardware(path: str | Path) -> HardwareConfig:
    data = _read_yaml(path)
    _keys(data, {"navigation_camera", "arm_camera"}, "hardware")
    navigation = _camera(data["navigation_camera"], "navigation_camera")
    arm = _camera(data["arm_camera"], "arm_camera")
    if navigation.mxid == arm.mxid:
        raise ValueError("navigation_camera and arm_camera must use different MXIDs")
    return HardwareConfig(navigation, arm)


def load_trajectory(path: str | Path) -> Trajectory:
    data = _read_yaml(path)
    _keys(data, {"name", "task_points"}, "trajectory")
    name = _text(data["name"], "trajectory.name")
    raw_points = data["task_points"]
    if not isinstance(raw_points, list) or not raw_points:
        raise ValueError("trajectory.task_points must be a nonempty list")

    points = []
    names = set()
    for index, raw in enumerate(raw_points):
        label = f"task_points[{index}]"
        item = _mapping(raw, label)
        _keys(
            item,
            {"name", "search", "navigation", "deployment", "manipulation"},
            label,
            required={"name"},
        )
        point_name = _text(item["name"], f"{label}.name")
        if point_name in names:
            raise ValueError(f"{label}.name must be unique")
        names.add(point_name)

        if "navigation" in item:
            navigation = _mapping(item["navigation"], f"{label}.navigation")
            _keys(navigation, {"tag_id", "goal"}, f"{label}.navigation")
            navigation_tag_id = _tag_id(
                navigation["tag_id"],
                f"{label}.navigation.tag_id",
            )
            navigation_goal = _vector(
                navigation["goal"],
                2,
                f"{label}.navigation.goal",
            )
        elif not points:
            raise ValueError(f"{label}.navigation is required for the first task point")
        else:
            navigation_tag_id = points[-1].navigation_tag_id
            navigation_goal = points[-1].navigation_goal.copy()

        if "search" in item:
            if "navigation" not in item:
                raise ValueError(f"{label}.search requires navigation")
            search = _mapping(item["search"], f"{label}.search")
            _keys(search, {"omega", "timeout_s"}, f"{label}.search")
            search_omega = _search_omega(search["omega"], f"{label}.search.omega")
            search_timeout_s = _positive(search["timeout_s"], f"{label}.search.timeout_s")
        else:
            search_omega = search_timeout_s = None

        if "manipulation" not in item:
            if "deployment" in item:
                raise ValueError(f"{label}.deployment requires manipulation")
            twist = deployment_timeout_s = manipulation_tag_id = policy = None
        else:
            manipulation = _mapping(item["manipulation"], f"{label}.manipulation")
            _keys(manipulation, {"tag_id", "policy"}, f"{label}.manipulation")
            manipulation_tag_id = _tag_id(
                manipulation["tag_id"],
                f"{label}.manipulation.tag_id",
            )
            policy = _text(
                manipulation["policy"],
                f"{label}.manipulation.policy",
            ).lower()
            if policy not in POLICIES:
                raise ValueError(f"{label}.manipulation.policy must be one of {sorted(POLICIES)}")

            if "deployment" in item:
                deployment = _mapping(item["deployment"], f"{label}.deployment")
                _keys(deployment, {"twist", "timeout_s"}, f"{label}.deployment")
                twist = _vector(deployment["twist"], 6, f"{label}.deployment.twist")
                deployment_timeout_s = _positive(
                    deployment["timeout_s"],
                    f"{label}.deployment.timeout_s",
                )
            else:
                twist = np.zeros(6, dtype=float)
                deployment_timeout_s = DEFAULT_DEPLOYMENT_TIMEOUT_S
            if np.any(np.abs(twist) > TASK_VELOCITY_LIMITS):
                raise ValueError(f"{label}.deployment.twist exceeds arm task-velocity limits")
        points.append(
            TaskPoint(
                name=point_name,
                navigation_tag_id=navigation_tag_id,
                navigation_goal=navigation_goal,
                search_omega=search_omega,
                search_timeout_s=search_timeout_s,
                deployment_twist=twist,
                deployment_timeout_s=deployment_timeout_s,
                manipulation_tag_id=manipulation_tag_id,
                policy=policy,
            )
        )
    return Trajectory(name, tuple(points))
