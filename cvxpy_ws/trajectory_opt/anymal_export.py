from pathlib import Path

import numpy as np


def _xy(point):
    return np.asarray(point, dtype=float).reshape(-1)[:2]


def _format_float(value):
    value = float(value)
    if abs(value) < 5.0e-13:
        value = 0.0

    text = f"{value:.6f}".rstrip("0").rstrip(".")
    if "." not in text:
        text += ".0"
    return text


def _format_tuple(values):
    return "(" + ", ".join(_format_float(v) for v in values) + ")"


def trajectory_to_anymal_waypoints(task, trajectory, ee_height=0.5):
    traj_points = [_xy(point) for point in trajectory]
    targets = task.get("targets", [])

    if len(traj_points) != len(targets):
        raise ValueError(
            "Trajectory export currently supports only one solved waypoint per target. "
            "Intermediate-point handling has not been added yet."
        )

    anymal_waypoints = []
    arm_waypoints = []
    for point, target in zip(traj_points, targets):
        target_center = _xy(target["center"])
        anymal_waypoints.append((-point[0], -point[1], 0.0))
        arm_waypoints.append((target_center[0] - point[0], target_center[1] - point[1], ee_height))

    return anymal_waypoints, arm_waypoints


def write_anymal_trajectory_module(output_path, anymal_waypoints, arm_waypoints, source_name=None):
    output_path = Path(output_path)

    lines = []
    if source_name is not None:
        lines.append(f"# Auto-generated from cvxpy task {source_name}\n")
    lines.append("# ANYmal waypoints: (dx, dy, dyaw) displacements from start pose [meters, rad]\n")
    lines.append("ANYMAL_WAYPOINTS = [\n")
    for idx, waypoint in enumerate(anymal_waypoints, start=1):
        lines.append(f"    {_format_tuple(waypoint)},  # waypoint {idx}\n")
    lines.append("]\n\n")
    lines.append("# Arm EE waypoints: (x, y, z) target positions in arm base frame [meters]\n")
    lines.append("ARM_WAYPOINTS = [\n")
    for idx, waypoint in enumerate(arm_waypoints, start=1):
        lines.append(f"    {_format_tuple(waypoint)},  # waypoint {idx}\n")
    lines.append("]\n")

    output_path.write_text("".join(lines), encoding="ascii")


def export_task_trajectory_module(task_module_name, task, trajectory, output_dir, ee_height=0.5):
    module_stem = Path(task_module_name).stem
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    anymal_waypoints, arm_waypoints = trajectory_to_anymal_waypoints(
        task,
        trajectory,
        ee_height=ee_height,
    )
    output_path = output_dir / f"{module_stem}.py"
    write_anymal_trajectory_module(
        output_path,
        anymal_waypoints,
        arm_waypoints,
        source_name=f"{module_stem}.py",
    )
    return output_path
