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
    formatted = []
    for value in values:
        if value is None:
            formatted.append("None")
        else:
            formatted.append(_format_float(value))
    return "(" + ", ".join(formatted) + ")"


def trajectory_to_anymal_waypoints(
    task,
    trajectory,
    ee_height=0.5,
    kept_indices=None,
    points_per_target=None,
    intermediate_arm_pose=(None, None, None),
):
    traj_points = [_xy(point) for point in trajectory]
    targets = task.get("targets", [])

    if len(traj_points) != len(targets) and (kept_indices is None or points_per_target is None):
        raise ValueError(
            "Trajectory export needs kept_indices and points_per_target when intermediate "
            "waypoints are present."
        )
    if kept_indices is not None and len(kept_indices) != len(traj_points):
        raise ValueError("kept_indices must align one-to-one with the pruned trajectory.")

    anymal_waypoints = []
    arm_waypoints = []
    if kept_indices is None:
        waypoint_info = [
            ("target", target_idx, point)
            for target_idx, point in enumerate(traj_points)
        ]
    else:
        target_indices = {
            (target_idx + 1) * points_per_target - 1
            for target_idx in range(len(targets))
        }
        waypoint_info = []
        for point, original_idx in zip(traj_points, kept_indices):
            if original_idx in target_indices:
                waypoint_info.append(("target", original_idx // points_per_target, point))
            else:
                waypoint_info.append(("intermediate", None, point))

    for waypoint_kind, target_idx, point in waypoint_info:
        anymal_waypoints.append((-point[0], -point[1], 0.0))
        if waypoint_kind == "target":
            target_center = _xy(targets[target_idx]["center"])
            arm_waypoints.append((target_center[0] - point[0], target_center[1] - point[1], ee_height))
        else:
            arm_waypoints.append(tuple(intermediate_arm_pose))

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


def export_task_trajectory_module(
    task_module_name,
    task,
    trajectory,
    output_dir,
    ee_height=0.5,
    kept_indices=None,
    points_per_target=None,
    intermediate_arm_pose=(None, None, None),
):
    module_stem = Path(task_module_name).stem
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    anymal_waypoints, arm_waypoints = trajectory_to_anymal_waypoints(
        task,
        trajectory,
        ee_height=ee_height,
        kept_indices=kept_indices,
        points_per_target=points_per_target,
        intermediate_arm_pose=intermediate_arm_pose,
    )
    output_path = output_dir / f"{module_stem}.py"
    write_anymal_trajectory_module(
        output_path,
        anymal_waypoints,
        arm_waypoints,
        source_name=f"{module_stem}.py",
    )
    return output_path
