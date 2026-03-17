import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.colors import LinearSegmentedColormap, Normalize
from matplotlib.cm import ScalarMappable
from pathlib import Path
import sys

FORCE_HEATMAP_CMAP = LinearSegmentedColormap.from_list(
    "task_force_penalty_heatmap",
    ["#2ca25f", "#ffff99", "#fdae61", "#d7191c"],
)


def _get_screen_size():
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()
        w = root.winfo_screenwidth()
        h = root.winfo_screenheight()
        root.destroy()
        if w > 0 and h > 0:
            return w, h
        return 1920, 1080
    except Exception:
        return 1920, 1080


def _iter_targets(task):
    for target in task.get("targets", []):
        yield np.asarray(target["center"]), float(target["radius"])


def _iter_targets_with_force(task):
    for target in task.get("targets", []):
        yield target, np.asarray(target["center"], dtype=float), float(target["radius"])


def _iter_obstacles(task):
    for obstacle in task.get("obstacles", []):
        yield np.asarray(obstacle["center"]), float(obstacle["radius"])


def _extract_traj_points(optimal_traj):
    if optimal_traj is None:
        return []

    if isinstance(optimal_traj, dict):
        if "points" in optimal_traj:
            return [np.asarray(pt).reshape(-1) for pt in optimal_traj["points"]]

        traj_points = []
        j = 0
        while f"x{j}" in optimal_traj:
            traj_points.append(np.asarray(optimal_traj[f"x{j}"]).reshape(-1))
            j += 1
        return traj_points

    return [np.asarray(pt).reshape(-1) for pt in optimal_traj]


def _trajectory_points_with_start(task, optimal_traj):
    traj_points = _extract_traj_points(optimal_traj)
    if not traj_points:
        return []

    start = np.asarray(task["start"], dtype=float).reshape(-1)
    if np.allclose(traj_points[0], start):
        return traj_points
    return [start] + traj_points


def _xy_center(center):
    center = np.asarray(center, dtype=float).reshape(-1)
    return center[:2]


def _force_application_point(target):
    center = np.asarray(target["center"], dtype=float).reshape(-1)
    if center.size >= 3:
        return center[:3]
    return np.array([center[0], center[1], 1.0], dtype=float)


def _compute_target_force_penalty(center, radius, force, radial_samples, angular_samples):
    center_xy = _xy_center(center)
    force_application_point = _force_application_point({"center": center})
    force = np.asarray(force, dtype=float).reshape(-1)

    r = np.linspace(0.0, radius, radial_samples)
    theta = np.linspace(0.0, 2.0 * np.pi, angular_samples, endpoint=False)

    radial_grid, angular_grid = np.meshgrid(r[1:], theta, indexing="xy")
    local_x = np.concatenate(([0.0], (radial_grid * np.cos(angular_grid)).ravel()))
    local_y = np.concatenate(([0.0], (radial_grid * np.sin(angular_grid)).ravel()))

    world_x = center_xy[0] + local_x
    world_y = center_xy[1] + local_y
    sampled_points = np.column_stack(
        (world_x, world_y, np.zeros_like(world_x))
    )

    force_norm = np.linalg.norm(force)
    if force_norm <= 0.0:
        penalty = np.zeros_like(world_x)
    else:
        lever_arm = force_application_point - sampled_points
        distance_to_task = np.linalg.norm(lever_arm, axis=-1)
        cross_norm = np.linalg.norm(np.cross(lever_arm, force), axis=-1)
        penalty = cross_norm / (distance_to_task * force_norm)

    return world_x, world_y, penalty


def _find_force_penalty_minimum(points_x, points_y, penalty, preferred_point):
    flat_x = np.asarray(points_x, dtype=float).ravel()
    flat_y = np.asarray(points_y, dtype=float).ravel()
    flat_penalty = np.asarray(penalty, dtype=float).ravel()
    preferred_xy = _xy_center(preferred_point)

    min_value = np.min(flat_penalty)
    candidate_mask = np.isclose(flat_penalty, min_value, rtol=1.0e-9, atol=1.0e-12)
    candidate_indices = np.flatnonzero(candidate_mask)
    candidate_points = np.column_stack((flat_x[candidate_indices], flat_y[candidate_indices]))
    distance_to_preferred = np.linalg.norm(candidate_points - preferred_xy, axis=1)
    tie_break_order = np.lexsort(
        (
            flat_y[candidate_indices],
            flat_x[candidate_indices],
            distance_to_preferred,
        )
    )
    min_index = candidate_indices[tie_break_order[0]]
    return flat_x[min_index], flat_y[min_index], flat_penalty[min_index]


def _fit_minimum_centered_surrogate(points_x, points_y, penalty, min_point, min_value):
    distances = np.sqrt((points_x - min_point[0]) ** 2 + (points_y - min_point[1]) ** 2)
    centered_penalty = penalty - min_value
    denominator = np.sum(distances**2)
    if denominator <= 0.0:
        scale = 0.0
    else:
        scale = max(np.sum(distances * centered_penalty) / denominator, 0.0)
    return min_value + scale * distances


def _build_force_target_fields(
    task,
    radial_samples,
    angular_samples,
    linearized=False,
):
    target_fields = []
    all_penalties = []
    all_x = []
    all_y = []
    all_r = []
    preferred_point = np.asarray(task["start"], dtype=float)

    for target, center, radius in _iter_targets_with_force(task):
        center_xy = _xy_center(center)
        force = np.asarray(target.get("force", np.zeros(3, dtype=float)), dtype=float)
        points_x, points_y, penalty = _compute_target_force_penalty(
            center,
            radius,
            force,
            radial_samples,
            angular_samples,
        )
        display_penalty = penalty
        if linearized:
            min_x, min_y, min_value = _find_force_penalty_minimum(
                points_x, points_y, penalty, preferred_point
            )
            display_penalty = _fit_minimum_centered_surrogate(
                points_x,
                points_y,
                penalty,
                np.array([min_x, min_y], dtype=float),
                min_value,
            )

        target_fields.append((center_xy, radius, points_x, points_y, display_penalty))
        all_penalties.append(display_penalty)
        all_x.append(center_xy[0])
        all_y.append(center_xy[1])
        all_r.append(radius)

    return target_fields, all_penalties, all_x, all_y, all_r


def _plot_force_target_fields(
    ax,
    task,
    optimal_traj,
    target_fields,
    all_penalties,
    all_x,
    all_y,
    all_r,
    title,
):
    if all_penalties:
        flat_penalties = np.concatenate(all_penalties)
        vmin = np.min(flat_penalties)
        vmax = np.max(flat_penalties)
        if np.isclose(vmin, vmax):
            vmax = vmin + 1.0e-6
        norm = Normalize(vmin=vmin, vmax=vmax)
        levels = np.linspace(vmin, vmax, 32)
    else:
        norm = Normalize(vmin=0.0, vmax=1.0)
        levels = np.linspace(0.0, 1.0, 32)

    first_center = True
    for center_xy, radius, points_x, points_y, penalty in target_fields:
        ax.tricontourf(
            points_x,
            points_y,
            penalty,
            levels=levels,
            cmap=FORCE_HEATMAP_CMAP,
            norm=norm,
            alpha=0.75,
            zorder=1,
        )
        ax.add_patch(
            Circle(
                center_xy,
                radius,
                facecolor="none",
                edgecolor="black",
                linewidth=0.75,
                zorder=2,
                label="task regions" if first_center else None,
            )
        )
        ax.scatter(
            center_xy[0],
            center_xy[1],
            color="black",
            s=45,
            zorder=4,
            label="task centers" if first_center else None,
        )
        first_center = False

    common_x, common_y, common_r = _plot_common_task_elements(ax, task, optimal_traj)
    if all_x:
        common_x = np.concatenate((common_x, np.asarray(all_x)))
        common_y = np.concatenate((common_y, np.asarray(all_y)))
        common_r = np.concatenate((common_r, np.asarray(all_r)))

    ax.set_aspect("equal", adjustable="box")

    xmin = np.min(common_x - common_r)
    xmax = np.max(common_x + common_r)
    ymin = np.min(common_y - common_r)
    ymax = np.max(common_y + common_r)

    dx = xmax - xmin
    dy = ymax - ymin
    pad = 0.15 * max(dx, dy, 1.0)

    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper right")
    return norm


def _plot_common_task_elements(ax, task, optimal_traj=None):
    start = np.asarray(task["start"], dtype=float)
    ax.scatter(start[0], start[1], color="black", s=60, zorder=4, label="start")

    all_x = [start[0]]
    all_y = [start[1]]
    all_r = [0.0]

    first_obstacle = True
    for center, radius in _iter_obstacles(task):
        center_xy = _xy_center(center)
        ax.add_patch(
            Circle(
                center_xy,
                radius,
                facecolor="orange",
                edgecolor="orange",
                alpha=0.35,
                zorder=3,
                label="obstacle" if first_obstacle else None,
            )
        )
        ax.scatter(
            center_xy[0],
            center_xy[1],
            color="orange",
            s=50,
            zorder=4,
        )
        first_obstacle = False

        all_x.append(center_xy[0])
        all_y.append(center_xy[1])
        all_r.append(radius)

    traj_points = _trajectory_points_with_start(task, optimal_traj)
    if len(traj_points) >= 2:
        traj_points = np.array(traj_points)
        all_x.extend(traj_points[:, 0].tolist())
        all_y.extend(traj_points[:, 1].tolist())
        all_r.extend([0.0] * len(traj_points))

        ax.plot(
            traj_points[:, 0],
            traj_points[:, 1],
            linestyle="--",
            color="blue",
            linewidth=2,
            zorder=5,
            label="optimal trajectory",
        )
        ax.scatter(
            traj_points[1:, 0],
            traj_points[1:, 1],
            color="blue",
            s=50,
            zorder=6,
        )

    return np.array(all_x), np.array(all_y), np.array(all_r)


def _add_smoothed_trajectory(ax, task, optimal_traj, all_x, all_y, all_r):
    traj_points = _trajectory_points_with_start(task, optimal_traj)
    if len(traj_points) < 2:
        return all_x, all_y, all_r

    traj_points = np.array(traj_points)
    all_x.extend(traj_points[:, 0].tolist())
    all_y.extend(traj_points[:, 1].tolist())
    all_r.extend([0.0] * len(traj_points))

    ax.plot(
        traj_points[:, 0],
        traj_points[:, 1],
        linestyle="--",
        color="blue",
        alpha=0.5,
        linewidth=1.5,
        zorder=4,
        label="polyline",
    )
    ax.scatter(
        traj_points[1:, 0],
        traj_points[1:, 1],
        color="blue",
        s=50,
        zorder=6,
    )

    d = np.linalg.norm(np.diff(traj_points, axis=0), axis=1)
    t = np.zeros(len(traj_points))
    t[1:] = np.cumsum(d)

    if t[-1] <= 0:
        return all_x, all_y, all_r

    m = np.zeros_like(traj_points)
    m[0] = (traj_points[1] - traj_points[0]) / max(t[1] - t[0], 1e-9)
    m[-1] = (traj_points[-1] - traj_points[-2]) / max(t[-1] - t[-2], 1e-9)

    for k in range(1, len(traj_points) - 1):
        denom = max(t[k + 1] - t[k - 1], 1e-9)
        m[k] = (traj_points[k + 1] - traj_points[k - 1]) / denom

    smooth_pts = []
    samples_per_segment = 100

    for k in range(len(traj_points) - 1):
        p0 = traj_points[k]
        p1 = traj_points[k + 1]
        m0 = m[k]
        m1 = m[k + 1]
        h = t[k + 1] - t[k]

        tau_vals = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)
        for tau in tau_vals:
            h00 = 2 * tau**3 - 3 * tau**2 + 1
            h10 = tau**3 - 2 * tau**2 + tau
            h01 = -2 * tau**3 + 3 * tau**2
            h11 = tau**3 - tau**2

            pt = h00 * p0 + h10 * h * m0 + h01 * p1 + h11 * h * m1
            smooth_pts.append(pt)

    smooth_pts.append(traj_points[-1])
    smooth_pts = np.array(smooth_pts)

    ax.plot(
        smooth_pts[:, 0],
        smooth_pts[:, 1],
        linestyle="-",
        color="red",
        linewidth=2.0,
        zorder=5,
        label="smoothed trajectory",
    )

    all_x.extend(smooth_pts[:, 0].tolist())
    all_y.extend(smooth_pts[:, 1].tolist())
    all_r.extend([0.0] * len(smooth_pts))
    return all_x, all_y, all_r


def plot_task(task, optimal_traj=None, save_path=None):
    """
    Visualize a task with:
      - start point as a black dot
      - target centers as green dots
      - target radii as transparent green circles
      - obstacle centers as orange dots
      - obstacle radii as transparent orange circles
      - optional optimal trajectory as dashed blue polyline

    Expected task format:
        task["start"]
        task["targets"] = [{"center": ..., "radius": ...}, ...]
        task["obstacles"] = [{"center": ..., "radius": ...}, ...]

    Expected optimal_traj format:
        {"points": [x0, x1, ...]} or [x0, x1, ...]
        Legacy {"x0", "x1", ...} dicts are also supported.

    If save_path is given the figure is saved there; otherwise plt.show() is called.
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.8 / dpi))

    start = np.asarray(task["start"])
    ax.scatter(start[0], start[1], color="black", s=60, zorder=4, label="start")

    all_x = [start[0]]
    all_y = [start[1]]
    all_r = [0.0]

    first_circle = True
    first_center = True
    for center, radius in _iter_targets(task):
        ax.add_patch(
            Circle(
                center,
                radius,
                facecolor="green",
                edgecolor="green",
                alpha=0.25,
                zorder=1,
                label="task regions" if first_circle else None,
            )
        )
        first_circle = False

        ax.scatter(
            center[0],
            center[1],
            color="green",
            s=50,
            zorder=2,
            label="task centers" if first_center else None,
        )
        first_center = False

        all_x.append(center[0])
        all_y.append(center[1])
        all_r.append(radius)

    first_obstacle = True
    for center, radius in _iter_obstacles(task):
        ax.add_patch(
            Circle(
                center,
                radius,
                facecolor="orange",
                edgecolor="orange",
                alpha=0.35,
                zorder=1,
                label="obstacle" if first_obstacle else None,
            )
        )
        ax.scatter(
            center[0],
            center[1],
            color="orange",
            s=50,
            zorder=2,
        )
        first_obstacle = False

        all_x.append(center[0])
        all_y.append(center[1])
        all_r.append(radius)

    traj_points = _extract_traj_points(optimal_traj)
    if len(traj_points) >= 2:
        traj_points = np.array(traj_points)
        all_x.extend(traj_points[:, 0].tolist())
        all_y.extend(traj_points[:, 1].tolist())
        all_r.extend([0.0] * len(traj_points))

        ax.plot(
            traj_points[:, 0],
            traj_points[:, 1],
            linestyle="--",
            color="blue",
            linewidth=2,
            zorder=5,
            label="optimal trajectory",
        )
        ax.scatter(
            traj_points[1:, 0],
            traj_points[1:, 1],
            color="blue",
            s=50,
            zorder=6,
        )

    ax.set_aspect("equal", adjustable="box")

    all_x = np.array(all_x)
    all_y = np.array(all_y)
    all_r = np.array(all_r)

    xmin = np.min(all_x - all_r)
    xmax = np.max(all_x + all_r)
    ymin = np.min(all_y - all_r)
    ymax = np.max(all_y + all_r)

    dx = xmax - xmin
    dy = ymax - ymin
    pad = 0.15 * max(dx, dy, 1.0)

    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Task visualization")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="upper right")
    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()


def plot_task_w_forces(
    task,
    optimal_traj=None,
    save_path=None,
    radial_samples=80,
    angular_samples=180,
):
    """
    Visualize a task with force-colored target regions.

    Each target is drawn as a disk heatmap of the alignment penalty sin(theta),
    using the target center as the force application point and the target
    force vector stored under target["force"].
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.8 / dpi))

    target_fields, all_penalties, all_x, all_y, all_r = _build_force_target_fields(
        task,
        radial_samples,
        angular_samples,
        linearized=False,
    )
    norm = _plot_force_target_fields(
        ax,
        task,
        optimal_traj,
        target_fields,
        all_penalties,
        all_x,
        all_y,
        all_r,
        "Task visualization with force penalty regions",
    )
    fig.colorbar(
        ScalarMappable(norm=norm, cmap=FORCE_HEATMAP_CMAP),
        ax=ax,
        shrink=0.67,
        label=r"$\sin(\theta)$",
    )

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()


def plot_task_w_linearized_forces(
    task,
    optimal_traj=None,
    save_path=None,
    radial_samples=80,
    angular_samples=180,
):
    """
    Visualize a task with per-target linearized force-penalty regions.

    Each target is sampled with the original alignment penalty and then
    approximated by a minimum-centered convex surrogate. If multiple sampled
    points share the minimum penalty, the one closest to task["start"] is used
    as the linearization point.
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.8 / dpi))

    target_fields, all_penalties, all_x, all_y, all_r = _build_force_target_fields(
        task,
        radial_samples,
        angular_samples,
        linearized=True,
    )
    norm = _plot_force_target_fields(
        ax,
        task,
        optimal_traj,
        target_fields,
        all_penalties,
        all_x,
        all_y,
        all_r,
        "Task visualization with linearized force penalty regions",
    )
    fig.colorbar(
        ScalarMappable(norm=norm, cmap=FORCE_HEATMAP_CMAP),
        ax=ax,
        shrink=0.67,
        label=r"$\sin(\theta)$",
    )

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight")
        plt.close(fig)
    else:
        plt.show()


def plot_task_smooth_w_forces(
    task,
    optimal_traj,
    save_path=None,
    masked=False,
    radial_samples=80,
    angular_samples=180,
    linearized=False,
):
    """
    Visualize force-colored task regions with the polyline and smoothed trajectory overlaid.

    Set linearized=True to show the minimum-centered convex surrogate regions used by
    the force-aware solver objective.
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.9 / dpi))

    target_fields, all_penalties, all_x, all_y, all_r = _build_force_target_fields(
        task,
        radial_samples,
        angular_samples,
        linearized=linearized,
    )

    if all_penalties:
        flat_penalties = np.concatenate(all_penalties)
        vmin = np.min(flat_penalties)
        vmax = np.max(flat_penalties)
        if np.isclose(vmin, vmax):
            vmax = vmin + 1.0e-6
        norm = Normalize(vmin=vmin, vmax=vmax)
        levels = np.linspace(vmin, vmax, 32)
    else:
        norm = Normalize(vmin=0.0, vmax=1.0)
        levels = np.linspace(0.0, 1.0, 32)

    start = np.asarray(task["start"], dtype=float)
    ax.scatter(start[0], start[1], color="black", s=60, zorder=6, label="start")
    plot_x = [start[0]]
    plot_y = [start[1]]
    plot_r = [0.0]

    first_center = True
    for center_xy, radius, points_x, points_y, penalty in target_fields:
        ax.tricontourf(
            points_x,
            points_y,
            penalty,
            levels=levels,
            cmap=FORCE_HEATMAP_CMAP,
            norm=norm,
            alpha=0.75,
            zorder=1,
        )
        ax.add_patch(
            Circle(
                center_xy,
                radius,
                facecolor="none",
                edgecolor="black",
                linewidth=0.75,
                zorder=2,
                label="task regions" if first_center else None,
            )
        )
        ax.scatter(
            center_xy[0],
            center_xy[1],
            color="black",
            s=45,
            zorder=4,
            label="task centers" if first_center else None,
        )
        first_center = False

        plot_x.append(center_xy[0])
        plot_y.append(center_xy[1])
        plot_r.append(radius)

    first_obstacle = True
    for center, radius in _iter_obstacles(task):
        center_xy = _xy_center(center)
        ax.add_patch(
            Circle(
                center_xy,
                radius,
                facecolor="orange",
                edgecolor="orange",
                alpha=0.35,
                zorder=3,
                label="obstacle" if first_obstacle else None,
            )
        )
        ax.scatter(
            center_xy[0],
            center_xy[1],
            color="orange",
            s=50,
            zorder=4,
        )
        first_obstacle = False

        plot_x.append(center_xy[0])
        plot_y.append(center_xy[1])
        plot_r.append(radius)

    plot_x, plot_y, plot_r = _add_smoothed_trajectory(
        ax,
        task,
        optimal_traj,
        plot_x,
        plot_y,
        plot_r,
    )

    ax.set_aspect("equal", adjustable="box")

    plot_x = np.asarray(plot_x)
    plot_y = np.asarray(plot_y)
    plot_r = np.asarray(plot_r)

    xmin = np.min(plot_x - plot_r)
    xmax = np.max(plot_x + plot_r)
    ymin = np.min(plot_y - plot_r)
    ymax = np.max(plot_y + plot_r)

    dx = xmax - xmin
    dy = ymax - ymin
    pad = 0.15 * max(dx, dy, 1.0)

    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    if masked:
        ax.axis("off")
        fig.patch.set_alpha(0.0)
        ax.patch.set_alpha(0.0)
    else:
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        if linearized:
            ax.set_title("Task visualization with linearized force regions and smoothed trajectory")
        else:
            ax.set_title("task/force regions and smoothed trajectory")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="upper right")
        fig.colorbar(
            ScalarMappable(norm=norm, cmap=FORCE_HEATMAP_CMAP),
            ax=ax,
            shrink=0.67,
            label=r"$\sin(\theta)$",
        )

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight", transparent=masked)
        plt.close(fig)
    else:
        plt.show()


def plot_task_smooth(task, optimal_traj, save_path=None, masked=False):
    """
    Visualize a task with:
      - start point as a black dot
      - target centers as green dots
      - target radii as transparent green circles
      - obstacle centers as orange dots
      - obstacle radii as transparent orange circles
      - original polyline in semi-transparent dashed blue
      - smoothed piecewise-cubic trajectory in solid green

    Expected task format:
        task["start"]
        task["targets"] = [{"center": ..., "radius": ...}, ...]
        task["obstacles"] = [{"center": ..., "radius": ...}, ...]

    Expected optimal_traj format:
        {"points": [x0, x1, ...]} or [x0, x1, ...]
        Legacy {"x0", "x1", ...} dicts are also supported.

    If save_path is given the figure is saved there; otherwise plt.show() is called.
    If masked=True, all axes/labels/title/legend/grid are hidden and the
    background is transparent (good for compositing onto other figures).
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.9 / dpi))

    start = np.asarray(task["start"])
    ax.scatter(start[0], start[1], color="black", s=60, zorder=4, label="start")

    all_x = [start[0]]
    all_y = [start[1]]
    all_r = [0.0]

    first_circle = True
    first_center = True
    for center, radius in _iter_targets(task):
        ax.add_patch(
            Circle(
                center,
                radius,
                facecolor="green",
                edgecolor="green",
                alpha=0.25,
                zorder=1,
                label="task regions" if first_circle else None,
            )
        )
        first_circle = False

        ax.scatter(
            center[0],
            center[1],
            color="green",
            s=50,
            zorder=2,
            label="task centers" if first_center else None,
        )
        first_center = False

        all_x.append(center[0])
        all_y.append(center[1])
        all_r.append(radius)

    first_obstacle = True
    for center, radius in _iter_obstacles(task):
        ax.add_patch(
            Circle(
                center,
                radius,
                facecolor="orange",
                edgecolor="orange",
                alpha=0.35,
                zorder=1,
                label="obstacle" if first_obstacle else None,
            )
        )
        ax.scatter(
            center[0],
            center[1],
            color="orange",
            s=50,
            zorder=2,
        )
        first_obstacle = False

        all_x.append(center[0])
        all_y.append(center[1])
        all_r.append(radius)

    traj_points = _trajectory_points_with_start(task, optimal_traj)
    if len(traj_points) >= 2:
        traj_points = np.array(traj_points)
        all_x.extend(traj_points[:, 0].tolist())
        all_y.extend(traj_points[:, 1].tolist())
        all_r.extend([0.0] * len(traj_points))

        ax.plot(
            traj_points[:, 0],
            traj_points[:, 1],
            linestyle="--",
            color="blue",
            alpha=0.5,
            linewidth=1.5,
            zorder=4,
            label="polyline",
        )
        ax.scatter(
            traj_points[1:, 0],
            traj_points[1:, 1],
            color="blue",
            s=50,
            zorder=6,
        )

        d = np.linalg.norm(np.diff(traj_points, axis=0), axis=1)
        t = np.zeros(len(traj_points))
        t[1:] = np.cumsum(d)

        if t[-1] > 0:
            m = np.zeros_like(traj_points)
            m[0] = (traj_points[1] - traj_points[0]) / max(t[1] - t[0], 1e-9)
            m[-1] = (traj_points[-1] - traj_points[-2]) / max(t[-1] - t[-2], 1e-9)

            for k in range(1, len(traj_points) - 1):
                denom = max(t[k + 1] - t[k - 1], 1e-9)
                m[k] = (traj_points[k + 1] - traj_points[k - 1]) / denom

            smooth_pts = []
            samples_per_segment = 100

            for k in range(len(traj_points) - 1):
                p0 = traj_points[k]
                p1 = traj_points[k + 1]
                m0 = m[k]
                m1 = m[k + 1]
                h = t[k + 1] - t[k]

                tau_vals = np.linspace(0.0, 1.0, samples_per_segment, endpoint=False)
                for tau in tau_vals:
                    h00 = 2 * tau**3 - 3 * tau**2 + 1
                    h10 = tau**3 - 2 * tau**2 + tau
                    h01 = -2 * tau**3 + 3 * tau**2
                    h11 = tau**3 - tau**2

                    pt = h00 * p0 + h10 * h * m0 + h01 * p1 + h11 * h * m1
                    smooth_pts.append(pt)

            smooth_pts.append(traj_points[-1])
            smooth_pts = np.array(smooth_pts)

            ax.plot(
                smooth_pts[:, 0],
                smooth_pts[:, 1],
                linestyle="-",
                color="red",
                linewidth=2.0,
                zorder=5,
                label="smoothed trajectory",
            )

            all_x.extend(smooth_pts[:, 0].tolist())
            all_y.extend(smooth_pts[:, 1].tolist())
            all_r.extend([0.0] * len(smooth_pts))

    ax.set_aspect("equal", adjustable="box")

    all_x = np.array(all_x)
    all_y = np.array(all_y)
    all_r = np.array(all_r)

    xmin = np.min(all_x - all_r)
    xmax = np.max(all_x + all_r)
    ymin = np.min(all_y - all_r)
    ymax = np.max(all_y + all_r)

    dx = xmax - xmin
    dy = ymax - ymin
    pad = 0.15 * max(dx, dy, 1.0)

    ax.set_xlim(xmin - pad, xmax + pad)
    ax.set_ylim(ymin - pad, ymax + pad)

    if masked:
        ax.axis("off")
        fig.patch.set_alpha(0.0)
        ax.patch.set_alpha(0.0)
    else:
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_title("Task visualization with smoothed trajectory")
        ax.grid(True, linestyle="--", alpha=0.4)
        ax.legend(loc="upper right")

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight", transparent=masked)
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    repo_root = Path(__file__).resolve().parents[2]
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))

    from cvxpy_ws.tasks.TASK_demo import TASK

    plot_task(TASK)
