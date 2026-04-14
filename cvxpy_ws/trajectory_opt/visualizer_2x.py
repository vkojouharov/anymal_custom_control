import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.font_manager import FontProperties

from trajectory_opt.visualizer import (
    _get_screen_size,
    _iter_obstacles,
    _iter_targets,
    _trajectory_points_with_start,
    _xy_center,
)


def _legend_fontsize_points(scale=1.0):
    base_size = plt.rcParams.get("legend.fontsize", plt.rcParams.get("font.size", 10))
    return FontProperties(size=base_size).get_size_in_points() * scale


def _add_smoothed_trajectory_on_axis(
    ax,
    task,
    optimal_traj,
    all_x,
    all_y,
    all_r,
    polyline_alpha=0.5,
):
    traj_points = _trajectory_points_with_start(task, optimal_traj)
    if len(traj_points) < 2:
        return all_x, all_y, all_r

    traj_points = np.asarray(traj_points, dtype=float)
    all_x.extend(traj_points[:, 0].tolist())
    all_y.extend(traj_points[:, 1].tolist())
    all_r.extend([0.0] * len(traj_points))

    ax.plot(
        traj_points[:, 0],
        traj_points[:, 1],
        linestyle="--",
        color="blue",
        alpha=polyline_alpha,
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
            smooth_pts.append(h00 * p0 + h10 * h * m0 + h01 * p1 + h11 * h * m1)

    smooth_pts.append(traj_points[-1])
    smooth_pts = np.asarray(smooth_pts, dtype=float)

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


def _plot_task_smooth_on_axis(
    ax,
    task,
    optimal_traj,
    title=None,
    show_legend=False,
    show_ylabel=True,
    legend_fontscale=1.0,
    legend_ncol=2,
    masked=False,
):
    start = _xy_center(task["start"])
    ax.scatter(start[0], start[1], color="black", s=60, zorder=4, label="start")

    all_x = [start[0]]
    all_y = [start[1]]
    all_r = [0.0]

    first_circle = True
    first_center = True
    for center, radius in _iter_targets(task):
        center_xy = _xy_center(center)
        ax.add_patch(
            Circle(
                center_xy,
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
            center_xy[0],
            center_xy[1],
            color="green",
            s=50,
            zorder=2,
            label="task centers" if first_center else None,
        )
        first_center = False

        all_x.append(center_xy[0])
        all_y.append(center_xy[1])
        all_r.append(radius)

    first_obstacle = True
    for center, radius in _iter_obstacles(task):
        center_xy = _xy_center(center)
        ax.add_patch(
            Circle(
                center_xy,
                radius,
                facecolor="orange",
                edgecolor="orange",
                alpha=1.0 if masked else 0.35,
                zorder=1,
                label="obstacle" if first_obstacle else None,
            )
        )
        ax.scatter(
            center_xy[0],
            center_xy[1],
            color="orange",
            s=50,
            zorder=2,
        )
        first_obstacle = False

        all_x.append(center_xy[0])
        all_y.append(center_xy[1])
        all_r.append(radius)

    all_x, all_y, all_r = _add_smoothed_trajectory_on_axis(
        ax,
        task,
        optimal_traj,
        all_x,
        all_y,
        all_r,
        polyline_alpha=1.0 if masked else 0.5,
    )
    all_x = np.asarray(all_x, dtype=float)
    all_y = np.asarray(all_y, dtype=float)
    all_r = np.asarray(all_r, dtype=float)

    ax.set_aspect("equal", adjustable="box")
    if not masked:
        if show_ylabel:
            ax.set_ylabel("y")
        ax.grid(True, linestyle="--", alpha=0.4)
        if title is not None:
            ax.set_title(title)
    if show_legend and not masked:
        ax.legend(
            loc="upper right",
            ncol=legend_ncol,
            fontsize=_legend_fontsize_points(legend_fontscale),
            columnspacing=1.0,
            handlelength=1.8,
            borderpad=0.5,
            labelspacing=0.4,
        )

    return all_x, all_y, all_r


def plot_tasks_smooth_2x(
    left_task,
    left_traj,
    right_task,
    right_traj,
    left_title="Left task",
    right_title="Right task",
    save_path=None,
    masked=False,
    legend_fontscale=1.0,
    legend_ncol=2,
):
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, axes = plt.subplots(
        1,
        2,
        sharex=True,
        sharey=True,
        figsize=(screen_w * 0.9 / dpi, screen_h * 0.48 / dpi),
    )

    left_x, left_y, left_r = _plot_task_smooth_on_axis(
        axes[0],
        left_task,
        left_traj,
        title=left_title,
        show_legend=not masked,
        show_ylabel=True,
        legend_fontscale=legend_fontscale,
        legend_ncol=legend_ncol,
        masked=masked,
    )
    right_x, right_y, right_r = _plot_task_smooth_on_axis(
        axes[1],
        right_task,
        right_traj,
        title=right_title,
        show_legend=False,
        show_ylabel=False,
        masked=masked,
    )

    all_x = np.concatenate((left_x, right_x))
    all_y = np.concatenate((left_y, right_y))
    all_r = np.concatenate((left_r, right_r))

    xmin = np.min(all_x - all_r)
    xmax = np.max(all_x + all_r)
    ymin = np.min(all_y - all_r)
    ymax = np.max(all_y + all_r)

    dx = xmax - xmin
    dy = ymax - ymin
    pad = 0.15 * max(dx, dy, 1.0)

    for ax in axes:
        ax.set_xlim(xmin - pad, xmax + pad)
        ax.set_ylim(ymin - pad, ymax + pad)

    if not masked:
        for ax in axes:
            ax.set_xlabel("x")

    if masked:
        for ax in axes:
            ax.axis("off")
            ax.patch.set_alpha(0.0)
        fig.patch.set_alpha(0.0)

    fig.tight_layout()
    fig.subplots_adjust(wspace=0.06)

    if save_path:
        plt.savefig(save_path, dpi=300, bbox_inches="tight", transparent=masked)
        plt.close(fig)
    else:
        plt.show()
