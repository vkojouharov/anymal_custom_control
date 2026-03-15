import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from pathlib import Path
import sys


def _get_screen_size():
    try:
        import tkinter as tk

        root = tk.Tk()
        root.withdraw()
        w = root.winfo_screenwidth()
        h = root.winfo_screenheight()
        root.destroy()
        return w, h
    except Exception:
        return 1920, 1080


def _iter_targets(task):
    for target in task.get("targets", []):
        yield np.asarray(target["center"]), float(target["radius"])


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
