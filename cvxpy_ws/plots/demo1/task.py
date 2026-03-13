import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

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


def plot_task(task, optimal_traj=None, save_path=None):
    """
    Visualize a task with:
      - start point x0 as a black dot
      - task centers c1, c2, ... as red dots
      - task radii r1, r2, ... as transparent red circles
      - optional optimal trajectory x0 -> x1 -> x2 -> ... as dashed blue polyline

    Expected task format:
        task["x0"]
        task["c1"], task["r1"]
        task["c2"], task["r2"]
        ...

    Expected optimal_traj format:
        optimal_traj["x0"], optimal_traj["x1"], optimal_traj["x2"], ...

    If save_path is given the figure is saved there; otherwise plt.show() is called.
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.8 / dpi))

    # Start point
    x0 = np.asarray(task["x0"])
    ax.scatter(x0[0], x0[1], color="black", s=60, zorder=4, label="start")

    all_x = [x0[0]]
    all_y = [x0[1]]
    all_r = [0.0]

    i = 1
    first_circle = True
    first_center = True
    while f"c{i}" in task and f"r{i}" in task:
        center = np.asarray(task[f"c{i}"])
        radius = float(task[f"r{i}"])

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
        i += 1

    # Optional optimal trajectory overlay
    if optimal_traj is not None:
        traj_points = []
        j = 0
        while f"x{j}" in optimal_traj:
            pt = np.asarray(optimal_traj[f"x{j}"]).reshape(-1)
            traj_points.append(pt)
            all_x.append(pt[0])
            all_y.append(pt[1])
            all_r.append(0.0)
            j += 1

        if len(traj_points) >= 2:
            traj_points = np.array(traj_points)
            ax.plot(
                traj_points[:, 0],
                traj_points[:, 1],
                linestyle="--",
                color="blue",
                linewidth=2,
                zorder=5,
                label="optimal trajectory",
            )
            # Blue dots at x1, x2, x3, ... (same size as red task center dots)
            if len(traj_points) > 1:
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
      - start point x0 as a black dot
      - task centers c1, c2, ... as red dots
      - task radii r1, r2, ... as transparent red circles
      - original polyline in semi-transparent dashed blue
      - smoothed piecewise-cubic trajectory in solid dark green

    Expected task format:
        task["x0"]
        task["c1"], task["r1"]
        task["c2"], task["r2"]
        ...

    Expected optimal_traj format:
        optimal_traj["x0"], optimal_traj["x1"], optimal_traj["x2"], ...

    If save_path is given the figure is saved there; otherwise plt.show() is called.
    If masked=True, all axes/labels/title/legend/grid are hidden and the
    background is transparent (good for compositing onto other figures).
    """
    screen_w, screen_h = _get_screen_size()
    dpi = plt.rcParams.get("figure.dpi", 100)
    fig, ax = plt.subplots(figsize=(screen_w / 2 / dpi, screen_h * 0.9 / dpi))

    # Start point
    x0 = np.asarray(task["x0"])
    ax.scatter(x0[0], x0[1], color="black", s=60, zorder=4, label="start")

    all_x = [x0[0]]
    all_y = [x0[1]]
    all_r = [0.0]

    # Plot task regions and centers
    i = 1
    first_circle = True
    first_center = True
    while f"c{i}" in task and f"r{i}" in task:
        center = np.asarray(task[f"c{i}"])
        radius = float(task[f"r{i}"])

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
        i += 1

    # Gather trajectory points
    traj_points = []
    j = 0
    while f"x{j}" in optimal_traj:
        pt = np.asarray(optimal_traj[f"x{j}"]).reshape(-1)
        traj_points.append(pt)
        all_x.append(pt[0])
        all_y.append(pt[1])
        all_r.append(0.0)
        j += 1

    if len(traj_points) >= 2:
        traj_points = np.array(traj_points)

        # Original polyline
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
        # Blue dots at x1, x2, x3, ... (same size as red task center dots)
        ax.scatter(
            traj_points[1:, 0],
            traj_points[1:, 1],
            color="blue",
            s=50,
            zorder=6,
        )

        # ---- Piecewise cubic Hermite spline ----
        # Chord-length parameterization
        d = np.linalg.norm(np.diff(traj_points, axis=0), axis=1)
        t = np.zeros(len(traj_points))
        t[1:] = np.cumsum(d)

        # Handle degenerate case where consecutive points coincide
        if t[-1] > 0:
            # Estimate tangents by finite differences
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

                    pt = (
                        h00 * p0
                        + h10 * h * m0
                        + h01 * p1
                        + h11 * h * m1
                    )
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
        plt.savefig(save_path, dpi=300, bbox_inches="tight",
                    transparent=masked)
        plt.close(fig)
    else:
        plt.show()
