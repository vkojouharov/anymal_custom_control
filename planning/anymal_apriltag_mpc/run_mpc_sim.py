#!/usr/bin/env python3
"""
Visual-servo MPC simulation for a holonomic ANYmal-style base.

This script simulates the receding-horizon MPC loop only. The optimized
velocity command is expressed in the AprilTag/global frame and is applied
directly to the simulated state. For hardware, convert the global velocity
command to the robot body frame before sending it to the controller.

The camera is modeled as looking along the robot's negative x-axis. With this
convention, the pose [0.5, 0, 0] is directly in front of the tag and has zero
tag bearing in the camera frame.
"""

import argparse
import os
import tempfile
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "xdg-cache"))

import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, writers
from matplotlib.patches import Polygon

if __package__:
    from .solve_mpc import bearing_to_tag, solve_visual_mpc_step
else:
    from solve_mpc import bearing_to_tag, solve_visual_mpc_step


TAG_BLUE = (14.0 / 255.0, 158.0 / 255.0, 213.0 / 255.0)
ROBOT_RED = (255.0 / 255.0, 82.0 / 255.0, 74.0 / 255.0)
PRED_ORANGE = (230.0 / 255.0, 126.0 / 255.0, 34.0 / 255.0)
SCRIPT_DIR = Path(__file__).resolve().parent

DEFAULT_DT = 0.2
DEFAULT_HORIZON = 20
DEFAULT_MAX_ITERS = 100
DEFAULT_VIDEO_FPS = 5


def wrap_angle(a):
    """Wrap an angle in radians to [-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def draw_oriented_rectangle(
    ax,
    pose,
    length,
    width,
    facecolor,
    edgecolor="black",
    linewidth=1.0,
    alpha=1.0,
    label=None,
    zorder=3.0,
):
    """
    Draw a rectangle centered at pose = [x, y, theta].

    The rectangle's local x-axis has size `length`, local y-axis has size
    `width`, and theta rotates the local frame into the plot/world frame.
    """
    px, py, theta = pose
    half_l = 0.5 * length
    half_w = 0.5 * width

    corners_local = np.array(
        [
            [half_l, half_w],
            [half_l, -half_w],
            [-half_l, -half_w],
            [-half_l, half_w],
        ]
    )

    c = np.cos(theta)
    s = np.sin(theta)
    rot = np.array([[c, -s], [s, c]])
    corners_world = corners_local @ rot.T + np.array([px, py])

    patch = Polygon(
        corners_world,
        closed=True,
        facecolor=facecolor,
        edgecolor=edgecolor,
        linewidth=linewidth,
        alpha=alpha,
        label=label,
        zorder=zorder,
    )
    ax.add_patch(patch)
    return patch


def draw_fov_cone(ax, pose, alpha_fov, max_range, color=TAG_BLUE, alpha=0.25):
    """
    Draw the camera field-of-view cone in the global/tag frame.

    The camera optical axis is the robot's negative x-axis, so the cone center
    direction is theta + pi in world coordinates.
    """
    px, py, theta = pose
    center_angle = theta + np.pi
    angles = np.linspace(center_angle - alpha_fov, center_angle + alpha_fov, 40)
    arc = np.column_stack(
        [
            px + max_range * np.cos(angles),
            py + max_range * np.sin(angles),
        ]
    )
    vertices = np.vstack(([px, py], arc))

    patch = Polygon(
        vertices,
        closed=True,
        facecolor=color,
        edgecolor=color,
        linewidth=0.8,
        alpha=alpha,
        zorder=0.5,
    )
    ax.add_patch(patch)
    return patch


def _draw_scene(
    ax,
    x,
    x_goal,
    realized_traj,
    predicted_traj,
    predicted_trajs,
    iteration,
    status,
    xlim,
    ylim,
    alpha_fov,
):
    """Render one simulation frame."""
    ax.clear()

    realized = np.asarray(realized_traj)
    ax.plot(
        realized[:, 0],
        realized[:, 1],
        color="black",
        marker="o",
        markersize=3.0,
        linewidth=1.5,
    )

    for pred_step, old_pred in predicted_trajs[:-1]:
        tail_start = iteration - pred_step
        if tail_start < 1 or tail_start >= len(old_pred):
            continue

        tail = np.asarray(old_pred[tail_start:])
        ax.plot(
            tail[:, 0],
            tail[:, 1],
            linestyle="--",
            color=PRED_ORANGE,
            alpha=0.5,
            linewidth=1.2,
            zorder=1.5,
        )

    if predicted_traj is not None:
        predicted = np.asarray(predicted_traj)
        ax.plot(
            predicted[:, 0],
            predicted[:, 1],
            linestyle="-",
            color=PRED_ORANGE,
            linewidth=2.0,
            marker=".",
            markersize=3.0,
            label="MPC horizon",
        )
        ax.plot(
            predicted[-1, 0],
            predicted[-1, 1],
            marker="s",
            color=PRED_ORANGE,
            markersize=6.0,
            linestyle="None",
            zorder=2.5,
        )

    draw_fov_cone(ax, x, alpha_fov=alpha_fov, max_range=6.0)

    # The tag is drawn as a thin rectangle with its front side facing +x,
    # which makes the goal pose [0.5, 0, 0] directly in front of it.
    draw_oriented_rectangle(
        ax,
        pose=np.array([0.0, 0.0, 0.0]),
        length=0.1,
        width=0.5,
        facecolor=TAG_BLUE,
    )

    draw_oriented_rectangle(
        ax,
        pose=x,
        length=0.7,
        width=0.4,
        facecolor=ROBOT_RED,
    )

    ax.plot(
        x_goal[0],
        x_goal[1],
        marker="x",
        color="green",
        markersize=8.0,
        mew=2.0,
        linestyle="None",
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_xlabel("+x [m] faces left")
    ax.set_ylabel("+y [m] faces down")
    ax.grid(True, linewidth=0.4, alpha=0.35)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, loc="upper left")
    ax.set_title(
        "iter {:03d} | x=[{:.3f}, {:.3f}, {:.1f} deg] | status={}".format(
            iteration,
            x[0],
            x[1],
            np.rad2deg(x[2]),
            status,
        )
    )


def _capture_frame(fig, video_writer):
    """Write the current Matplotlib figure to the video, if recording."""
    if video_writer is None:
        return
    fig.canvas.draw()
    video_writer.grab_frame()


def run_simulation(
    dt=DEFAULT_DT,
    horizon=DEFAULT_HORIZON,
    max_iters=DEFAULT_MAX_ITERS,
    pos_tol=0.1,
    yaw_tol=np.deg2rad(5.0),
    live=True,
    pause_s=0.03,
    verbose=True,
    save_mp4=None,
    video_fps=DEFAULT_VIDEO_FPS,
):
    """
    Run receding-horizon visual-servo MPC until convergence or failure.

    Returns:
        history: dict containing realized states, applied controls, solver
            statuses, and the final predicted MPC horizon.
    """
    x = np.array([5.0, -0.5, np.pi / 12.0])
    x_goal = np.array([1.0, 0.0, 0.0])
    u_prev = np.zeros(3)

    p_weight = np.diag([20.0, 20.0, 10.0])
    r_weight = np.diag([0.1, 0.1, 0.05])
    s_weight = np.diag([2.0, 2.0, 1.0])

    u_min = np.array([-0.5, -0.5, -1.0])
    u_max = np.array([0.5, 0.5, 1.0])

    # Command-to-command changes per MPC cycle. Define these from physical
    # acceleration limits so changing dt preserves the same robot behavior.
    max_xy_accel = 0.75
    max_yaw_accel = 1.0
    du_max = np.array([max_xy_accel * dt, max_xy_accel * dt, max_yaw_accel * dt])
    du_min = -du_max

    alpha_fov = np.deg2rad(35.0)
    # Invert both axes for this camera-style view: the AprilTag at x=0 is on
    # the right, +x points left, and +y points down.
    xlim = (5.5, -0.5)
    ylim = (2.0, -2.0)

    realized_traj = [x.copy()]
    applied_controls = []
    statuses = []
    solve_times = []
    predicted_traj = None
    predicted_trajs = []

    initial_beta = bearing_to_tag(x)
    if abs(initial_beta) > alpha_fov:
        return {
            "states": np.asarray(realized_traj),
            "controls": np.asarray(applied_controls),
            "statuses": ["initial_fov_infeasible"],
            "solve_times": np.asarray(solve_times),
            "predicted_traj": predicted_traj,
            "predicted_trajs": predicted_trajs,
            "x_goal": x_goal.copy(),
            "alpha_fov": alpha_fov,
            "converged": False,
            "stop_reason": "initial_fov_infeasible",
        }

    fig = None
    ax = None
    video_writer = None
    save_mp4 = Path(save_mp4) if save_mp4 is not None else None
    if save_mp4 is not None and not writers.is_available("ffmpeg"):
        raise RuntimeError("Matplotlib ffmpeg writer is not available; cannot save MP4.")

    if live or save_mp4 is not None:
        if live:
            plt.ion()
        fig, ax = plt.subplots(figsize=(9, 5))
        _draw_scene(
            ax,
            x,
            x_goal,
            realized_traj,
            predicted_traj,
            predicted_trajs,
            0,
            "initial",
            xlim,
            ylim,
            alpha_fov,
        )
        fig.canvas.draw()
        if save_mp4 is not None:
            save_mp4.parent.mkdir(parents=True, exist_ok=True)
            video_writer = FFMpegWriter(
                fps=video_fps,
                metadata={"title": "AprilTag visual-servo MPC simulation"},
            )
            video_writer.setup(fig, str(save_mp4), dpi=150)
        _capture_frame(fig, video_writer)
        if live:
            plt.pause(pause_s)

    converged = False
    stop_reason = "max_iters"

    try:
        for iteration in range(1, max_iters + 1):
            # The solver linearizes the FOV constraint around the current measured
            # state when nominal_traj is left as None.
            solve_start = time.perf_counter()
            u0, x_pred, _, status = solve_visual_mpc_step(
                x0=x,
                x_goal=x_goal,
                u_prev=u_prev,
                dt=dt,
                N=horizon,
                P=p_weight,
                R=r_weight,
                S=s_weight,
                u_min=u_min,
                u_max=u_max,
                du_min=du_min,
                du_max=du_max,
                alpha_fov=alpha_fov,
                nominal_traj=None,
            )
            solve_times.append(time.perf_counter() - solve_start)
            statuses.append(status)

            if u0 is None:
                stop_reason = "solver_failed"
                applied_controls.append(np.zeros(3))
                if fig is not None:
                    _draw_scene(
                        ax,
                        x,
                        x_goal,
                        realized_traj,
                        predicted_traj,
                        predicted_trajs,
                        iteration,
                        status,
                        xlim,
                        ylim,
                        alpha_fov,
                    )
                    _capture_frame(fig, video_writer)
                    if live:
                        plt.pause(pause_s)
                break

            predicted_traj = x_pred.copy()
            predicted_trajs.append((iteration, predicted_traj))
            if verbose:
                pred_path_length = np.sum(np.linalg.norm(np.diff(predicted_traj[:, :2], axis=0), axis=1))
                print(
                    "iter={:03d} status={} horizon={} pred_points={} pred_path={:.3f} "
                    "pred_end=[{:.3f}, {:.3f}, {:.3f}]".format(
                        iteration,
                        status,
                        horizon,
                        len(predicted_traj),
                        pred_path_length,
                        predicted_traj[-1, 0],
                        predicted_traj[-1, 1],
                        predicted_traj[-1, 2],
                    )
                )

            if fig is not None:
                _draw_scene(
                    ax,
                    x,
                    x_goal,
                    realized_traj,
                    predicted_traj,
                    predicted_trajs,
                    iteration,
                    status,
                    xlim,
                    ylim,
                    alpha_fov,
                )
                _capture_frame(fig, video_writer)
                if live:
                    plt.pause(pause_s)

            # Simulation applies global/tag-frame velocity directly. Check the
            # true FOV before accepting the next state, since the optimizer uses
            # a linearized FOV constraint.
            x_next = x + dt * u0
            x_next[2] = wrap_angle(x_next[2])
            if abs(bearing_to_tag(x_next)) > alpha_fov:
                stop_reason = "next_state_fov_infeasible"
                applied_controls.append(np.zeros(3))
                if fig is not None:
                    _draw_scene(
                        ax,
                        x,
                        x_goal,
                        realized_traj,
                        predicted_traj,
                        predicted_trajs,
                        iteration,
                        stop_reason,
                        xlim,
                        ylim,
                        alpha_fov,
                    )
                    _capture_frame(fig, video_writer)
                    if live:
                        plt.pause(pause_s)
                break

            applied_controls.append(u0.copy())
            x = x_next

            # For hardware, convert the command before sending it:
            # vx_body = cos(theta) * vx_world + sin(theta) * vy_world
            # vy_body = -sin(theta) * vx_world + cos(theta) * vy_world
            # omega_body = omega_world
            u_prev = u0
            realized_traj.append(x.copy())

            pos_error = np.linalg.norm(x[:2] - x_goal[:2])
            yaw_error = abs(wrap_angle(x[2] - x_goal[2]))
            converged = pos_error < pos_tol and yaw_error < yaw_tol

            if converged:
                stop_reason = "converged"
                break

        if fig is not None:
            final_status = statuses[-1] if statuses else "initial"
            _draw_scene(
                ax,
                x,
                x_goal,
                realized_traj,
                predicted_traj,
                predicted_trajs,
                len(realized_traj) - 1,
                final_status,
                xlim,
                ylim,
                alpha_fov,
            )
            _capture_frame(fig, video_writer)
            fig.canvas.draw_idle()
    finally:
        if video_writer is not None:
            video_writer.finish()
            print("Saved MP4: {}".format(save_mp4))

    if live and fig is not None:
        plt.ioff()
        plt.show()
    elif fig is not None:
        plt.close(fig)

    return {
        "states": np.asarray(realized_traj),
        "controls": np.asarray(applied_controls),
        "statuses": statuses,
        "solve_times": np.asarray(solve_times),
        "predicted_traj": predicted_traj,
        "predicted_trajs": predicted_trajs,
        "x_goal": x_goal.copy(),
        "alpha_fov": alpha_fov,
        "converged": converged,
        "stop_reason": stop_reason,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--horizon", type=int, default=DEFAULT_HORIZON)
    parser.add_argument("--max-iters", type=int, default=DEFAULT_MAX_ITERS)
    parser.add_argument("--pause", type=float, default=0.03)
    parser.add_argument(
        "--output-mp4",
        type=Path,
        default=SCRIPT_DIR / "mpc_sim.mp4",
        help="Path for the saved MP4 simulation video.",
    )
    parser.add_argument("--fps", type=int, default=DEFAULT_VIDEO_FPS)
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="Do not save an MP4 video.",
    )
    parser.add_argument(
        "--no-live",
        action="store_true",
        help="Run the simulation without opening a Matplotlib animation window.",
    )
    args = parser.parse_args()

    history = run_simulation(
        dt=args.dt,
        horizon=args.horizon,
        max_iters=args.max_iters,
        live=not args.no_live,
        pause_s=args.pause,
        save_mp4=None if args.no_video else args.output_mp4,
        video_fps=args.fps,
    )

    final_state = history["states"][-1]
    print(
        "stop_reason={} converged={} final_state=[{:.4f}, {:.4f}, {:.4f}]".format(
            history["stop_reason"],
            history["converged"],
            final_state[0],
            final_state[1],
            final_state[2],
        )
    )


if __name__ == "__main__":
    main()
