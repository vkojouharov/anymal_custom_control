#!/usr/bin/env python3
"""
Headless horizon sweep for the visual-servo MPC simulation.

Runs closed-loop MPC episodes at dt=0.2 for horizons 5, 10, ..., 50,
records episode-level metrics, and saves a subplot summary.
"""

import argparse
import csv
import os
import tempfile
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")
os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", os.path.join(tempfile.gettempdir(), "xdg-cache"))

import matplotlib.pyplot as plt
import numpy as np

if __package__:
    from .run_mpc_sim import run_simulation, wrap_angle
    from .solve_mpc import bearing_to_tag
else:
    from run_mpc_sim import run_simulation, wrap_angle
    from solve_mpc import bearing_to_tag


SCRIPT_DIR = Path(__file__).resolve().parent


def compute_episode_metrics(history, dt, horizon):
    """Convert one closed-loop simulation history into scalar metrics."""
    states = np.asarray(history["states"])
    controls = np.asarray(history["controls"])
    solve_times = np.asarray(history["solve_times"])
    x_goal = np.asarray(history["x_goal"])
    alpha_fov = float(history["alpha_fov"])

    num_applied_steps = max(len(states) - 1, 0)
    executed_controls = controls[:num_applied_steps]

    if len(states) > 1:
        step_distances = np.linalg.norm(np.diff(states[:, :2], axis=0), axis=1)
        total_distance = float(np.sum(step_distances))
    else:
        total_distance = 0.0

    fov_margins = np.array([alpha_fov - abs(bearing_to_tag(state)) for state in states])
    initial_fov_margin_deg = float(np.rad2deg(fov_margins[0]))
    final_state = states[-1]

    if len(executed_controls) > 0:
        max_xy_speed = float(np.max(np.linalg.norm(executed_controls[:, :2], axis=1)))
        max_yaw_rate = float(np.max(np.abs(executed_controls[:, 2])))
        control_effort = float(dt * np.sum(np.sum(executed_controls**2, axis=1)))
    else:
        max_xy_speed = 0.0
        max_yaw_rate = 0.0
        control_effort = 0.0

    if len(executed_controls) > 1:
        accel_like = np.diff(executed_controls, axis=0) / dt
        smoothness = float(dt * np.sum(np.sum(accel_like**2, axis=1)))
    else:
        smoothness = 0.0

    if len(solve_times) > 0:
        mean_solve_ms = float(1000.0 * np.mean(solve_times))
        max_solve_ms = float(1000.0 * np.max(solve_times))
        total_solve_s = float(np.sum(solve_times))
    else:
        mean_solve_ms = np.nan
        max_solve_ms = np.nan
        total_solve_s = 0.0

    return {
        "horizon": int(horizon),
        "lookahead_s": float(horizon * dt),
        "converged": bool(history["converged"]),
        "stop_reason": history["stop_reason"],
        "num_steps": int(num_applied_steps),
        "total_time_s": float(num_applied_steps * dt),
        "total_distance_m": total_distance,
        "initial_fov_margin_deg": initial_fov_margin_deg,
        "min_fov_margin_deg": float(np.rad2deg(np.min(fov_margins))),
        "final_fov_margin_deg": float(np.rad2deg(fov_margins[-1])),
        "final_pos_error_m": float(np.linalg.norm(final_state[:2] - x_goal[:2])),
        "final_yaw_error_deg": float(np.rad2deg(abs(wrap_angle(final_state[2] - x_goal[2])))),
        "mean_solve_ms": mean_solve_ms,
        "max_solve_ms": max_solve_ms,
        "total_solve_s": total_solve_s,
        "control_effort": control_effort,
        "smoothness": smoothness,
        "max_xy_speed_mps": max_xy_speed,
        "max_yaw_rate_radps": max_yaw_rate,
    }


def write_csv(rows, output_csv):
    """Write sweep metrics to CSV."""
    fieldnames = list(rows[0].keys())
    with output_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot_metrics(rows, output_png):
    """Create the horizon-sweep subplot figure."""
    horizons = np.array([row["horizon"] for row in rows])
    converged = np.array([row["converged"] for row in rows])

    series = [
        ("total_time_s", "Total time [s]", "tab:blue"),
        ("total_distance_m", "Total distance [m]", "tab:orange"),
        ("min_fov_margin_deg", "Worst FOV margin [deg]", "tab:green"),
        ("mean_solve_ms", "Mean solve time [ms]", "tab:red"),
        ("final_pos_error_m", "Final position error [m]", "tab:purple"),
        ("smoothness", "Command smoothness", "tab:brown"),
    ]

    with plt.rc_context(
        {
            "font.size": 12.5,
            "axes.titlesize": 15,
            "axes.labelsize": 12.5,
            "xtick.labelsize": 11,
            "ytick.labelsize": 11,
            "legend.fontsize": 11,
            "figure.titlesize": 17,
        }
    ):
        fig, axes = plt.subplots(2, 3, figsize=(17, 9), sharex=True)
        axes = axes.ravel()

        for ax, (key, ylabel, color) in zip(axes, series):
            values = np.array([row[key] for row in rows], dtype=float)
            ax.plot(horizons, values, color=color, marker="o", linewidth=2.0)
            if key == "min_fov_margin_deg":
                initial_margin = rows[0]["initial_fov_margin_deg"]
                ax.axhline(
                    initial_margin,
                    color="0.35",
                    linestyle="--",
                    linewidth=1.5,
                    label="initial pose margin",
                )
            if not np.all(converged):
                ax.scatter(
                    horizons[~converged],
                    values[~converged],
                    color="black",
                    marker="x",
                    s=80,
                    zorder=3,
                    label="not converged",
                )
            if key == "min_fov_margin_deg" or not np.all(converged):
                ax.legend(loc="best")
            ax.set_ylabel(ylabel)
            ax.grid(True, linewidth=0.5, alpha=0.4)

        for ax in axes[-3:]:
            ax.set_xlabel("MPC horizon steps")

        fig.suptitle("MPC Horizon Sweep, dt = 0.2 s")
        fig.tight_layout()
        fig.savefig(output_png, dpi=180)
        plt.close(fig)


def run_sweep(dt=0.2, horizons=range(5, 51, 5), max_iters=1000):
    """Run all horizon settings and return metric rows."""
    rows = []
    for horizon in horizons:
        print("Running dt={:.3f}, horizon={}...".format(dt, horizon))
        history = run_simulation(
            dt=dt,
            horizon=horizon,
            max_iters=max_iters,
            live=False,
            verbose=False,
        )
        metrics = compute_episode_metrics(history, dt, horizon)
        rows.append(metrics)
        print(
            "  stop_reason={} converged={} time={:.2f}s dist={:.2f}m "
            "min_fov_margin={:.2f}deg mean_solve={:.1f}ms".format(
                metrics["stop_reason"],
                metrics["converged"],
                metrics["total_time_s"],
                metrics["total_distance_m"],
                metrics["min_fov_margin_deg"],
                metrics["mean_solve_ms"],
            )
        )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=0.2)
    parser.add_argument("--min-horizon", type=int, default=5)
    parser.add_argument("--max-horizon", type=int, default=50)
    parser.add_argument("--step", type=int, default=5)
    parser.add_argument("--max-iters", type=int, default=1000)
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=SCRIPT_DIR / "horizon_sweep_dt_0p2.csv",
    )
    parser.add_argument(
        "--output-png",
        type=Path,
        default=SCRIPT_DIR / "horizon_sweep_dt_0p2.png",
    )
    args = parser.parse_args()

    horizons = range(args.min_horizon, args.max_horizon + 1, args.step)
    rows = run_sweep(dt=args.dt, horizons=horizons, max_iters=args.max_iters)

    write_csv(rows, args.output_csv)
    plot_metrics(rows, args.output_png)

    print("Wrote CSV: {}".format(args.output_csv))
    print("Wrote plot: {}".format(args.output_png))


if __name__ == "__main__":
    main()
