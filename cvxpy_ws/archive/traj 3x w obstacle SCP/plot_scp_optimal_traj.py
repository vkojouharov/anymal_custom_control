"""
Load the SCP iteration history CSV and plot the optimal (last row) trajectory
using plot_task_smooth from task_w_obstacle.

Usage:
    python cvxpy_ws/plot_scp_result.py                        # saves to plots/
    python cvxpy_ws/plot_scp_result.py --csv path/to/scp.csv  # custom CSV
    python cvxpy_ws/plot_scp_result.py --show                  # display instead of save
"""
import argparse
import csv
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from task_w_obstacle import task, plot_task_smooth


def load_last_row(csv_path):
    """Return the last row of the CSV as a dict with numpy arrays for waypoints."""
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"CSV is empty: {csv_path}")
    row = rows[-1]
    return {
        "iteration":    int(row["iteration"]),
        "cost":         float(row["cost"]),
        "x1":           np.array([float(row["x1_0"]), float(row["x1_1"])]),
        "x2":           np.array([float(row["x2_0"]), float(row["x2_1"])]),
        "x3":           np.array([float(row["x3_0"]), float(row["x3_1"])]),
        "max_step":     float(row["max_step"]),
        "status":       row["status"],
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv",  default=None, help="Path to scp_history.csv")
    parser.add_argument("--show", action="store_true", help="Display plot instead of saving")
    args = parser.parse_args()

    base = os.path.dirname(__file__)
    csv_path = args.csv or os.path.join(base, "data", "scp_history.csv")

    result = load_last_row(csv_path)

    print(f"Plotting optimal solution from iteration {result['iteration']}:")
    print(f"  cost     = {result['cost']:.6f}")
    print(f"  max_step = {result['max_step']:.6f}")
    print(f"  status   = {result['status']}")
    print(f"  x1 = {result['x1']}")
    print(f"  x2 = {result['x2']}")
    print(f"  x3 = {result['x3']}")

    traj = {
        "x0": task["x0"],
        "x1": result["x1"],
        "x2": result["x2"],
        "x3": result["x3"],
    }

    if args.show:
        plot_task_smooth(task, traj)
    else:
        plots_dir = os.path.join(base, "plots")
        os.makedirs(plots_dir, exist_ok=True)
        save_path = os.path.join(plots_dir, "scp_obstacle_traj.png")
        plot_task_smooth(task, traj, save_path=save_path)
        print(f"\nPlot saved to {save_path}")
