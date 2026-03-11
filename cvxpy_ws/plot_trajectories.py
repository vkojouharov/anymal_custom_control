"""
Run trajectory optimisation for several task-region radii and save plots.

Flags:
  --masked   strip axes/labels/title/grid and use transparent background;
             appends _mask to each output filename.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from task import make_task, plot_task_smooth
from traj_opt import make_solver

parser = argparse.ArgumentParser()
parser.add_argument("--masked", action="store_true",
                    help="Save clean masked plots with transparent background")
args = parser.parse_args()

PLOTS_DIR = os.path.join(os.path.dirname(__file__), "plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

radii = [0.1, 0.5, 1, 2, 3, 4, 5]

fast_solve = make_solver()

for r in radii:
    t = make_task(r)
    traj = fast_solve(t)

    stem = f"task_3x_r{r}"
    if args.masked:
        stem += "_mask"
    save_path = os.path.join(PLOTS_DIR, f"{stem}.png")
    plot_task_smooth(t, traj, save_path=save_path, masked=args.masked)
    print(f"r={r}: saved {save_path}")
    print(f"  x1={traj['x1']}")
    print(f"  x2={traj['x2']}")
    print(f"  x3={traj['x3']}")
