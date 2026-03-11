"""
Animate SCP iterations as a GIF.

Each frame shows the trajectory at that SCP iteration via plot_task_smooth,
held for 0.5 s. A small annotation (iteration / cost / step) is overlaid on
each frame using PIL.

Usage:
    python cvxpy_ws/scp_obstacle_traj_animate.py
    python cvxpy_ws/scp_obstacle_traj_animate.py --csv path/to/scp_history.csv
    python cvxpy_ws/scp_obstacle_traj_animate.py --out path/to/output.gif
"""
import argparse
import csv
import os
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, os.path.dirname(__file__))
from task_w_obstacle import task, plot_task_smooth


def load_history(csv_path):
    """Load all rows from the SCP history CSV as a list of dicts."""
    with open(csv_path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"CSV is empty: {csv_path}")
    history = []
    for row in rows:
        history.append({
            "iteration": int(row["iteration"]),
            "cost":      float(row["cost"]),
            "x1":        np.array([float(row["x1_0"]), float(row["x1_1"])]),
            "x2":        np.array([float(row["x2_0"]), float(row["x2_1"])]),
            "x3":        np.array([float(row["x3_0"]), float(row["x3_1"])]),
            "max_step":  float(row["max_step"]),
            "status":    row["status"],
        })
    return history


def _annotate(img, text, margin=8):
    """Overlay a single line of text in the bottom-left corner of a PIL Image."""
    draw = ImageDraw.Draw(img)
    # Thin white halo for readability, then dark text on top
    for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
        draw.text((margin + dx, img.height - 20 + dy), text, fill=(255, 255, 255))
    draw.text((margin, img.height - 20), text, fill=(30, 30, 30))
    return img


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", default=None, help="Path to scp_history.csv")
    parser.add_argument("--out", default=None, help="Path to output GIF")
    args = parser.parse_args()

    base     = os.path.dirname(__file__)
    csv_path = args.csv or os.path.join(base, "data", "scp_history.csv")
    out_path = args.out or os.path.join(base, "plots", "scp_animation.gif")

    history = load_history(csv_path)
    print(f"Loaded {len(history)} iterations from {csv_path}")

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    frames = []
    with tempfile.TemporaryDirectory() as tmpdir:
        for entry in history:
            traj = {
                "x0": task["x0"],
                "x1": entry["x1"],
                "x2": entry["x2"],
                "x3": entry["x3"],
            }
            frame_path = os.path.join(tmpdir, f"frame_{entry['iteration']:04d}.png")
            plot_task_smooth(task, traj, save_path=frame_path)

            img = Image.open(frame_path).convert("RGB")
            label = (f"iter {entry['iteration']}   "
                     f"cost={entry['cost']:.4f}   "
                     f"step={entry['max_step']:.4f}   "
                     f"[{entry['status']}]")
            _annotate(img, label)
            frames.append(img)

            print(f"  iter {entry['iteration']:3d}  cost={entry['cost']:.4f}  "
                  f"max_step={entry['max_step']:.6f}")

    # duration is in milliseconds; loop=0 means loop forever
    frames[0].save(
        out_path,
        save_all=True,
        append_images=frames[1:],
        duration=333,
        loop=0,
        optimize=False,
    )
    print(f"\nGIF saved to {out_path}  ({len(frames)} frames, 0.5 s each)")
