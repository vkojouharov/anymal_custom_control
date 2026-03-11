"""
Sweep task-region radius r over linspace(0.01, 1, 100).
For each r:
  - solve the optimal trajectory and record the minimum polyline distance
  - time solve() over N_TRIALS repetitions and record the avg wall-clock time

Saves a two-panel figure: optimal distance vs r  /  avg solve time vs r.
"""
import os
import sys
import time

import numpy as np
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))

from task import make_task
from traj_opt import make_solver

# ── Experiment parameters ──────────────────────────────────────────────────────
RADII    = np.linspace(0.01, 5.0, 100)
N_TRIALS = 100
PLOTS_DIR = os.path.join(os.path.dirname(__file__), "plots")
os.makedirs(PLOTS_DIR, exist_ok=True)

# Build the compiled solver once (problem graph constructed here)
print("Compiling CVXPY problem...")
fast_solve = make_solver()
print("Done.")

# ── Sweep ──────────────────────────────────────────────────────────────────────
opt_distances = []
avg_times     = []

for i, r in enumerate(RADII):
    t = make_task(r)

    # Solve once to get the optimal waypoints
    traj = fast_solve(t)
    dist = traj["cost"]
    opt_distances.append(dist)

    # Time N_TRIALS solves (pure re-solve, no reconstruction)
    t0 = time.perf_counter()
    for _ in range(N_TRIALS):
        fast_solve(t)
    elapsed = time.perf_counter() - t0
    avg_times.append(elapsed / N_TRIALS)

    if (i + 1) % 10 == 0:
        print(f"  {i+1}/{len(RADII)}  r={r:.3f}  dist={dist:.4f}  "
              f"avg_time={avg_times[-1]*1e3:.3f} ms")

opt_distances = np.array(opt_distances)
avg_times     = np.array(avg_times)

# ── Plot ───────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(8, 5))

ax.plot(RADII, opt_distances, color="steelblue", linewidth=2)
ax.set_xlabel("radius r", fontsize=15)
ax.set_ylabel("optimal polyline distance", fontsize=15)
ax.set_title("trajectory distance vs task-region radii", fontsize=16.25)
ax.tick_params(axis="both", labelsize=13.75)
ax.grid(True, linestyle="--", alpha=0.4)

fig.tight_layout()
save_path = os.path.join(PLOTS_DIR, "r_sweep.png")
plt.savefig(save_path, dpi=300, bbox_inches="tight")
plt.close(fig)
print(f"\nSaved {save_path}")
