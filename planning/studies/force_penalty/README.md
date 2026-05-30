# Force Penalty Study

Exploratory scripts and outputs for force-alignment penalty models.

## Files

- `penalty_model.py`: wedge-constrained convex penalty visualization.
- `penalty_model.md`: notes on the wedge penalty model.
- `visualize_force_penalty.py`: visualizes the sampled `sin(theta)` force-alignment penalty and its minimum-centered approximation.
- `visualize_force_accuracy_penalty.py`: compares alignment, distance, and mixed costs.
- `outputs/`: generated PNG figures from the visualization scripts.

These scripts are study utilities, not the active trajectory run path. The active solver-side force logic is in `trajectory_opt/solver.py`.
