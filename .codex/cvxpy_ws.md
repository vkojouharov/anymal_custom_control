# `cvxpy_ws` Notes

## Current Layout
- `cvxpy_ws/tasks/TASK_demo.py`: example task in the new list-based format
- `cvxpy_ws/trajectory_opt/solver.py`: active trajectory solvers and helper utilities
- `cvxpy_ws/trajectory_opt/visualizer.py`: plotting utilities for task regions, obstacles, polylines, and smoothed trajectories
- `cvxpy_ws/RUN.py`: current entrypoint for running the active solver and plotting the result
- `cvxpy_ws/archive/`: old no-obstacle and obstacle-SCP code snapshots kept for reference only

## Active Task Format
Tasks are now defined as a Python dict with explicit lists instead of numbered keys:

```python
TASK = {
    "start": np.array([...], dtype=float),
    "targets": [
        {"center": np.array([...], dtype=float), "radius": ...},
        ...
    ],
    "obstacles": [
        {"center": np.array([...], dtype=float), "radius": ...},
        ...
    ],
}
```

This replaced the old `x0`, `c1`, `r1`, `c2`, `r2`, ... structure.

## Solver Status
`solver.py` currently contains two solver paths:

### `solve_trajectory(...)`
- DCCP-based nonconvex solver
- supports `intermediate_points`
- supports `pruning_threshold`
- supports `init_mode`
- returns construction time and solve time separately

### `solve_trajectory_SCP(...)`
- newer generalized SCP solver based on the archived SCP implementation
- default initializer is `init_mode="offset_interp"`
- uses a parameterized convex subproblem and updates the linearization each iteration
- applies pruning after convergence
- returns:
  - pruned trajectory
  - construction time
  - total solve time across all SCP iterations
  - kept waypoint indices after pruning
  - per-iteration history

## Initialization Modes
`solver.py` currently supports:

### `interp_line`
- straight interpolation between the previous anchor and the current target center

### `offset_interp`
- starts from the same interpolated points
- checks whether obstacles intersect the current segment
- tries shifting intermediate points along the segment normal in `0.1 m` increments
- checks both normal directions and keeps the first side that clears the obstructing obstacles
- this is a heuristic initializer, not a proof of strict feasibility

## Pruning
Post-solve pruning removes approximately collinear intermediate waypoints:
- controlled by `pruning_threshold`
- preserves the target-anchor waypoints
- currently geometric only; it does not re-check obstacle clearance after pruning

## Visualization
`visualizer.py` supports:
- task plotting with the new task format
- optional trajectory overlays
- smoothed trajectory plotting using piecewise cubic Hermite interpolation
- smoothed trajectory is currently colored red to match the legacy style

Important caveat:
- the smoothed curve is for visualization only and is not guaranteed to remain obstacle-free

## Timing
Recent changes split timing into:
- construction time: variable setup, constraints, initialization, and problem construction
- solve time: optimizer runtime only

`RUN.py` currently prints both for the SCP path.

## Known Caveats
- `.DS_Store`, `__pycache__`, and `dccp.log` are present in the workspace and are not part of the solver logic
- archived code under `cvxpy_ws/archive/` is historical reference, not active code
- `visualizer.py` can be run directly, but GUI behavior depends on the local environment
- the plotter accepts dense waypoint trajectories, including the added intermediate points

## Suggested Next Cleanup
- remove or ignore generated/macOS files (`.DS_Store`, `__pycache__`, logs)
- decide whether to keep both DCCP and SCP solvers long-term or treat one as experimental
- add obstacle-clearance re-check after pruning if the pruned trajectory will be used as an actual motion path
