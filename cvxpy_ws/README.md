# `cvxpy_ws`

Active trajectory-optimization workspace for the CVXPY/DCCP/SCP experiments.

## Main Files
- [RUN.py](/Users/stanleywang/Documents/GitHub/anymal_custom_control/cvxpy_ws/RUN.py): loads a task, runs the active solver, prints timing, and visualizes the result
- [solver.py](/Users/stanleywang/Documents/GitHub/anymal_custom_control/cvxpy_ws/trajectory_opt/solver.py): trajectory solvers and helper logic
- [visualizer.py](/Users/stanleywang/Documents/GitHub/anymal_custom_control/cvxpy_ws/trajectory_opt/visualizer.py): task and trajectory plotting
- [TASK_demo.py](/Users/stanleywang/Documents/GitHub/anymal_custom_control/cvxpy_ws/tasks/TASK_demo.py): example task definition

## Folders
- `tasks/`: task definitions
- `trajectory_opt/`: active solver and plotting code
- `archive/`: old code snapshots and legacy outputs

## Task Format
Tasks are defined as:

```python
TASK = {
    "start": np.array([...]),
    "targets": [{"center": ..., "radius": ...}, ...],
    "obstacles": [{"center": ..., "radius": ...}, ...],
}
```

## Where Things Happen
- Tasks are defined in `tasks/`
- Waypoint trajectories are computed in `trajectory_opt/solver.py`
- Plotting is handled in `trajectory_opt/visualizer.py`

The returned trajectory is an ordered set of 2D waypoints and may include intermediate points and post-solve pruning.

## Solver Functions
- `solve_trajectory(...)`: DCCP-based solver
- `solve_trajectory_SCP(...)`: SCP-based solver

Current solver options include:
- adjustable `intermediate_points`
- initialization modes like `interp_line` and `offset_interp`
- optional waypoint pruning

## Notes
- The active code uses the new list-based task format, not the old numbered-key format.
- Timing is split into construction time and solve time.
- The smoothed plotted curve is only for visualization and is not guaranteed to remain obstacle-free.
