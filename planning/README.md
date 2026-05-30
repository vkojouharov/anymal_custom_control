# `planning`

CVXPY/DCCP/SCP workspace for 2D task-region trajectory optimization and ANYmal waypoint export.

## Quick Start

Run the active obstacle comparison:

```bash
python planning/RUN.py
```

Run the no-obstacle radius comparison:

```bash
python planning/RUN_demo.py
```

The root runner files are compatibility wrappers. The implementation lives in `experiments/`.

## Main Entry Points

- `experiments/run_obstacle_comparison.py`: solves and plots the current obstacle-aware comparison.
- `experiments/run_radius_comparison.py`: solves and plots the source-radius comparison without obstacles.
- `trajectory_opt/solver.py`: core DCCP, force-aware, and SCP solvers.
- `trajectory_opt/anymal_export.py`: converts optimized 2D points to ANYmal base and arm waypoint modules.
- `trajectory_opt/plotting.py`: single-task plotting helpers.
- `trajectory_opt/comparison_plotting.py`: side-by-side comparison plotting helpers.

## Task Files

Task definitions live in `tasks/`. Each task module exposes one `TASK` dictionary:

```python
TASK = {
    "start": np.array([...]),
    "targets": [{"center": ..., "radius": ...}, ...],
    "obstacles": [{"center": ..., "radius": ...}, ...],
}
```

Current task modules:

- `source_radius_0p5.py`
- `source_radius_1p75.py`
- `source_obstacle_radius_0p5.py`
- `source_obstacle_radius_1p75.py`
- `demo_obstacle_task.py`
- `demo_force_task.py`

Historical task aliases are kept in `tasks/__init__.py` so older runner imports remain easier to map.

## Outputs

`experiments/*` export generated trajectory modules to:

```text
../catkin_ws/src/anymal_custom_control/scripts/trajectories/
```

Plots and scratch data can go in `plots/` and `data/`. Study-specific figures belong next to their study under `studies/*/outputs/`.

## More Detail

See `STRUCTURE.md` for the full folder layout and guidance on where new code should go.
