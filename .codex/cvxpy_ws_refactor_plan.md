# `cvxpy_ws` Refactor Plan

This file is an implementation brief for a later execution pass. It is not a record of completed work. The repository should be treated as still using the current pre-refactor layout unless a future change explicitly implements this plan.

## Intent

Refactor `cvxpy_ws` into a cleaner trajectory-optimization package with one obvious active path:

`task definition -> adapter -> DCCP solve -> plot/export`

The package should stop presenting experimental solver paths as co-equal with the production path. The active package should be boring, explicit, and easy to trace. The goal is cleanup of structure and interfaces, not a redesign of the underlying optimization problem.

## Locked Decisions

These decisions are already made and should not be revisited during implementation unless there is a concrete technical blocker:

- DCCP is the only active supported solver path.
- Default DCCP configuration keeps `tau=10`.
- SCP is not the default, not public, and not part of the active package API.
- SCP code should be retained only as experimental / research code.
- Force-penalty modeling and force-penalty plotting should be retained only as experimental / research code.
- ANYmal export remains active and supported.
- ANYmal base XY output keeps the existing sign-flipped convention.
- Arm waypoint output keeps the existing target-relative XY convention with fixed `z=0.5`.
- Intermediate-point export behavior is not part of this refactor and must fail loudly instead of guessing.

## Current Problems

The current `cvxpy_ws` structure has several issues:

- `trajectory_opt/solver.py` mixes DCCP, SCP, force-penalty helpers, initialization logic, pruning logic, and solver-specific utilities in one file.
- `trajectory_opt/visualizer.py` includes both active task plotting and force-related visualization, while `visualizer_2x.py` duplicates some active plotting logic.
- `RUN.py` and `RUN_demo.py` are acting as both runnable entrypoints and informal APIs.
- Active code and experimental code are not clearly separated.
- The package has no small stable internal data model for tasks, solver configuration, and solve results.
- The active package surface is harder to read than it should be because research code paths still live next to the production path.

## Refactor Goal

After the refactor:

- there is one active solver module for DCCP
- there is one active plotting module
- there is one active export module
- there is a small explicit task / config / result model
- experimental solver and force code is not imported from the active path
- scripts are orchestration-only
- the active package can be understood without reading experimental code

## Target Active Package Layout

Implement the active package around this structure:

```text
cvxpy_ws/
  trajectory_opt/
    __init__.py
    models.py
    io.py
    plotting.py
    export.py
    solver.py              # thin compatibility shim only
    visualizer.py          # thin compatibility shim only
    visualizer_2x.py       # thin compatibility shim only
    solvers/
      __init__.py
      dccp.py
  experimental/
    __init__.py
    scp/
      __init__.py
      solver.py
    force_penalty/
      __init__.py
      penalty_model.py
      visualize_force_penalty.py
      visualize_force_accuracy_penalty.py
      plotting.py          # if needed for active visualizer extraction leftovers
```

Notes:

- `trajectory_opt/solver.py` should remain only as a compatibility layer for old imports.
- `trajectory_opt/visualizer.py` and `trajectory_opt/visualizer_2x.py` should remain only as compatibility wrappers, not as the canonical home of the plotting logic.
- The active plotting implementation should live in `trajectory_opt/plotting.py`.
- The active solver implementation should live in `trajectory_opt/solvers/dccp.py`.

## Required Internal Models

Add explicit internal models in `trajectory_opt/models.py`. Use dataclasses unless there is a strong reason not to.

Recommended definitions:

- `Target(center: np.ndarray, radius: float, force: np.ndarray | None = None)`
- `Obstacle(center: np.ndarray, radius: float)`
- `TaskSpec(start: np.ndarray, targets: tuple[Target, ...], obstacles: tuple[Obstacle, ...] = ())`
- `SolverConfig(intermediate_points: int = 0, init_mode: str = "interp_line", pruning_threshold: float = 0.1, tau: float = 10.0, solver = cp.SCS, segment_length_factor: float = 0.7)`
- `SolveResult(trajectory: np.ndarray, objective_value: float | None, status: str, construction_time_s: float, solve_time_s: float, kept_indices: list[int], metadata: dict[str, object])`

Model rules:

- Internal active code should consume `TaskSpec`, not raw task dicts.
- The current task modules in `tasks/` do not need to be rewritten immediately.
- The adapter layer should make existing dict-based task modules usable during the transition.
- `SolveResult.metadata` can hold temporary extras like the raw DCCP result value while the API stabilizes.

## Task Adapter Layer

Create `trajectory_opt/io.py` to bridge current task dicts into the new model layer.

Required adapter functions:

- `task_from_dict(task_dict) -> TaskSpec`
- `task_to_dict(task: TaskSpec) -> dict`
- optional: `coerce_task(task_or_dict) -> TaskSpec`

Behavior:

- centers should be converted to `np.ndarray`
- start should be converted to `np.ndarray`
- missing obstacle list should normalize to empty tuple/list
- optional target `"force"` should be preserved if present

The active code path should call this adapter at the outer boundary so downstream logic works against one normalized shape.

## Active Solver API

The canonical active solver API should be:

```python
task = task_from_dict(TASK)
config = SolverConfig(intermediate_points=0, init_mode="straight_line", tau=10)
result = solve_dccp(task, config)
```

Implement `solve_dccp(task: TaskSpec, config: SolverConfig) -> SolveResult` in `trajectory_opt/solvers/dccp.py`.

The DCCP solver module should own:

- target-center helpers
- obstacle distance helpers
- init-point generation
- obstacle-offset interpolation logic
- straight-line initialization logic
- pruning logic
- DCCP problem build + solve

The DCCP module should not own:

- SCP setup / SCP iterations
- force-penalty sampling
- force-linearization logic
- plotting logic
- ANYmal export formatting

## DCCP Solver Behavior To Preserve

These current behaviors should be preserved unless a bug is found during migration:

- final waypoint in each target block must lie within the target region
- obstacle avoidance is still enforced via DCCP waypoint distance constraints
- the segment-length constraint is only enabled when there are obstacles or when `intermediate_points > 0`
- supported init modes remain:
  - `interp_line`
  - `offset_interp`
  - `straight_line`
- `tau=10` remains the default
- trajectory pruning remains active after solve

Return shape:

- the stable API returns a `SolveResult`
- temporary compatibility wrappers may continue returning the legacy tuple until callers are migrated

## Compatibility Layer

Keep old import paths working temporarily.

### `trajectory_opt/solver.py`

This file should become a thin wrapper that:

- imports the new active DCCP API
- re-exports `solve_trajectory(...)` as a compatibility wrapper over `solve_dccp(...)`
- emits a deprecation warning for old entrypoints

Compatibility behavior:

- `solve_trajectory(...)` should keep its current argument shape for existing callers
- internally it should build `SolverConfig`, call `solve_dccp`, and unwrap the `SolveResult` into the legacy tuple
- do not keep SCP as an active compatibility entrypoint here
- do not keep force-aware DCCP as an active compatibility entrypoint here

If needed, SCP compatibility should move under `experimental/`

### `trajectory_opt/visualizer.py`

This file should become a thin wrapper that re-exports active plotting functions from `trajectory_opt/plotting.py`. Do not keep force-specific plotting in the active wrapper.

### `trajectory_opt/visualizer_2x.py`

This file should become a thin wrapper over the shared plotting implementation in `trajectory_opt/plotting.py`. Keep the public function name if existing scripts rely on it, but the actual logic should live in one place.

## Plotting Refactor

Create `trajectory_opt/plotting.py` as the single active plotting implementation.

Required active functions:

- `plot_task(task, optimal_traj=None, save_path=None)`
- `plot_task_smooth(task, optimal_traj, save_path=None, masked=False)`
- `plot_tasks_smooth_2x(left_task, left_traj, right_task, right_traj, left_title="Left task", right_title="Right task", save_path=None, masked=False, legend_fontscale=1.0, legend_ncol=2)`

Plotting requirements:

- remove duplicated task/trajectory drawing logic between single-view and 2-panel plots
- share helper functions for:
  - extracting trajectory points
  - collecting task / obstacle / trajectory bounds
  - drawing start / targets / obstacles
  - adding smoothed trajectory overlays
  - shared-axis limit computation
- preserve current Hermite-style smoothed display path
- preserve current note that smoothed curves are visualization-only and not guaranteed obstacle-safe
- preserve current shared-axis comparison behavior for the left/right 2-panel plot
- preserve current legend compactness settings for the comparison plot unless a better shared helper emerges

Move out of active plotting:

- force-penalty region plots
- force-linearized plots
- force-colored smooth plots

Those should live under `experimental/force_penalty/`

## ANYmal Export Refactor

Move the canonical ANYmal export logic into `trajectory_opt/export.py`.

Required functions:

- `trajectory_to_anymal_waypoints(task, trajectory_or_result, ee_height=0.5)`
- `write_anymal_trajectory_module(output_path, anymal_waypoints, arm_waypoints, source_name=None)`
- `export_anymal_module(task, result_or_trajectory, output_path, ee_height=0.5)`
- `export_task_trajectory_module(task_module_name, task, trajectory_or_result, output_dir, ee_height=0.5)` if still helpful for `RUN_demo.py`

Required behavior:

- ANYmal waypoint remains `(-x, -y, 0.0)`
- arm waypoint remains `(target_center_x - x, target_center_y - y, ee_height)`
- default `ee_height` remains `0.5`
- output file remains a Python module with `ANYMAL_WAYPOINTS` and `ARM_WAYPOINTS`

Required guard:

- if solved trajectory length does not match the number of targets, raise a clear error indicating that intermediate-point export behavior has not yet been implemented
- do not silently invent arm behavior for intermediate points during this refactor

## Run Script Refactor

The runnable scripts should become orchestration-only.

### `RUN.py`

Keep as a local comparison / visualization entrypoint.

It should:

- import task modules
- call the active DCCP path
- call the active shared plotting path
- print concise timing / status if useful

It should not:

- contain solver math
- contain plotting internals
- import SCP
- import force-penalty code

### `RUN_demo.py`

Keep as the export-focused demo entrypoint.

It should:

- import task modules
- call the active DCCP path with `intermediate_points=0`
- export resulting trajectories into `catkin_ws/src/anymal_custom_control/scripts/trajectories`
- name the output files after the source task module names
- optionally visualize the solved trajectories using the active 2-panel plot

It should not:

- contain export formatting logic
- contain plotting internals
- contain direct ANYmal coordinate-conversion math

## Experimental Code Placement

### SCP

Move SCP logic out of the active solver module into `experimental/scp/solver.py`.

This includes:

- SCP problem construction helpers
- linearized obstacle half-space helpers
- SCP iteration loop
- SCP history reporting

The SCP code can preserve its current interface if useful, but it must not be imported by the active package surface.

### Force-Penalty

Move force-related active code out of the active package surface into `experimental/force_penalty/`.

This includes:

- force-penalty sampling helpers
- minimum-centered surrogate fitting logic
- force-aware solver variant
- force visualization utilities
- existing `force_penalty/*.py` text-code assets

Generated PNG assets can remain where they are if moving them is not worth the churn, but the active code path must not depend on them.

## Migration Order

Execute in this order to minimize breakage:

1. Add `models.py`
2. Add `io.py`
3. Extract DCCP implementation into `solvers/dccp.py`
4. Extract active export implementation into `export.py`
5. Extract active plotting implementation into `plotting.py`
6. Turn `solver.py` into a compatibility wrapper
7. Turn `visualizer.py` and `visualizer_2x.py` into compatibility wrappers
8. Update `RUN.py` and `RUN_demo.py` to use the new active APIs
9. Move SCP code into `experimental/scp/`
10. Move force-penalty code into `experimental/force_penalty/`
11. Update `README.md` to describe the new active package shape

## Acceptance Criteria

The refactor is complete only when all of the following are true:

- the active import path does not import SCP code
- the active import path does not import force-penalty code
- DCCP solve still works for current no-obstacle demo tasks
- DCCP solve still works for current obstacle demo tasks
- `RUN_demo.py` still exports valid ANYmal trajectory files into the catkin trajectory folder
- `RUN.py` still produces the left/right comparison plot
- current task modules in `tasks/` still run via the adapter layer
- the old active file names remain only as compatibility shims or are removed intentionally after all callers are migrated

## Required Tests

Add or preserve these checks during implementation:

- task adapter test from dict task to `TaskSpec`
- DCCP no-obstacle smoke test
- DCCP obstacle smoke test
- init-mode tests for:
  - `interp_line`
  - `offset_interp`
  - `straight_line`
- segment-length constraint gating test
- pruning test ensuring target-final waypoints are preserved
- export sign-flip test for ANYmal base
- export arm-offset test
- export failure test for intermediate-point trajectories
- plotting smoke tests under `Agg` for:
  - `plot_task_smooth`
  - `plot_tasks_smooth_2x`

## Non-Goals

Do not do these in the same refactor:

- redesign the DCCP optimization formulation
- retune solver hyperparameters beyond preserving `tau=10`
- implement intermediate-point arm export behavior
- delete research code outright
- force a total rewrite of all task definition files if adapters are sufficient

## Final Guidance For The Implementing Agent

When there is a tradeoff between abstraction and clarity, prefer clarity.

This workspace does not need a clever architecture. It needs:

- one active solver path
- one active plotting path
- one active export path
- one stable internal task/config/result model
- explicit separation of active and experimental code

If a decision comes up during execution and the options are:

- preserve old experimental code in active files, or
- move it into `experimental/` and keep the active package simple

choose the latter.
