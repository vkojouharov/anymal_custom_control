# `planning` Structure

This workspace is organized around a small active optimization package, runnable experiments, exploratory studies, and preserved archives.

## Top Level

```text
planning/
  RUN.py
  RUN_demo.py
  experiments/
  tasks/
  trajectory_opt/
  studies/
  archive/
  generated/
  data/
  plots/
```

## Active Code

`experiments/` contains runnable scripts. Add new experiment scripts here when they combine a task, solver settings, export behavior, and plotting.

`tasks/` contains task definitions only. A task file should define data, not run a solve. Prefer descriptive names such as `source_obstacle_radius_0p5.py`.

`trajectory_opt/` contains reusable implementation code:

- `solver.py`: optimization formulations and helper logic.
- `anymal_export.py`: conversion from optimized points to robot waypoint modules.
- `plotting.py`: plotting for one task.
- `comparison_plotting.py`: plotting for paired task comparisons.

## Studies

`studies/` is for exploratory analysis that supports the main solver but is not part of the normal run path.

Current study:

- `studies/force_penalty/`: force-alignment penalty exploration, model notes, visualization scripts, and generated figures under `outputs/`.

## Archives

`archive/` preserves older experiments and their outputs. These folders are intentionally retained for reference and are not expected to follow the active package style.

Current archive folders:

- `2026-03-12_src_demo/`
- `3x_no_obstacle/`
- `3x_obstacle_scp/`

## Generated Files

`generated/` stores files that are useful to keep around but should not be part of the active source tree:

- `generated/python_cache/`: previously tracked Python bytecode caches.
- `generated/macos/`: previously tracked macOS Finder metadata.

New Python caches and `.DS_Store` files are ignored by `.gitignore`.

## Adding New Work

Use this default placement:

- New task data: `tasks/new_task_name.py`
- New solver or export function: `trajectory_opt/`
- New runnable workflow: `experiments/run_new_workflow.py`
- New exploratory analysis: `studies/new_study_name/`
- New generated study plots: `studies/new_study_name/outputs/`
- Old snapshots or one-off historical artifacts: `archive/`
