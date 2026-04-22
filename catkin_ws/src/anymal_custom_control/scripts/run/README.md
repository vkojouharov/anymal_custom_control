# `scripts/run`

Preferred user-facing entrypoints live here.

These scripts should stay thin. Their job is to launch the intended runtime
path, not to own the underlying control or perception logic.

Current preferred entrypoints:

- `run_operator_station.py`
- `run_arm_jparse_teleop.py`
- `run_perception_dashboard.py`

Legacy `RUN_*` scripts still exist during migration and remain callable for
compatibility.
