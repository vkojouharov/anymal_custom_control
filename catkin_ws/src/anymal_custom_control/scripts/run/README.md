# `scripts/run`

Preferred user-facing entrypoints live here.

These scripts should stay thin. Their job is to launch the intended runtime
path, not to own the underlying control or perception logic.

Current preferred entrypoints:

- `run_operator_station.py`
- `run_arm_jparse_teleop.py`
- `run_perception_dashboard.py`
- `run_giraf_arm_controller.py`
- `run_giraf_arm_teleop.py`

Notes:

- `run_arm_jparse_teleop.py` launches the new GIRAF arm controller + joystick
  teleop pair.
- `run_giraf_arm_controller.py` and `run_giraf_arm_teleop.py` are lower-level
  runnables for debugging or split-process bring-up.
- Historical `RUN_*` scripts now live under `scripts/legacy/`.
