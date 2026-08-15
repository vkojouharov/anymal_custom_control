# Cable outfitting

## Quick start

From this folder:

```bash
cp config/hardware.example.yaml config/hardware.yaml
cp config/trajectory.example.yaml config/trajectory.yaml
rosrun anymal_custom_control list_oakd_cameras.py
```

1. Put the forward camera MXID under `navigation_camera` and the arm-camera
   MXID under `arm_camera` in `config/hardware.yaml`. The defaults are 30 Hz
   with the large navigation-tag size and 10 Hz with the small manipulation-tag
   size.
2. Edit `config/trajectory.yaml`. Each task point names its navigation tag and
   goal, open-loop deployment twist and timeout, manipulation tag, and
   `pick`, `place`, or `hook` policy. The example IDs and motions are
   placeholders and must be checked for the real setup.
3. Put ANYmal in Walk and run:

```bash
rosrun anymal_custom_control run_cable_trajectory.py "$PWD/config/hardware.yaml" "$PWD/config/trajectory.yaml"
```

`Y` starts or resumes, `B` pauses, and `X` stops. The completed trajectory
holds the final arm pose until `X` or Ctrl-C.

## Folder layout

- `run_cable_trajectory.py`: thin executable for the integrated runtime.
- `executor.py`: task-point state machine and operator controls.
- `camera.py`: two-camera ownership, calibration, and AprilTag detection.
- `arm.py`, `kinematics.py`: internally integrated arm control.
- `mpc.py`: ANYmal tag-relative MPC.
- `trajectory.py`: hardware and trajectory YAML loading and validation.
- `policies/`: separate pick, place, and hook policies.
- `config/`: editable examples.
- `debug/run_cable_outfitting.py`: preserved single-camera arm debug tool.

Both cameras remain open and drain frames continuously, but only the camera
needed by the current phase runs AprilTag detection. Intrinsics and distortion
are read independently from each camera's EEPROM and applied before pose
estimation. The navigation camera must negotiate USB3; the arm camera may use
USB2.

Navigation goals are `[x_m, y_m, yaw_rad]` in the large-tag frame. Deployment
twists are `[vx, vy, vz, wx, wy, wz]` in the arm base frame and run until the
requested manipulation tag is stable or the timeout expires. Policies are
selected explicitly rather than inferred from tag IDs.

The original standalone base MPC remains at
`scripts/run/run_mpc_barebones_test.py` relative to the ROS package root.
