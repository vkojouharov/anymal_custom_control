# AprilTag Visual-Servo MPC

Small MPC sandbox for navigating a holonomic ANYmal-style base to a pose in
front of a fixed AprilTag. State and controls are expressed in the AprilTag /
global frame.

## Files

- `solve_mpc.py`: CVXPY MPC solver. Builds one convex QP, solves for a horizon
  of global velocity commands, and returns only the first command for use in a
  receding-horizon loop.
- `run_mpc_sim.py`: Closed-loop simulator and live Matplotlib visualization.
  Draws the AprilTag, robot, realized trajectory, current MPC horizon, old
  prediction tails, and camera FOV cone.
- `sweep_mpc_horizon.py`: Headless parameter sweep over MPC horizon length.
  Runs full closed-loop episodes and saves metrics as CSV plus a subplot PNG.

## Requirements

Install the project Python requirements first:

```bash
pip install -r requirements.txt
```

The MPC scripts require `cvxpy`, `numpy`, and `matplotlib`.

## Live MPC Simulation

Run with live plotting:

```bash
python3 planning/anymal_apriltag_mpc/run_mpc_sim.py
```

By default this also saves an MP4 video at:

```text
planning/anymal_apriltag_mpc/mpc_sim.mp4
```

Run without opening a plot window:

```bash
python3 planning/anymal_apriltag_mpc/run_mpc_sim.py --no-live
```

Useful options:

```bash
python3 planning/anymal_apriltag_mpc/run_mpc_sim.py \
  --dt 0.2 \
  --horizon 30 \
  --max-iters 100 \
  --pause 0.03 \
  --output-mp4 planning/anymal_apriltag_mpc/mpc_sim.mp4 \
  --fps 5
```

- `--dt`: simulation/control timestep in seconds.
- `--horizon`: number of MPC steps planned each solve.
- `--max-iters`: maximum closed-loop MPC cycles.
- `--pause`: Matplotlib pause time between live frames.
- `--output-mp4`: output path for the saved simulation video.
- `--fps`: MP4 frame rate.
- `--no-video`: do not save an MP4.
- `--no-live`: run headless.

The physical lookahead time is:

```text
T_horizon = dt * horizon
```

## Horizon Sweep

Run the default sweep:

```bash
python3 planning/anymal_apriltag_mpc/sweep_mpc_horizon.py
```

Default sweep:

```text
dt = 0.2
horizons = 5, 10, ..., 50
max_iters = 1000
```

Outputs:

```text
planning/anymal_apriltag_mpc/horizon_sweep_dt_0p2.csv
planning/anymal_apriltag_mpc/horizon_sweep_dt_0p2.png
```

Useful options:

```bash
python3 planning/anymal_apriltag_mpc/sweep_mpc_horizon.py \
  --dt 0.2 \
  --min-horizon 5 \
  --max-horizon 50 \
  --step 5 \
  --max-iters 1000
```

Optional output paths:

```bash
python3 planning/anymal_apriltag_mpc/sweep_mpc_horizon.py \
  --output-csv /tmp/mpc_sweep.csv \
  --output-png /tmp/mpc_sweep.png
```

The sweep records episode-level metrics such as convergence, total time, total
distance, minimum FOV margin, final error, control effort, smoothness, and solve
time.
