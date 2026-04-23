"""Preflight timing benchmark for the CBF-QP arm controller.

Measures the two unknowns that determine whether a 200 Hz CBF-QP control
loop is viable on this arm:
  1. Cost of evaluating the sympy-lambdified 6x6 Jacobian (plus det + inv).
  2. Cost of a warm-started QP solve per installed cvxpy solver.

Run inside the container:
    docker compose run --rm anymal-control \
        python3 /catkin_ws/src/anymal_custom_control/scripts/preflight_cbf_bench.py
"""

import time

import numpy as np
import cvxpy as cp

print(f"cvxpy version: {cp.__version__}")
print(f"installed solvers: {cp.installed_solvers()}")

from anymal_custom_control.RRPRRR_kinematic_model import num_jacobian

# Representative mid-workspace samples (kinematic coords, with *_KIN_OFFSET
# applied — same convention as joint_coords in RUN_arm_wrist_teleop.py).
q_samples = [
    (0.0,  np.pi/2 + 0.1, 0.5, np.pi/2,       5*np.pi/6 + 0.1,  0.0),
    (0.3,  np.pi/2 + 0.3, 0.8, np.pi/2 + 0.1, 5*np.pi/6 + 0.2,  0.2),
    (-0.2, np.pi/2 + 0.5, 1.2, np.pi/2 - 0.1, 5*np.pi/6 - 0.1, -0.3),
]

for q in q_samples:
    _ = num_jacobian(q)

# --- J_num alone -----------------------------------------------------------
N = 2000
t0 = time.perf_counter()
for _ in range(N):
    for q in q_samples:
        _ = np.asarray(num_jacobian(q), dtype=float)
dt_j = (time.perf_counter() - t0) / (N * len(q_samples)) * 1e6
print(f"J_num:              {dt_j:7.1f} us/call")

# --- J_num + det + inv -----------------------------------------------------
t0 = time.perf_counter()
for _ in range(N):
    for q in q_samples:
        J = np.asarray(num_jacobian(q), dtype=float)
        _ = np.linalg.det(J)
        _ = np.linalg.inv(J)
dt_linalg = (time.perf_counter() - t0) / (N * len(q_samples)) * 1e6
print(f"J_num + det + inv:  {dt_linalg:7.1f} us/call")

# --- QP (mirrors CBFQPController shape) ------------------------------------
n, m = 6, 6
qdot  = cp.Variable(n)
slack = cp.Variable(nonneg=True)
Jp       = cp.Parameter((m, n))
vdes_p   = cp.Parameter(m)
grad_h_p = cp.Parameter(n)
h_p      = cp.Parameter()
qdot_lb_p = cp.Parameter(n)
qdot_ub_p = cp.Parameter(n)

objective = cp.Minimize(
    0.5 * cp.sum_squares(Jp @ qdot - vdes_p)
    + 0.5 * 1e-3 * cp.sum_squares(qdot)
    + 0.5 * 5e3  * cp.square(slack)
)
constraints = [
    grad_h_p @ qdot >= -3.0 * h_p - slack,
    qdot >= qdot_lb_p,
    qdot <= qdot_ub_p,
]
prob = cp.Problem(objective, constraints)

J0 = np.asarray(num_jacobian(q_samples[0]), dtype=float)
Jp.value        = J0
vdes_p.value    = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
grad_h_p.value  = np.full(n, 0.1)
h_p.value       = 0.5
qdot_lb_p.value = -np.array([3.0, 2.0, 0.5, 3.0, 3.0, 3.0])
qdot_ub_p.value =  np.array([3.0, 2.0, 0.5, 3.0, 3.0, 3.0])

solvers_to_try = [s for s in ("OSQP", "CLARABEL", "SCS", "PROXQP", "PIQP")
                  if s in cp.installed_solvers()]

print()
for solver in solvers_to_try:
    try:
        prob.solve(solver=solver, warm_start=False, verbose=False)
        N2 = 500
        t0 = time.perf_counter()
        for _ in range(N2):
            prob.solve(solver=solver, warm_start=True, verbose=False)
        dt_solve = (time.perf_counter() - t0) / N2 * 1e6
        print(f"{solver:10s} warm solve: {dt_solve:7.1f} us/solve")
    except Exception as e:
        print(f"{solver:10s} FAILED: {e}")

print()
print("Budget at 200 Hz = 5000 us/tick.")
print("Full CBF step ≈ 7*(J_num) + (det+inv) + 6*(6x6 trace) + solve.")
