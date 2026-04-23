"""Smoke test for CBFQPController.

In a well-conditioned region where the CBF constraint is inactive and
velocity limits are loose, the QP's solution should closely match the
damped pseudoinverse of J applied to v_des.  This confirms the objective
and parameter plumbing are correct.

Run:
    docker compose run --rm anymal-control \
        python3 /catkin_ws/src/anymal_custom_control/scripts/test_cbf_controller.py
"""

import time

import numpy as np

from anymal_custom_control.RRPRRR_kinematic_model import (
    num_jacobian,
    num_manipulability_and_grad,
)
from anymal_custom_control.cbf_controller import CBFQPController

DT = 0.005
CBF_EPS = 0.05
QDOT_MAX = np.array([5.0, 3.0, 1.0, 5.0, 5.0, 5.0])

# Mid-workspace q (kinematic coords, with *_KIN_OFFSET already applied).
q_kin = (0.2, np.pi / 2 + 0.4, 0.9, np.pi / 2 + 0.1, 5 * np.pi / 6 + 0.2, -0.1)
# Raw integrator state (no offsets) for pos_lb/ub arithmetic.
q_vec = np.array([0.2, 0.4, 0.9, 0.1, 0.2, -0.1])
pos_lb = np.array([-np.pi / 2, 0.0, 0.310, -np.inf, -1.7, -np.inf])
pos_ub = np.array([np.pi / 2, np.pi / 2, 2.0, np.inf, np.inf, np.inf])

J = np.asarray(num_jacobian(q_kin), dtype=float)
w, grad_w = num_manipulability_and_grad(q_kin)
print(f"q_kin = {q_kin}")
print(f"w = {w:.4f}  (CBF_EPS = {CBF_EPS}, so h = {w - CBF_EPS:.4f})")
print(f"grad_w = {grad_w}")

v_des = np.array([0.1, 0.05, 0.0, 0.0, 0.02, 0.0])

# Reference: pinv
qdot_pinv = np.linalg.pinv(J) @ v_des
v_achieved_pinv = J @ qdot_pinv
print(f"\npinv qdot:      {qdot_pinv}")
print(f"pinv residual:  {np.linalg.norm(J @ qdot_pinv - v_des):.2e}")

# CBF controller
cbf = CBFQPController(n=6, m=6, gamma=3.0, reg_lambda=1e-3, rho=5e3,
                      solver="CLARABEL")

# Warm up
for _ in range(3):
    cbf.solve(J, v_des, w - CBF_EPS, grad_w, q_vec, DT, pos_lb, pos_ub, QDOT_MAX)

qdot_cbf, slack, solve_ms = cbf.solve(
    J, v_des, w - CBF_EPS, grad_w, q_vec, DT, pos_lb, pos_ub, QDOT_MAX
)
print(f"\ncbf qdot:       {qdot_cbf}")
print(f"cbf residual:   {np.linalg.norm(J @ qdot_cbf - v_des):.2e}")
print(f"cbf slack:      {slack:.2e}")
print(f"cbf solve:      {solve_ms:.2f} ms")

# In well-conditioned region with inactive CBF, qdot_cbf ~ damped pinv.
# Check residual is small — CBF should be able to track v_des.
residual_cbf = np.linalg.norm(J @ qdot_cbf - v_des)
assert residual_cbf < 1e-2, f"CBF residual too large: {residual_cbf}"
assert slack < 1e-4, f"CBF slack nonzero in well-conditioned region: {slack}"

# --- Solve time under realistic load ---
N = 500
t0 = time.perf_counter()
for _ in range(N):
    cbf.solve(J, v_des, w - CBF_EPS, grad_w, q_vec, DT, pos_lb, pos_ub, QDOT_MAX)
dt_mean = (time.perf_counter() - t0) / N * 1e3
print(f"\n500-iteration mean solve time: {dt_mean:.2f} ms")

# --- Near-singularity: set a small h, grad_h pointing into safe region ---
print("\n--- Near-singularity scenario (h = 0.01) ---")
h_small = 0.01
# Pretend grad_h[2] < 0 so extending d3 decreases w (pushes toward singularity)
grad_h_sing = np.array([0.0, 0.1, -0.5, 0.0, -0.1, 0.0])
# Command v_des that asks to extend d3
v_cmd = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])
qdot_sing, slack_sing, _ = cbf.solve(
    J, v_cmd, h_small, grad_h_sing, q_vec, DT, pos_lb, pos_ub, QDOT_MAX
)
print(f"v_cmd = {v_cmd}")
print(f"qdot  = {qdot_sing}")
print(f"slack = {slack_sing:.2e}")
# With h=0.01 and ∇h·qdot ≥ -γh = -0.03, the QP should reduce the
# component of qdot along -∇h, which here means limiting d3-extension.
# Heuristic check: grad_h · qdot + γ·h + slack ≥ 0
cbf_margin = grad_h_sing @ qdot_sing + 3.0 * h_small + slack_sing
print(f"CBF margin (should be ≥ 0): {cbf_margin:.4e}")
assert cbf_margin >= -1e-4, "CBF constraint violated"

print("\nAll smoke tests passed.")
