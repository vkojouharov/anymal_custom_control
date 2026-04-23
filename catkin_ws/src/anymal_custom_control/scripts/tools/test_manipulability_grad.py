"""Verify the analytic manipulability gradient against central finite
difference at several q samples in the safe workspace.

Run:
    docker compose run --rm anymal-control \
        python3 /catkin_ws/src/anymal_custom_control/scripts/test_manipulability_grad.py
"""

import time

import numpy as np

from anymal_custom_control.RRPRRR_kinematic_model import (
    J_num,
    num_manipulability_and_grad,
)


def w_scalar(q):
    J = np.asarray(J_num(*q), dtype=float)
    return abs(np.linalg.det(J))


def fd_grad(q, eps=1e-6):
    q = np.asarray(q, dtype=float)
    g = np.empty(6)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = eps
        g[i] = (w_scalar(q + dq) - w_scalar(q - dq)) / (2.0 * eps)
    return g


rng = np.random.default_rng(42)

# Safe workspace samples (kinematic coords, with *_KIN_OFFSET applied).
q_nominal = np.array([0.0, np.pi/2, 0.8, np.pi/2, 5*np.pi/6, 0.0])

# Tolerance: |a-b| <= atol + rtol*|b|. atol dominates near zero (where FD
# noise swamps rel error); rtol dominates away from zero.
ATOL = 1e-6
RTOL = 1e-4

max_abs = 0.0
failures = 0

print(f"{'sample':>6}  {'w':>10}  {'grad_analytic':>60}  {'max_abs_err':>12}")
for k in range(10):
    jitter = rng.uniform(-0.3, 0.3, size=6)
    q = q_nominal + jitter
    w_a, grad_a = num_manipulability_and_grad(tuple(q))
    grad_fd = fd_grad(q)
    abs_err = np.abs(grad_a - grad_fd)
    tol = ATOL + RTOL * np.abs(grad_fd)
    ok = bool(np.all(abs_err <= tol))
    max_abs = max(max_abs, float(abs_err.max()))
    failures += 0 if ok else 1
    grad_str = "[" + ", ".join(f"{g:+.4f}" for g in grad_a) + "]"
    print(f"{k:>6}  {w_a:>10.5f}  {grad_str:>60}  {abs_err.max():>12.2e}"
          f"{'' if ok else '  *FAIL*'}")

print()
print(f"max absolute error across all samples: {max_abs:.2e}")
print(f"failures (|a-b| > {ATOL:.0e} + {RTOL:.0e}*|b|): {failures}")

# --- Runtime cost of the full helper ---------------------------------------
N = 500
q0 = tuple(q_nominal + rng.uniform(-0.1, 0.1, size=6))
# Warm
for _ in range(5):
    _ = num_manipulability_and_grad(q0)
t0 = time.perf_counter()
for _ in range(N):
    _ = num_manipulability_and_grad(q0)
dt = (time.perf_counter() - t0) / N * 1e6
print(f"\nnum_manipulability_and_grad: {dt:.1f} us/call")

assert failures == 0, f"{failures} gradient samples exceeded 1e-4 rel tolerance"
