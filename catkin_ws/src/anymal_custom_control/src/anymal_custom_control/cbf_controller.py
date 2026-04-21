"""CBF-QP resolved-rate controller for the RRPRRR arm.

Replaces the Jacobian pseudoinverse: where ``qdot = pinv(J) @ v_des``
amplifies near kinematic singularities (σ_min(J) → 0 ⇒ ‖pinv(J)‖ → ∞),
this picks ``qdot`` by solving a small QP each tick with a Yoshikawa
manipulability barrier baked in as a linear inequality.

Problem (one cvxpy ``Problem`` built once; parameters swapped per tick):

    minimize_{qdot, ε}
        ½·‖J·qdot − v_des‖²  +  ½·λ·‖qdot‖²
        +  ½·α·(qdot₄²)  +  ½·β·‖qdot − qdot_ref‖²_W  +  ½·ρ·ε²
    s.t.
        ∇hᵀ·qdot  ≥  −γ·h − ε              (CBF, ε slack)
        qdot_lb  ≤  qdot  ≤  qdot_ub        (box, joint pos limits folded in)
        ε  ≥  0

- h(q) = w(q) − ε_barrier with w the Yoshikawa manipulability.
- γ sets how hard the CBF pushes off the boundary (∇hᵀqdot ≥ −γh ⇒
  h(t) ≥ h(0)·exp(−γt), so {h≥0} is forward-invariant).
- ρ large keeps the QP feasible if pose limits and the CBF constraint
  cannot be satisfied simultaneously; slack is last-resort.
- The box folds in joint-position limits: qdot_ub = min(qdot_max,
  (q_max − q)/dt). At q = q_max the upper bound clips to 0.
"""

import time

import numpy as np
import cvxpy as cp


class CBFQPController:
    def __init__(
        self,
        n=6,
        m=6,
        gamma=3.0,
        reg_lambda=1e-3,
        wrist_rate_alpha=0.0,
        posture_weights=None,
        rho=5e3,
        solver="CLARABEL",
    ):
        self.solver = solver
        self.n, self.m = n, m

        self.qdot = cp.Variable(n)
        self.slack = cp.Variable(nonneg=True)

        self.Jp = cp.Parameter((m, n))
        self.vdes_p = cp.Parameter(m)
        self.grad_h_p = cp.Parameter(n)
        self.h_p = cp.Parameter()
        self.qdot_lb_p = cp.Parameter(n)
        self.qdot_ub_p = cp.Parameter(n)
        self.qdot_ref_p = cp.Parameter(n)

        if posture_weights is None:
            posture_weights = np.zeros(n)
        self.posture_weight_sqrt = np.sqrt(np.asarray(posture_weights, dtype=float))

        objective = cp.Minimize(
            0.5 * cp.sum_squares(self.Jp @ self.qdot - self.vdes_p)
            + 0.5 * reg_lambda * cp.sum_squares(self.qdot)
            # + 0.5 * cp.sum_squares(
            #     cp.multiply(self.posture_weight_sqrt, self.qdot - self.qdot_ref_p)
            # )
            + 0.5 * rho * cp.square(self.slack)
        )
        constraints = [
            self.grad_h_p @ self.qdot >= -gamma * self.h_p - self.slack,
            self.qdot >= self.qdot_lb_p,
            self.qdot <= self.qdot_ub_p,
        ]
        self.prob = cp.Problem(objective, constraints)

        self.last_solve_ms = 0.0

    def solve(self, J, v_des, h_val, grad_h, q, dt, pos_lb, pos_ub, qdot_max, q_ref=None):
        """Solve one tick and return (qdot, slack, solve_ms).

        On solver failure returns (zeros, +inf, solve_ms) — zero velocity
        is the hardware-safe default; motors in impedance mode hold pose.
        """
        qdot_lb_pos = (pos_lb - q) / dt
        qdot_ub_pos = (pos_ub - q) / dt
        qdot_lb = np.maximum(-qdot_max, qdot_lb_pos)
        qdot_ub = np.minimum(qdot_max, qdot_ub_pos)
        if q_ref is None:
            qdot_ref = np.zeros(self.n)
        else:
            qdot_ref = (np.asarray(q_ref, dtype=float) - np.asarray(q, dtype=float)) / dt

        self.Jp.value = np.asarray(J, dtype=float)
        self.vdes_p.value = np.asarray(v_des, dtype=float)
        self.grad_h_p.value = np.asarray(grad_h, dtype=float)
        self.h_p.value = float(h_val)
        self.qdot_lb_p.value = qdot_lb
        self.qdot_ub_p.value = qdot_ub
        self.qdot_ref_p.value = qdot_ref

        t0 = time.perf_counter()
        try:
            self.prob.solve(solver=self.solver, warm_start=True, verbose=False)
        except Exception:
            self.prob.solve(warm_start=True, verbose=False)
        self.last_solve_ms = (time.perf_counter() - t0) * 1000.0

        if self.qdot.value is None:
            print("[CBF] solver failed; freezing qdot.")
            return np.zeros(self.n), float("inf"), self.last_solve_ms

        return (
            np.asarray(self.qdot.value, dtype=float),
            float(self.slack.value),
            self.last_solve_ms,
        )
