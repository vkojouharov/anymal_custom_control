from __future__ import annotations

import math
import time
from dataclasses import dataclass

import cvxpy as cp
import numpy as np


MPC_HZ = 10.0
CONTROL_HZ = 30.0
DT = 1.0 / MPC_HZ
HORIZON = 30
P_XY, P_THETA = 50.0, 25.0
R = np.diag([0.1, 0.1, 0.05])
S = np.diag([2.0, 2.0, 0.2])
U_MAX = np.array([0.4, 0.4, 0.4])
# U_MAX = np.array([0.2, 0.2, 0.2])
DU_MAX = np.array([10.0, 10.0, 10.0])
ALPHA = math.radians(30.0)
BASE_LINEAR_SIGN = -1.0


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def state_from_tag(T_camera_tag: np.ndarray) -> np.ndarray:
    transform = np.asarray(T_camera_tag, dtype=float).reshape(4, 4)
    rotation_tag_camera = transform[:3, :3].T
    camera_position_tag = -rotation_tag_camera @ transform[:3, 3]
    x, y = -camera_position_tag[2], camera_position_tag[0]
    body_x_tag = -(rotation_tag_camera @ np.array([0.0, 0.0, 1.0]))
    heading_x, heading_y = -body_x_tag[2], body_x_tag[0]
    return np.array([x, y, wrap(-math.atan2(heading_y, heading_x))], dtype=float)


def median_state(states) -> np.ndarray:
    values = np.asarray(states, dtype=float).reshape(-1, 3)
    state = np.median(values, axis=0)
    state[2] = math.atan2(float(np.median(np.sin(values[:, 2]))), float(np.median(np.cos(values[:, 2]))))
    return state


def axis_command(value: float, slope: float, intercept: float, deadband: float = 0.0, minimum: float = 0.0) -> float:
    if abs(value) < deadband or value == 0.0:
        return 0.0
    return float(np.clip(math.copysign(max(slope * abs(value) + intercept, minimum), value), -1.0, 1.0))


def movement_axes(state: np.ndarray, control: np.ndarray) -> tuple[float, float, float]:
    ux, uy, theta_dot = np.asarray(control, dtype=float).reshape(3)
    c, s = math.cos(float(state[2])), math.sin(float(state[2]))
    forward = float(np.clip(-c * ux + s * uy, -0.4, 0.4))
    left = float(np.clip(-s * ux - c * uy, -0.4, 0.4))
    omega = 0.5 * float(np.clip(theta_dot, -0.4, 0.4))
    return (
        BASE_LINEAR_SIGN * axis_command(forward, 1.23, 0.035),
        BASE_LINEAR_SIGN * axis_command(left, 1.23, 0.035, deadband=0.02, minimum=0.4),
        -axis_command(omega, 1.22, 0.024, deadband=0.02, minimum=0.1),
    )


def goal_reached(state: np.ndarray, goal: np.ndarray) -> bool:
    error = np.asarray(state, dtype=float) - np.asarray(goal, dtype=float)
    return abs(error[0]) <= 0.1 and abs(error[1]) <= 0.25 and abs(wrap(float(error[2]))) <= 0.2


@dataclass(frozen=True)
class MpcResult:
    trajectory: np.ndarray
    status: str
    solve_ms: float
    state_cost: float


class BaseMpc:
    def __init__(self):
        self.x = cp.Variable((HORIZON + 1, 3))
        self.u = cp.Variable((HORIZON, 3))
        self.x0 = cp.Parameter(3)
        self.u_prev = cp.Parameter(3)
        self.terminal_factor = cp.Parameter((3, 3))
        self.terminal_target = cp.Parameter(3)
        self.bearing_gradient = cp.Parameter(3)
        self.bearing_offset = cp.Parameter()
        self.weighted_gradient = cp.Parameter(3)
        self.weighted_offset = cp.Parameter()

        constraints = [self.x[0] == self.x0]
        for k in range(HORIZON):
            constraints += [self.x[k + 1] == self.x[k] + DT * self.u[k], -U_MAX <= self.u[k], self.u[k] <= U_MAX]
        constraints += [-DU_MAX <= self.u[0] - self.u_prev, self.u[0] - self.u_prev <= DU_MAX]
        for k in range(1, HORIZON):
            constraints += [-DU_MAX <= self.u[k] - self.u[k - 1], self.u[k] - self.u[k - 1] <= DU_MAX]
        bearing = [self.bearing_offset + self.bearing_gradient @ self.x[k] for k in range(HORIZON + 1)]
        constraints += [item <= ALPHA for item in bearing] + [item >= -ALPHA for item in bearing]

        cost = cp.sum_squares(self.terminal_factor @ self.x[HORIZON] - self.terminal_target)
        cost += cp.sum_squares(cp.hstack([self.weighted_offset + self.weighted_gradient @ self.x[k] for k in range(1, HORIZON + 1)]))
        r_factor, s_factor = np.sqrt(R), np.sqrt(S)
        for k in range(HORIZON):
            cost += cp.sum_squares(r_factor @ self.u[k])
        cost += cp.sum_squares(s_factor @ (self.u[0] - self.u_prev))
        for k in range(1, HORIZON):
            cost += cp.sum_squares(s_factor @ (self.u[k] - self.u[k - 1]))
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
        if not self.problem.is_dcp(dpp=True):
            raise RuntimeError("MPC is not DPP compliant")

    def solve(self, state, goal, previous_control) -> MpcResult | None:
        state = np.asarray(state, dtype=float).reshape(3)
        goal = np.asarray(goal, dtype=float).reshape(3).copy()
        goal[2] = state[2] + wrap(float(goal[2] - state[2]))
        # distance = max(float(np.hypot(state[0], state[1])), 1e-6)
        # gradient = np.array([-state[1] / distance**2, state[0] / distance**2, 1.0])
        tag_distance = max(float(np.hypot(state[0], state[1])), 1e-6)
        distance = max(float(np.hypot(state[0] - goal[0], state[1] - goal[1])), 0.1)
        gradient = np.array([-state[1] / distance**2, state[0] / distance**2, 1.0])
        beta = wrap(math.atan2(state[1], state[0]) + state[2])
        offset = beta - gradient @ state
        factor = np.diag(np.sqrt([P_XY, P_XY, P_THETA / distance**2]))
        self.x0.value = state
        self.u_prev.value = np.asarray(previous_control, dtype=float).reshape(3)
        self.terminal_factor.value = factor
        self.terminal_target.value = factor @ goal
        self.bearing_gradient.value = gradient
        self.bearing_offset.value = offset
        self.weighted_gradient.value = math.sqrt(distance) * gradient
        self.weighted_offset.value = math.sqrt(distance) * offset
        started = time.monotonic()
        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        except cp.SolverError:
            return None
        solve_ms = 1000.0 * (time.monotonic() - started)
        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or self.u.value is None:
            return None
        error = state - goal
        error[2] = wrap(float(error[2]))
        return MpcResult(np.asarray(self.u.value, dtype=float).copy(), str(self.problem.status), solve_ms, float(np.sum(np.square(factor @ error))))
