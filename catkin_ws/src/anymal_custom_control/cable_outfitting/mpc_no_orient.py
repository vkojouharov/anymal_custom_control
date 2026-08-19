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
P_XY = 50.0
W_BEARING = 1.0
R = np.diag([0.1, 0.1, 0.05])
S = np.diag([2.0, 2.0, 0.2])
U_MAX = np.array([0.4, 0.4, 0.4])
DU_MAX = np.array([10.0, 10.0, 10.0])
ALPHA = math.radians(30.0)
POSITION_TOLERANCE = np.array([0.1, 0.1])
BEARING_TOLERANCE = 0.2
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


def tag_bearing(state: np.ndarray) -> float:
    state = np.asarray(state, dtype=float).reshape(3)
    return wrap(math.atan2(float(state[1]), float(state[0])) + float(state[2]))


def linearize_tag_bearing(state: np.ndarray) -> tuple[np.ndarray, float, float]:
    state = np.asarray(state, dtype=float).reshape(3)
    tag_distance = max(float(np.hypot(state[0], state[1])), 1e-6)
    gradient = np.array([-state[1] / tag_distance**2, state[0] / tag_distance**2, 1.0])
    beta = tag_bearing(state)
    return gradient, beta - gradient @ state, beta


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
        BASE_LINEAR_SIGN * axis_command(left, 1.23, 0.035, deadband=0.02, minimum=0.1),
        -axis_command(omega, 1.22, 0.024),
    )


def goal_reached(state: np.ndarray, goal: np.ndarray) -> bool:
    state = np.asarray(state, dtype=float).reshape(3)
    goal = np.asarray(goal, dtype=float).reshape(2)
    position_reached = np.all(np.abs(state[:2] - goal) <= POSITION_TOLERANCE)
    return bool(position_reached and abs(tag_bearing(state)) <= BEARING_TOLERANCE)


@dataclass(frozen=True)
class MpcResult:
    trajectory: np.ndarray
    status: str
    solve_ms: float
    state_cost: float


class BaseMpc:
    """Position-only MPC that continuously points the base camera at the tag."""

    def __init__(self):
        self.x = cp.Variable((HORIZON + 1, 3))
        self.u = cp.Variable((HORIZON, 3))
        self.x0 = cp.Parameter(3)
        self.goal = cp.Parameter(2)
        self.u_prev = cp.Parameter(3)
        self.bearing_gradient = cp.Parameter(3)
        self.bearing_offset = cp.Parameter()

        constraints = [self.x[0] == self.x0]
        for k in range(HORIZON):
            constraints += [
                self.x[k + 1] == self.x[k] + DT * self.u[k],
                -U_MAX <= self.u[k],
                self.u[k] <= U_MAX,
            ]
        constraints += [-DU_MAX <= self.u[0] - self.u_prev, self.u[0] - self.u_prev <= DU_MAX]
        for k in range(1, HORIZON):
            constraints += [-DU_MAX <= self.u[k] - self.u[k - 1], self.u[k] - self.u[k - 1] <= DU_MAX]

        bearing = [self.bearing_offset + self.bearing_gradient @ self.x[k] for k in range(HORIZON + 1)]
        constraints += [item <= ALPHA for item in bearing] + [item >= -ALPHA for item in bearing]

        cost = P_XY * cp.sum_squares(self.x[HORIZON, :2] - self.goal)
        cost += W_BEARING * cp.sum_squares(cp.hstack(bearing[1:]))
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
        goal = np.asarray(goal, dtype=float).reshape(2)
        gradient, offset, beta = linearize_tag_bearing(state)

        self.x0.value = state
        self.goal.value = goal
        self.u_prev.value = np.asarray(previous_control, dtype=float).reshape(3)
        self.bearing_gradient.value = gradient
        self.bearing_offset.value = offset

        started = time.monotonic()
        try:
            self.problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
        except cp.SolverError:
            return None
        solve_ms = 1000.0 * (time.monotonic() - started)
        if self.problem.status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) or self.u.value is None:
            return None

        position_error = state[:2] - goal
        state_cost = P_XY * float(position_error @ position_error) + W_BEARING * beta**2
        return MpcResult(np.asarray(self.u.value, dtype=float).copy(), str(self.problem.status), solve_ms, state_cost)
