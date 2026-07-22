"""MPC utilities for ANYmal egocentric AprilTag servoing."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .constants import (
    AXIS_FORWARD_INTERCEPT,
    AXIS_FORWARD_SLOPE,
    AXIS_LATERAL_INTERCEPT,
    AXIS_LATERAL_MIN_COMMAND,
    AXIS_LATERAL_MIN_MPS,
    AXIS_LATERAL_SLOPE,
    AXIS_TURNING_INTERCEPT,
    AXIS_TURNING_SIGN,
    AXIS_TURNING_SLOPE,
    MPC_ALPHA_FOV_RAD,
    MPC_BODY_FORWARD_MAX_MPS,
    MPC_BODY_LATERAL_MAX_MPS,
    MPC_BODY_TURNING_MAX_RADPS,
    MPC_DT_SEC,
    MPC_DU_MAX,
    MPC_DU_MIN,
    MPC_HORIZON,
    MPC_MAX_SOLVE_TIME_SEC,
    MPC_P_WEIGHT,
    MPC_R_WEIGHT,
    MPC_S_WEIGHT,
    MPC_TARGET_LATERAL_TOLERANCE_M,
    MPC_TARGET_RANGE_TOLERANCE_M,
    MPC_TARGET_YAW_TOLERANCE_RAD,
    MPC_U_MAX,
    MPC_U_MIN,
)
from .messages import OdomPose, TagPose, odom_relative_xy, wrap_angle_rad


@dataclass(frozen=True)
class MpcState:
    x: float
    y: float
    theta: float

    def as_array(self) -> np.ndarray:
        return np.asarray([self.x, self.y, self.theta], dtype=float)


@dataclass(frozen=True)
class MpcCommand:
    heading: float
    lateral: float
    turning: float
    vx_body_mps: float
    vy_body_mps: float
    omega_radps: float
    u_world: tuple[float, float, float]
    range_error_m: float
    lateral_error_m: float
    yaw_error_rad: float
    solver_status: str
    solve_time_ms: float
    command_source: str
    open_loop_step: int
    target_reached: bool


@dataclass(frozen=True)
class MpcSolveResult:
    command: MpcCommand
    predicted_states: list[list[float]]
    predicted_controls: list[list[float]]


def zero_mpc_command(
    *,
    solver_status: str = "idle",
    command_source: str = "zero",
    solve_time_ms: float = 0.0,
) -> MpcCommand:
    return MpcCommand(
        heading=0.0,
        lateral=0.0,
        turning=0.0,
        vx_body_mps=0.0,
        vy_body_mps=0.0,
        omega_radps=0.0,
        u_world=(0.0, 0.0, 0.0),
        range_error_m=0.0,
        lateral_error_m=0.0,
        yaw_error_rad=0.0,
        solver_status=solver_status,
        solve_time_ms=float(solve_time_ms),
        command_source=command_source,
        open_loop_step=0,
        target_reached=False,
    )


def tag_pose_to_mpc_state(tag: TagPose) -> Optional[MpcState]:
    """Convert OpenCV camera-frame AprilTag pose to the solver's planar state.

    The detector gives the tag pose in camera coordinates:
        p_camera_tag, R_camera_tag.

    We first invert that transform to localize the camera/robot in the tag
    frame:
        p_tag_camera = -R_tag_camera @ p_camera_tag.

    The solver uses a planar frame whose +X axis is the approach direction from
    the tag toward the starting robot side, and whose +Y axis is tag-left. With
    the AprilTag detector convention observed in logs, a camera in front of the
    tag lies on negative tag Z, so -Z_tag maps to solver +X and -X_tag maps to
    solver +Y. The camera optical axis expressed in the tag frame gives the
    robot heading in the same planar solver frame.
    """

    if tag.rotation_camera_tag is None:
        return None
    rotation_tag_camera = tag.rotation_camera_tag.T
    camera_position_tag = -rotation_tag_camera @ tag.position_camera_m
    x_mpc, y_mpc = _tag_vector_to_mpc_xy(camera_position_tag)

    camera_forward_tag = rotation_tag_camera @ np.asarray([0.0, 0.0, 1.0], dtype=float)
    solver_body_x_tag = -camera_forward_tag
    heading_x, heading_y = _tag_vector_to_mpc_xy(solver_body_x_tag)
    if not math.isfinite(x_mpc) or not math.isfinite(y_mpc) or not math.isfinite(heading_x) or not math.isfinite(heading_y):
        return None
    if abs(heading_x) + abs(heading_y) <= 1e-9:
        return None
    theta = math.atan2(heading_y, heading_x)
    return MpcState(float(x_mpc), float(y_mpc), wrap_angle_rad(float(theta)))


def median_mpc_state(tags: Sequence[TagPose]) -> Optional[MpcState]:
    states = [state for state in (tag_pose_to_mpc_state(tag) for tag in tags) if state is not None]
    if not states:
        return None
    xs = np.asarray([state.x for state in states], dtype=float)
    ys = np.asarray([state.y for state in states], dtype=float)
    thetas = np.asarray([state.theta for state in states], dtype=float)
    theta = math.atan2(float(np.median(np.sin(thetas))), float(np.median(np.cos(thetas))))
    return MpcState(float(np.median(xs)), float(np.median(ys)), float(theta))


def odom_to_mpc_state(current: Optional[OdomPose], origin: Optional[OdomPose], origin_state: Optional[MpcState]) -> Optional[MpcState]:
    if current is None or origin is None or origin_state is None:
        return None
    rel_forward, rel_left, rel_yaw = odom_relative_xy(current, origin)
    solver_body_delta = np.asarray([-rel_forward, rel_left], dtype=float)
    c = math.cos(origin_state.theta)
    s = math.sin(origin_state.theta)
    dx = c * solver_body_delta[0] - s * solver_body_delta[1]
    dy = s * solver_body_delta[0] + c * solver_body_delta[1]
    return MpcState(
        x=float(origin_state.x + dx),
        y=float(origin_state.y + dy),
        theta=wrap_angle_rad(origin_state.theta - rel_yaw),
    )


def solve_mpc_command(
    *,
    state: MpcState,
    target_distance_m: float,
    u_prev_world: np.ndarray,
) -> Optional[MpcSolveResult]:
    goal = np.asarray([float(target_distance_m), 0.0, 0.0], dtype=float)
    x0 = state.as_array()
    distance_to_tag_m = float(np.hypot(x0[0], x0[1]))
    safe_distance_to_tag_m = max(distance_to_tag_m, np.finfo(float).eps)
    p_weight = np.asarray(MPC_P_WEIGHT, dtype=float).copy()
    p_weight[2] /= safe_distance_to_tag_m
    solve_start = time.perf_counter()
    u0_world, x_pred, u_pred, status = solve_visual_mpc_step(
        x0=x0,
        x_goal=goal,
        u_prev=u_prev_world,
        dt=MPC_DT_SEC,
        N=MPC_HORIZON,
        P=np.diag(p_weight),
        R=np.diag(MPC_R_WEIGHT),
        S=np.diag(MPC_S_WEIGHT),
        u_min=np.asarray(MPC_U_MIN, dtype=float),
        u_max=np.asarray(MPC_U_MAX, dtype=float),
        du_min=np.asarray(MPC_DU_MIN, dtype=float),
        du_max=np.asarray(MPC_DU_MAX, dtype=float),
        alpha_fov=MPC_ALPHA_FOV_RAD,
        bearing_weight=distance_to_tag_m,
        nominal_traj=None,
    )
    solve_time_ms = (time.perf_counter() - solve_start) * 1000.0
    if u0_world is None or x_pred is None or u_pred is None:
        return None
    command = command_from_world_control(
        state=state,
        target_distance_m=target_distance_m,
        u_world=np.asarray(u0_world, dtype=float),
        solver_status=str(status),
        solve_time_ms=solve_time_ms,
        command_source="fresh_solve",
        open_loop_step=0,
    )
    return MpcSolveResult(
        command=command,
        predicted_states=_array_to_rows(x_pred),
        predicted_controls=_array_to_rows(u_pred),
    )


def warm_up_mpc_solver(target_distance_m: float) -> float:
    """Compile and solve the cached MPC once before robot motion begins."""
    warmup_state = MpcState(max(float(target_distance_m) + 0.5, 1.0), 0.0, 0.0)
    result = solve_mpc_command(
        state=warmup_state,
        target_distance_m=target_distance_m,
        u_prev_world=np.zeros(3, dtype=float),
    )
    if result is None:
        raise RuntimeError("MPC warm-up solve failed")
    return result.command.solve_time_ms


def solve_overran_budget(command: MpcCommand) -> bool:
    return command.solve_time_ms > (MPC_MAX_SOLVE_TIME_SEC * 1000.0)


def command_from_world_control(
    *,
    state: MpcState,
    target_distance_m: float,
    u_world: np.ndarray,
    solver_status: str,
    solve_time_ms: float,
    command_source: str,
    open_loop_step: int,
) -> MpcCommand:
    body_solver = world_to_body_velocity(u_world, state.theta)
    vx_body_mps = -float(body_solver[0])
    vy_body_mps = float(body_solver[1])
    omega_radps = float(body_solver[2])
    vx_body_mps = float(np.clip(vx_body_mps, -MPC_BODY_FORWARD_MAX_MPS, MPC_BODY_FORWARD_MAX_MPS))
    vy_body_mps = float(np.clip(vy_body_mps, -MPC_BODY_LATERAL_MAX_MPS, MPC_BODY_LATERAL_MAX_MPS))
    omega_radps = float(np.clip(omega_radps, -MPC_BODY_TURNING_MAX_RADPS, MPC_BODY_TURNING_MAX_RADPS))
    u_world = body_to_world_velocity(np.asarray([-vx_body_mps, vy_body_mps, omega_radps], dtype=float), state.theta)
    heading, lateral, turning = physical_velocity_to_axes(vx_body_mps, vy_body_mps, omega_radps)
    range_error = float(state.x - target_distance_m)
    lateral_error = float(state.y)
    yaw_error = wrap_angle_rad(float(state.theta))
    target_reached = (
        abs(range_error) <= MPC_TARGET_RANGE_TOLERANCE_M
        and abs(lateral_error) <= MPC_TARGET_LATERAL_TOLERANCE_M
        and abs(yaw_error) <= MPC_TARGET_YAW_TOLERANCE_RAD
    )
    if target_reached:
        heading = lateral = turning = 0.0
        vx_body_mps = vy_body_mps = omega_radps = 0.0
        u_world = np.zeros(3, dtype=float)
    return MpcCommand(
        heading=float(heading),
        lateral=float(lateral),
        turning=float(turning),
        vx_body_mps=float(vx_body_mps),
        vy_body_mps=float(vy_body_mps),
        omega_radps=float(omega_radps),
        u_world=(float(u_world[0]), float(u_world[1]), float(u_world[2])),
        range_error_m=range_error,
        lateral_error_m=lateral_error,
        yaw_error_rad=yaw_error,
        solver_status=solver_status,
        solve_time_ms=float(solve_time_ms),
        command_source=command_source,
        open_loop_step=int(open_loop_step),
        target_reached=target_reached,
    )


def physical_velocity_to_axes(v_forward_mps: float, v_left_mps: float, omega_radps: float) -> tuple[float, float, float]:
    heading = _affine_axis(v_forward_mps, AXIS_FORWARD_SLOPE, AXIS_FORWARD_INTERCEPT)
    lateral = _affine_axis(
        v_left_mps,
        AXIS_LATERAL_SLOPE,
        AXIS_LATERAL_INTERCEPT,
        min_abs_input=AXIS_LATERAL_MIN_MPS,
        min_abs_output=AXIS_LATERAL_MIN_COMMAND,
    )
    turning = AXIS_TURNING_SIGN * _affine_axis(omega_radps, AXIS_TURNING_SLOPE, AXIS_TURNING_INTERCEPT)
    return heading, lateral, turning


def _affine_axis(
    value: float,
    slope: float,
    intercept: float,
    *,
    min_abs_input: float = 0.0,
    min_abs_output: float = 0.0,
) -> float:
    value = float(value)
    magnitude = abs(value)
    if magnitude <= 0.0 or magnitude < float(min_abs_input):
        return 0.0
    axis = float(slope) * magnitude + float(intercept)
    axis = max(axis, float(min_abs_output))
    return float(np.clip(math.copysign(axis, value), -1.0, 1.0))


def _quadratic_factor(matrix: np.ndarray) -> np.ndarray:
    """Return L such that ||L x||^2 equals x.T @ matrix @ x."""
    matrix = np.asarray(matrix, dtype=float)
    symmetric = 0.5 * (matrix + matrix.T)
    eigenvalues, eigenvectors = np.linalg.eigh(symmetric)
    if float(np.min(eigenvalues)) < -1e-9:
        raise ValueError("MPC quadratic weight must be positive semidefinite")
    eigenvalues = np.maximum(eigenvalues, 0.0)
    return np.sqrt(eigenvalues)[:, None] * eigenvectors.T


def _cache_array_key(values) -> tuple:
    array = np.asarray(values, dtype=float)
    return (array.shape, *array.reshape(-1).tolist())


class _ParameterizedVisualMpc:
    """Reusable DPP-compliant CVXPY problem for low-latency MPC solves."""

    def __init__(self, *, dt, N, R, S, u_min, u_max, du_min, du_max, alpha_fov) -> None:
        import cvxpy as cp

        self._cp = cp
        self._N = int(N)
        nx = 3
        nu = 3

        self._x0 = cp.Parameter(nx)
        self._u_prev = cp.Parameter(nu)
        self._terminal_factor = cp.Parameter((nx, nx))
        self._terminal_target = cp.Parameter(nx)
        self._bearing_gradient = cp.Parameter((self._N + 1, nx))
        self._bearing_offset = cp.Parameter(self._N + 1)
        self._weighted_bearing_gradient = cp.Parameter((self._N + 1, nx))
        self._weighted_bearing_offset = cp.Parameter(self._N + 1)

        self._x = cp.Variable((self._N + 1, nx))
        self._u = cp.Variable((self._N, nu))
        constraints = [self._x[0, :] == self._x0]
        for k in range(self._N):
            constraints.append(self._x[k + 1, :] == self._x[k, :] + float(dt) * self._u[k, :])
            constraints.append(np.asarray(u_min, dtype=float) <= self._u[k, :])
            constraints.append(self._u[k, :] <= np.asarray(u_max, dtype=float))
        constraints.append(np.asarray(du_min, dtype=float) <= self._u[0, :] - self._u_prev)
        constraints.append(self._u[0, :] - self._u_prev <= np.asarray(du_max, dtype=float))
        for k in range(1, self._N):
            constraints.append(np.asarray(du_min, dtype=float) <= self._u[k, :] - self._u[k - 1, :])
            constraints.append(self._u[k, :] - self._u[k - 1, :] <= np.asarray(du_max, dtype=float))

        weighted_beta_lin_traj = []
        for k in range(self._N + 1):
            beta_lin = self._bearing_offset[k] + self._bearing_gradient[k, :] @ self._x[k, :]
            constraints.append(beta_lin <= float(alpha_fov))
            constraints.append(beta_lin >= -float(alpha_fov))
            weighted_beta_lin_traj.append(
                self._weighted_bearing_offset[k]
                + self._weighted_bearing_gradient[k, :] @ self._x[k, :]
            )

        weighted_terminal_error = self._terminal_factor @ self._x[self._N, :] - self._terminal_target
        cost = cp.sum_squares(weighted_terminal_error)
        cost += cp.sum_squares(cp.hstack(weighted_beta_lin_traj[1:]))

        r_factor = _quadratic_factor(R)
        s_factor = _quadratic_factor(S)
        for k in range(self._N):
            cost += cp.sum_squares(r_factor @ self._u[k, :])
        cost += cp.sum_squares(s_factor @ (self._u[0, :] - self._u_prev))
        for k in range(1, self._N):
            cost += cp.sum_squares(s_factor @ (self._u[k, :] - self._u[k - 1, :]))

        self._problem = cp.Problem(cp.Minimize(cost), constraints)
        if not self._problem.is_dcp(dpp=True):
            raise RuntimeError("Parameterized MPC problem is not DPP-compliant")

    def solve(self, *, x0, x_goal, u_prev, P, bearing_weight, nominal_traj, solver):
        gradients = np.asarray([bearing_gradient(state) for state in nominal_traj], dtype=float)
        beta_bars = np.asarray([bearing_to_tag(state) for state in nominal_traj], dtype=float)
        offsets = beta_bars - np.einsum("ij,ij->i", gradients, nominal_traj)
        terminal_factor = _quadratic_factor(P)
        sqrt_bearing_weight = math.sqrt(bearing_weight)

        self._x0.value = x0
        self._u_prev.value = u_prev
        self._terminal_factor.value = terminal_factor
        self._terminal_target.value = terminal_factor @ x_goal
        self._bearing_gradient.value = gradients
        self._bearing_offset.value = offsets
        self._weighted_bearing_gradient.value = sqrt_bearing_weight * gradients
        self._weighted_bearing_offset.value = sqrt_bearing_weight * offsets

        cp = self._cp
        selected_solver = cp.OSQP if solver is None else solver
        try:
            self._problem.solve(solver=selected_solver, warm_start=True)
        except cp.SolverError:
            self._problem.solve(solver=cp.CLARABEL, warm_start=True)
        if self._problem.status not in ["optimal", "optimal_inaccurate"]:
            return None, None, None, self._problem.status
        return (
            np.asarray(self._u.value[0], dtype=float).copy(),
            np.asarray(self._x.value, dtype=float).copy(),
            np.asarray(self._u.value, dtype=float).copy(),
            self._problem.status,
        )


_VISUAL_MPC_CACHE: dict[tuple, _ParameterizedVisualMpc] = {}


def _parameterized_mpc(*, dt, N, R, S, u_min, u_max, du_min, du_max, alpha_fov) -> _ParameterizedVisualMpc:
    key = (
        float(dt),
        int(N),
        _cache_array_key(R),
        _cache_array_key(S),
        _cache_array_key(u_min),
        _cache_array_key(u_max),
        _cache_array_key(du_min),
        _cache_array_key(du_max),
        float(alpha_fov),
    )
    workspace = _VISUAL_MPC_CACHE.get(key)
    if workspace is None:
        workspace = _ParameterizedVisualMpc(
            dt=dt,
            N=N,
            R=R,
            S=S,
            u_min=u_min,
            u_max=u_max,
            du_min=du_min,
            du_max=du_max,
            alpha_fov=alpha_fov,
        )
        _VISUAL_MPC_CACHE[key] = workspace
    return workspace


def solve_visual_mpc_step(
    x0,
    x_goal,
    u_prev,
    dt=MPC_DT_SEC,
    N=MPC_HORIZON,
    P=np.diag(MPC_P_WEIGHT),
    R=np.diag(MPC_R_WEIGHT),
    S=np.diag(MPC_S_WEIGHT),
    u_min=np.asarray(MPC_U_MIN, dtype=float),
    u_max=np.asarray(MPC_U_MAX, dtype=float),
    du_min=np.asarray(MPC_DU_MIN, dtype=float),
    du_max=np.asarray(MPC_DU_MAX, dtype=float),
    alpha_fov=MPC_ALPHA_FOV_RAD,
    bearing_weight=0.0,
    nominal_traj=None,
    solver=None,
):
    x0 = np.asarray(x0, dtype=float)
    x_goal = np.asarray(x_goal, dtype=float)
    u_prev = np.asarray(u_prev, dtype=float)
    bearing_weight = float(bearing_weight)
    if bearing_weight < 0.0:
        raise ValueError("bearing_weight must be nonnegative")
    nx = 3
    if nominal_traj is None:
        nominal_traj = np.tile(x0, (N + 1, 1))
    else:
        nominal_traj = np.asarray(nominal_traj, dtype=float)
        assert nominal_traj.shape == (N + 1, nx)
    workspace = _parameterized_mpc(
        dt=dt,
        N=N,
        R=R,
        S=S,
        u_min=u_min,
        u_max=u_max,
        du_min=du_min,
        du_max=du_max,
        alpha_fov=alpha_fov,
    )
    return workspace.solve(
        x0=x0,
        x_goal=x_goal,
        u_prev=u_prev,
        P=P,
        bearing_weight=bearing_weight,
        nominal_traj=nominal_traj,
        solver=solver,
    )


def bearing_to_tag(state) -> float:
    x, y, theta = np.asarray(state, dtype=float)
    return wrap_angle_rad(float(np.arctan2(y, x) - theta))


def bearing_gradient(state, eps: float = 1e-6) -> np.ndarray:
    x, y, _ = np.asarray(state, dtype=float)
    r2 = x**2 + y**2 + eps
    return np.asarray([-y / r2, x / r2, -1.0], dtype=float)


def world_to_body_velocity(u_world, theta: float) -> np.ndarray:
    vx_w, vy_w, omega = np.asarray(u_world, dtype=float)
    c = math.cos(theta)
    s = math.sin(theta)
    vx_b = c * vx_w + s * vy_w
    vy_b = -s * vx_w + c * vy_w
    return np.asarray([vx_b, vy_b, omega], dtype=float)


def body_to_world_velocity(u_body, theta: float) -> np.ndarray:
    vx_b, vy_b, omega = np.asarray(u_body, dtype=float)
    c = math.cos(theta)
    s = math.sin(theta)
    vx_w = c * vx_b - s * vy_b
    vy_w = s * vx_b + c * vy_b
    return np.asarray([vx_w, vy_w, omega], dtype=float)


def _tag_vector_to_mpc_xy(vector_tag: np.ndarray) -> tuple[float, float]:
    return -float(vector_tag[2]), -float(vector_tag[0])


def _array_to_rows(values: np.ndarray) -> list[list[float]]:
    arr = np.asarray(values, dtype=float)
    return [[float(item) for item in row] for row in arr.reshape(arr.shape[0], -1)]
