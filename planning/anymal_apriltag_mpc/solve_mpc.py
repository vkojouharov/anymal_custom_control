import numpy as np


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def bearing_to_tag(state):
    """
    Bearing of tag in robot/camera frame.

    State is [x, y, theta] in tag frame.
    Tag is assumed at origin. The camera is modeled as looking along the
    robot's negative x-axis, so x_goal = [0.5, 0, 0] has zero tag bearing.
    """
    x, y, theta = state
    beta = np.arctan2(y, x) - theta
    return wrap_angle(beta)


def bearing_gradient(state, eps=1e-6):
    """
    Gradient of beta(state) = atan2(y, x) - theta
    with respect to [x, y, theta].
    """
    x, y, _ = state
    r2 = x**2 + y**2 + eps

    return np.array([
        -y / r2,
         x / r2,
        -1.0
    ])


def solve_visual_mpc_step(
    x0,
    x_goal,
    u_prev,
    dt=0.2,
    N=20,
    P=np.diag([20.0, 20.0, 10.0]),
    R=np.diag([0.1, 0.1, 0.05]),
    S=np.diag([2.0, 2.0, 1.0]),
    u_min=np.array([-0.5, -0.5, -1.0]),
    u_max=np.array([ 0.5,  0.5,  1.0]),
    du_min=np.array([-0.15, -0.15, -0.20]),
    du_max=np.array([ 0.15,  0.15,  0.20]),
    alpha_fov=np.deg2rad(35.0),
    nominal_traj=None,
    solver=None,
):
    """
    Solve one MPC step.

    State:
        x_k = [x, y, theta] in tag/world frame

    Control:
        u_k = [xdot, ydot, thetadot] in tag/world frame

    Returns:
        u0_world: first optimized global-frame velocity command
        x_pred: predicted state trajectory, shape (N+1, 3)
        u_pred: predicted control trajectory, shape (N, 3)
        status: CVXPY solve status
    """

    import cvxpy as cp

    if solver is None:
        solver = cp.OSQP

    x0 = np.asarray(x0, dtype=float)
    x_goal = np.asarray(x_goal, dtype=float)
    u_prev = np.asarray(u_prev, dtype=float)

    nx = 3
    nu = 3

    # Decision variables
    x = cp.Variable((N + 1, nx))
    u = cp.Variable((N, nu))

    constraints = []

    # Initial condition
    constraints.append(x[0, :] == x0)

    # Dynamics
    for k in range(N):
        constraints.append(x[k + 1, :] == x[k, :] + dt * u[k, :])

    # Control limits
    for k in range(N):
        constraints.append(u_min <= u[k, :])
        constraints.append(u[k, :] <= u_max)

    # Control rate limits
    constraints.append(du_min <= u[0, :] - u_prev)
    constraints.append(u[0, :] - u_prev <= du_max)

    for k in range(1, N):
        constraints.append(du_min <= u[k, :] - u[k - 1, :])
        constraints.append(u[k, :] - u[k - 1, :] <= du_max)

    # Nominal trajectory for FOV linearization
    if nominal_traj is None:
        # Simplest option: linearize around current measured pose for all k
        nominal_traj = np.tile(x0, (N + 1, 1))
    else:
        nominal_traj = np.asarray(nominal_traj, dtype=float)
        assert nominal_traj.shape == (N + 1, nx)

    # Linearized FOV constraints
    for k in range(N + 1):
        xbar = nominal_traj[k]

        beta_bar = bearing_to_tag(xbar)
        g = bearing_gradient(xbar)

        # beta(x_k) ≈ beta_bar + g^T (x_k - xbar)
        beta_lin = beta_bar + g @ (x[k, :] - xbar)

        constraints.append(beta_lin <= alpha_fov)
        constraints.append(beta_lin >= -alpha_fov)

    # Objective
    cost = 0

    # Terminal pose cost
    eN = x[N, :] - x_goal
    cost += cp.quad_form(eN, P)

    # Control effort and smoothness
    for k in range(N):
        cost += cp.quad_form(u[k, :], R)

    cost += cp.quad_form(u[0, :] - u_prev, S)

    for k in range(1, N):
        cost += cp.quad_form(u[k, :] - u[k - 1, :], S)

    problem = cp.Problem(cp.Minimize(cost), constraints)

    try:
        problem.solve(solver=solver, warm_start=True)
    except cp.SolverError:
        problem.solve(solver=cp.CLARABEL)

    if problem.status not in ["optimal", "optimal_inaccurate"]:
        return None, None, None, problem.status

    u_pred = u.value
    x_pred = x.value
    u0_world = u_pred[0]

    return u0_world, x_pred, u_pred, problem.status


def world_to_body_velocity(u_world, theta):
    """
    Convert global/tag-frame velocity to body-frame command.

    u_world = [xdot, ydot, thetadot]
    theta is robot heading in the same global/tag frame.
    """
    vx_w, vy_w, omega = u_world

    c = np.cos(theta)
    s = np.sin(theta)

    vx_b =  c * vx_w + s * vy_w
    vy_b = -s * vx_w + c * vy_w

    return np.array([vx_b, vy_b, omega])
