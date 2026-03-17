import cvxpy as cp
import dccp
import numpy as np
import time


def _target_center_xy(target):
    center = np.asarray(target["center"], dtype=float).reshape(-1)
    return center[:2]


def _target_force_application_point(target):
    center = np.asarray(target["center"], dtype=float).reshape(-1)
    if center.size >= 3:
        return center[:3]
    return np.array([center[0], center[1], 1.0], dtype=float)


def _point_line_distance(point, line_start, line_end):
    segment = line_end - line_start
    norm = np.linalg.norm(segment)
    if norm < 1e-9:
        return np.linalg.norm(point - line_start)
    return abs(np.cross(segment, point - line_start)) / norm


def _segment_hits_obstacle(segment_start, segment_end, obstacle):
    return _point_line_distance(np.asarray(obstacle["center"], dtype=float), segment_start, segment_end) <= float(obstacle["radius"])


def _points_clear_obstacles(points, obstacles):
    for point in points:
        for obstacle in obstacles:
            center = np.asarray(obstacle["center"], dtype=float)
            radius = float(obstacle["radius"])
            if np.linalg.norm(point - center) < radius:
                return False
    return True


def _build_init_points(task, points_per_target, init_mode, offset_step=0.1, max_offset_steps=50):
    init_points = np.zeros((len(task["targets"]) * points_per_target, 2), dtype=float)
    previous_point = np.asarray(task["start"], dtype=float).reshape(-1)[:2]
    obstacles = task.get("obstacles", [])

    if init_mode == "straight_line":
        final_center = _target_center_xy(task["targets"][-1])
        for idx in range(len(init_points)):
            alpha = (idx + 1) / len(init_points)
            init_points[idx] = (1.0 - alpha) * previous_point + alpha * final_center
        return init_points

    for i, target in enumerate(task["targets"]):
        target_center = _target_center_xy(target)
        base_idx = i * points_per_target
        block_points = np.zeros((points_per_target, 2), dtype=float)
        for j in range(points_per_target):
            alpha = (j + 1) / points_per_target
            block_points[j] = (1.0 - alpha) * previous_point + alpha * target_center

        if init_mode == "offset_interp" and points_per_target > 1:
            segment = target_center - previous_point
            segment_norm = np.linalg.norm(segment)
            obstructing = [
                obstacle for obstacle in obstacles
                if _segment_hits_obstacle(previous_point, target_center, obstacle)
            ]
            if obstructing and segment_norm > 1e-9:
                normal = np.array([-segment[1], segment[0]]) / segment_norm
                shifted = None
                intermediate = block_points[:-1]
                for step_idx in range(1, max_offset_steps + 1):
                    offset = step_idx * offset_step
                    for direction in (1.0, -1.0):
                        candidate = intermediate + direction * offset * normal
                        if _points_clear_obstacles(candidate, obstructing):
                            shifted = candidate
                            break
                    if shifted is not None:
                        block_points[:-1] = shifted
                        break

        init_points[base_idx:base_idx + points_per_target] = block_points
        previous_point = target_center

    return init_points


def _obstacle_halfspace(normal, center, radius, point_expr):
    return normal @ center + radius * cp.norm(normal, 2) - normal @ point_expr <= 0


def _check_init_clear(points, obstacles):
    for idx, point in enumerate(points):
        for obs_idx, obstacle in enumerate(obstacles):
            center = np.asarray(obstacle["center"], dtype=float)
            radius = float(obstacle["radius"])
            if np.linalg.norm(point - center) < radius:
                raise ValueError(
                    f"Initial point {idx} lies inside obstacle {obs_idx}; SCP needs obstacle-clear initialization."
                )


def _build_scp_problem(task, points_per_target, trust_region_radius, sample_fractions):
    num_tasks = len(task["targets"])
    num_points = num_tasks * points_per_target
    num_obstacles = len(task["obstacles"])

    x = cp.Variable((num_points, 2))
    previous_points = cp.Parameter((num_points, 2))
    trust_radius = cp.Parameter(nonneg=True, value=trust_region_radius)
    waypoint_normals = [
        [cp.Parameter(2) for _ in range(num_points)]
        for _ in range(num_obstacles)
    ]
    segment_normals = [
        [[cp.Parameter(2) for _ in sample_fractions] for _ in range(num_points - 1)]
        for _ in range(num_obstacles)
    ]

    objective = cp.Minimize(
        sum(cp.norm(x[i + 1] - x[i], 2) for i in range(num_points - 1))
        + cp.norm(x[0] - task["start"], 2)
    )

    constraints = []
    for i, target in enumerate(task["targets"]):
        target_idx = (i + 1) * points_per_target - 1
        constraints.append(
            cp.norm(x[target_idx] - _target_center_xy(target), 2)
            <= float(target["radius"])
        )

    for i in range(num_points):
        constraints.append(cp.norm(x[i] - previous_points[i], 2) <= trust_radius)

    for obs_idx, obstacle in enumerate(task["obstacles"]):
        center = np.asarray(obstacle["center"], dtype=float)
        radius = float(obstacle["radius"])

        for point_idx in range(num_points):
            constraints.append(
                _obstacle_halfspace(waypoint_normals[obs_idx][point_idx], center, radius, x[point_idx])
            )

        for seg_idx in range(num_points - 1):
            for frac_idx, frac in enumerate(sample_fractions):
                point_expr = (1.0 - frac) * x[seg_idx] + frac * x[seg_idx + 1]
                constraints.append(
                    _obstacle_halfspace(segment_normals[obs_idx][seg_idx][frac_idx], center, radius, point_expr)
                )

    problem = cp.Problem(objective, constraints)
    return problem, x, previous_points, trust_radius, waypoint_normals, segment_normals


def _update_scp_linearization(task, current_points, sample_fractions, waypoint_normals, segment_normals):
    for obs_idx, obstacle in enumerate(task["obstacles"]):
        center = np.asarray(obstacle["center"], dtype=float)

        for point_idx, point in enumerate(current_points):
            waypoint_normals[obs_idx][point_idx].value = point - center

        for seg_idx in range(len(current_points) - 1):
            for frac_idx, frac in enumerate(sample_fractions):
                sample = (1.0 - frac) * current_points[seg_idx] + frac * current_points[seg_idx + 1]
                segment_normals[obs_idx][seg_idx][frac_idx].value = sample - center


def _prune_trajectory(points, num_tasks, points_per_target, pruning_threshold):
    target_indices = {points_per_target * i + (points_per_target - 1) for i in range(num_tasks)}
    pruned_points = [points[0]]
    original_indices = [0]

    for idx in range(1, len(points) - 1):
        if idx in target_indices:
            pruned_points.append(points[idx])
            original_indices.append(idx)
            continue

        dist = _point_line_distance(points[idx], pruned_points[-1], points[idx + 1])
        if dist > pruning_threshold:
            pruned_points.append(points[idx])
            original_indices.append(idx)

    pruned_points.append(points[-1])
    original_indices.append(len(points) - 1)
    return np.asarray(pruned_points), original_indices


def _sample_target_force_penalty(target, radial_samples, angular_samples):
    center_xy = _target_center_xy(target)
    force_application_point = _target_force_application_point(target)
    force = np.asarray(target.get("force", np.zeros(3, dtype=float)), dtype=float).reshape(-1)

    r = np.linspace(0.0, float(target["radius"]), radial_samples)
    theta = np.linspace(0.0, 2.0 * np.pi, angular_samples, endpoint=False)
    radial_grid, angular_grid = np.meshgrid(r[1:], theta, indexing="xy")

    local_x = np.concatenate(([0.0], (radial_grid * np.cos(angular_grid)).ravel()))
    local_y = np.concatenate(([0.0], (radial_grid * np.sin(angular_grid)).ravel()))

    world_x = center_xy[0] + local_x
    world_y = center_xy[1] + local_y
    sampled_points = np.column_stack((world_x, world_y, np.zeros_like(world_x)))

    force_norm = np.linalg.norm(force)
    if force_norm <= 0.0:
        penalty = np.zeros_like(world_x)
    else:
        lever_arm = force_application_point - sampled_points
        distance_to_task = np.linalg.norm(lever_arm, axis=-1)
        cross_norm = np.linalg.norm(np.cross(lever_arm, force), axis=-1)
        penalty = cross_norm / (distance_to_task * force_norm)

    return world_x, world_y, penalty


def _find_force_penalty_minimum(points_x, points_y, penalty, preferred_point):
    flat_x = np.asarray(points_x, dtype=float).ravel()
    flat_y = np.asarray(points_y, dtype=float).ravel()
    flat_penalty = np.asarray(penalty, dtype=float).ravel()
    preferred_xy = np.asarray(preferred_point, dtype=float).reshape(-1)[:2]

    min_value = np.min(flat_penalty)
    candidate_mask = np.isclose(flat_penalty, min_value, rtol=1.0e-9, atol=1.0e-12)
    candidate_indices = np.flatnonzero(candidate_mask)
    candidate_points = np.column_stack((flat_x[candidate_indices], flat_y[candidate_indices]))
    distance_to_preferred = np.linalg.norm(candidate_points - preferred_xy, axis=1)
    tie_break_order = np.lexsort(
        (
            flat_y[candidate_indices],
            flat_x[candidate_indices],
            distance_to_preferred,
        )
    )
    min_index = candidate_indices[tie_break_order[0]]
    return flat_x[min_index], flat_y[min_index], flat_penalty[min_index]


def _fit_force_penalty_linearization(points_x, points_y, penalty, min_point, min_value):
    distances = np.sqrt((points_x - min_point[0]) ** 2 + (points_y - min_point[1]) ** 2)
    centered_penalty = penalty - min_value
    denominator = np.sum(distances**2)
    if denominator <= 0.0:
        scale = 0.0
    else:
        scale = max(np.sum(distances * centered_penalty) / denominator, 0.0)
    return {
        "min_point": np.asarray(min_point, dtype=float),
        "min_value": float(min_value),
        "scale": float(scale),
    }


def _build_force_penalty_linearizations(task, radial_samples, angular_samples):
    preferred_point = np.asarray(task["start"], dtype=float)
    linearizations = []
    for target in task["targets"]:
        points_x, points_y, penalty = _sample_target_force_penalty(
            target, radial_samples, angular_samples
        )
        min_x, min_y, min_value = _find_force_penalty_minimum(
            points_x, points_y, penalty, preferred_point
        )
        linearizations.append(
            _fit_force_penalty_linearization(
                points_x,
                points_y,
                penalty,
                np.array([min_x, min_y], dtype=float),
                min_value,
            )
        )
    return linearizations


def _linearized_force_penalty_expr(point_expr, linearization):
    return linearization["min_value"] + linearization["scale"] * cp.norm(
        point_expr - linearization["min_point"], 2
    )


def solve_trajectory(TASK, pruning_threshold=0.1, intermediate_points=3, init_mode="interp_line"):
    construct_start = time.perf_counter()

    # Retrieve task parameters
    num_tasks = len(TASK["targets"])
    obstacles = TASK.get("obstacles", [])
    num_obstacles = len(obstacles)
    min_obstacle_size = min(obs["radius"] for obs in obstacles) if num_obstacles > 0 else 1.0
    points_per_target = intermediate_points + 1

    # Define optimization variabkes
    x = cp.Variable((num_tasks * points_per_target, 2))

    # Objecive: Minimize total distance traveled
    objective = cp.Minimize(
        sum(cp.norm(x[i + 1] - x[i], 2) for i in range(num_tasks * points_per_target - 1))
        + cp.norm(x[0] - TASK["start"], 2)
    )

    # Constraints
    constraints = []
    # final waypoint in each target block must lie within the target region
    for i in range(num_tasks):
        constraints.append(
            cp.norm(x[(i + 1) * points_per_target - 1] - _target_center_xy(TASK["targets"][i]), 2)
            <= TASK["targets"][i]["radius"]
        )
    # Limit segment length when obstacle clearance sampling matters or when
    # intermediate waypoints are present and should stay locally connected.
    if num_obstacles > 0 or intermediate_points != 0:
        for i in range(num_tasks * points_per_target - 1):
            constraints.append(cp.norm(x[i+1] - x[i], 2) <= 0.7 * min_obstacle_size)
    # all waypoints must avoid obstacles (NOT convex, need DCCP)
    for i in range(num_tasks * points_per_target):
        for j in range(num_obstacles):
            constraints.append(
                obstacles[j]["radius"] - cp.norm(x[i] - obstacles[j]["center"], 2) <= 0
            )

    if init_mode not in {"interp_line", "offset_interp", "straight_line"}:
        raise ValueError(f"Unsupported init_mode: {init_mode}")
    init_points = _build_init_points(TASK, points_per_target, init_mode)
    x.value = init_points

    # form problem
    problem = cp.Problem(objective, constraints)
    construction_time_s = time.perf_counter() - construct_start

    solve_start = time.perf_counter()
    solve_result = problem.solve(method="dccp", solver=cp.SCS, tau=10)
    solve_time_s = time.perf_counter() - solve_start
    if x.value is None:
        raise RuntimeError(
            f"DCCP solve failed: status={problem.status}, result={solve_result}. "
            "The current task/obstacle layout is likely infeasible for the present constraints or initialization."
        )
    trajectory = np.asarray(x.value)
    trajectory, kept_indices = _prune_trajectory(trajectory, num_tasks, points_per_target, pruning_threshold)

    return trajectory, solve_result, construction_time_s, solve_time_s, kept_indices


def solve_trajectory_w_forces(
    TASK,
    pruning_threshold=0.1,
    intermediate_points=0,
    distance_weight=0.1,
    radial_samples=120,
    angular_samples=240,
):
    construct_start = time.perf_counter()

    num_tasks = len(TASK["targets"])
    obstacles = TASK.get("obstacles", [])
    num_obstacles = len(obstacles)
    points_per_target = intermediate_points + 1
    min_obstacle_size = min(obs["radius"] for obs in obstacles) if num_obstacles > 0 else None

    x = cp.Variable((num_tasks * points_per_target, 2))
    linearizations = _build_force_penalty_linearizations(TASK, radial_samples, angular_samples)

    distance_cost = (
        sum(cp.norm(x[i + 1] - x[i], 2) for i in range(num_tasks * points_per_target - 1))
        + cp.norm(x[0] - np.asarray(TASK["start"], dtype=float), 2)
    )
    force_cost = 0
    for i, linearization in enumerate(linearizations):
        target_idx = (i + 1) * points_per_target - 1
        force_cost += _linearized_force_penalty_expr(x[target_idx], linearization)
    objective = cp.Minimize(force_cost + distance_weight * distance_cost)

    constraints = []
    for i, target in enumerate(TASK["targets"]):
        target_idx = (i + 1) * points_per_target - 1
        constraints.append(
            cp.norm(x[target_idx] - _target_center_xy(target), 2) <= float(target["radius"])
        )

    if min_obstacle_size is not None:
        for i in range(num_tasks * points_per_target - 1):
            constraints.append(cp.norm(x[i + 1] - x[i], 2) <= 0.75 * min_obstacle_size)

    for i in range(num_tasks * points_per_target):
        for obstacle in obstacles:
            constraints.append(
                float(obstacle["radius"])
                - cp.norm(x[i] - np.asarray(obstacle["center"], dtype=float), 2)
                <= 0
            )

    init_points = _build_init_points(TASK, points_per_target, "interp_line")
    x.value = init_points

    problem = cp.Problem(objective, constraints)
    construction_time_s = time.perf_counter() - construct_start

    solve_start = time.perf_counter()
    solve_result = problem.solve(method="dccp", solver=cp.SCS, tau=10)
    solve_time_s = time.perf_counter() - solve_start
    if x.value is None:
        raise RuntimeError(
            f"DCCP force solve failed: status={problem.status}, result={solve_result}. "
            "The current task/obstacle layout is likely infeasible for the present constraints or initialization."
        )
    trajectory = np.asarray(x.value)
    trajectory, kept_indices = _prune_trajectory(
        trajectory, num_tasks, points_per_target, pruning_threshold
    )

    return trajectory, solve_result, construction_time_s, solve_time_s, kept_indices


def solve_trajectory_SCP(
    TASK,
    pruning_threshold=0.1,
    intermediate_points=3,
    init_mode="offset_interp",
    trust_region_radius=1.0,
    tol=1e-2,
    max_iter=20,
    sample_fractions=(0.25, 0.5, 0.75),
    solver=cp.SCS,
):
    construct_start = time.perf_counter()

    num_tasks = len(TASK["targets"])
    points_per_target = intermediate_points + 1
    if init_mode not in {"interp_line", "offset_interp", "straight_line"}:
        raise ValueError(f"Unsupported init_mode: {init_mode}")

    init_points = _build_init_points(TASK, points_per_target, init_mode)
    _check_init_clear(init_points, TASK.get("obstacles", []))

    (
        problem,
        x,
        previous_points,
        trust_radius,
        waypoint_normals,
        segment_normals,
    ) = _build_scp_problem(TASK, points_per_target, trust_region_radius, sample_fractions)
    construction_time_s = time.perf_counter() - construct_start

    current_points = init_points.copy()
    trust_radius.value = trust_region_radius
    total_solve_time_s = 0.0
    history = []

    for iteration in range(max_iter):
        previous_points.value = current_points
        _update_scp_linearization(TASK, current_points, sample_fractions, waypoint_normals, segment_normals)
        x.value = current_points

        iter_start = time.perf_counter()
        solve_result = problem.solve(solver=solver, warm_start=True, verbose=False)
        iter_solve_time_s = time.perf_counter() - iter_start
        total_solve_time_s += iter_solve_time_s

        if x.value is None or problem.status not in ("optimal", "optimal_inaccurate"):
            raise RuntimeError(
                f"SCP solve failed at iteration {iteration}: status={problem.status}, result={solve_result}"
            )

        next_points = np.asarray(x.value).copy()
        max_step = np.max(np.linalg.norm(next_points - current_points, axis=1))
        history.append(
            {
                "iteration": iteration,
                "status": problem.status,
                "cost": float(problem.value),
                "max_step": float(max_step),
                "solve_time_s": iter_solve_time_s,
            }
        )

        current_points = next_points
        if max_step < tol:
            break

    trajectory, kept_indices = _prune_trajectory(current_points, num_tasks, points_per_target, pruning_threshold)
    return trajectory, construction_time_s, total_solve_time_s, kept_indices, history
    
