import cvxpy as cp
import numpy as np

from task import plot_task_smooth


def solve(t):
    """
    Solve the minimum-length trajectory problem for task dict `t`.

    Expects t to have keys: x0, c1, r1, c2, r2, c3, r3.
    Returns a dict with keys x0, x1, x2, x3 (optimal waypoints),
    or raises RuntimeError if the problem is infeasible / not DCP.
    """
    x1 = cp.Variable(2)
    x2 = cp.Variable(2)
    x3 = cp.Variable(2)

    objective = cp.norm(x1 - t["x0"], 2) + cp.norm(x2 - x1, 2) + cp.norm(x3 - x2, 2)
    constraints = [
        cp.norm(x1 - t["c1"], 2) <= t["r1"],
        cp.norm(x2 - t["c2"], 2) <= t["r2"],
        cp.norm(x3 - t["c3"], 2) <= t["r3"],
    ]

    problem = cp.Problem(cp.Minimize(objective), constraints)
    if not problem.is_dcp():
        raise RuntimeError("Problem is not DCP!")

    problem.solve()

    if problem.status not in ("optimal", "optimal_inaccurate"):
        raise RuntimeError(f"Solver status: {problem.status}")

    return {
        "x0": t["x0"],
        "x1": x1.value,
        "x2": x2.value,
        "x3": x3.value,
    }


def make_solver():
    """
    Build and compile the CVXPY problem once using Parameters.
    Returns a callable  solver(t)  that only updates parameter values and
    re-solves — no problem reconstruction overhead.
    """
    x0_p  = cp.Parameter(2)
    c1_p  = cp.Parameter(2); r1_p = cp.Parameter(nonneg=True)
    c2_p  = cp.Parameter(2); r2_p = cp.Parameter(nonneg=True)
    c3_p  = cp.Parameter(2); r3_p = cp.Parameter(nonneg=True)

    x1 = cp.Variable(2)
    x2 = cp.Variable(2)
    x3 = cp.Variable(2)

    objective = cp.norm(x1 - x0_p, 2) + cp.norm(x2 - x1, 2) + cp.norm(x3 - x2, 2)
    constraints = [
        cp.norm(x1 - c1_p, 2) <= r1_p,
        cp.norm(x2 - c2_p, 2) <= r2_p,
        cp.norm(x3 - c3_p, 2) <= r3_p,
    ]
    problem = cp.Problem(cp.Minimize(objective), constraints)

    # Warm-compile: trigger canonicalization now with dummy values
    x0_p.value  = np.zeros(2)
    c1_p.value  = np.ones(2);  r1_p.value = 0.5
    c2_p.value  = np.ones(2);  r2_p.value = 0.5
    c3_p.value  = np.ones(2);  r3_p.value = 0.5
    problem.solve(warm_start=True)

    def solver(t):
        x0_p.value = np.asarray(t["x0"], dtype=float)
        c1_p.value = np.asarray(t["c1"], dtype=float); r1_p.value = float(t["r1"])
        c2_p.value = np.asarray(t["c2"], dtype=float); r2_p.value = float(t["r2"])
        c3_p.value = np.asarray(t["c3"], dtype=float); r3_p.value = float(t["r3"])
        problem.solve(warm_start=True)
        if problem.status not in ("optimal", "optimal_inaccurate"):
            raise RuntimeError(f"Solver status: {problem.status}")
        return {
            "x0": t["x0"],
            "x1": x1.value.copy(),
            "x2": x2.value.copy(),
            "x3": x3.value.copy(),
            "cost": problem.value,
        }

    return solver


if __name__ == "__main__":
    task1 = {
        "x0": np.array([0.0, 0.0]),
        "c1": np.array([1.0, 1.0]),  "r1": 0.5,
        "c2": np.array([1.5, -1.0]), "r2": 0.5,
        "c3": np.array([3.0, 0.0]),  "r3": 0.5,
    }
    task2 = {
        "x0": np.array([0.0, 0.0]),
        "c1": np.array([1.0, 1.0]),  "r1": 1.5,
        "c2": np.array([1.5, -1.0]), "r2": 1.5,
        "c3": np.array([3.0, 0.0]),  "r3": 1.5,
    }

    optimal_traj1 = solve(task1)
    optimal_traj2 = solve(task2)

    fmt = lambda vec: np.array2string(np.asarray(vec), precision=3, floatmode="fixed")
    print("---------- TASK 1 (r = 0.5) ----------")
    print("[X1] base: ", fmt(-optimal_traj1["x1"]), "arm: ", fmt(task1["c1"] - optimal_traj1["x1"]))
    print("[X2] base: ", fmt(-optimal_traj1["x2"]), "arm: ", fmt(task1["c2"] - optimal_traj1["x2"]))
    print("[X3] base: ", fmt(-optimal_traj1["x3"]), "arm: ", fmt(task1["c3"] - optimal_traj1["x3"]))
    print("---------- TASK 2 (r = 1.5) ----------")
    print("[X1] base: ", fmt(-optimal_traj2["x1"]), "arm: ", fmt(task2["c1"] - optimal_traj2["x1"]))
    print("[X2] base: ", fmt(-optimal_traj2["x2"]), "arm: ", fmt(task2["c2"] - optimal_traj2["x2"]))
    print("[X3] base: ", fmt(-optimal_traj2["x3"]), "arm: ", fmt(task2["c3"] - optimal_traj2["x3"]))
    
    plot_task_smooth(task1, optimal_traj1, save_path="cvxpy_ws/plots/demo1/optimal_traj_0.5.png")
    plot_task_smooth(task2, optimal_traj2, save_path="cvxpy_ws/plots/demo1/optimal_traj_1.5.png")
