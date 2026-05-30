import csv
import os
import time

import cvxpy as cp
import numpy as np

from task_w_obstacle import task, plot_task, plot_task_smooth


# ── Helpers ────────────────────────────────────────────────────────────────────

def interp(a, b):
    """Quarter-step interpolations along segment a → b (at t=0.75, 0.5, 0.25)."""
    return 0.25*a + 0.75*b, 0.5*(a + b), 0.75*a + 0.25*b


def _obstacle_halfspace(n, co, ro, point_expr):
    """Linearised obstacle-avoidance constraint: n'co + ro||n|| - n'point <= 0."""
    return n @ co + ro * np.linalg.norm(n) - n @ point_expr <= 0


# ── SCP solver ─────────────────────────────────────────────────────────────────

def scp_solve(t, x1_init, x2_init, x3_init, eps=0.1, tol=0.01, max_iter=50):
    """
    Iterative Sequential Convex Programming for obstacle-aware minimum-length
    trajectory through three task regions.

    Parameters
    ----------
    t : dict
        Task dict with keys x0, c1, r1, c2, r2, c3, r3, co, ro.
    x1_init, x2_init, x3_init : array-like
        Initial feasible guesses for the three waypoints (must lie outside the
        obstacle, i.e. ||xi - co|| >= ro).
    eps : float
        Trust-region radius (fixed).
    tol : float
        Convergence tolerance on max waypoint displacement between iterates.
    max_iter : int
        Maximum number of SCP iterations.

    Returns
    -------
    traj : dict   {"x0", "x1", "x2", "x3"}  compatible with plot_task_smooth()
    history : list[dict]  per-iteration metrics
    """
    x0 = np.asarray(t["x0"], dtype=float)
    co = np.asarray(t["co"], dtype=float)
    ro = float(t["ro"])

    x1_prev = np.asarray(x1_init, dtype=float)
    x2_prev = np.asarray(x2_init, dtype=float)
    x3_prev = np.asarray(x3_init, dtype=float)

    # Feasibility guard: initial guesses must lie outside obstacle
    for label, xi in [("x1_init", x1_prev), ("x2_init", x2_prev), ("x3_init", x3_prev)]:
        if np.linalg.norm(xi - co) < ro:
            raise ValueError(f"{label} is inside the obstacle (dist={np.linalg.norm(xi - co):.4f} < ro={ro})")

    history = []

    for k in range(max_iter):
        # ── Linearisation points ──
        # Intermediate points along each segment
        x01_a, x01_b, x01_c = interp(x0, x1_prev)
        x12_a, x12_b, x12_c = interp(x1_prev, x2_prev)
        x23_a, x23_b, x23_c = interp(x2_prev, x3_prev)

        # Outward normals (point − obstacle centre)
        n1  = x1_prev - co
        n2  = x2_prev - co
        n3  = x3_prev - co
        n01 = [x01_a - co, x01_b - co, x01_c - co]
        n12 = [x12_a - co, x12_b - co, x12_c - co]
        n23 = [x23_a - co, x23_b - co, x23_c - co]

        # ── Build SOCP ──
        x1 = cp.Variable(2)
        x2 = cp.Variable(2)
        x3 = cp.Variable(2)

        objective = cp.norm(x1 - x0, 2) + cp.norm(x2 - x1, 2) + cp.norm(x3 - x2, 2)

        constraints = [
            # Task-region containment
            cp.norm(x1 - t["c1"], 2) <= t["r1"],
            cp.norm(x2 - t["c2"], 2) <= t["r2"],
            cp.norm(x3 - t["c3"], 2) <= t["r3"],
            # Linearised obstacle avoidance — waypoints
            _obstacle_halfspace(n1, co, ro, x1),
            _obstacle_halfspace(n2, co, ro, x2),
            _obstacle_halfspace(n3, co, ro, x3),
            # Linearised obstacle avoidance — intermediate points on segment x0→x1
            _obstacle_halfspace(n01[0], co, ro, 0.25*x0 + 0.75*x1),
            _obstacle_halfspace(n01[1], co, ro, 0.50*x0 + 0.50*x1),
            _obstacle_halfspace(n01[2], co, ro, 0.75*x0 + 0.25*x1),
            # segment x1→x2
            _obstacle_halfspace(n12[0], co, ro, 0.25*x1 + 0.75*x2),
            _obstacle_halfspace(n12[1], co, ro, 0.50*x1 + 0.50*x2),
            _obstacle_halfspace(n12[2], co, ro, 0.75*x1 + 0.25*x2),
            # segment x2→x3
            _obstacle_halfspace(n23[0], co, ro, 0.25*x2 + 0.75*x3),
            _obstacle_halfspace(n23[1], co, ro, 0.50*x2 + 0.50*x3),
            _obstacle_halfspace(n23[2], co, ro, 0.75*x2 + 0.25*x3),
            # Trust region
            cp.norm(x1 - x1_prev, 2) <= eps,
            cp.norm(x2 - x2_prev, 2) <= eps,
            cp.norm(x3 - x3_prev, 2) <= eps,
        ]

        problem = cp.Problem(cp.Minimize(objective), constraints)

        t0 = time.perf_counter()
        problem.solve()
        solve_time = time.perf_counter() - t0

        if problem.status not in ("optimal", "optimal_inaccurate"):
            print(f"[SCP iter {k}] solver status: {problem.status} — stopping.")
            break

        # ── Convergence check ──
        max_step = max(
            np.linalg.norm(x1.value - x1_prev),
            np.linalg.norm(x2.value - x2_prev),
            np.linalg.norm(x3.value - x3_prev),
        )

        history.append({
            "iteration":    k,
            "solve_time_s": solve_time,
            "cost":         problem.value,
            "x1":           x1.value.copy(),
            "x2":           x2.value.copy(),
            "x3":           x3.value.copy(),
            "max_step":     max_step,
            "status":       problem.status,
        })

        # Update linearisation point
        x1_prev = x1.value.copy()
        x2_prev = x2.value.copy()
        x3_prev = x3.value.copy()

        if max_step < tol:
            print(f"[SCP] converged at iteration {k}  (max_step={max_step:.6f} < tol={tol})")
            break
    else:
        print(f"[SCP] reached max_iter={max_iter} without converging  (max_step={max_step:.6f})")

    traj = {"x0": x0, "x1": x1_prev, "x2": x2_prev, "x3": x3_prev}
    return traj, history


# ── CSV logging ────────────────────────────────────────────────────────────────

def save_history_csv(history, path):
    """Write the SCP iteration history to a CSV file."""
    fieldnames = [
        "iteration", "solve_time_s", "cost",
        "x1_0", "x1_1", "x2_0", "x2_1", "x3_0", "x3_1",
        "max_step", "status",
    ]
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in history:
            writer.writerow({
                "iteration":    row["iteration"],
                "solve_time_s": f'{row["solve_time_s"]:.6f}',
                "cost":         f'{row["cost"]:.6f}',
                "x1_0":         f'{row["x1"][0]:.6f}',
                "x1_1":         f'{row["x1"][1]:.6f}',
                "x2_0":         f'{row["x2"][0]:.6f}',
                "x2_1":         f'{row["x2"][1]:.6f}',
                "x3_0":         f'{row["x3"][0]:.6f}',
                "x3_1":         f'{row["x3"][1]:.6f}',
                "max_step":     f'{row["max_step"]:.6f}',
                "status":       row["status"],
            })


# ── Main ───────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    PLOTS_DIR = os.path.join(os.path.dirname(__file__), "plots")
    DATA_DIR  = os.path.join(os.path.dirname(__file__), "data")

    traj, history = scp_solve(
        task,
        x1_init=np.array([0.5, 1.0]),
        x2_init=np.array([-1.0, 2.5]),
        x3_init=np.array([-1.0, 4.0]),
        eps=0.1,
        tol=0.01,
        max_iter=50,
    )

    # Print convergence table
    print(f"\n{'iter':>4}  {'cost':>9}  {'max_step':>10}  {'time_ms':>8}  {'status'}")
    print("-" * 52)
    for row in history:
        print(f"{row['iteration']:4d}  {row['cost']:9.4f}  {row['max_step']:10.6f}  "
              f"{row['solve_time_s']*1e3:8.3f}  {row['status']}")

    print(f"\nFinal waypoints:")
    print(f"  x1 = {traj['x1']}")
    print(f"  x2 = {traj['x2']}")
    print(f"  x3 = {traj['x3']}")

    # Save CSV
    csv_path = os.path.join(DATA_DIR, "scp_history.csv")
    save_history_csv(history, csv_path)
    print(f"\nHistory saved to {csv_path}")

    # Plot
    plot_path = os.path.join(PLOTS_DIR, "scp_obstacle_traj.png")
    plot_task_smooth(task, traj, save_path=plot_path)
    print(f"Plot saved to {plot_path}")
