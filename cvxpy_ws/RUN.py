import warnings
warnings.filterwarnings("ignore", category=FutureWarning, module="dccp.*")

from tasks.SRC_demo_obstacle_031726_r0p5 import TASK as TASK_obstacle_small_r
from tasks.SRC_demo_obstacle_031726_r2 import TASK as TASK_obstacle_large_r

from trajectory_opt.solver import solve_trajectory
from trajectory_opt.visualizer_2x import plot_tasks_smooth_2x

small_traj, small_result, small_construct_s, small_solve_s, small_kept = solve_trajectory(
    TASK_obstacle_small_r,
    intermediate_points=6,
    pruning_threshold=0.1,
)

large_traj, large_result, large_construct_s, large_solve_s, large_kept = solve_trajectory(
    TASK_obstacle_large_r,
    init_mode="straight_line",
    intermediate_points=3,
    pruning_threshold=0.1,
)

plot_tasks_smooth_2x(
    TASK_obstacle_small_r,
    small_traj,
    TASK_obstacle_large_r,
    large_traj,
    left_title="low reach w/ obstacle",
    right_title="high reach w/ obstacle",
)

# from tasks.TASK_demo import TASK
# from tasks.TASK_demo_w_forces import TASK as TASK_WITH_FORCES

# from trajectory_opt.solver import solve_trajectory_SCP, solve_trajectory, solve_trajectory_w_forces
# from trajectory_opt.visualizer import plot_task, plot_task_smooth, plot_task_w_forces, plot_task_w_linearized_forces, plot_task_smooth_w_forces

# trajectory, solve_result, construction_time_s, solve_time_s, kept_indices = solve_trajectory_w_forces(TASK_WITH_FORCES, distance_weight=0.1)
# print(f"Construction time: {construction_time_s:.6f} sec | Solve time (all SCP iterations): {solve_time_s:.2f} sec")
# plot_task_smooth_w_forces(TASK_WITH_FORCES, trajectory)

# plot_task_w_forces(TASK_WITH_FORCES)
# plot_task_w_linearized_forces(TASK_WITH_FORCES)

# trajectory, construction_time_s, solve_time_s, kept_indices, history = solve_trajectory_SCP(TASK)
# trajectory, solve_result, construction_time_s, solve_time_s, kept_indices = solve_trajectory(TASK)

# print(f"Construction time: {construction_time_s:.6f} sec | Solve time (all SCP iterations): {solve_time_s:.2f} sec")
# plot_task_smooth(TASK, trajectory)
