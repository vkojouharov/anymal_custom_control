import warnings
warnings.filterwarnings("ignore", category=FutureWarning, module="dccp.*")

from tasks.TASK_demo import TASK

from trajectory_opt.solver import solve_trajectory_SCP
from trajectory_opt.visualizer import plot_task, plot_task_smooth

trajectory, construction_time_s, solve_time_s, kept_indices, history = solve_trajectory_SCP(TASK)
print(f"Construction time: {construction_time_s:.6f} sec | Solve time (all SCP iterations): {solve_time_s:.2f} sec")
plot_task_smooth(TASK, trajectory)
