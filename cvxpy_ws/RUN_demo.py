import warnings
from pathlib import Path

warnings.filterwarnings("ignore", category=FutureWarning, module="dccp.*")

from tasks import SRC_demo_031726_r0p5, SRC_demo_031726_r2

from trajectory_opt.anymal_export import export_task_trajectory_module
from trajectory_opt.solver import solve_trajectory
from trajectory_opt.visualizer_2x import plot_tasks_smooth_2x


TRAJECTORY_OUTPUT_DIR = (
    Path(__file__).resolve().parents[1]
    / "catkin_ws"
    / "src"
    / "anymal_custom_control"
    / "scripts"
    / "trajectories"
)


def _solve_and_export(task_module, intermediate_points=0, ee_height=0.5):
    trajectory, solve_result, construction_time_s, solve_time_s, kept_indices = solve_trajectory(
        task_module.TASK,
        intermediate_points=intermediate_points,
    )
    output_path = export_task_trajectory_module(
        task_module.__name__.split(".")[-1],
        task_module.TASK,
        trajectory,
        TRAJECTORY_OUTPUT_DIR,
        ee_height=ee_height,
    )
    print(
        f"{output_path.name}: status={solve_result}, "
        f"construction={construction_time_s:.4f}s, solve={solve_time_s:.4f}s"
    )
    return trajectory, output_path, kept_indices


small_traj, small_output_path, small_kept = _solve_and_export(SRC_demo_031726_r0p5, intermediate_points=0)
large_traj, large_output_path, large_kept = _solve_and_export(SRC_demo_031726_r2, intermediate_points=0)

plot_tasks_smooth_2x(
    SRC_demo_031726_r0p5.TASK,
    small_traj,
    SRC_demo_031726_r2.TASK,
    large_traj,
    left_title=small_output_path.stem,
    right_title=large_output_path.stem,
)
