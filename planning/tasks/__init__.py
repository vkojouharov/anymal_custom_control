"""Task definitions used by the CVXPY trajectory experiments."""

from . import demo_force_task
from . import demo_obstacle_task
from . import source_obstacle_radius_0p5
from . import source_obstacle_radius_1p75
from . import source_radius_0p5
from . import source_radius_1p75

# Historical aliases used by older runner scripts.
SRC_demo_031726_r0p5 = source_radius_0p5
SRC_demo_031726_r2 = source_radius_1p75
SRC_demo_obstacle_031726_r0p5 = source_obstacle_radius_0p5
SRC_demo_obstacle_031726_r2 = source_obstacle_radius_1p75
TASK_demo = demo_obstacle_task
TASK_demo_w_forces = demo_force_task
