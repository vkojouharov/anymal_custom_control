"""Shared constants for ANYmal egocentric AprilTag visual servoing."""

from __future__ import annotations

# Critical pose-scale constant: physical black-square AprilTag edge length.
# Field-test tag is 7.9 in.
APRILTAG_TAG_LENGTH_M = 0.20066

APRILTAG_FAMILY = "tag16h5"

RGB_COMPRESSED_TOPIC = "/oakd/rgb/image_color/compressed"
APRILTAG_STATS_TOPIC = "/oakd/apriltag/stats_json"
APRILTAG_DETECTIONS_TOPIC = "/oakd/apriltag/detections_json"
ODOM_TOPIC = "/legged_odometry/pose_in_odom"

COMMAND_TOPIC = "/anymal/egocentric_servo/command_json"
STATUS_TOPIC = "/anymal/egocentric_servo/status_json"
TRAJECTORY_TOPIC = "/anymal/egocentric_servo/trajectory_json"

DEFAULT_RECORD_DIR = "/experiments/egocentric_servo_runs"
DEFAULT_ARCHIVE_DIR = "/experiments/egocentric_servo_runs"
DEFAULT_TARGET_DISTANCE_M = 0.5
DEFAULT_LOOP_HZ = 20.0
DEFAULT_TAG_TIMEOUT_SEC = 0.45
DEFAULT_TAG_LOSS_PAUSE_SEC = 2.0
DEFAULT_ODOM_TIMEOUT_SEC = 1.0

MPC_DT_SEC = 0.2
MPC_LOOP_HZ = 5.0
MPC_HORIZON = 20
MPC_MAX_SOLVE_TIME_SEC = 0.2
MPC_MAX_OPEN_LOOP_STEPS = 2
MPC_START_SAMPLE_SEC = 1.0
MPC_START_MIN_SAMPLES = 20
MPC_FILTER_WINDOW = 9
MPC_STATE_SMOOTH_ALPHA = 0.35
MPC_STATE_MAX_POSITION_STEP_M = 0.25
MPC_STATE_MAX_THETA_STEP_RAD = 0.25

MPC_TARGET_RANGE_TOLERANCE_M = 0.05
MPC_TARGET_LATERAL_TOLERANCE_M = 0.05
MPC_TARGET_YAW_TOLERANCE_RAD = 0.10

MPC_P_WEIGHT = (20.0, 20.0, 10.0)
MPC_R_WEIGHT = (0.1, 0.1, 0.05)
MPC_S_WEIGHT = (2.0, 2.0, 1.0)
# Conservative hardware safety limits in the MPC/tag frame.
MPC_U_MIN = (-0.2, -0.2, -0.5)
MPC_U_MAX = (0.2, 0.2, 0.5)
MPC_DU_MIN = (-0.06, -0.06, -0.15)
MPC_DU_MAX = (0.06, 0.06, 0.15)
MPC_BODY_FORWARD_MAX_MPS = 0.2
MPC_BODY_LATERAL_MAX_MPS = 0.2
MPC_BODY_TURNING_MAX_RADPS = 0.5
MPC_ALPHA_FOV_RAD = 0.5235987755982988  # 30 deg

# Calibrated physical body velocity -> normalized ANYmal movement axes.
# Each mapping is applied to magnitude first, then the original sign is restored.
AXIS_FORWARD_SLOPE = 1.23
AXIS_FORWARD_INTERCEPT = 0.035
AXIS_LATERAL_SLOPE = AXIS_FORWARD_SLOPE
AXIS_LATERAL_INTERCEPT = AXIS_FORWARD_INTERCEPT
AXIS_LATERAL_MIN_MPS = 0.02
AXIS_LATERAL_MIN_COMMAND = 0.1
AXIS_TURNING_SLOPE = 1.22
AXIS_TURNING_INTERCEPT = 0.024
AXIS_TURNING_SIGN = -1.0

RECENT_TRAJECTORY_POINTS = 600
