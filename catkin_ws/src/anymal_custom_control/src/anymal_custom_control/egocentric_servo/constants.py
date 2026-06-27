"""Shared constants for ANYmal egocentric AprilTag visual servoing."""

from __future__ import annotations

# Critical pose-scale constant: physical black-square AprilTag edge length.
# Field-test tag is 5.75 in.
APRILTAG_TAG_LENGTH_M = 0.14605

APRILTAG_FAMILY = "tag16h5"

RGB_COMPRESSED_TOPIC = "/oakd/rgb/image_color/compressed"
APRILTAG_STATS_TOPIC = "/oakd/apriltag/stats_json"
APRILTAG_DETECTIONS_TOPIC = "/oakd/apriltag/detections_json"
ODOM_TOPIC = "/legged_odometry/pose_in_odom"
OAKD_IMU_QUATERNION_TOPIC = "/oakd/imu/game_rotation_vector"

COMMAND_TOPIC = "/anymal/egocentric_servo/command_json"
STATUS_TOPIC = "/anymal/egocentric_servo/status_json"
TRAJECTORY_TOPIC = "/anymal/egocentric_servo/trajectory_json"

DEFAULT_RECORD_DIR = "/tmp/anymal_egocentric_servo_runs"
DEFAULT_ARCHIVE_DIR = "/catkin_ws/src/anymal_custom_control/data/egocentric_servo_runs"
DEFAULT_TARGET_DISTANCE_M = 0.5
DEFAULT_LOOP_HZ = 20.0
DEFAULT_TAG_TIMEOUT_SEC = 0.45
DEFAULT_ODOM_TIMEOUT_SEC = 1.0
DEFAULT_IMU_TIMEOUT_SEC = 1.0

DEFAULT_MAX_HEADING = 0.18
DEFAULT_MAX_LATERAL = 0.12
DEFAULT_MAX_TURNING = 0.16

RANGE_KP = 0.75
LATERAL_KP = 1.0
YAW_KP = 0.9

TARGET_RANGE_TOLERANCE_M = 0.04
TARGET_LATERAL_TOLERANCE_M = 0.06
TARGET_YAW_TOLERANCE_RAD = 0.10

RECENT_TRAJECTORY_POINTS = 600
