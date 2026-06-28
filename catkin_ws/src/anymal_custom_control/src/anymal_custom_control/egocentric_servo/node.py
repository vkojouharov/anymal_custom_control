"""ROS node for ANYmal egocentric AprilTag visual servoing."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from anymal_custom_control import ModeController, MovementController
from anymal_custom_control.modes import DEFAULT_TOPIC as MODE_GOAL_TOPIC
from operational_mode_manager_msgs.msg import SwitchOperationalModeActionGoal

from .constants import (
    APRILTAG_DETECTIONS_TOPIC,
    APRILTAG_TAG_LENGTH_M,
    COMMAND_TOPIC,
    RGB_COMPRESSED_TOPIC,
    DEFAULT_IMU_TIMEOUT_SEC,
    DEFAULT_LOOP_HZ,
    DEFAULT_TAG_LOSS_PAUSE_SEC,
    DEFAULT_MAX_HEADING,
    DEFAULT_MAX_LATERAL,
    DEFAULT_MAX_TURNING,
    DEFAULT_MIN_COMMAND,
    DEFAULT_ODOM_TIMEOUT_SEC,
    DEFAULT_ARCHIVE_DIR,
    DEFAULT_RECORD_DIR,
    DEFAULT_TAG_TIMEOUT_SEC,
    DEFAULT_TARGET_DISTANCE_M,
    OAKD_IMU_QUATERNION_TOPIC,
    ODOM_TOPIC,
    RECENT_TRAJECTORY_POINTS,
    STATUS_TOPIC,
    TRAJECTORY_TOPIC,
)
from .messages import ImuQuat, OdomPose, TagPose, parse_tag_detections_json, yaw_from_quaternion
from .policy import ServoCommand, ServoLimits, compute_servo_command
from .recorder import TrajectoryRecorder


STATE_IDLE = "IDLE"
STATE_ARMED = "ARMED"
STATE_TRACKING = "TRACKING"
STATE_PAUSED = "PAUSED"
STATE_PAUSED_LOST_TAG = "PAUSED_LOST_TAG"
STATE_TARGET_REACHED = "TARGET_REACHED"
STATE_STOPPED = "STOPPED"
STATE_FAULT = "FAULT"


class EgocentricServoNode:
    def __init__(self, args: argparse.Namespace) -> None:
        self._target_tag_id: Optional[int] = args.target_tag_id
        self._target_distance_m = float(args.target_distance_m)
        self._tag_timeout_sec = float(args.tag_timeout_sec)
        self._tag_loss_pause_sec = float(args.tag_loss_pause_sec)
        self._odom_timeout_sec = float(args.odom_timeout_sec)
        self._imu_timeout_sec = float(args.imu_timeout_sec)
        self._limits = ServoLimits(
            max_heading=float(args.max_heading),
            max_lateral=float(args.max_lateral),
            max_turning=float(args.max_turning),
            min_command=float(args.min_command),
        )
        self._lock = threading.Lock()
        self._state = STATE_IDLE
        self._message = "Waiting for operator"
        self._requested_mode: Optional[str] = None
        self._observed_mode: Optional[str] = None
        self._motion_publishing = False
        self._latest_tag: Optional[TagPose] = None
        self._latest_tags: list[TagPose] = []
        self._latest_odom: Optional[OdomPose] = None
        self._latest_imu: Optional[ImuQuat] = None
        self._latest_command = _zero_command()
        self._recent_points: deque[dict] = deque(maxlen=RECENT_TRAJECTORY_POINTS)
        self._last_status: dict = {}
        self._record_video = bool(args.record_video)
        self._video_topic = str(args.video_topic)
        self._last_video_error_logged: Optional[str] = None

        self._movement = MovementController(rate_hz=max(10, int(args.publish_rate_hz)))
        self._modes = ModeController(movement_controller=self._movement)
        self._recorder = TrajectoryRecorder(
            args.record_dir,
            archive_root=args.archive_dir,
            record_video=self._record_video,
            video_fps=float(args.video_fps),
        )

        self._status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)
        self._trajectory_pub = rospy.Publisher(TRAJECTORY_TOPIC, String, queue_size=1, latch=True)

        rospy.Subscriber(COMMAND_TOPIC, String, self._command_cb, queue_size=10, tcp_nodelay=True)
        rospy.Subscriber(MODE_GOAL_TOPIC, SwitchOperationalModeActionGoal, self._mode_goal_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(APRILTAG_DETECTIONS_TOPIC, String, self._tag_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(ODOM_TOPIC, PoseWithCovarianceStamped, self._odom_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(OAKD_IMU_QUATERNION_TOPIC, QuaternionStamped, self._imu_cb, queue_size=1, tcp_nodelay=True)
        if self._record_video:
            rospy.Subscriber(self._video_topic, CompressedImage, self._rgb_cb, queue_size=1, tcp_nodelay=True)

    def _command_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            command = str(payload.get("command", "")).strip().lower()
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Bad egocentric servo command JSON: %s", exc)
            return

        with self._lock:
            try:
                rospy.loginfo("Egocentric servo command received: %s", payload)
                if command == "mode":
                    mode = str(payload.get("mode", "")).strip()
                    self._request_mode_locked(mode)
                elif command == "arm":
                    self._arm_locked()
                elif command == "start":
                    self._start_locked()
                elif command == "pause":
                    self._pause_locked()
                elif command == "resume":
                    self._resume_locked()
                elif command == "stop":
                    self._stop_locked("Stopped by operator")
                elif command == "select_tag":
                    self._select_tag_locked(payload.get("tag_id"))
                else:
                    self._message = f"Unknown command: {command}"
                    rospy.logwarn("Unknown egocentric servo command: %s", command)
            except Exception as exc:
                self._halt_motion_locked(publish_zero=True)
                self._state = STATE_FAULT
                self._message = f"Command {command} failed: {exc}"
                rospy.logerr("Egocentric servo command %s failed: %s", command, exc)

    def _tag_cb(self, msg: String) -> None:
        try:
            tags = parse_tag_detections_json(msg.data, target_tag_id=self._target_tag_id)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse AprilTag detections: %s", exc)
            return
        with self._lock:
            self._latest_tags = tags
            if tags:
                self._latest_tag = tags[0]

    def _rgb_cb(self, msg: CompressedImage) -> None:
        if not self._recorder.active:
            return
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("OpenCV failed to decode compressed RGB frame")
            stamp_sec = float(msg.header.stamp.to_sec()) if msg.header.stamp else rospy.get_time()
            video_error = self._recorder.write_video_frame(stamp_sec=stamp_sec, frame_bgr=frame)
            if video_error and video_error != self._last_video_error_logged:
                self._last_video_error_logged = video_error
                rospy.logwarn("Egocentric servo video recording failed: %s", video_error)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "Egocentric servo RGB video frame skipped: %s", exc)

    def _odom_cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._latest_odom = OdomPose(
                stamp_sec=float(msg.header.stamp.to_sec()) if msg.header.stamp else rospy.get_time(),
                x=float(pose.position.x),
                y=float(pose.position.y),
                yaw=float(yaw),
            )

    def _imu_cb(self, msg: QuaternionStamped) -> None:
        quat = msg.quaternion
        with self._lock:
            self._latest_imu = ImuQuat(
                stamp_sec=float(msg.header.stamp.to_sec()) if msg.header.stamp else rospy.get_time(),
                x=float(quat.x),
                y=float(quat.y),
                z=float(quat.z),
                w=float(quat.w),
            )

    def _mode_goal_cb(self, msg: SwitchOperationalModeActionGoal) -> None:
        mode = str(msg.goal.target.name).strip()
        if mode not in ModeController.VALID_MODES:
            return
        with self._lock:
            self._observed_mode = mode
            if mode != ModeController.WALK and self._state == STATE_TRACKING:
                self._halt_motion_locked(publish_zero=True)
                self._state = STATE_PAUSED
                self._message = f"Paused: observed mode request {mode}"

    def _request_mode_locked(self, mode: str) -> None:
        if mode not in ModeController.VALID_MODES:
            self._message = f"Invalid mode request: {mode}"
            return
        was_tracking = self._state == STATE_TRACKING
        self._halt_motion_locked(publish_zero=was_tracking)
        self._modes.switch_mode(mode)
        self._requested_mode = mode
        self._observed_mode = mode
        if was_tracking:
            self._state = STATE_PAUSED
        self._message = f"Requested {mode}"

    def _arm_locked(self) -> None:
        if self._state == STATE_TRACKING:
            self._halt_motion_locked(publish_zero=True)
            self._state = STATE_PAUSED
            self._message = "Paused active trajectory; press Resume or Stop"
            return
        self._halt_motion_locked(publish_zero=False)
        if not self._walk_mode_locked():
            self._message = "Switch to Walk before Arm"
            return
        self._state = STATE_ARMED
        self._message = "Armed; press Start Trajectory to move"

    def _start_locked(self) -> None:
        now_sec = rospy.get_time()
        if self._state != STATE_ARMED:
            self._halt_motion_locked(publish_zero=False)
            self._message = "Press Arm before Start Trajectory"
            return
        if not self._walk_mode_locked():
            self._halt_motion_locked(publish_zero=False)
            self._message = f"Cannot start: active mode is {self._active_mode_locked() or 'unknown'}, expected Walk"
            return
        if not self._tag_fresh_locked(now_sec):
            self._halt_motion_locked(publish_zero=False)
            age = now_sec - self._latest_tag.stamp_sec if self._latest_tag else None
            self._message = "Cannot start: no fresh AprilTag pose" if age is None else f"Cannot start: AprilTag stale ({age:.2f}s)"
            return
        if not self._recorder.active:
            run_dir = self._recorder.start(
                odom=self._latest_odom,
                tag=self._latest_tag,
                imu=self._latest_imu,
            )
            self._last_video_error_logged = None
            rospy.loginfo("Egocentric servo recording: %s", run_dir)
        self._ensure_motion_publisher_locked()
        self._state = STATE_TRACKING
        self._message = "Tracking AprilTag"

    def _pause_locked(self) -> None:
        self._halt_motion_locked(publish_zero=self._state == STATE_TRACKING)
        if self._state in {STATE_TRACKING, STATE_PAUSED_LOST_TAG}:
            self._state = STATE_PAUSED
        self._message = "Paused by operator"

    def _resume_locked(self) -> None:
        now_sec = rospy.get_time()
        if self._state not in {STATE_PAUSED, STATE_PAUSED_LOST_TAG}:
            self._message = f"Resume ignored from {self._state}"
            return
        if not self._walk_mode_locked():
            self._halt_motion_locked(publish_zero=False)
            self._message = "Cannot resume: robot is not in Walk mode"
            return
        if not self._tag_fresh_locked(now_sec):
            self._halt_motion_locked(publish_zero=False)
            self._state = STATE_PAUSED_LOST_TAG
            self._message = "Cannot resume: no fresh AprilTag pose"
            return
        self._ensure_motion_publisher_locked()
        self._state = STATE_TRACKING
        self._message = "Resumed tracking"

    def _stop_locked(self, message: str) -> None:
        self._halt_motion_locked(publish_zero=self._state == STATE_TRACKING)
        self._latest_command = _zero_command()
        archive_dir = self._recorder.stop()
        self._state = STATE_STOPPED
        if archive_dir is not None:
            self._message = f"{message}; archived to {archive_dir}"
        elif self._recorder.archive_error:
            self._message = f"{message}; archive failed: {self._recorder.archive_error}"
        else:
            self._message = message

    def _select_tag_locked(self, tag_id_value: object) -> None:
        try:
            self._target_tag_id = int(tag_id_value) if tag_id_value not in {None, ""} else None
        except (TypeError, ValueError):
            self._message = f"Invalid tag id: {tag_id_value}"
            return
        self._latest_tag = None
        self._latest_tags = []
        self._message = f"Selected tag {self._target_tag_id}" if self._target_tag_id is not None else "Selected best visible tag"

    def _active_mode_locked(self) -> Optional[str]:
        return self._observed_mode or self._requested_mode

    def _walk_mode_locked(self) -> bool:
        return self._active_mode_locked() == ModeController.WALK

    def _ensure_motion_publisher_locked(self) -> None:
        if self._motion_publishing:
            return
        self._movement.start()
        self._motion_publishing = True

    def _halt_motion_locked(self, *, publish_zero: bool) -> None:
        self._movement.stop()
        if not self._motion_publishing:
            return
        if publish_zero:
            self._movement.publish_once()
        self._movement.shutdown()
        self._motion_publishing = False

    def _tag_fresh_locked(self, now_sec: float) -> bool:
        return (
            self._latest_tag is not None
            and math.isfinite(self._latest_tag.forward_m)
            and self._latest_tag.forward_m > 0.05
            and (now_sec - self._latest_tag.stamp_sec) <= self._tag_timeout_sec
        )

    def _freshness_locked(self, now_sec: float) -> dict[str, object]:
        tag_age = now_sec - self._latest_tag.stamp_sec if self._latest_tag else None
        odom_age = now_sec - self._latest_odom.stamp_sec if self._latest_odom else None
        imu_age = now_sec - self._latest_imu.stamp_sec if self._latest_imu else None
        return {
            "tag_age_sec": tag_age,
            "tag_fresh": tag_age is not None and tag_age <= self._tag_timeout_sec,
            "tag_loss_pause_sec": self._tag_loss_pause_sec,
            "odom_age_sec": odom_age,
            "odom_fresh": odom_age is not None and odom_age <= self._odom_timeout_sec,
            "imu_age_sec": imu_age,
            "imu_fresh": imu_age is not None and imu_age <= self._imu_timeout_sec,
        }

    def spin(self) -> None:
        rate = rospy.Rate(float(rospy.get_param("~loop_hz", 20.0)))
        try:
            while not rospy.is_shutdown():
                self._step()
                rate.sleep()
        finally:
            with self._lock:
                self._halt_motion_locked(publish_zero=True)
            self._recorder.stop()

    def _step(self) -> None:
        now_sec = rospy.get_time()
        with self._lock:
            command = _zero_command()

            if self._state == STATE_PAUSED_LOST_TAG and self._tag_fresh_locked(now_sec):
                if self._walk_mode_locked():
                    self._state = STATE_TRACKING
                    self._message = "Reacquired AprilTag; resumed tracking"
                else:
                    self._message = "AprilTag reacquired; waiting for Walk mode"

            if self._state == STATE_TRACKING:
                if not self._tag_fresh_locked(now_sec):
                    self._halt_motion_locked(publish_zero=True)
                    tag_age = now_sec - self._latest_tag.stamp_sec if self._latest_tag else None
                    if tag_age is None or tag_age > self._tag_loss_pause_sec:
                        self._state = STATE_PAUSED_LOST_TAG
                        self._message = "Paused: lost AprilTag pose for more than %.1fs" % self._tag_loss_pause_sec
                    else:
                        self._message = "Holding: AprilTag stale for %.2fs; will auto-resume" % tag_age
                else:
                    command = compute_servo_command(
                        self._latest_tag,
                        target_distance_m=self._target_distance_m,
                        limits=self._limits,
                    )
                    if command.target_reached:
                        self._halt_motion_locked(publish_zero=True)
                        self._state = STATE_TARGET_REACHED
                        self._message = "Target reached"
                    else:
                        self._ensure_motion_publisher_locked()
                        self._movement.set_velocity(
                            heading=command.heading,
                            lateral=command.lateral,
                            turning=command.turning,
                        )
                        self._message = "Tracking AprilTag"
            elif self._state in {STATE_PAUSED, STATE_PAUSED_LOST_TAG, STATE_IDLE, STATE_ARMED, STATE_STOPPED, STATE_TARGET_REACHED}:
                if self._motion_publishing:
                    self._halt_motion_locked(publish_zero=True)

            self._latest_command = command
            self._record_locked(now_sec, command)
            self._publish_locked(now_sec)

    def _record_locked(self, now_sec: float, command: ServoCommand) -> None:
        if not self._recorder.active:
            return
        self._recorder.write_sample(
            stamp_sec=now_sec,
            state=self._state,
            message=self._message,
            tag=self._latest_tag,
            odom=self._latest_odom,
            imu=self._latest_imu,
            command=command,
            requested_mode=self._requested_mode,
        )
        self._recent_points.append(
            {
                "t": now_sec,
                "state": self._state,
                "tag_range_m": self._latest_tag.range_m if self._latest_tag else None,
                "tag_x_camera_m": float(self._latest_tag.position_camera_m[0]) if self._latest_tag else None,
                "tag_z_camera_m": float(self._latest_tag.position_camera_m[2]) if self._latest_tag else None,
                "tag_face_yaw_error_rad": self._latest_tag.face_yaw_error_rad if self._latest_tag else None,
                "tag_face_normal_x_camera": self._latest_tag.face_normal_camera[0] if self._latest_tag and self._latest_tag.face_normal_camera else None,
                "tag_face_normal_z_camera": self._latest_tag.face_normal_camera[2] if self._latest_tag and self._latest_tag.face_normal_camera else None,
                "odom_x": self._latest_odom.x if self._latest_odom else None,
                "odom_y": self._latest_odom.y if self._latest_odom else None,
                "cmd_heading": command.heading,
                "cmd_lateral": command.lateral,
                "cmd_turning": command.turning,
            }
        )

    def _publish_locked(self, now_sec: float) -> None:
        tag = self._latest_tag
        status = {
            "stamp_sec": now_sec,
            "state": self._state,
            "message": self._message,
            "apriltag_tag_length_m": APRILTAG_TAG_LENGTH_M,
            "target_distance_m": self._target_distance_m,
            "target_tag_id": self._target_tag_id,
            "requested_mode": self._requested_mode,
            "observed_mode": self._observed_mode,
            "active_mode": self._active_mode_locked(),
            "motion_publishing": self._motion_publishing,
            "freshness": self._freshness_locked(now_sec),
            "tag": _tag_status(tag),
            "command": {
                "heading": self._latest_command.heading,
                "lateral": self._latest_command.lateral,
                "turning": self._latest_command.turning,
                "range_error_m": self._latest_command.range_error_m,
                "lateral_error_m": self._latest_command.lateral_error_m,
                "yaw_error_rad": self._latest_command.yaw_error_rad,
                "face_yaw_error_rad": self._latest_command.face_yaw_error_rad,
                "face_blend": self._latest_command.face_blend,
                "target_reached": self._latest_command.target_reached,
            },
            "recording": {
                "active": self._recorder.active,
                "run_dir": str(self._recorder.run_dir) if self._recorder.run_dir else None,
                "archive_dir": str(self._recorder.archive_dir) if self._recorder.archive_dir else None,
                "archive_error": self._recorder.archive_error,
                "sample_count": self._recorder.sample_count,
                "video": self._recorder.video_status(),
            },
        }
        self._last_status = status
        self._status_pub.publish(String(data=json.dumps(status, separators=(",", ":"), sort_keys=True)))
        trajectory = {
            "stamp_sec": now_sec,
            "recording": status["recording"],
            "points": list(self._recent_points),
        }
        self._trajectory_pub.publish(String(data=json.dumps(trajectory, separators=(",", ":"), sort_keys=True)))


def _tag_status(tag: Optional[TagPose]) -> Optional[dict]:
    if tag is None:
        return None
    return {
        "id": tag.tag_id,
        "stamp_sec": tag.stamp_sec,
        "decision_margin": tag.decision_margin,
        "tag_size_m": tag.tag_size_m,
        "pose_t_camera_m": [float(item) for item in tag.position_camera_m],
        "range_m": tag.range_m,
        "forward_m": tag.forward_m,
        "lateral_right_m": tag.lateral_right_m,
        "bearing_rad": tag.bearing_rad,
        "face_yaw_error_rad": tag.face_yaw_error_rad,
        "face_normal_camera": list(tag.face_normal_camera) if tag.face_normal_camera else None,
    }


def _zero_command() -> ServoCommand:
    return ServoCommand(
        heading=0.0,
        lateral=0.0,
        turning=0.0,
        range_error_m=0.0,
        lateral_error_m=0.0,
        yaw_error_rad=0.0,
        face_yaw_error_rad=None,
        face_blend=0.0,
        target_reached=False,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ANYmal egocentric AprilTag visual servo node")
    parser.add_argument("--target-tag-id", type=int, default=None, help="Target tag ID; default uses best visible tag")
    parser.add_argument("--target-distance-m", type=float, default=DEFAULT_TARGET_DISTANCE_M)
    parser.add_argument("--record-dir", default=DEFAULT_RECORD_DIR)
    parser.add_argument("--archive-dir", default=DEFAULT_ARCHIVE_DIR)
    parser.add_argument("--loop-hz", type=float, default=DEFAULT_LOOP_HZ)
    parser.add_argument("--publish-rate-hz", type=float, default=20.0)
    parser.add_argument("--no-record-video", dest="record_video", action="store_false", help="Disable RGB MP4 recording")
    parser.set_defaults(record_video=True)
    parser.add_argument("--video-fps", type=float, default=30.0, help="RGB MP4 recording frame rate")
    parser.add_argument("--video-topic", default=RGB_COMPRESSED_TOPIC, help="Compressed RGB topic to record")
    parser.add_argument("--tag-timeout-sec", type=float, default=DEFAULT_TAG_TIMEOUT_SEC)
    parser.add_argument("--tag-loss-pause-sec", type=float, default=DEFAULT_TAG_LOSS_PAUSE_SEC)
    parser.add_argument("--odom-timeout-sec", type=float, default=DEFAULT_ODOM_TIMEOUT_SEC)
    parser.add_argument("--imu-timeout-sec", type=float, default=DEFAULT_IMU_TIMEOUT_SEC)
    parser.add_argument("--max-heading", type=float, default=DEFAULT_MAX_HEADING)
    parser.add_argument("--max-lateral", type=float, default=DEFAULT_MAX_LATERAL)
    parser.add_argument("--max-turning", type=float, default=DEFAULT_MAX_TURNING)
    parser.add_argument("--min-command", type=float, default=DEFAULT_MIN_COMMAND)
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(rospy.myargv(argv)[1:] if argv is not None else rospy.myargv()[1:])
    rospy.init_node("anymal_egocentric_servo", anonymous=False)
    rospy.set_param("~loop_hz", float(args.loop_hz))
    node = EgocentricServoNode(args)
    rospy.loginfo(
        "ANYmal egocentric servo ready: tag_length=%.5fm target_distance=%.2fm command_topic=%s",
        APRILTAG_TAG_LENGTH_M,
        args.target_distance_m,
        COMMAND_TOPIC,
    )
    node.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
