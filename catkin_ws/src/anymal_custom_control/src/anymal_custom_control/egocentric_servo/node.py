"""ROS node for ANYmal MPC AprilTag visual servoing."""

from __future__ import annotations

import argparse
import json
import math
import threading
from collections import deque
from typing import Optional

import cv2
import numpy as np

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from anymal_custom_control import ModeController, MovementController
from anymal_custom_control.modes import DEFAULT_TOPIC as MODE_GOAL_TOPIC
from operational_mode_manager_msgs.msg import SwitchOperationalModeActionGoal

from .constants import (
    APRILTAG_DETECTIONS_TOPIC,
    APRILTAG_TAG_LENGTH_M,
    COMMAND_TOPIC,
    DEFAULT_ARCHIVE_DIR,
    DEFAULT_LOOP_HZ,
    DEFAULT_ODOM_TIMEOUT_SEC,
    DEFAULT_RECORD_DIR,
    DEFAULT_TAG_LOSS_PAUSE_SEC,
    DEFAULT_TAG_TIMEOUT_SEC,
    DEFAULT_TARGET_DISTANCE_M,
    EGOCENTRIC_CAMERA_ROTATE_180,
    MPC_DT_SEC,
    MPC_FILTER_WINDOW,
    MPC_HORIZON,
    MPC_LOOP_HZ,
    MPC_MAX_SOLVE_TIME_SEC,
    MPC_MAX_OPEN_LOOP_STEPS,
    MPC_STATE_MAX_POSITION_STEP_M,
    MPC_STATE_MAX_THETA_STEP_RAD,
    MPC_STATE_SMOOTH_ALPHA,
    MPC_START_MIN_SAMPLES,
    MPC_START_SAMPLE_SEC,
    ODOM_TOPIC,
    RECENT_TRAJECTORY_POINTS,
    RGB_COMPRESSED_TOPIC,
    STATUS_TOPIC,
    TRAJECTORY_TOPIC,
)
from .messages import (
    OdomPose,
    TagPose,
    parse_tag_detections_json,
    rotate_tag_pose_camera_180,
    wrap_angle_rad,
    yaw_from_quaternion,
)
from .mpc import (
    MpcCommand,
    MpcState,
    command_from_world_control,
    median_mpc_state,
    odom_to_mpc_state,
    solve_mpc_command,
    solve_overran_budget,
    tag_pose_to_mpc_state,
    zero_mpc_command,
)
from .recorder import TrajectoryRecorder


STATE_IDLE = "IDLE"
STATE_ARMED = "ARMED"
STATE_INITIALIZING = "INITIALIZING"
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
        self._lock = threading.Lock()
        self._state = STATE_IDLE
        self._message = "Waiting for operator"
        self._requested_mode: Optional[str] = None
        self._observed_mode: Optional[str] = None
        self._motion_publishing = False
        self._latest_tag: Optional[TagPose] = None
        self._latest_tags: list[TagPose] = []
        self._tag_buffer: deque[TagPose] = deque(maxlen=120)
        self._latest_odom: Optional[OdomPose] = None
        self._latest_command: MpcCommand = zero_mpc_command()
        self._recent_points: deque[dict] = deque(maxlen=RECENT_TRAJECTORY_POINTS)
        self._last_status: dict = {}
        self._record_video = bool(args.record_video)
        self._video_topic = str(args.video_topic)
        self._last_video_error_logged: Optional[str] = None

        self._init_start_sec: Optional[float] = None
        self._init_end_sec: Optional[float] = None
        self._origin_odom: Optional[OdomPose] = None
        self._origin_tag: Optional[TagPose] = None
        self._origin_mpc_state: Optional[MpcState] = None
        self._current_mpc_state: Optional[MpcState] = None
        self._current_odom_mpc_state: Optional[MpcState] = None
        self._last_mpc_tick_sec: Optional[float] = None
        self._last_world_command = np.zeros(3, dtype=float)
        self._plan_states: list[list[float]] = []
        self._plan_controls: list[list[float]] = []
        self._plan_next_index = 0
        self._open_loop_step = 0

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
                rospy.loginfo("Egocentric MPC servo command received: %s", payload)
                if command == "mode":
                    self._request_mode_locked(str(payload.get("mode", "")).strip())
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
            if EGOCENTRIC_CAMERA_ROTATE_180:
                tags = [rotate_tag_pose_camera_180(tag) for tag in tags]
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse AprilTag detections: %s", exc)
            return
        with self._lock:
            self._latest_tags = tags
            if tags:
                self._latest_tag = tags[0]
                self._tag_buffer.append(tags[0])

    def _rgb_cb(self, msg: CompressedImage) -> None:
        if not self._recorder.active:
            return
        try:
            np_arr = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("OpenCV failed to decode compressed RGB frame")
            if EGOCENTRIC_CAMERA_ROTATE_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
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

    def _mode_goal_cb(self, msg: SwitchOperationalModeActionGoal) -> None:
        mode = str(msg.goal.target.name).strip()
        if mode not in ModeController.VALID_MODES:
            return
        with self._lock:
            self._observed_mode = mode
            if mode != ModeController.WALK and self._state in {STATE_INITIALIZING, STATE_TRACKING}:
                self._halt_motion_locked(publish_zero=True)
                self._state = STATE_PAUSED
                self._message = f"Paused: observed mode request {mode}"

    def _request_mode_locked(self, mode: str) -> None:
        if mode not in ModeController.VALID_MODES:
            self._message = f"Invalid mode request: {mode}"
            return
        was_moving = self._state == STATE_TRACKING
        self._halt_motion_locked(publish_zero=was_moving)
        self._modes.switch_mode(mode)
        self._requested_mode = mode
        self._observed_mode = mode
        if was_moving:
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
        self._reset_mpc_run_locked()
        self._state = STATE_ARMED
        self._message = "Armed; press Start Trajectory to initialize MPC"

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
            run_dir = self._recorder.start()
            self._last_video_error_logged = None
            rospy.loginfo("Egocentric MPC servo recording: %s", run_dir)
        self._init_start_sec = now_sec
        self._init_end_sec = now_sec + MPC_START_SAMPLE_SEC
        self._state = STATE_INITIALIZING
        self._message = "Collecting AprilTag pose samples for MPC initialization"

    def _pause_locked(self) -> None:
        self._halt_motion_locked(publish_zero=self._state == STATE_TRACKING)
        if self._state in {STATE_INITIALIZING, STATE_TRACKING, STATE_PAUSED_LOST_TAG}:
            self._state = STATE_PAUSED
        self._message = "Paused by operator"

    def _resume_locked(self) -> None:
        now_sec = rospy.get_time()
        if self._state not in {STATE_PAUSED, STATE_PAUSED_LOST_TAG}:
            self._message = f"Resume ignored from {self._state}"
            return
        if self._origin_mpc_state is None or self._origin_odom is None:
            self._message = "Cannot resume: MPC origin is not initialized"
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
        self._last_mpc_tick_sec = None
        self._open_loop_step = 0
        self._state = STATE_TRACKING
        self._message = "Resumed MPC tracking"

    def _stop_locked(self, message: str) -> None:
        self._halt_motion_locked(publish_zero=self._state == STATE_TRACKING)
        self._latest_command = zero_mpc_command(command_source="stop")
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
        self._tag_buffer.clear()
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

    def _fresh_tags_locked(self, now_sec: float, *, limit: int = MPC_FILTER_WINDOW) -> list[TagPose]:
        fresh = [
            tag
            for tag in self._tag_buffer
            if (now_sec - tag.stamp_sec) <= self._tag_timeout_sec
            and math.isfinite(tag.forward_m)
            and tag.forward_m > 0.05
            and tag_pose_to_mpc_state(tag) is not None
        ]
        return fresh[-limit:]

    def _freshness_locked(self, now_sec: float) -> dict[str, object]:
        tag_age = now_sec - self._latest_tag.stamp_sec if self._latest_tag else None
        odom_age = now_sec - self._latest_odom.stamp_sec if self._latest_odom else None
        return {
            "tag_age_sec": tag_age,
            "tag_fresh": tag_age is not None and tag_age <= self._tag_timeout_sec,
            "tag_loss_pause_sec": self._tag_loss_pause_sec,
            "odom_age_sec": odom_age,
            "odom_fresh": odom_age is not None and odom_age <= self._odom_timeout_sec,
        }

    def spin(self) -> None:
        rate = rospy.Rate(float(rospy.get_param("~loop_hz", DEFAULT_LOOP_HZ)))
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
            if self._state == STATE_INITIALIZING:
                self._finish_initialization_if_ready_locked(now_sec)
            if self._state == STATE_TRACKING and self._mpc_tick_due_locked(now_sec):
                self._run_mpc_tick_locked(now_sec)
            elif self._state in {STATE_PAUSED, STATE_PAUSED_LOST_TAG, STATE_IDLE, STATE_ARMED, STATE_STOPPED, STATE_TARGET_REACHED}:
                if self._motion_publishing:
                    self._halt_motion_locked(publish_zero=True)
            self._publish_locked(now_sec)

    def _finish_initialization_if_ready_locked(self, now_sec: float) -> None:
        if self._init_end_sec is None or now_sec < self._init_end_sec:
            count = self._initialization_sample_count_locked(now_sec)
            self._message = f"Collecting AprilTag pose samples: {count}/{MPC_START_MIN_SAMPLES}"
            return
        samples = self._initialization_samples_locked(now_sec)
        median_state = median_mpc_state(samples)
        if len(samples) < MPC_START_MIN_SAMPLES or median_state is None:
            self._recorder.stop()
            self._state = STATE_ARMED
            self._message = f"Initialization failed: only {len(samples)} valid tag samples"
            return
        if self._latest_odom is None:
            self._recorder.stop()
            self._state = STATE_ARMED
            self._message = "Initialization failed: no legged odometry pose"
            return
        self._origin_tag = samples[-1]
        self._origin_odom = self._latest_odom
        self._origin_mpc_state = median_state
        self._current_mpc_state = median_state
        self._current_odom_mpc_state = odom_to_mpc_state(self._latest_odom, self._origin_odom, self._origin_mpc_state)
        self._recorder.set_origin(odom=self._origin_odom, tag=self._origin_tag, mpc_state=self._origin_mpc_state)
        self._last_world_command = np.zeros(3, dtype=float)
        self._plan_states = []
        self._plan_controls = []
        self._plan_next_index = 0
        self._open_loop_step = 0
        self._last_mpc_tick_sec = None
        self._ensure_motion_publisher_locked()
        self._state = STATE_TRACKING
        self._message = "MPC tracking AprilTag"

    def _initialization_samples_locked(self, now_sec: float) -> list[TagPose]:
        if self._init_start_sec is None:
            return []
        return [
            tag
            for tag in self._tag_buffer
            if self._init_start_sec <= tag.stamp_sec <= now_sec
            and math.isfinite(tag.forward_m)
            and tag.forward_m > 0.05
            and tag_pose_to_mpc_state(tag) is not None
        ]

    def _initialization_sample_count_locked(self, now_sec: float) -> int:
        return len(self._initialization_samples_locked(now_sec))

    def _mpc_tick_due_locked(self, now_sec: float) -> bool:
        return self._last_mpc_tick_sec is None or (now_sec - self._last_mpc_tick_sec) >= MPC_DT_SEC

    def _run_mpc_tick_locked(self, now_sec: float) -> None:
        self._last_mpc_tick_sec = now_sec
        command = zero_mpc_command()
        predicted_states: list[list[float]] = list(self._plan_states)
        predicted_controls: list[list[float]] = list(self._plan_controls)
        tag_fresh = False
        fresh_tags = self._fresh_tags_locked(now_sec)
        filtered_state = median_mpc_state(fresh_tags)
        if filtered_state is not None:
            tag_fresh = True
            solve_state = self._smooth_mpc_state_locked(filtered_state)
            solve_target_distance_m = self._target_distance_m
            solve_u_prev_world = self._last_world_command.copy()
            self._lock.release()
            try:
                solve_result = solve_mpc_command(
                    state=solve_state,
                    target_distance_m=solve_target_distance_m,
                    u_prev_world=solve_u_prev_world,
                )
            finally:
                self._lock.acquire()
            if self._state != STATE_TRACKING:
                return
            if solve_result is None:
                record_sec = rospy.get_time()
                self._halt_motion_locked(publish_zero=True)
                self._state = STATE_PAUSED
                self._latest_command = zero_mpc_command(solver_status="solver_failed", command_source="solver_failed")
                self._message = "Paused: MPC solver failed"
                self._record_locked(record_sec, tag_fresh, solve_state, self._latest_command, predicted_states, predicted_controls)
                return
            command = solve_result.command
            if solve_overran_budget(command):
                record_sec = rospy.get_time()
                self._halt_motion_locked(publish_zero=True)
                self._state = STATE_PAUSED
                self._latest_command = zero_mpc_command(
                    solver_status="solve_overrun",
                    command_source="solve_overrun",
                    solve_time_ms=command.solve_time_ms,
                )
                self._message = f"Paused: MPC solve overran {command.solve_time_ms:.0f} ms budget"
                self._record_locked(record_sec, tag_fresh, solve_state, self._latest_command, predicted_states, predicted_controls)
                return
            predicted_states = solve_result.predicted_states
            predicted_controls = solve_result.predicted_controls
            self._plan_states = predicted_states
            self._plan_controls = predicted_controls
            self._plan_next_index = 1
            self._open_loop_step = 0
            self._current_mpc_state = solve_state
        elif self._can_execute_open_loop_locked():
            index = self._plan_next_index
            state_estimate = MpcState(*self._plan_states[index]) if index < len(self._plan_states) else self._current_mpc_state
            if state_estimate is None:
                self._pause_for_lost_tag_locked()
                record_sec = rospy.get_time()
                self._record_locked(record_sec, False, self._current_mpc_state, self._latest_command, predicted_states, predicted_controls)
                return
            command = command_from_world_control(
                state=state_estimate,
                target_distance_m=self._target_distance_m,
                u_world=np.asarray(self._plan_controls[index], dtype=float),
                solver_status="stale_tag_open_loop",
                solve_time_ms=0.0,
                command_source="open_loop_plan",
                open_loop_step=self._open_loop_step + 1,
            )
            self._open_loop_step += 1
            self._plan_next_index += 1
            self._current_mpc_state = state_estimate
        else:
            self._pause_for_lost_tag_locked()
            record_sec = rospy.get_time()
            self._record_locked(record_sec, False, self._current_mpc_state, self._latest_command, predicted_states, predicted_controls)
            return

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
            self._message = "MPC tracking AprilTag" if tag_fresh else f"MPC open-loop step {command.open_loop_step}"

        self._latest_command = command
        self._last_world_command = np.asarray(command.u_world, dtype=float)
        record_sec = rospy.get_time()
        self._record_locked(record_sec, tag_fresh, self._current_mpc_state, command, predicted_states, predicted_controls)

    def _can_execute_open_loop_locked(self) -> bool:
        return (
            self._open_loop_step < MPC_MAX_OPEN_LOOP_STEPS
            and self._plan_controls
            and self._plan_next_index < len(self._plan_controls)
        )

    def _smooth_mpc_state_locked(self, measurement: MpcState) -> MpcState:
        previous = self._current_mpc_state
        if previous is None:
            return measurement

        dx = float(measurement.x - previous.x)
        dy = float(measurement.y - previous.y)
        distance = math.hypot(dx, dy)
        if distance > MPC_STATE_MAX_POSITION_STEP_M > 0.0:
            scale = MPC_STATE_MAX_POSITION_STEP_M / distance
            dx *= scale
            dy *= scale

        dtheta = wrap_angle_rad(float(measurement.theta - previous.theta))
        if abs(dtheta) > MPC_STATE_MAX_THETA_STEP_RAD:
            dtheta = math.copysign(MPC_STATE_MAX_THETA_STEP_RAD, dtheta)

        alpha = min(1.0, max(0.0, float(MPC_STATE_SMOOTH_ALPHA)))
        return MpcState(
            x=float(previous.x + alpha * dx),
            y=float(previous.y + alpha * dy),
            theta=wrap_angle_rad(float(previous.theta + alpha * dtheta)),
        )

    def _pause_for_lost_tag_locked(self) -> None:
        self._halt_motion_locked(publish_zero=True)
        self._latest_command = zero_mpc_command(solver_status="stale_tag", command_source="stale_tag")
        self._state = STATE_PAUSED_LOST_TAG
        self._message = f"Paused: no fresh AprilTag pose after {MPC_MAX_OPEN_LOOP_STEPS} open-loop MPC step(s)"

    def _record_locked(
        self,
        now_sec: float,
        tag_fresh: bool,
        mpc_state: Optional[MpcState],
        command: MpcCommand,
        predicted_states: list[list[float]],
        predicted_controls: list[list[float]],
    ) -> None:
        self._current_odom_mpc_state = odom_to_mpc_state(self._latest_odom, self._origin_odom, self._origin_mpc_state)
        if self._recorder.active:
            self._recorder.write_sample(
                stamp_sec=now_sec,
                state=self._state,
                message=self._message,
                tag_fresh=tag_fresh,
                tag=self._latest_tag,
                mpc_state=mpc_state,
                odom=self._latest_odom,
                odom_mpc_state=self._current_odom_mpc_state,
                command=command,
                predicted_states=predicted_states,
                predicted_controls=predicted_controls,
                requested_mode=self._requested_mode,
            )
        self._recent_points.append(
            {
                "t": now_sec,
                "state": self._state,
                "tag_range_m": self._latest_tag.range_m if self._latest_tag else None,
                "mpc_x_tag_m": mpc_state.x if mpc_state else None,
                "mpc_y_tag_m": mpc_state.y if mpc_state else None,
                "odom_mpc_x_tag_m": self._current_odom_mpc_state.x if self._current_odom_mpc_state else None,
                "odom_mpc_y_tag_m": self._current_odom_mpc_state.y if self._current_odom_mpc_state else None,
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
            "mpc": {
                "dt_sec": MPC_DT_SEC,
                "loop_hz": MPC_LOOP_HZ,
                "horizon": MPC_HORIZON,
                "max_solve_time_sec": MPC_MAX_SOLVE_TIME_SEC,
                "max_open_loop_steps": MPC_MAX_OPEN_LOOP_STEPS,
                "filter_window": MPC_FILTER_WINDOW,
                "state_smooth_alpha": MPC_STATE_SMOOTH_ALPHA,
                "state_max_position_step_m": MPC_STATE_MAX_POSITION_STEP_M,
                "state_max_theta_step_rad": MPC_STATE_MAX_THETA_STEP_RAD,
                "current_state": _mpc_state_status(self._current_mpc_state),
                "odom_state": _mpc_state_status(self._current_odom_mpc_state),
                "origin_state": _mpc_state_status(self._origin_mpc_state),
                "predicted_states": self._plan_states,
                "predicted_controls": self._plan_controls,
            },
            "command": _command_status(self._latest_command),
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

    def _reset_mpc_run_locked(self) -> None:
        self._init_start_sec = None
        self._init_end_sec = None
        self._origin_odom = None
        self._origin_tag = None
        self._origin_mpc_state = None
        self._current_mpc_state = None
        self._current_odom_mpc_state = None
        self._last_mpc_tick_sec = None
        self._last_world_command = np.zeros(3, dtype=float)
        self._plan_states = []
        self._plan_controls = []
        self._plan_next_index = 0
        self._open_loop_step = 0
        self._latest_command = zero_mpc_command()


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
    }


def _mpc_state_status(state: Optional[MpcState]) -> Optional[dict]:
    if state is None:
        return None
    return {"x": state.x, "y": state.y, "theta": state.theta}


def _command_status(command: MpcCommand) -> dict:
    return {
        "heading": command.heading,
        "lateral": command.lateral,
        "turning": command.turning,
        "vx_body_mps": command.vx_body_mps,
        "vy_body_mps": command.vy_body_mps,
        "omega_radps": command.omega_radps,
        "u_world": list(command.u_world),
        "range_error_m": command.range_error_m,
        "lateral_error_m": command.lateral_error_m,
        "yaw_error_rad": command.yaw_error_rad,
        "solver_status": command.solver_status,
        "solve_time_ms": command.solve_time_ms,
        "command_source": command.command_source,
        "open_loop_step": command.open_loop_step,
        "target_reached": command.target_reached,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ANYmal egocentric AprilTag MPC servo node")
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
    return parser


def main(argv: Optional[list[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(rospy.myargv(argv)[1:] if argv is not None else rospy.myargv()[1:])
    rospy.init_node("anymal_egocentric_servo", anonymous=False)
    rospy.set_param("~loop_hz", float(args.loop_hz))
    node = EgocentricServoNode(args)
    rospy.loginfo(
        "ANYmal egocentric MPC servo ready: tag_length=%.5fm target_distance=%.2fm command_topic=%s",
        APRILTAG_TAG_LENGTH_M,
        args.target_distance_m,
        COMMAND_TOPIC,
    )
    node.spin()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
