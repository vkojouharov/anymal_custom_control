#!/usr/bin/env python3
"""Prototype AprilTag visual servo policy for a cautious pick approach."""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rospy
from flask import Flask, Response, jsonify
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64, String

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3
from anymal_custom_control.RRPRRR_kinematic_model import num_forward_transform
from anymal_custom_control.control.giraf_arm_common import (
    AUTO_GRIPPER_VELOCITY_TOPIC,
    AUTO_TASK_VELOCITY_TOPIC,
    COMMAND_SOURCE_TOPIC,
    GRIPPER_SPEED,
    PITCH_KIN_OFFSET,
    STATE_TOPIC,
    TASK_VELOCITY_LIMITS,
    THETA4_KIN_OFFSET,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
    clamp_task_velocity,
)


RGB_COMPRESSED_TOPIC = "/oakd/rgb/image_color/compressed"
APRILTAG_DETECTIONS_TOPIC = "/oakd/apriltag/detections_json"
VISUAL_SERVO_STATUS_TOPIC = "/giraf_arm/visual_servo_status_json"

TARGET_TAG_ID = 1
TARGET_MARGIN_MIN = 50.0
TAG_TIMEOUT_SEC = 0.25
STATE_TIMEOUT_SEC = 0.5
SUCCESS_DISTANCE_M = 0.08
WAYPOINT_FRACTION = 0.50
WAYPOINT_TOLERANCE_FRACTION = 0.10
WAYPOINT_TOLERANCE_MIN_M = 0.005
MAX_PID_SPEED_M_S = 0.06
MIN_PID_SPEED_M_S = 0.012
DISTANCE_SPEED_GAIN = 0.35
KP = 0.8
KI = 0.08
KD = 0.03
INTEGRAL_LIMIT_M_S = 0.04
GRASP_CLOSE_SEC = 1.5
POST_GRASP_LIFT_M = 0.30
POST_GRASP_LIFT_TOLERANCE_M = 0.02
CAMERA_DOWNWARD_PITCH_DEG = 15.0
DIAGNOSTICS_PRINT_HZ = 10.0


@dataclass
class ArmState:
    stamp_sec: float
    roll: float
    pitch: float
    boom: float
    th4: float
    th5: float
    th6: float


@dataclass
class TagDetection:
    stamp_sec: float
    decision_margin: float
    pose_t_camera_m: np.ndarray
    tag_size_m: Optional[float]


def camera_rotation_tool_from_selected_mapping() -> np.ndarray:
    """Return R_tool_camera for the FK terminal/tool frame.

    Columns are OpenCV camera +X/+Y/+Z expressed in the FK tool frame. In the
    current FK convention, tool +Z is physical gripper forward, tool +Y tracks
    physical gripper up, and tool -X tracks physical gripper right.
    """
    theta = math.radians(CAMERA_DOWNWARD_PITCH_DEG)
    x_camera_in_tool = np.array([-1.0, 0.0, 0.0], dtype=float)
    y_camera_in_tool = np.array([0.0, -math.cos(theta), -math.sin(theta)], dtype=float)
    z_camera_in_tool = np.array([0.0, -math.sin(theta), math.cos(theta)], dtype=float)
    return np.column_stack((x_camera_in_tool, y_camera_in_tool, z_camera_in_tool))


R_TOOL_CAMERA = camera_rotation_tool_from_selected_mapping()


def state_from_json(msg: String) -> Optional[ArmState]:
    try:
        payload = json.loads(msg.data)
        arm = payload["arm"]
        return ArmState(
            stamp_sec=float(payload.get("stamp_sec", rospy.get_time())),
            roll=float(arm["roll"]),
            pitch=float(arm["pitch"]),
            boom=float(arm["boom"]),
            th4=float(arm["th4"]),
            th5=float(arm["th5"]),
            th6=float(arm["th6"]),
        )
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        rospy.logwarn_throttle(2.0, "Failed to parse /giraf_arm/state: %s", exc)
        return None


def joint_coords_from_state(state: ArmState) -> list[float]:
    return [
        state.roll,
        state.pitch + PITCH_KIN_OFFSET,
        get_boom_length_d3(state.boom),
        state.th4 + THETA4_KIN_OFFSET,
        state.th5 + THETA5_KIN_OFFSET,
        state.th6 + THETA6_KIN_OFFSET,
    ]


def transform_from_state(state: ArmState) -> Optional[np.ndarray]:
    try:
        return np.asarray(num_forward_transform(joint_coords_from_state(state)), dtype=float)
    except Exception as exc:
        rospy.logwarn_throttle(2.0, "Failed to compute FK for visual servo: %s", exc)
        return None


def vector_or_none(value: Optional[np.ndarray]) -> Optional[list[float]]:
    if value is None:
        return None
    return [float(x) for x in np.asarray(value, dtype=float).reshape(-1)]


def matrix_to_rows(value: np.ndarray) -> list[list[float]]:
    return [
        [float(item) for item in row]
        for row in np.asarray(value, dtype=float)
    ]


def xyz_components(value: Optional[np.ndarray]) -> Optional[dict[str, float]]:
    if value is None:
        return None
    vec = np.asarray(value, dtype=float).reshape(3)
    return {
        "global_x_forward": float(vec[0]),
        "global_y_left": float(vec[1]),
        "global_z_up": float(vec[2]),
        "global_z_down": float(-vec[2]),
    }


def finite_pose_vector(value: object) -> Optional[np.ndarray]:
    if not isinstance(value, list) or len(value) != 3:
        return None
    try:
        vec = np.asarray([float(x) for x in value], dtype=float)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(vec)):
        return None
    return vec


class VisualServoAutonomy:
    def __init__(self, port: int, loop_hz: float, diagnostics: bool = False) -> None:
        self.port = int(port)
        self.loop_hz = float(loop_hz)
        self.diagnostics = bool(diagnostics)
        self._last_diagnostics_print_sec = 0.0
        self._lock = threading.Lock()
        self.command_source = "teleop"
        self.latest_state: Optional[ArmState] = None
        self.latest_tag: Optional[TagDetection] = None
        self.latest_rgb: Optional[bytes] = None
        self.mode_state = "IDLE"
        self.message = "Waiting for teleop"
        self.waypoint_global: Optional[np.ndarray] = None
        self.waypoint_distance_m: Optional[float] = None
        self.waypoint_tolerance_m: Optional[float] = None
        self.ray_camera: Optional[np.ndarray] = None
        self.ray_tool: Optional[np.ndarray] = None
        self.ray_global: Optional[np.ndarray] = None
        self.lift_waypoint_global: Optional[np.ndarray] = None
        self.integral_error = np.zeros(3, dtype=float)
        self.previous_error: Optional[np.ndarray] = None
        self.previous_pid_time = time.monotonic()
        self.grasp_start_time: Optional[float] = None
        self.latest_status = self._status_payload(rospy.get_time(), np.zeros(3), None, None)

        self.task_pub = rospy.Publisher(AUTO_TASK_VELOCITY_TOPIC, TwistStamped, queue_size=1)
        self.gripper_pub = rospy.Publisher(AUTO_GRIPPER_VELOCITY_TOPIC, Float64, queue_size=1)
        self.command_source_pub = rospy.Publisher(COMMAND_SOURCE_TOPIC, String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(VISUAL_SERVO_STATUS_TOPIC, String, queue_size=1, latch=True)

        rospy.Subscriber(STATE_TOPIC, String, self._state_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(COMMAND_SOURCE_TOPIC, String, self._command_source_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(APRILTAG_DETECTIONS_TOPIC, String, self._tag_detections_cb, queue_size=1, tcp_nodelay=True)
        rospy.Subscriber(RGB_COMPRESSED_TOPIC, CompressedImage, self._rgb_cb, queue_size=1, tcp_nodelay=True)

        self._app = self._build_app()

    def _state_cb(self, msg: String) -> None:
        parsed = state_from_json(msg)
        if parsed is not None:
            with self._lock:
                self.latest_state = parsed

    def _command_source_cb(self, msg: String) -> None:
        source = msg.data.strip().lower()
        if source not in {"teleop", "auto"}:
            return
        with self._lock:
            previous = self.command_source
            self.command_source = source
            if source != previous:
                self._reset_pid_locked()
                self.waypoint_global = None
                self.waypoint_distance_m = None
                self.waypoint_tolerance_m = None
                self.ray_camera = None
                self.ray_tool = None
                self.ray_global = None
                self.lift_waypoint_global = None
                self.grasp_start_time = None
                self.mode_state = "APPROACH" if source == "auto" else "IDLE"
                self.message = "Autonomous mode active" if source == "auto" else "Waiting for teleop"

    def _tag_detections_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            stamp_sec = float(payload.get("stamp_sec", rospy.get_time()))
            tags = payload.get("tags", [])
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            rospy.logwarn_throttle(2.0, "Failed to parse AprilTag detections: %s", exc)
            return

        best: Optional[TagDetection] = None
        for tag in tags:
            if not isinstance(tag, dict):
                continue
            try:
                tag_id = int(tag.get("id", -1))
                margin = float(tag.get("decision_margin", 0.0))
            except (TypeError, ValueError):
                continue
            if tag_id != TARGET_TAG_ID:
                continue
            if not bool(tag.get("known_size", False)):
                continue
            pose = finite_pose_vector(tag.get("pose_t_camera_m"))
            if pose is None:
                continue
            candidate = TagDetection(
                stamp_sec=stamp_sec,
                decision_margin=margin,
                pose_t_camera_m=pose,
                tag_size_m=tag.get("tag_size_m"),
            )
            if best is None or candidate.decision_margin > best.decision_margin:
                best = candidate

        with self._lock:
            self.latest_tag = best

    def _rgb_cb(self, msg: CompressedImage) -> None:
        with self._lock:
            self.latest_rgb = bytes(msg.data)

    def _reset_pid_locked(self) -> None:
        self.integral_error = np.zeros(3, dtype=float)
        self.previous_error = None
        self.previous_pid_time = time.monotonic()

    def _tag_ready_locked(self, now_sec: float) -> bool:
        if self.latest_tag is None:
            return False
        if (now_sec - self.latest_tag.stamp_sec) > TAG_TIMEOUT_SEC:
            return False
        if self.latest_tag.decision_margin < TARGET_MARGIN_MIN:
            return False
        distance = float(np.linalg.norm(self.latest_tag.pose_t_camera_m))
        return math.isfinite(distance) and distance > 1e-6

    def _state_ready_locked(self, now_sec: float) -> bool:
        return self.latest_state is not None and (now_sec - self.latest_state.stamp_sec) <= STATE_TIMEOUT_SEC

    def _current_transform_locked(self) -> Optional[np.ndarray]:
        if self.latest_state is None:
            return None
        return transform_from_state(self.latest_state)

    def _compute_waypoint_locked(self, transform: np.ndarray) -> bool:
        if self.latest_tag is None:
            return False
        pose_camera = self.latest_tag.pose_t_camera_m
        distance = float(np.linalg.norm(pose_camera))
        if distance <= 1e-6:
            return False
        ray_camera = pose_camera / distance
        ray_tool = R_TOOL_CAMERA.dot(ray_camera)
        ray_global = transform[:3, :3].dot(ray_tool)
        ray_norm = float(np.linalg.norm(ray_global))
        if ray_norm <= 1e-9:
            return False
        ray_global /= ray_norm
        current_global = transform[:3, 3]
        self.ray_camera = ray_camera
        self.ray_tool = ray_tool / max(float(np.linalg.norm(ray_tool)), 1e-9)
        self.ray_global = ray_global
        self.waypoint_global = current_global + WAYPOINT_FRACTION * distance * ray_global
        self.waypoint_distance_m = distance
        self.waypoint_tolerance_m = max(WAYPOINT_TOLERANCE_MIN_M, WAYPOINT_TOLERANCE_FRACTION * distance)
        self._reset_pid_locked()
        return True

    def _set_lift_waypoint_locked(self, current_global: np.ndarray) -> None:
        self.lift_waypoint_global = current_global + np.array([0.0, 0.0, POST_GRASP_LIFT_M], dtype=float)
        self.waypoint_global = self.lift_waypoint_global.copy()
        self.waypoint_distance_m = POST_GRASP_LIFT_M
        self.waypoint_tolerance_m = POST_GRASP_LIFT_TOLERANCE_M
        self.ray_camera = None
        self.ray_tool = None
        self.ray_global = np.array([0.0, 0.0, 1.0], dtype=float)
        self._reset_pid_locked()

    def _pid_velocity_locked(self, error: np.ndarray) -> np.ndarray:
        now = time.monotonic()
        dt = max(1e-3, now - self.previous_pid_time)
        self.previous_pid_time = now

        self.integral_error += error * dt
        self.integral_error = np.clip(self.integral_error, -INTEGRAL_LIMIT_M_S, INTEGRAL_LIMIT_M_S)

        if self.previous_error is None:
            derivative = np.zeros(3, dtype=float)
        else:
            derivative = (error - self.previous_error) / dt
        self.previous_error = error.copy()

        velocity = KP * error + KI * self.integral_error + KD * derivative
        if self.waypoint_distance_m is None:
            max_speed = MIN_PID_SPEED_M_S
        else:
            max_speed = min(MAX_PID_SPEED_M_S, max(MIN_PID_SPEED_M_S, DISTANCE_SPEED_GAIN * self.waypoint_distance_m))
        norm = float(np.linalg.norm(velocity))
        if norm > max_speed and norm > 1e-9:
            velocity = velocity * (max_speed / norm)
        return velocity

    def _twist(self, linear: np.ndarray) -> TwistStamped:
        clamped = clamp_task_velocity(np.concatenate((linear, np.zeros(3, dtype=float))))
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = float(clamped[0])
        msg.twist.linear.y = float(clamped[1])
        msg.twist.linear.z = float(clamped[2])
        msg.twist.angular.x = float(clamped[3])
        msg.twist.angular.y = float(clamped[4])
        msg.twist.angular.z = float(clamped[5])
        return msg

    def _publish_zero(self) -> None:
        self.task_pub.publish(self._twist(np.zeros(3, dtype=float)))
        self.gripper_pub.publish(Float64(data=0.0))

    def _publish_close(self) -> None:
        self.task_pub.publish(self._twist(np.zeros(3, dtype=float)))
        self.gripper_pub.publish(Float64(data=-GRIPPER_SPEED))

    def _status_payload(
        self,
        now_sec: float,
        command_linear: np.ndarray,
        current_global: Optional[np.ndarray],
        error_global: Optional[np.ndarray],
    ) -> dict:
        tag = self.latest_tag
        visible = tag is not None and (now_sec - tag.stamp_sec) <= TAG_TIMEOUT_SEC
        distance = float(np.linalg.norm(tag.pose_t_camera_m)) if tag is not None else None
        ready = self._tag_ready_locked(now_sec)
        transform = self._current_transform_locked() if self.latest_state is not None else None
        camera_axes_global = transform[:3, :3].dot(R_TOOL_CAMERA) if transform is not None else None
        return {
            "stamp_sec": round(float(now_sec), 6),
            "mode": self.command_source,
            "state": self.mode_state,
            "ready": bool(ready),
            "active": self.command_source == "auto",
            "tag": {
                "id": TARGET_TAG_ID,
                "visible": bool(visible),
                "age_sec": float(now_sec - tag.stamp_sec) if tag is not None else None,
                "decision_margin": float(tag.decision_margin) if tag is not None else None,
                "tag_size_m": tag.tag_size_m if tag is not None else None,
                "pose_t_camera_m": vector_or_none(tag.pose_t_camera_m if tag is not None else None),
                "distance_m": distance,
            },
            "waypoint_global": vector_or_none(self.waypoint_global),
            "lift_waypoint_global": vector_or_none(self.lift_waypoint_global),
            "waypoint_distance_m": self.waypoint_distance_m,
            "waypoint_tolerance_m": self.waypoint_tolerance_m,
            "ray_camera": vector_or_none(self.ray_camera),
            "ray_tool": vector_or_none(self.ray_tool),
            "ray_global": vector_or_none(self.ray_global),
            "ray_global_components": xyz_components(self.ray_global),
            "R_tool_camera": matrix_to_rows(R_TOOL_CAMERA),
            "camera_axes_global": matrix_to_rows(camera_axes_global) if camera_axes_global is not None else None,
            "current_global": vector_or_none(current_global),
            "error_global": vector_or_none(error_global),
            "command_linear": vector_or_none(command_linear),
            "command_components": xyz_components(command_linear),
            "limits_linear": [float(x) for x in TASK_VELOCITY_LIMITS[:3]],
            "message": self.message,
        }

    def _publish_status_locked(
        self,
        now_sec: float,
        command_linear: np.ndarray,
        current_global: Optional[np.ndarray],
        error_global: Optional[np.ndarray],
    ) -> None:
        self.latest_status = self._status_payload(now_sec, command_linear, current_global, error_global)
        self.status_pub.publish(String(data=json.dumps(self.latest_status, separators=(",", ":"), sort_keys=True)))

    def _diagnostics_payload_locked(self) -> dict:
        status = dict(self.latest_status)
        tag = status.get("tag") or {}
        return {
            "stamp_sec": status.get("stamp_sec"),
            "mode": status.get("mode"),
            "state": status.get("state"),
            "ready": status.get("ready"),
            "tag_visible": tag.get("visible"),
            "tag_age_sec": tag.get("age_sec"),
            "tag_margin": tag.get("decision_margin"),
            "tag_pose_camera_m": tag.get("pose_t_camera_m"),
            "tag_distance_m": tag.get("distance_m"),
            "ray_camera": status.get("ray_camera"),
            "ray_tool": status.get("ray_tool"),
            "ray_global": status.get("ray_global"),
            "ray_global_components": status.get("ray_global_components"),
            "camera_axes_global": status.get("camera_axes_global"),
            "R_tool_camera": status.get("R_tool_camera"),
            "current_global": status.get("current_global"),
            "waypoint_global": status.get("waypoint_global"),
            "lift_waypoint_global": status.get("lift_waypoint_global"),
            "waypoint_distance_m": status.get("waypoint_distance_m"),
            "waypoint_tolerance_m": status.get("waypoint_tolerance_m"),
            "error_global": status.get("error_global"),
            "command_linear": status.get("command_linear"),
            "command_components": status.get("command_components"),
            "message": status.get("message"),
        }

    def _maybe_print_diagnostics_locked(self, now_sec: float) -> None:
        if not self.diagnostics or self.command_source != "auto":
            return
        if now_sec - self._last_diagnostics_print_sec < (1.0 / DIAGNOSTICS_PRINT_HZ):
            return
        self._last_diagnostics_print_sec = now_sec
        print(
            "VISUAL_SERVO_DIAG "
            + json.dumps(self._diagnostics_payload_locked(), separators=(",", ":"), sort_keys=True),
            flush=True,
        )

    def step(self) -> None:
        now_sec = rospy.get_time()
        command_linear = np.zeros(3, dtype=float)
        current_global: Optional[np.ndarray] = None
        error_global: Optional[np.ndarray] = None
        close_gripper = False
        switch_to_teleop = False

        with self._lock:
            if self.command_source != "auto":
                self.mode_state = "IDLE"
                self.message = "Waiting for teleop"
                self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                self._maybe_print_diagnostics_locked(now_sec)
                publish_zero = True
            elif not self._state_ready_locked(now_sec):
                self.mode_state = "WAITING_FOR_STATE"
                self.message = "Waiting for fresh arm state"
                self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                self._maybe_print_diagnostics_locked(now_sec)
                publish_zero = True
            else:
                publish_zero = False
                transform = self._current_transform_locked()
                if transform is None:
                    self.mode_state = "WAITING_FOR_STATE"
                    self.message = "FK unavailable"
                    self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                    self._maybe_print_diagnostics_locked(now_sec)
                    publish_zero = True
                else:
                    current_global = transform[:3, 3]
                    if self.mode_state == "LIFT":
                        if self.lift_waypoint_global is None:
                            self._set_lift_waypoint_locked(current_global)
                        error_global = self.lift_waypoint_global - current_global
                        error_norm = float(np.linalg.norm(error_global))
                        if error_norm <= POST_GRASP_LIFT_TOLERANCE_M:
                            self.command_source = "teleop"
                            self.mode_state = "DONE"
                            self.message = "Lift complete; returning to teleop"
                            self._reset_pid_locked()
                            publish_zero = True
                            switch_to_teleop = True
                        else:
                            self.waypoint_global = self.lift_waypoint_global.copy()
                            self.waypoint_distance_m = POST_GRASP_LIFT_M
                            self.waypoint_tolerance_m = POST_GRASP_LIFT_TOLERANCE_M
                            self.message = "Lifting grasped object"
                            command_linear = self._pid_velocity_locked(error_global)
                        self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                        self._maybe_print_diagnostics_locked(now_sec)
                    elif not self._tag_ready_locked(now_sec):
                        self.mode_state = "PAUSED_LOST_TAG"
                        self.message = "Waiting for fresh ID1 pose"
                        self.waypoint_global = None
                        self.waypoint_distance_m = None
                        self.waypoint_tolerance_m = None
                        self.ray_camera = None
                        self.ray_tool = None
                        self.ray_global = None
                        self._reset_pid_locked()
                        self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                        self._maybe_print_diagnostics_locked(now_sec)
                        publish_zero = True
                    else:
                        tag_distance = float(np.linalg.norm(self.latest_tag.pose_t_camera_m)) if self.latest_tag is not None else float("inf")
                        if tag_distance <= SUCCESS_DISTANCE_M:
                            self.waypoint_global = None
                            self.waypoint_distance_m = tag_distance
                            self.waypoint_tolerance_m = None
                            self.ray_camera = None
                            self.ray_tool = None
                            self.ray_global = None
                            self._reset_pid_locked()
                            if self.grasp_start_time is None:
                                self.grasp_start_time = time.monotonic()
                            elapsed = time.monotonic() - self.grasp_start_time
                            if elapsed <= GRASP_CLOSE_SEC:
                                self.mode_state = "GRASPING"
                                self.message = "Tag within grasp distance; closing gripper"
                                close_gripper = True
                            else:
                                self.mode_state = "LIFT"
                                self.message = "Grasp command complete; starting lift"
                                self._set_lift_waypoint_locked(current_global)
                            self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                            self._maybe_print_diagnostics_locked(now_sec)
                        else:
                            self.grasp_start_time = None
                            if self.waypoint_global is None and not self._compute_waypoint_locked(transform):
                                self.mode_state = "PAUSED_LOST_TAG"
                                self.message = "Unable to compute waypoint"
                                self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                                self._maybe_print_diagnostics_locked(now_sec)
                                publish_zero = True
                            else:
                                error_global = self.waypoint_global - current_global
                                error_norm = float(np.linalg.norm(error_global))
                                tolerance = self.waypoint_tolerance_m or WAYPOINT_TOLERANCE_MIN_M
                                if error_norm <= tolerance:
                                    self.mode_state = "WAYPOINT_REACHED"
                                    self.message = "Waypoint reached; recomputing"
                                    if self._compute_waypoint_locked(transform):
                                        error_global = self.waypoint_global - current_global
                                    else:
                                        publish_zero = True
                                if not publish_zero:
                                    self.mode_state = "APPROACH"
                                    self.message = "Driving toward AprilTag waypoint"
                                    command_linear = self._pid_velocity_locked(error_global)
                                self._publish_status_locked(now_sec, command_linear, current_global, error_global)
                                self._maybe_print_diagnostics_locked(now_sec)

        if switch_to_teleop:
            self._publish_zero()
            self.command_source_pub.publish(String(data="teleop"))
        elif close_gripper:
            self._publish_close()
        elif publish_zero:
            self._publish_zero()
        else:
            self.task_pub.publish(self._twist(command_linear))
            self.gripper_pub.publish(Float64(data=0.0))

    def _build_app(self) -> Flask:
        app = Flask(__name__)
        node = self

        @app.route("/")
        def index() -> str:
            return """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>GIRAF Visual Servo Prototype</title>
  <style>
    body { margin: 0; font-family: system-ui, sans-serif; background: #101418; color: #e8edf2; }
    main { display: grid; grid-template-columns: minmax(320px, 1fr) 360px; gap: 18px; padding: 18px; }
    img { width: 100%; background: #050607; border: 1px solid #2b333b; }
    section { border: 1px solid #2b333b; padding: 14px; background: #151b21; }
    h1 { font-size: 18px; margin: 0 0 12px; }
    dl { display: grid; grid-template-columns: 150px 1fr; gap: 8px 12px; margin: 0; font-size: 14px; }
    dt { color: #9fb0bf; }
    dd { margin: 0; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; word-break: break-word; }
    @media (max-width: 860px) { main { grid-template-columns: 1fr; } }
  </style>
</head>
<body>
  <main>
    <img src="/feed/rgb" alt="RGB AprilTag view">
    <section>
      <h1>Visual Servo</h1>
      <dl id="status"></dl>
    </section>
  </main>
  <script>
    const fmt = (value) => value === null || value === undefined ? "null" :
      Array.isArray(value) ? "[" + value.map(v => Number(v).toFixed(4)).join(", ") + "]" :
      typeof value === "number" ? value.toFixed(4) : String(value);
    async function refresh() {
      const data = await fetch("/status").then(r => r.json());
      const tag = data.tag || {};
      const rows = [
        ["mode", data.mode],
        ["state", data.state],
        ["ready", data.ready],
        ["message", data.message],
        ["tag pose camera", tag.pose_t_camera_m],
        ["tag distance", tag.distance_m],
        ["tag margin", tag.decision_margin],
        ["ray camera", data.ray_camera],
        ["ray tool", data.ray_tool],
        ["ray global", data.ray_global],
        ["ray components", data.ray_global_components],
        ["command components", data.command_components],
        ["camera axes global", data.camera_axes_global],
        ["waypoint", data.waypoint_global],
        ["current", data.current_global],
        ["error", data.error_global],
        ["command", data.command_linear],
      ];
      document.getElementById("status").innerHTML = rows
        .map(([k, v]) => `<dt>${k}</dt><dd>${fmt(v)}</dd>`).join("");
    }
    refresh();
    setInterval(refresh, 200);
  </script>
</body>
</html>
"""

        @app.route("/status")
        def status() -> Response:
            with node._lock:
                payload = dict(node.latest_status)
            return jsonify(payload)

        @app.route("/feed/rgb")
        def rgb_feed() -> Response:
            return Response(node._rgb_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")

        return app

    def _rgb_stream(self):
        while not rospy.is_shutdown():
            with self._lock:
                frame = self.latest_rgb
            if frame is None:
                time.sleep(0.05)
                continue
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            time.sleep(0.05)

    def start_web(self) -> threading.Thread:
        thread = threading.Thread(
            target=lambda: self._app.run(host="0.0.0.0", port=self.port, debug=False, use_reloader=False, threaded=True),
            daemon=True,
        )
        thread.start()
        return thread

    def run(self) -> int:
        self.start_web()
        rospy.loginfo("Visual servo dashboard: http://localhost:%d", self.port)
        rate = rospy.Rate(self.loop_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()
        self._publish_zero()
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Run prototype AprilTag visual-servo autonomy node.")
    parser.add_argument("--port", type=int, default=5010, help="Flask status dashboard port")
    parser.add_argument("--loop-hz", type=float, default=50.0, help="Autonomy command loop rate")
    parser.add_argument(
        "--diagnostics",
        action="store_true",
        help="Print compact visual-servo diagnostics JSON lines at 10 Hz while auto is active",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("giraf_arm_visual_servo_autonomy", anonymous=False)
    node = VisualServoAutonomy(port=args.port, loop_hz=args.loop_hz, diagnostics=args.diagnostics)
    try:
        return node.run()
    except KeyboardInterrupt:
        print("\nShutting down visual-servo autonomy...")
        node._publish_zero()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
