#!/usr/bin/env python3
"""Barebones threaded runtime for autonomous cable-outfitting experiments."""

import math
import threading
import time
import traceback
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rospy

from anymal_custom_control.RRP_kinematic_model import get_boom_length_d3, get_boom_motor_rad
from anymal_custom_control.RRPRRR_kinematic_model import num_forward_transform, num_jacobian
from anymal_custom_control.control.giraf_arm_common import (
    ARM_WY_SPEED,
    ARM_WZ_SPEED,
    ARM_X_SPEED,
    ARM_Y_SPEED,
    ARM_Z_SPEED,
    BOOM_MAX,
    BOOM_MIN,
    COMMAND_TIMEOUT_SEC,
    CONTROL_LOOP_HZ,
    D3_MIN,
    GRIPPER_SPEED,
    PITCH_KIN_OFFSET,
    PITCH_MAX,
    PITCH_MIN,
    QDOT_LIMITS,
    ROLL_LIMIT,
    TASK_VELOCITY_LIMITS,
    TELEOP_PUBLISH_HZ,
    THETA4_DXL_SIGN,
    THETA4_KIN_OFFSET,
    THETA5_DXL_SIGN,
    THETA5_KIN_OFFSET,
    THETA6_KIN_OFFSET,
)
from anymal_custom_control.dynamixel import (
    ARM_HOME,
    ARM_IDS,
    ARM_TICK_LIMITS,
    GRIPPER_IDS,
    GRIPPER_OPEN,
    GRIPPER_STROKE,
    dynamixel_connect,
    dynamixel_disconnect,
    dynamixel_drive,
    radians_to_ticks,
    ticks_to_radians,
)
from anymal_custom_control.joystick_driver import joystick_connect, joystick_disconnect, joystick_read
from anymal_custom_control.motor_driver import motor_connect, motor_disconnect, motor_drive

from cable_policy import CablePolicy, PolicyCommand, PolicyObservation


CAMERA_HZ = 30.0
COMMAND_HZ = 100.0
DIAGNOSTICS_HZ = 10.0
WIDTH, HEIGHT = 640, 360
TAG_FAMILY, TAG_ID, TAG_SIZE_M = "tag16h5", 1, 0.049
TAG_MARGIN_MIN = 35.0
DLS_DAMPING = 0.05
CONTROL_DT = 1.0 / CONTROL_LOOP_HZ

MD80_GAIN_OVERRIDES = {
    11: {"kp": 200.0, "kd": 10.0, "max_torque": 10.0},
}


def joint_limit_rad(mid, sign=1.0):
    lo_tick, hi_tick = ARM_TICK_LIMITS[mid]
    home_tick = ARM_HOME[mid]
    lo_rad = sign * ticks_to_radians(lo_tick - home_tick)
    hi_rad = sign * ticks_to_radians(hi_tick - home_tick)
    return min(lo_rad, hi_rad), max(lo_rad, hi_rad)


THETA4_MIN, THETA4_MAX = joint_limit_rad(ARM_IDS[0], THETA4_DXL_SIGN)
THETA5_MIN, THETA5_MAX = joint_limit_rad(ARM_IDS[1], THETA5_DXL_SIGN)
THETA6_MIN, THETA6_MAX = joint_limit_rad(ARM_IDS[2])


@dataclass(frozen=True)
class TagPose:
    T_camera_tag: np.ndarray
    decision_margin: float
    stamp_sec: float


lock = threading.Lock()
stop = threading.Event()

auto_enabled = False
mode_epoch = 0
camera_available = False
camera_failed = False
camera_fps = 0.0
detection_fps = 0.0
tag_visible = False
latest_tag = None
policy_available = True

teleop_task_velocity = np.zeros(6, dtype=float)
teleop_gripper_velocity = 0.0
selected_task_velocity = np.zeros(6, dtype=float)
selected_gripper_velocity = 0.0
selected_command_time = 0.0

arm_joint_position = np.array([0.0, 0.0, D3_MIN, 0.0, 0.0, 0.0], dtype=float)
T_base_tool = np.eye(4, dtype=float)
fatal_failure = None


def fatal(name, exc):
    global fatal_failure
    message = f"{name} thread failed: {exc}"
    with lock:
        if fatal_failure is None:
            fatal_failure = message
        zero_selected_command_locked()
    print(message)
    trace = traceback.format_exc()
    if trace != "NoneType: None\n":
        print(trace, end="")
    stop.set()


def zero_selected_command_locked():
    global selected_task_velocity, selected_gripper_velocity, selected_command_time
    selected_task_velocity = np.zeros(6, dtype=float)
    selected_gripper_velocity = 0.0
    selected_command_time = time.monotonic()


def camera_thread():
    global auto_enabled, camera_available, camera_failed, camera_fps, detection_fps
    global latest_tag, mode_epoch, tag_visible

    try:
        import cv2
        import depthai as dai
        from pupil_apriltags import Detector

        pipeline = dai.Pipeline()
        camera = pipeline.create(dai.node.ColorCamera)
        output = pipeline.create(dai.node.XLinkOut)
        output.setStreamName("rgb")
        camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camera.setPreviewSize(WIDTH, HEIGHT)
        camera.setPreviewKeepAspectRatio(False)
        camera.setInterleaved(False)
        camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camera.setFps(CAMERA_HZ)
        camera.preview.link(output.input)
        detector = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0)

        with dai.Device(pipeline) as device:
            calibration = device.readCalibration()
            raw_K = np.asarray(
                calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, WIDTH, HEIGHT),
                dtype=float,
            )
            distortion = np.asarray(
                calibration.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A),
                dtype=float,
            ).reshape(-1)
            corrected_K, valid_roi = cv2.getOptimalNewCameraMatrix(
                raw_K,
                distortion,
                (WIDTH, HEIGHT),
                0.0,
                (WIDTH, HEIGHT),
            )
            map_1, map_2 = cv2.initUndistortRectifyMap(
                raw_K,
                distortion,
                None,
                corrected_K,
                (WIDTH, HEIGHT),
                cv2.CV_16SC2,
            )
            camera_params = (
                float(corrected_K[0, 0]),
                float(corrected_K[1, 1]),
                float(corrected_K[0, 2]),
                float(corrected_K[1, 2]),
            )
            queue = device.getOutputQueue("rgb", maxSize=1, blocking=False)
            with lock:
                camera_available = True
            print(
                f"camera ready: {WIDTH}x{HEIGHT} {CAMERA_HZ:.0f} Hz, "
                f"{len(distortion)} distortion coefficients, ROI={valid_roi}"
            )

            frames = 0
            detections_run = 0
            rate_time = time.monotonic()
            was_visible = False
            while not stop.is_set() and not rospy.is_shutdown():
                raw_frame = queue.get().getCvFrame()
                frame = cv2.remap(
                    raw_frame,
                    map_1,
                    map_2,
                    interpolation=cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT,
                )
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=camera_params,
                    tag_size=TAG_SIZE_M,
                )
                matches = [
                    detection
                    for detection in detections
                    if detection.tag_id == TAG_ID
                    and detection.decision_margin > TAG_MARGIN_MIN
                ]
                best = max(matches, key=lambda item: item.decision_margin) if matches else None
                now = time.monotonic()
                frames += 1
                detections_run += 1

                pose = None
                if best is not None:
                    transform = np.eye(4, dtype=float)
                    transform[:3, :3] = np.asarray(best.pose_R, dtype=float).reshape(3, 3)
                    transform[:3, 3] = np.asarray(best.pose_t, dtype=float).reshape(3)
                    if np.all(np.isfinite(transform)):
                        pose = TagPose(transform, float(best.decision_margin), now)

                elapsed = now - rate_time
                with lock:
                    tag_visible = pose is not None
                    if pose is not None:
                        latest_tag = pose
                    if elapsed >= 1.0:
                        camera_fps = frames / elapsed
                        detection_fps = detections_run / elapsed

                if pose is not None and not was_visible:
                    print(f"tag acquired: {TAG_FAMILY} ID {TAG_ID}")
                elif pose is None and was_visible:
                    print(f"tag lost: {TAG_FAMILY} ID {TAG_ID}")
                was_visible = pose is not None

                if elapsed >= 1.0:
                    frames = 0
                    detections_run = 0
                    rate_time = now
    except Exception as exc:
        if stop.is_set() or rospy.is_shutdown():
            return
        with lock:
            camera_available = False
            camera_failed = True
            tag_visible = False
            if auto_enabled:
                auto_enabled = False
                mode_epoch += 1
            zero_selected_command_locked()
        print(f"camera thread failed; teleop remains available: {exc}")
        traceback.print_exc()
    finally:
        with lock:
            camera_available = False


def build_teleop_command(data):
    task = np.zeros(6, dtype=float)
    gripper = 0.0
    if data["LB"] and data["RB"]:
        task[0] = ARM_X_SPEED * data["LY"]
        task[1] = -ARM_Y_SPEED * data["LX"]
        if data["RT"] and not data["LT"]:
            task[2] = ARM_Z_SPEED * data["RT"]
        elif data["LT"] and not data["RT"]:
            task[2] = -ARM_Z_SPEED * data["LT"]
        task[4] = ARM_WY_SPEED * data["RY"]
        task[5] = -ARM_WZ_SPEED * data["RX"]
        if data["AB"] and not data["BB"]:
            gripper = -GRIPPER_SPEED
        elif data["BB"] and not data["AB"]:
            gripper = GRIPPER_SPEED
    return task, gripper


def joystick_thread():
    global auto_enabled, mode_epoch, teleop_gripper_velocity, teleop_task_velocity

    joystick = None
    previous_x = 0
    previous_y = 0
    try:
        joystick = joystick_connect()
        print(f"joystick ready: {joystick['device'].name}")
        while not stop.is_set() and not rospy.is_shutdown():
            data = joystick_read(joystick)
            task, gripper = build_teleop_command(data)
            with lock:
                teleop_task_velocity = task
                teleop_gripper_velocity = gripper

            if data["XB"] and not previous_x:
                print("X pressed: emergency stop")
                with lock:
                    zero_selected_command_locked()
                stop.set()

            if data["YB"] and not previous_y:
                with lock:
                    if not auto_enabled and (camera_failed or not policy_available):
                        message = "Y ignored: autonomous mode is unavailable until restart"
                    else:
                        auto_enabled = not auto_enabled
                        mode_epoch += 1
                        zero_selected_command_locked()
                        message = f"command mode: {'auto' if auto_enabled else 'teleop'}"
                print(message)

            previous_x = data["XB"]
            previous_y = data["YB"]
            stop.wait(1.0 / TELEOP_PUBLISH_HZ)
    except Exception as exc:
        fatal("joystick", exc)
    finally:
        with lock:
            teleop_task_velocity = np.zeros(6, dtype=float)
            teleop_gripper_velocity = 0.0
        if joystick is not None:
            joystick_disconnect(joystick)


def validate_policy_command(command):
    if not isinstance(command, PolicyCommand):
        raise TypeError("policy must return PolicyCommand")
    task = np.asarray(command.task_velocity_base, dtype=float).reshape(-1)
    if task.shape != (6,) or not np.all(np.isfinite(task)):
        raise ValueError("policy task velocity must contain six finite values")
    gripper = float(command.gripper_velocity)
    if not math.isfinite(gripper):
        raise ValueError("policy gripper velocity must be finite")
    return (
        np.clip(task, -TASK_VELOCITY_LIMITS, TASK_VELOCITY_LIMITS),
        float(np.clip(gripper, -GRIPPER_SPEED, GRIPPER_SPEED)),
    )


def command_thread():
    global auto_enabled, mode_epoch, policy_available
    global selected_gripper_velocity, selected_task_velocity, selected_command_time

    policy = CablePolicy()
    previous_epoch = -1
    previous_step_time = time.monotonic()
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            tick = time.monotonic()
            with lock:
                epoch = mode_epoch
                use_auto = auto_enabled
                manual_task = teleop_task_velocity.copy()
                manual_gripper = teleop_gripper_velocity
                tag = latest_tag
                visible = tag_visible
                vision_ok = camera_available
                joints = arm_joint_position.copy()
                tool_transform = T_base_tool.copy()

            if epoch != previous_epoch:
                policy.reset()
                task = np.zeros(6, dtype=float)
                gripper = 0.0
                previous_epoch = epoch
            elif use_auto:
                age = float("inf") if tag is None else max(0.0, tick - tag.stamp_sec)
                observation = PolicyObservation(
                    T_camera_tag=None if tag is None else tag.T_camera_tag.copy(),
                    tag_age_sec=age,
                    tag_visible=visible,
                    tag_decision_margin=None if tag is None else tag.decision_margin,
                    camera_available=vision_ok,
                    joint_position=joints,
                    T_base_tool=tool_transform,
                    dt_sec=max(1e-6, tick - previous_step_time),
                )
                try:
                    task, gripper = validate_policy_command(policy.step(observation))
                except Exception as exc:
                    with lock:
                        policy_available = False
                        auto_enabled = False
                        mode_epoch += 1
                        zero_selected_command_locked()
                    print(f"policy failed; returning to teleop: {exc}")
                    traceback.print_exc()
                    previous_step_time = tick
                    stop.wait(1.0 / COMMAND_HZ)
                    continue
            else:
                task = np.clip(manual_task, -TASK_VELOCITY_LIMITS, TASK_VELOCITY_LIMITS)
                gripper = float(np.clip(manual_gripper, -GRIPPER_SPEED, GRIPPER_SPEED))

            with lock:
                if epoch == mode_epoch:
                    selected_task_velocity = task
                    selected_gripper_velocity = gripper
                    selected_command_time = tick
                else:
                    zero_selected_command_locked()
            previous_step_time = tick
            stop.wait(max(0.0, 1.0 / COMMAND_HZ - (time.monotonic() - tick)))
    except Exception as exc:
        fatal("command", exc)


def damped_joint_velocity(jacobian, task_velocity, damping=DLS_DAMPING):
    jacobian = np.asarray(jacobian, dtype=float)
    task_velocity = np.asarray(task_velocity, dtype=float).reshape(6)
    regularized = jacobian @ jacobian.T + (damping * damping) * np.eye(6)
    return jacobian.T @ np.linalg.solve(regularized, task_velocity)


def dxl_ticks(joints, gripper_position):
    fraction = float(np.clip(gripper_position, 0.0, 1.0))
    gripper_id = GRIPPER_IDS[0]
    return [
        ARM_HOME[ARM_IDS[0]] + radians_to_ticks(THETA4_DXL_SIGN * joints[3]),
        ARM_HOME[ARM_IDS[1]] + radians_to_ticks(THETA5_DXL_SIGN * joints[4]),
        ARM_HOME[ARM_IDS[2]] + radians_to_ticks(joints[5]),
        int(round(GRIPPER_OPEN[gripper_id] - GRIPPER_STROKE * (1.0 - fraction))),
    ]


def diagnostic_line(now, joints, task):
    with lock:
        mode = "auto" if auto_enabled else "teleop"
        tag = latest_tag
        visible = tag_visible
        vision_ok = camera_available
        camera_rate = camera_fps
        detector_rate = detection_fps
    if tag is None:
        tag_text = "tag=none"
    else:
        xyz = tag.T_camera_tag[:3, 3]
        age = max(0.0, now - tag.stamp_sec)
        tag_text = f"tag={'seen' if visible else 'lost'} age={age:.2f}s xyz={np.round(xyz, 3)}"
    print(
        f"mode={mode} camera={'ok' if vision_ok else 'off'} "
        f"cam/det={camera_rate:.1f}/{detector_rate:.1f}Hz {tag_text} "
        f"v={np.round(task, 3)} q={np.round(joints, 3)}"
    )


def control_thread():
    global T_base_tool, arm_joint_position

    joints = np.array([0.0, 0.0, D3_MIN, 0.0, 0.0, 0.0], dtype=float)
    gripper_position = 1.0
    md80 = None
    dxl = None
    next_diagnostic = 0.0
    try:
        print("connecting MD80 and Dynamixel hardware")
        md80 = motor_connect(gain_overrides=MD80_GAIN_OVERRIDES)
        dxl = dynamixel_connect(baudrate=1_000_000)
        print("arm hardware ready")

        while not stop.is_set() and not rospy.is_shutdown():
            tick = time.monotonic()
            with lock:
                task = selected_task_velocity.copy()
                gripper_velocity = selected_gripper_velocity
                command_age = tick - selected_command_time
            if command_age > COMMAND_TIMEOUT_SEC:
                task[:] = 0.0
                gripper_velocity = 0.0

            kinematic_joints = np.array(
                [
                    joints[0],
                    joints[1] + PITCH_KIN_OFFSET,
                    joints[2],
                    joints[3] + THETA4_KIN_OFFSET,
                    joints[4] + THETA5_KIN_OFFSET,
                    joints[5] + THETA6_KIN_OFFSET,
                ],
                dtype=float,
            )
            jacobian = np.asarray(num_jacobian(kinematic_joints), dtype=float)
            joint_velocity = damped_joint_velocity(jacobian, task)
            joint_velocity = np.clip(joint_velocity, -QDOT_LIMITS, QDOT_LIMITS)
            if not np.all(np.isfinite(joint_velocity)):
                raise ValueError("DLS produced a non-finite joint velocity")

            joints += CONTROL_DT * joint_velocity
            gripper_position += CONTROL_DT * gripper_velocity
            joints[0] = np.clip(joints[0], -ROLL_LIMIT, ROLL_LIMIT)
            joints[1] = np.clip(joints[1], PITCH_MIN, PITCH_MAX)
            joints[2] = max(joints[2], D3_MIN)
            joints[3] = np.clip(joints[3], THETA4_MIN, THETA4_MAX)
            joints[4] = np.clip(joints[4], THETA5_MIN, THETA5_MAX)
            joints[5] = np.clip(joints[5], THETA6_MIN, THETA6_MAX)
            gripper_position = float(np.clip(gripper_position, 0.0, 1.0))

            boom = float(np.clip(get_boom_motor_rad(joints[2]), BOOM_MIN, BOOM_MAX))
            joints[2] = get_boom_length_d3(boom)
            kinematic_joints = np.array(
                [
                    joints[0],
                    joints[1] + PITCH_KIN_OFFSET,
                    joints[2],
                    joints[3] + THETA4_KIN_OFFSET,
                    joints[4] + THETA5_KIN_OFFSET,
                    joints[5] + THETA6_KIN_OFFSET,
                ],
                dtype=float,
            )
            tool_transform = np.asarray(num_forward_transform(kinematic_joints), dtype=float)

            motor_drive(md80, joints[0], joints[1], boom)
            if not dynamixel_drive(dxl, dxl_ticks(joints, gripper_position)):
                raise RuntimeError("Dynamixel command failed")

            with lock:
                arm_joint_position = joints.copy()
                T_base_tool = tool_transform

            if tick >= next_diagnostic:
                diagnostic_line(tick, joints, task)
                next_diagnostic = tick + 1.0 / DIAGNOSTICS_HZ
            stop.wait(max(0.0, CONTROL_DT - (time.monotonic() - tick)))
    except Exception as exc:
        fatal("control", exc)
    finally:
        with lock:
            zero_selected_command_locked()
        if md80 is not None:
            try:
                motor_disconnect()
            except Exception as exc:
                print(f"MD80 shutdown warning: {exc}")
        if dxl is not None:
            try:
                dynamixel_disconnect(dxl)
            except Exception as exc:
                print(f"Dynamixel shutdown warning: {exc}")
        print("arm hardware stopped")


def main():
    rospy.init_node("cable_outfitting_barebones", anonymous=False)
    workers = {
        "camera": threading.Thread(target=camera_thread, name="camera"),
        "joystick": threading.Thread(target=joystick_thread, name="joystick"),
        "command": threading.Thread(target=command_thread, name="command"),
        "control": threading.Thread(target=control_thread, name="control"),
    }
    print("X: emergency stop | Y: toggle teleop/auto | LB+RB: manual dead-man")
    for worker in workers.values():
        worker.start()

    try:
        while not stop.is_set() and not rospy.is_shutdown():
            for name in ("joystick", "command", "control"):
                if not workers[name].is_alive():
                    fatal(name, RuntimeError("thread exited unexpectedly"))
                    break
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Ctrl-C: stopping")
    finally:
        stop.set()
        with lock:
            zero_selected_command_locked()
        for worker in workers.values():
            worker.join(timeout=3.0)
        alive = [worker.name for worker in workers.values() if worker.is_alive()]
        if alive:
            print(f"shutdown warning: threads still alive: {alive}")
    return 1 if fatal_failure is not None else 0


if __name__ == "__main__":
    raise SystemExit(main())
