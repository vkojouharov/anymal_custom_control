#!/usr/bin/env python3
"""Barebones three-thread AprilTag MPC for ANYmal hardware."""

import argparse
import math
import threading
import time
from collections import deque

import cv2
import cvxpy as cp
import depthai as dai
import numpy as np
import rospy
from pupil_apriltags import Detector

from anymal_custom_control import MovementController


# CAMERA_HZ, MPC_HZ, CONTROL_HZ = 10.0, 5.0, 10.0
CAMERA_HZ, MPC_HZ, CONTROL_HZ = 30.0, 30.0, 30.0
WIDTH, HEIGHT, TAG_SIZE = 640, 360, 0.20066
DT, N, TAG_TIMEOUT = 1.0 / MPC_HZ, 20, 0.5
GOAL = np.array([1.0, 0.0, 0.0])
P_XY, P_THETA = 50.0, 10.0
R = np.diag([0.1, 0.1, 0.05])
S = np.diag([2.0, 2.0, 0.2])
U_MAX = np.array([0.35, 0.35, 0.2])
# DU_MAX = np.array([0.06, 0.06, 0.15])
DU_MAX = np.array([10.0, 10.0, 10.0])
ALPHA = math.radians(30.0)

lock = threading.Lock()
stop = threading.Event()
robot_state = None
robot_state_time = 0.0
control_trajectory = None
control_trajectory_time = 0.0


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def state_from_detection(det, camera_matrix):
    half = TAG_SIZE / 2.0
    tag_corners = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    ok, rvec, tvec = cv2.solvePnP(
        tag_corners,
        np.asarray(det.corners, dtype=np.float64),
        camera_matrix,
        None,
        flags=getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE),
    )
    if not ok:
        return None
    rotation_camera_tag = cv2.Rodrigues(rvec)[0]
    rotation_tag_camera = rotation_camera_tag.T
    camera_position_tag = -rotation_tag_camera @ np.asarray(tvec).reshape(3)
    x, y = -camera_position_tag[2], camera_position_tag[0]
    body_x_tag = -(rotation_tag_camera @ np.array([0.0, 0.0, 1.0]))
    heading_x, heading_y = -body_x_tag[2], body_x_tag[0]
    return np.array([x, y, wrap(-math.atan2(heading_y, heading_x))])


def camera_thread(target_tag_id):
    global robot_state, robot_state_time
    try:
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
        detector = Detector(families="tag16h5", nthreads=2, quad_decimate=1.0)

        with dai.Device() as device:
            intrinsics = np.asarray(
                device.readFactoryCalibration().getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, WIDTH, HEIGHT),
                dtype=np.float64,
            )
            device.startPipeline(pipeline)
            queue = device.getOutputQueue("rgb", maxSize=1, blocking=False)
            selected_id = target_tag_id
            while not stop.is_set() and not rospy.is_shutdown():
                packet = queue.tryGet()
                if packet is None:
                    time.sleep(0.002)
                    continue
                gray = cv2.cvtColor(packet.getCvFrame(), cv2.COLOR_BGR2GRAY)
                detections = [d for d in detector.detect(gray) if d.decision_margin > 35.0]
                if selected_id is None and detections:
                    selected_id = max(detections, key=lambda d: d.decision_margin).tag_id
                    print(f"tracking tag {selected_id}")
                detections = [d for d in detections if d.tag_id == selected_id]
                if not detections:
                    continue
                state = state_from_detection(max(detections, key=lambda d: d.decision_margin), intrinsics)
                if state is not None and np.all(np.isfinite(state)):
                    with lock:
                        robot_state = state
                        robot_state_time = time.monotonic()
                # print(state)
    except Exception as exc:
        print(f"camera thread failed: {exc}")
        stop.set()


def mpc_thread():
    global control_trajectory, control_trajectory_time
    x = cp.Variable((N + 1, 3))
    u = cp.Variable((N, 3))
    x0 = cp.Parameter(3)
    u_prev_param = cp.Parameter(3)
    terminal_factor = cp.Parameter((3, 3))
    terminal_target = cp.Parameter(3)
    bearing_gradient = cp.Parameter(3)
    bearing_offset = cp.Parameter()
    weighted_gradient = cp.Parameter(3)
    weighted_offset = cp.Parameter()

    constraints = [x[0] == x0]
    for k in range(N):
        constraints += [x[k + 1] == x[k] + DT * u[k], -U_MAX <= u[k], u[k] <= U_MAX]
    constraints += [-DU_MAX <= u[0] - u_prev_param, u[0] - u_prev_param <= DU_MAX]
    for k in range(1, N):
        constraints += [-DU_MAX <= u[k] - u[k - 1], u[k] - u[k - 1] <= DU_MAX]
    beta = [bearing_offset + bearing_gradient @ x[k] for k in range(N + 1)]
    constraints += [item <= ALPHA for item in beta] + [item >= -ALPHA for item in beta]

    cost = cp.sum_squares(terminal_factor @ x[N] - terminal_target)
    cost += cp.sum_squares(cp.hstack([weighted_offset + weighted_gradient @ x[k] for k in range(1, N + 1)]))
    r_factor, s_factor = np.sqrt(R), np.sqrt(S)
    for k in range(N):
        cost += cp.sum_squares(r_factor @ u[k])
    cost += cp.sum_squares(s_factor @ (u[0] - u_prev_param))
    for k in range(1, N):
        cost += cp.sum_squares(s_factor @ (u[k] - u[k - 1]))
    problem = cp.Problem(cp.Minimize(cost), constraints)
    if not problem.is_dcp(dpp=True):
        raise RuntimeError("MPC is not DPP compliant")

    samples = deque(maxlen=3)
    last_sample_time = -1.0
    next_solve_time = 0.0
    u_prev = np.zeros(3)
    while not stop.is_set() and not rospy.is_shutdown():
        tick = time.monotonic()
        with lock:
            measured = None if robot_state is None else robot_state.copy()
            measured_time = robot_state_time
        if measured is not None and measured_time != last_sample_time:
            samples.append(measured)
            last_sample_time = measured_time
        if len(samples) == 3 and tick >= next_solve_time:
            next_solve_time = tick + DT
            states = np.asarray(samples)
            state = np.median(states, axis=0)
            state[2] = math.atan2(np.median(np.sin(states[:, 2])), np.median(np.cos(states[:, 2])))
            distance = max(float(np.hypot(state[0], state[1])), 1e-6)
            gradient = np.array([-state[1] / distance**2, state[0] / distance**2, 1.0])
            beta0 = wrap(math.atan2(state[1], state[0]) + state[2])
            offset = beta0 - gradient @ state
            factor = np.diag(np.sqrt([P_XY, P_XY, P_THETA / (distance**2)]))
            x0.value = state
            u_prev_param.value = u_prev
            terminal_factor.value = factor
            terminal_target.value = factor @ GOAL
            bearing_gradient.value = gradient
            bearing_offset.value = offset
            weighted_gradient.value = math.sqrt(distance) * gradient
            weighted_offset.value = math.sqrt(distance) * offset
            solve_start = time.monotonic()
            solved = True
            try:
                problem.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            except cp.SolverError as exc:
                solved = False
                print(f"MPC solve error: {exc}")
            solve_ms = 1000.0 * (time.monotonic() - solve_start)
            if solved and problem.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE) and u.value is not None:
                trajectory = np.asarray(u.value).copy()
                u_prev = trajectory[0].copy()
                with lock:
                    control_trajectory = trajectory
                    control_trajectory_time = time.monotonic()
                print(f"state={np.round(state, 3)} u0={np.round(trajectory[0], 3)} solve={solve_ms:.1f}ms")
            else:
                with lock:
                    control_trajectory = None
                print(f"MPC failed: {problem.status} ({solve_ms:.1f}ms)")
        stop.wait(max(0.0, 1.0 / CAMERA_HZ - (time.monotonic() - tick)))


def axis_command(value, slope, intercept, deadband=0.0, minimum=0.0):
    if abs(value) < deadband or value == 0.0:
        return 0.0
    return float(np.clip(math.copysign(max(slope * abs(value) + intercept, minimum), value), -1.0, 1.0))


def control_thread(movement):
    tracking_started = False
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            tick = time.monotonic()
            with lock:
                state = None if robot_state is None else robot_state.copy()
                state_time = robot_state_time
                trajectory = None if control_trajectory is None else control_trajectory.copy()
                trajectory_time = control_trajectory_time
            if state is not None:
                tracking_started = True
            if tracking_started and tick - state_time > TAG_TIMEOUT:
                print(f"tag lost for more than {TAG_TIMEOUT:.1f}s; stopping")
                stop.set()
            if state is None or trajectory is None or tick - trajectory_time > TAG_TIMEOUT or stop.is_set():
                movement.stop()
            else:
                command = trajectory[min(int((tick - trajectory_time) / DT), N - 1)]
                ux, uy, theta_dot = command
                c, s = math.cos(state[2]), math.sin(state[2])
                forward = float(np.clip(-c * ux + s * uy, -1.0, 1.0))
                left = float(np.clip(-s * ux - c * uy, -0.2, 0.2))
                omega = 0.5 * float(np.clip(theta_dot, -0.2, 0.2))
                movement.set_velocity(
                    heading=axis_command(forward, 1.23, 0.035),
                    lateral=axis_command(left, 1.23, 0.035, deadband=0.02, minimum=0.1),
                    turning=-axis_command(omega, 1.22, 0.024),
                )
                # movement.set_velocity(
                #     heading=axis_command(forward, 1.23, 0.035),
                #     lateral=axis_command(left, 1.5, 0.36, deadband=0.025, minimum=0.0),
                #     turning=-axis_command(omega, 1.22, 0.024),
                # )
            movement.publish_once()
            stop.wait(max(0.0, 1.0 / CONTROL_HZ - (time.monotonic() - tick)))
    finally:
        movement.stop()
        movement.publish_once()


def main():
    global GOAL
    parser = argparse.ArgumentParser()
    parser.add_argument("--tag-id", type=int, default=None)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("anymal_barebones_apriltag_mpc", anonymous=False)
    movement = MovementController(rate_hz=int(CONTROL_HZ))
    threads = [
        threading.Thread(target=camera_thread, args=(args.tag_id,), name="camera"),
        threading.Thread(target=mpc_thread, name="mpc"),
        threading.Thread(target=control_thread, args=(movement,), name="control"),
    ]
    print("starting immediately when three tag samples are available; Ctrl-C to stop")
    for thread in threads:
        thread.start()
    try:
        while not stop.is_set() and not rospy.is_shutdown():
            if any(not thread.is_alive() for thread in threads):
                stop.set()
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop.set()
    finally:
        stop.set()
        movement.stop()
        movement.publish_once()
        for thread in threads:
            thread.join(timeout=2.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
