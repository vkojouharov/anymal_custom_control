from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import depthai as dai
import numpy as np
from pupil_apriltags import Detector

from .trajectory import CameraConfig


WIDTH, HEIGHT = 640, 360
TAG_FAMILY = "tag16h5"
TAG_MARGIN_MIN = 35.0


@dataclass(frozen=True)
class TagPose:
    T_camera_tag: np.ndarray
    decision_margin: float
    stamp_sec: float


@dataclass(frozen=True)
class CameraSnapshot:
    visible_tags: dict[int, TagPose]
    latest_tags: dict[int, TagPose]
    frame_fps: float
    detection_fps: float
    usb_speed: str | None


def _device_info(mxid: str):
    for info in dai.Device.getAllAvailableDevices():
        if info.getMxId() == mxid:
            return info
    raise RuntimeError(f"DepthAI device {mxid} is not available")


def _usb_speed(device) -> str:
    return str(device.getUsbSpeed()).rsplit(".", 1)[-1].upper()


def _pipeline(fps: float) -> dai.Pipeline:
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
    camera.setFps(fps)
    camera.preview.link(output.input)
    return pipeline


class CameraWorker:
    def __init__(self, role: str, config: CameraConfig, stop: threading.Event, require_usb3: bool = False):
        self.role = role
        self.config = config
        self.stop = stop
        self.require_usb3 = require_usb3
        self.active = threading.Event()
        self.ready = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name=f"{role}_camera")
        self._visible_tags: dict[int, TagPose] = {}
        self._latest_tags: dict[int, TagPose] = {}
        self._frame_fps = 0.0
        self._detection_fps = 0.0
        self._usb_speed: str | None = None
        self._failure: str | None = None

    def start(self) -> None:
        self._thread.start()

    def join(self, timeout: float | None = None) -> None:
        self._thread.join(timeout)

    def set_active(self, enabled: bool) -> None:
        with self._lock:
            self._visible_tags = {}
            self._latest_tags = {}
        if enabled:
            self.active.set()
        else:
            self.active.clear()

    @property
    def failure(self) -> str | None:
        with self._lock:
            return self._failure

    def snapshot(self) -> CameraSnapshot:
        def clone(tags):
            return {tag_id: TagPose(tag.T_camera_tag.copy(), tag.decision_margin, tag.stamp_sec) for tag_id, tag in tags.items()}

        with self._lock:
            return CameraSnapshot(clone(self._visible_tags), clone(self._latest_tags), self._frame_fps, self._detection_fps, self._usb_speed)

    def _run(self) -> None:
        try:
            detector = Detector(families=TAG_FAMILY, nthreads=2, quad_decimate=1.0)
            info = _device_info(self.config.mxid)
            with dai.Device(info) as device:
                speed = _usb_speed(device)
                if self.require_usb3 and speed not in {"SUPER", "SUPER_PLUS"}:
                    raise RuntimeError(f"{self.role} camera {self.config.mxid} negotiated {speed}, expected USB3")
                calibration = device.readCalibration()
                raw_K = np.asarray(calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, WIDTH, HEIGHT), dtype=float)
                distortion = np.asarray(calibration.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A), dtype=float).reshape(-1)
                corrected_K, _ = cv2.getOptimalNewCameraMatrix(raw_K, distortion, (WIDTH, HEIGHT), 0.0, (WIDTH, HEIGHT))
                map_1, map_2 = cv2.initUndistortRectifyMap(raw_K, distortion, None, corrected_K, (WIDTH, HEIGHT), cv2.CV_16SC2)
                camera_params = (float(corrected_K[0, 0]), float(corrected_K[1, 1]), float(corrected_K[0, 2]), float(corrected_K[1, 2]))
                with self._lock:
                    self._usb_speed = speed
                device.startPipeline(_pipeline(self.config.fps))
                queue = device.getOutputQueue("rgb", maxSize=1, blocking=False)
                rate_start = last_frame = time.monotonic()
                frames = detections_run = 0
                warned = False
                was_active = self.active.is_set()
                while not self.stop.is_set():
                    packet = queue.tryGet()
                    now = time.monotonic()
                    if packet is None:
                        if now - last_frame > 1.0:
                            raise RuntimeError(f"{self.role} camera produced no frames for 1 second")
                        self.stop.wait(0.002)
                        continue
                    last_frame = now
                    frames += 1
                    if not self.ready.is_set():
                        print(
                            f"{self.role} camera ready: mxid={self.config.mxid} usb={speed} "
                            f"fps={self.config.fps:g} calibration=EEPROM distortion_coefficients={len(distortion)}"
                        )
                        self.ready.set()
                    active = self.active.is_set()
                    if active != was_active:
                        rate_start = now
                        frames = detections_run = 0
                        warned = False
                        was_active = active
                    if active:
                        frame = cv2.remap(packet.getCvFrame(), map_1, map_2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
                        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                        detections = detector.detect(gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=self.config.tag_size_m)
                        detections_run += 1
                        best = {}
                        for detection in detections:
                            if detection.decision_margin <= TAG_MARGIN_MIN:
                                continue
                            tag_id = int(detection.tag_id)
                            if tag_id not in best or detection.decision_margin > best[tag_id].decision_margin:
                                best[tag_id] = detection
                        poses = {}
                        for tag_id, detection in best.items():
                            transform = np.eye(4, dtype=float)
                            transform[:3, :3] = np.asarray(detection.pose_R, dtype=float).reshape(3, 3)
                            transform[:3, 3] = np.asarray(detection.pose_t, dtype=float).reshape(3)
                            if np.all(np.isfinite(transform)):
                                poses[tag_id] = TagPose(transform, float(detection.decision_margin), now)
                        with self._lock:
                            self._visible_tags = poses
                            self._latest_tags.update(poses)
                    elapsed = now - rate_start
                    if elapsed >= 1.0:
                        frame_fps = frames / elapsed
                        detection_fps = detections_run / elapsed
                        with self._lock:
                            self._frame_fps = frame_fps
                            self._detection_fps = detection_fps
                        threshold = 25.0 if self.require_usb3 else 8.0
                        if active and detection_fps < threshold and not warned:
                            print(f"warning: {self.role} detection rate is {detection_fps:.1f} Hz (target >= {threshold:.0f})")
                            warned = True
                        elif detection_fps >= threshold:
                            warned = False
                        frames = detections_run = 0
                        rate_start = now
        except Exception as exc:
            with self._lock:
                self._failure = f"{self.role} camera failed: {exc}"
            print(self._failure)
            self.ready.set()
            self.stop.set()
