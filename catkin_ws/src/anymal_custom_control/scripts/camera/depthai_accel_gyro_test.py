#!/usr/bin/env python3
"""Stream OAK-D IMU data to a small browser dashboard.

Shows:
  - raw accelerometer and gyroscope time series over the past 5 seconds
  - latest numeric accel/gyro values
  - fused outputs when supported by the IMU and enabled

Usage:
    python3 depthai_accel_gyro_test.py
    python3 depthai_accel_gyro_test.py --port 5005
    python3 depthai_accel_gyro_test.py --imu-rate 100 --plot-window-sec 5
"""

import argparse
import math
import socket
import threading
import time
from collections import deque

import depthai as dai
from flask import Flask, jsonify, render_template_string

app = Flask(__name__)

DEFAULT_PORT = 5005
DEFAULT_IMU_RATE_HZ = 100
DEFAULT_PLOT_WINDOW_SEC = 5.0
MAX_PLOT_POINTS = 250
RAW_BATCH_THRESHOLD = 1
RAW_MAX_BATCH_REPORTS = 10

state_lock = threading.Lock()
imu_status = {
    "connected": False,
    "imu_type": "unknown",
    "firmware_version": "unknown",
    "fused_enabled": False,
    "raw_rate_hz": 0,
    "latest": {
        "accel": None,
        "gyro": None,
        "linear_accel": None,
        "gravity": None,
        "rotation_vector": None,
    },
    "counts": {
        "accel": 0,
        "gyro": 0,
        "linear_accel": 0,
        "gravity": 0,
        "rotation_vector": 0,
    },
    "fps": {
        "accel": 0.0,
        "gyro": 0.0,
    },
}
history = deque()

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>DepthAI IMU Test</title>
    <style>
        :root {
            --bg: #111111;
            --panel: #1b1b1b;
            --line: #2f2f2f;
            --text: #e8e8e8;
            --muted: #bdbdbd;
            --accel-x: #ff6b6b;
            --accel-y: #ffd166;
            --accel-z: #4ecdc4;
            --gyro-x: #7b9cff;
            --gyro-y: #ff9ff3;
            --gyro-z: #8bd450;
        }
        body {
            background: var(--bg);
            color: var(--text);
            font-family: monospace;
            margin: 20px;
        }
        h1 {
            text-align: center;
            color: #4caf50;
            margin-bottom: 18px;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(420px, 1fr));
            gap: 18px;
        }
        .panel {
            background: var(--panel);
            border: 1px solid var(--line);
            border-radius: 8px;
            padding: 14px;
        }
        .panel h2 {
            margin: 0 0 10px 0;
            color: var(--text);
        }
        .stats {
            color: var(--muted);
            font-size: 15px;
            line-height: 1.6;
            white-space: pre-wrap;
        }
        canvas {
            width: 100%;
            height: 220px;
            background: #0d0d0d;
            border: 1px solid var(--line);
            border-radius: 6px;
            display: block;
        }
        .legend {
            margin-top: 10px;
            font-size: 14px;
            color: var(--muted);
        }
    </style>
</head>
<body>
    <h1>DepthAI IMU Test</h1>
    <div class="grid">
        <div class="panel">
            <h2>Device</h2>
            <div class="stats" id="device-stats">Waiting for IMU...</div>
        </div>
        <div class="panel">
            <h2>Latest Values</h2>
            <div class="stats" id="latest-stats">Waiting for samples...</div>
        </div>
        <div class="panel">
            <h2>Fused Outputs</h2>
            <div class="stats" id="fused-stats">Waiting for fused outputs...</div>
        </div>
        <div class="panel">
            <h2>Accelerometer</h2>
            <canvas id="accel-canvas" width="760" height="220"></canvas>
            <div class="legend">X red, Y yellow, Z cyan</div>
        </div>
        <div class="panel">
            <h2>Gyroscope</h2>
            <canvas id="gyro-canvas" width="760" height="220"></canvas>
            <div class="legend">X blue, Y pink, Z green</div>
        </div>
    </div>
    <script>
        const accelColors = { x: '#ff6b6b', y: '#ffd166', z: '#4ecdc4' };
        const gyroColors = { x: '#7b9cff', y: '#ff9ff3', z: '#8bd450' };

        function fmtVec(vec, unit) {
            if (!vec) return 'unavailable';
            return `x ${vec.x.toFixed(3)} ${unit}, y ${vec.y.toFixed(3)} ${unit}, z ${vec.z.toFixed(3)} ${unit}`;
        }

        function fmtQuat(q) {
            if (!q) return 'unavailable';
            return `i ${q.i.toFixed(4)}, j ${q.j.toFixed(4)}, k ${q.k.toFixed(4)}, real ${q.real.toFixed(4)}, acc ${q.accuracy.toFixed(4)} rad`;
        }

        function drawPlot(canvasId, samples, key, colors, unitLabel) {
            const canvas = document.getElementById(canvasId);
            const ctx = canvas.getContext('2d');
            const width = canvas.width;
            const height = canvas.height;
            const pad = 28;

            ctx.clearRect(0, 0, width, height);
            ctx.fillStyle = '#0d0d0d';
            ctx.fillRect(0, 0, width, height);

            if (!samples.length) {
                ctx.fillStyle = '#888';
                ctx.font = '16px monospace';
                ctx.fillText('No data', width / 2 - 34, height / 2);
                return;
            }

            const values = [];
            for (const sample of samples) {
                values.push(sample[key].x, sample[key].y, sample[key].z);
            }
            let minVal = Math.min(...values);
            let maxVal = Math.max(...values);
            if (minVal === maxVal) {
                minVal -= 1.0;
                maxVal += 1.0;
            }
            const span = maxVal - minVal;
            minVal -= 0.1 * span;
            maxVal += 0.1 * span;

            const t0 = samples[0].t;
            const t1 = samples[samples.length - 1].t;
            const dt = Math.max(t1 - t0, 1e-6);

            ctx.strokeStyle = '#2f2f2f';
            ctx.lineWidth = 1;
            for (let i = 0; i <= 4; i++) {
                const y = pad + (height - 2 * pad) * i / 4;
                ctx.beginPath();
                ctx.moveTo(pad, y);
                ctx.lineTo(width - pad, y);
                ctx.stroke();
            }

            ctx.fillStyle = '#bbb';
            ctx.font = '12px monospace';
            ctx.fillText(`${maxVal.toFixed(2)} ${unitLabel}`, 4, pad + 4);
            ctx.fillText(`${minVal.toFixed(2)} ${unitLabel}`, 4, height - pad + 4);

            function yMap(v) {
                const alpha = (v - minVal) / (maxVal - minVal);
                return height - pad - alpha * (height - 2 * pad);
            }

            function xMap(t) {
                const alpha = (t - t0) / dt;
                return pad + alpha * (width - 2 * pad);
            }

            for (const axis of ['x', 'y', 'z']) {
                ctx.strokeStyle = colors[axis];
                ctx.lineWidth = 2;
                ctx.beginPath();
                samples.forEach((sample, idx) => {
                    const x = xMap(sample.t);
                    const y = yMap(sample[key][axis]);
                    if (idx === 0) ctx.moveTo(x, y);
                    else ctx.lineTo(x, y);
                });
                ctx.stroke();
            }
        }

        async function refresh() {
            try {
                const res = await fetch('/imu');
                const data = await res.json();

                document.getElementById('device-stats').textContent =
                    `Connected: ${data.connected}\n` +
                    `IMU type: ${data.imu_type}\n` +
                    `Firmware: ${data.firmware_version}\n` +
                    `Fused enabled: ${data.fused_enabled}\n` +
                    `Configured rate: ${data.raw_rate_hz} Hz\n` +
                    `Accel FPS: ${data.fps.accel.toFixed(1)}\n` +
                    `Gyro FPS: ${data.fps.gyro.toFixed(1)}`;

                document.getElementById('latest-stats').textContent =
                    `Accel [m/s^2]: ${fmtVec(data.latest.accel, '')}\n` +
                    `Gyro [rad/s]: ${fmtVec(data.latest.gyro, '')}`;

                document.getElementById('fused-stats').textContent =
                    `Linear accel [m/s^2]: ${fmtVec(data.latest.linear_accel, '')}\n` +
                    `Gravity [m/s^2]: ${fmtVec(data.latest.gravity, '')}\n` +
                    `Rotation vector: ${fmtQuat(data.latest.rotation_vector)}`;

                drawPlot('accel-canvas', data.history, 'accel', accelColors, 'm/s^2');
                drawPlot('gyro-canvas', data.history, 'gyro', gyroColors, 'rad/s');
            } catch (err) {
                document.getElementById('device-stats').textContent = 'IMU endpoint unavailable';
                document.getElementById('latest-stats').textContent = 'IMU endpoint unavailable';
                document.getElementById('fused-stats').textContent = 'IMU endpoint unavailable';
            }
        }

        refresh();
        setInterval(refresh, 100);
    </script>
</body>
</html>
"""


def vec_to_dict(vec):
    if vec is None:
        return None
    return {"x": float(vec.x), "y": float(vec.y), "z": float(vec.z)}


def rotation_to_dict(rot):
    if rot is None:
        return None
    return {
        "i": float(rot.i),
        "j": float(rot.j),
        "k": float(rot.k),
        "real": float(rot.real),
        "accuracy": float(getattr(rot, "rotationVectorAccuracy", math.nan)),
    }


def prune_history(now_monotonic, window_sec):
    while history and (now_monotonic - history[0]["host_time"]) > window_sec:
        history.popleft()


def downsample_history(samples, max_points):
    if len(samples) <= max_points:
        return samples
    step = max(1, len(samples) // max_points)
    return samples[::step]


def build_pipeline(enable_fused, imu_type, imu_rate_hz):
    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xout = pipeline.create(dai.node.XLinkOut)

    xout.setStreamName("imu")

    imu.enableIMUSensor(
        [dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW],
        imu_rate_hz,
    )

    if enable_fused and imu_type == "BNO086":
        imu.enableIMUSensor(
            [
                dai.IMUSensor.LINEAR_ACCELERATION,
                dai.IMUSensor.ROTATION_VECTOR,
            ],
            imu_rate_hz,
        )

    imu.setBatchReportThreshold(RAW_BATCH_THRESHOLD)
    imu.setMaxBatchReports(RAW_MAX_BATCH_REPORTS)
    imu.out.link(xout.input)
    return pipeline


def update_fps(counter_name, current_time, fps_state):
    fps_state[counter_name]["count"] += 1
    elapsed = current_time - fps_state[counter_name]["t0"]
    if elapsed >= 1.0:
        with state_lock:
            imu_status["fps"][counter_name] = fps_state[counter_name]["count"] / elapsed
        fps_state[counter_name]["count"] = 0
        fps_state[counter_name]["t0"] = current_time


def imu_capture_loop(imu_rate_hz, window_sec, enable_fused):
    try:
        with dai.Device() as device:
            imu_type = str(device.getConnectedIMU())
            firmware = str(device.getIMUFirmwareVersion())
            if imu_type in {"", "NONE", "UNKNOWN", "None"}:
                with state_lock:
                    imu_status["connected"] = False
                    imu_status["imu_type"] = imu_type or "none"
                    imu_status["firmware_version"] = firmware
                return

            fused_active = enable_fused and imu_type == "BNO086"
            pipeline = build_pipeline(fused_active, imu_type, imu_rate_hz)
            device.startPipeline(pipeline)
            queue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)

            with state_lock:
                imu_status["connected"] = True
                imu_status["imu_type"] = imu_type
                imu_status["firmware_version"] = firmware
                imu_status["fused_enabled"] = fused_active
                imu_status["raw_rate_hz"] = imu_rate_hz

            fps_state = {
                "accel": {"count": 0, "t0": time.perf_counter()},
                "gyro": {"count": 0, "t0": time.perf_counter()},
            }

            while True:
                imu_data = queue.get()
                for packet in imu_data.packets:
                    host_time = time.monotonic()
                    sample = {"host_time": host_time}

                    accel = getattr(packet, "acceleroMeter", None)
                    gyro = getattr(packet, "gyroscope", None)
                    linear_accel = getattr(packet, "linearAcceleration", None)
                    gravity = getattr(packet, "gravity", None)
                    rotation_vector = getattr(packet, "rotationVector", None)

                    if accel is not None:
                        sample["accel"] = {
                            "x": float(accel.x),
                            "y": float(accel.y),
                            "z": float(accel.z),
                        }
                        with state_lock:
                            imu_status["latest"]["accel"] = sample["accel"]
                            imu_status["counts"]["accel"] += 1
                        update_fps("accel", time.perf_counter(), fps_state)

                    if gyro is not None:
                        sample["gyro"] = {
                            "x": float(gyro.x),
                            "y": float(gyro.y),
                            "z": float(gyro.z),
                        }
                        with state_lock:
                            imu_status["latest"]["gyro"] = sample["gyro"]
                            imu_status["counts"]["gyro"] += 1
                        update_fps("gyro", time.perf_counter(), fps_state)

                    if linear_accel is not None:
                        with state_lock:
                            imu_status["latest"]["linear_accel"] = {
                                "x": float(linear_accel.x),
                                "y": float(linear_accel.y),
                                "z": float(linear_accel.z),
                            }
                            imu_status["counts"]["linear_accel"] += 1

                    if gravity is not None:
                        with state_lock:
                            imu_status["latest"]["gravity"] = {
                                "x": float(gravity.x),
                                "y": float(gravity.y),
                                "z": float(gravity.z),
                            }
                            imu_status["counts"]["gravity"] += 1

                    if rotation_vector is not None:
                        with state_lock:
                            imu_status["latest"]["rotation_vector"] = {
                                "i": float(rotation_vector.i),
                                "j": float(rotation_vector.j),
                                "k": float(rotation_vector.k),
                                "real": float(rotation_vector.real),
                                "accuracy": float(
                                    getattr(rotation_vector, "rotationVectorAccuracy", math.nan)
                                ),
                            }
                            imu_status["counts"]["rotation_vector"] += 1

                    if "accel" in sample and "gyro" in sample:
                        with state_lock:
                            history.append(sample)
                            prune_history(host_time, window_sec)
    except Exception as exc:
        with state_lock:
            imu_status["connected"] = False
            imu_status["imu_type"] = "error"
            imu_status["firmware_version"] = str(exc)


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/imu")
def imu():
    with state_lock:
        now = time.monotonic()
        prune_history(now, app.config["PLOT_WINDOW_SEC"])
        history_samples = [
            {
                "t": sample["host_time"],
                "accel": sample["accel"],
                "gyro": sample["gyro"],
            }
            for sample in history
        ]
        history_samples = downsample_history(history_samples, MAX_PLOT_POINTS)

        if history_samples:
            t0 = history_samples[0]["t"]
            for sample in history_samples:
                sample["t"] -= t0

        return jsonify(
            {
                "connected": imu_status["connected"],
                "imu_type": imu_status["imu_type"],
                "firmware_version": imu_status["firmware_version"],
                "fused_enabled": imu_status["fused_enabled"],
                "raw_rate_hz": imu_status["raw_rate_hz"],
                "latest": imu_status["latest"],
                "counts": imu_status["counts"],
                "fps": imu_status["fps"],
                "history": history_samples,
            }
        )


def main():
    parser = argparse.ArgumentParser(description="DepthAI accelerometer/gyroscope browser test")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="HTTP port")
    parser.add_argument(
        "--imu-rate",
        type=int,
        default=DEFAULT_IMU_RATE_HZ,
        help="Requested IMU rate for raw accel/gyro and fused outputs when enabled",
    )
    parser.add_argument(
        "--plot-window-sec",
        type=float,
        default=DEFAULT_PLOT_WINDOW_SEC,
        help="Seconds of history to keep in the browser plots",
    )
    parser.add_argument(
        "--no-fused",
        action="store_true",
        help="Disable fused outputs even if the IMU supports them",
    )
    args = parser.parse_args()

    app.config["PLOT_WINDOW_SEC"] = args.plot_window_sec

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)
    enable_fused = not args.no_fused

    thread = threading.Thread(
        target=imu_capture_loop,
        args=(args.imu_rate, args.plot_window_sec, enable_fused),
        daemon=True,
    )
    thread.start()

    print("IMU dashboard available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")
    print(f"Requested IMU rate: {args.imu_rate} Hz")
    print(f"Plot window: {args.plot_window_sec:.1f} s")
    print(f"Fused outputs requested: {enable_fused}")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
