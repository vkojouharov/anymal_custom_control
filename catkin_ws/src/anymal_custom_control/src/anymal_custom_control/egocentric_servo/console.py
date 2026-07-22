#!/usr/bin/env python3
"""Dedicated web console for ANYmal egocentric AprilTag visual servo tests."""

from __future__ import annotations

import argparse
import json
import logging
import socket
import threading
import time
from typing import Optional

from flask import Flask, Response, jsonify, render_template_string, request

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from anymal_custom_control.egocentric_servo.constants import (
    APRILTAG_DETECTIONS_TOPIC,
    APRILTAG_STATS_TOPIC,
    APRILTAG_TAG_LENGTH_M,
    COMMAND_TOPIC,
    EGOCENTRIC_CAMERA_ROTATE_180,
    RGB_COMPRESSED_TOPIC,
    STATUS_TOPIC,
    TRAJECTORY_TOPIC,
)


app = Flask(__name__)
lock = threading.Lock()
new_frame = threading.Event()
latest_rgb: Optional[bytes] = None
latest_rgb_stamp_sec = 0.0
apriltag_stats: dict = {}
apriltag_detections: dict = {}
servo_status: dict = {
    "state": "WAITING",
    "message": "Waiting for /anymal/egocentric_servo/status_json",
    "apriltag_tag_length_m": APRILTAG_TAG_LENGTH_M,
}
trajectory: dict = {"points": []}
command_pub: Optional[rospy.Publisher] = None


HTML_PAGE = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ANYmal Egocentric Servo</title>
  <style>
    :root {
      --bg: #101418;
      --panel: #172028;
      --line: #2d3c48;
      --text: #edf4f8;
      --muted: #9dacb8;
      --accent: #63d2bf;
      --warn: #ffbd5a;
      --danger: #ff6f6f;
      --ok: #73d07d;
    }
    * { box-sizing: border-box; }
    body { margin: 0; background: var(--bg); color: var(--text); font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
    main { max-width: 1500px; margin: 0 auto; padding: 18px; }
    header { display: flex; align-items: flex-start; justify-content: space-between; gap: 18px; margin-bottom: 16px; }
    h1 { margin: 0 0 6px; font-size: 28px; }
    p { margin: 0; color: var(--muted); }
    .grid { display: grid; grid-template-columns: minmax(420px, 1.35fr) minmax(340px, 0.8fr); gap: 16px; align-items: start; }
    section { border: 1px solid var(--line); background: var(--panel); padding: 14px; border-radius: 6px; }
    h2 { margin: 0 0 12px; font-size: 17px; }
    img { display: block; width: 100%; background: #050607; border: 1px solid var(--line); }
    img.rotate-180 { transform: rotate(180deg); }
    .controls { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
    button { min-height: 44px; border: 1px solid var(--line); background: #22303a; color: var(--text); font-weight: 700; border-radius: 4px; cursor: pointer; }
    button:hover { border-color: var(--accent); }
    button:disabled { opacity: 0.25; cursor: not-allowed; border-color: var(--line); }
    button.primary { background: #1f4b45; border-color: #3b8c7f; }
    button.warn { background: #60471d; border-color: #9d762d; }
    button.danger { background: #5a2427; border-color: #9b4147; }
    button.active-mode { outline: 2px solid var(--accent); outline-offset: 1px; }
    .metrics { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    .metric { border: 1px solid var(--line); padding: 10px; border-radius: 4px; background: rgba(255,255,255,0.02); min-height: 72px; }
    .label { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: .08em; margin-bottom: 6px; }
    .value { font-size: 18px; font-weight: 750; overflow-wrap: anywhere; }
    .note { color: var(--muted); font-size: 12px; margin-top: 5px; }
    .state { padding: 10px 12px; border: 1px solid var(--line); border-radius: 4px; background: #0f151a; }
    .state.ok { border-color: var(--ok); }
    .state.warn { border-color: var(--warn); }
    .state.fault { border-color: var(--danger); }
    canvas { width: 100%; height: 220px; border: 1px solid var(--line); background: #0b1014; }
    pre { white-space: pre-wrap; overflow-wrap: anywhere; color: var(--muted); font-size: 12px; margin: 0; }
    @media (max-width: 920px) { .grid { grid-template-columns: 1fr; } .controls { grid-template-columns: repeat(2, minmax(0, 1fr)); } }
  </style>
</head>
<body>
<main>
  <header>
    <div>
      <h1>ANYmal Egocentric Servo</h1>
      <p>AprilTag camera-relative walking servo and trajectory recording.</p>
    </div>
    <div class="state" id="state-box">
      <div class="label">Servo State</div>
      <div class="value" id="state">Waiting</div>
      <div class="note" id="message">No status yet.</div>
    </div>
  </header>
  <div class="grid">
    <div>
      <section>
        <h2>Oak-D RGB / AprilTag</h2>
        <img src="/feed/rgb" alt="Oak-D RGB stream"{% if camera_rotate_180 %} class="rotate-180"{% endif %}>
        <div class="note" id="rgb-note">Waiting for RGB stream.</div>
      </section>
      <section style="margin-top:16px;">
        <h2>Trajectory History</h2>
        <canvas id="history" width="900" height="260"></canvas>
        <div class="note" id="recording">No active run.</div>
      </section>
    </div>
    <div>
      <section>
        <h2>Controls</h2>
        <div class="controls">
          <button id="btn-rest" onclick="sendMode('Rest')">Rest</button>
          <button id="btn-stand" onclick="sendMode('Stand')">Stand</button>
          <button id="btn-walk" onclick="sendMode('Walk')" class="primary">Walk</button>
          <button id="btn-arm" onclick="sendCommand('arm')">Arm</button>
          <button id="btn-start" onclick="sendCommand('start')" class="primary">Start Trajectory</button>
          <button id="btn-pause" onclick="sendCommand('pause')" class="warn">Pause</button>
          <button id="btn-resume" onclick="sendCommand('resume')">Resume</button>
          <button id="btn-stop" onclick="sendCommand('stop')" class="danger">Stop</button>
        </div>
        <div class="note" id="command-result">Commands go to the servo node, not directly to robot motion topics.</div>
      </section>
      <section style="margin-top:16px;">
        <h2>Telemetry</h2>
        <div class="metrics">
          <div class="metric"><div class="label">Tag Length</div><div class="value" id="tag-length">-</div></div>
          <div class="metric"><div class="label">Target</div><div class="value" id="target">-</div></div>
          <div class="metric"><div class="label">Range</div><div class="value" id="range">-</div></div>
          <div class="metric"><div class="label">Lateral</div><div class="value" id="lateral">-</div></div>
          <div class="metric"><div class="label">Bearing</div><div class="value" id="bearing">-</div></div>
          <div class="metric"><div class="label">Command</div><div class="value" id="command">0 0 0</div></div>
          <div class="metric"><div class="label">Freshness</div><div class="value" id="freshness">-</div></div>
          <div class="metric"><div class="label">Active Mode</div><div class="value" id="mode">-</div></div>
        </div>
      </section>
      <section style="margin-top:16px;">
        <h2>Raw Status</h2>
        <pre id="raw">Waiting...</pre>
      </section>
    </div>
  </div>
</main>
<script>
function fmt(v, digits = 3) {
  if (v === null || v === undefined || Number.isNaN(Number(v))) return "-";
  return Number(v).toFixed(digits);
}
async function sendCommand(command) {
  const res = await fetch("/api/command", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({command})
  });
  const data = await res.json();
  document.getElementById("command-result").textContent = data.ok ? `Sent ${command}` : `Failed: ${data.error}`;
}
async function sendMode(mode) {
  const res = await fetch("/api/command", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({command: "mode", mode})
  });
  const data = await res.json();
  document.getElementById("command-result").textContent = data.ok ? `Requested ${mode}` : `Failed: ${data.error}`;
}
function classifyState(state) {
  if (["TRACKING", "ARMED", "TARGET_REACHED"].includes(state)) return "state ok";
  if (["PAUSED", "PAUSED_LOST_TAG", "STOPPED", "IDLE"].includes(state)) return "state warn";
  if (["FAULT"].includes(state)) return "state fault";
  return "state";
}
function activeMode(status) {
  return status.active_mode || status.observed_mode || status.requested_mode || null;
}
function setEnabled(id, enabled) {
  const el = document.getElementById(id);
  if (el) el.disabled = !enabled;
}
function setModeActive(id, active) {
  const el = document.getElementById(id);
  if (el) el.classList.toggle("active-mode", active);
}
function updateControls(status) {
  const state = status.state || "";
  const mode = activeMode(status);
  const walk = mode === "Walk";
  const tagFresh = Boolean((status.freshness || {}).tag_fresh);
  const tracking = state === "TRACKING";
  setEnabled("btn-rest", !tracking);
  setEnabled("btn-stand", !tracking);
  setEnabled("btn-walk", !tracking);
  setEnabled("btn-arm", walk && !["TRACKING", "ARMED"].includes(state));
  setEnabled("btn-start", state === "ARMED" && walk && tagFresh);
  setEnabled("btn-pause", tracking);
  setEnabled("btn-resume", ["PAUSED", "PAUSED_LOST_TAG"].includes(state) && walk && tagFresh);
  setEnabled("btn-stop", !["IDLE", "STOPPED", "WAITING"].includes(state));
  setModeActive("btn-rest", mode === "Rest");
  setModeActive("btn-stand", mode === "Stand");
  setModeActive("btn-walk", mode === "Walk");
}
function drawHistory(points) {
  const canvas = document.getElementById("history");
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = "#2d3c48";
  ctx.strokeRect(0, 0, canvas.width, canvas.height);
  if (!points || points.length < 2) return;
  const valid = points.filter(p => Number.isFinite(Number(p.tag_range_m)));
  if (valid.length < 2) return;
  const minT = valid[0].t;
  const maxT = valid[valid.length - 1].t;
  const maxRange = Math.max(...valid.map(p => Number(p.tag_range_m)), 0.6);
  ctx.strokeStyle = "#63d2bf";
  ctx.lineWidth = 2;
  ctx.beginPath();
  valid.forEach((p, i) => {
    const x = ((p.t - minT) / Math.max(0.001, maxT - minT)) * (canvas.width - 30) + 15;
    const y = canvas.height - 15 - (Number(p.tag_range_m) / maxRange) * (canvas.height - 30);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}
async function refresh() {
  try {
    const data = await fetch("/api/status").then(r => r.json());
    const status = data.servo_status || {};
    const tag = status.tag || {};
    const cmd = status.command || {};
    const fresh = status.freshness || {};
    const rec = status.recording || {};
    document.getElementById("state").textContent = status.state || "Waiting";
    document.getElementById("message").textContent = status.message || "";
    document.getElementById("state-box").className = classifyState(status.state || "");
    document.getElementById("rgb-note").textContent = `RGB age ${fmt(data.rgb_age_sec, 2)} s`;
    document.getElementById("tag-length").textContent = `${fmt(status.apriltag_tag_length_m, 5)} m`;
    document.getElementById("target").textContent = `${status.target_tag_id ?? "best"} @ ${fmt(status.target_distance_m, 2)} m`;
    document.getElementById("range").textContent = `${fmt(tag.range_m)} m`;
    document.getElementById("lateral").textContent = `${fmt(tag.lateral_right_m)} m right`;
    document.getElementById("bearing").textContent = `${fmt((tag.bearing_rad || 0) * 180 / Math.PI, 1)} deg`;
    document.getElementById("command").textContent = `${fmt(cmd.heading, 2)} ${fmt(cmd.lateral, 2)} ${fmt(cmd.turning, 2)}`;
    document.getElementById("freshness").textContent = `tag ${fresh.tag_fresh ? "fresh" : "stale"} | odom ${fresh.odom_fresh ? "fresh" : "stale"}`;
    const mode = activeMode(status);
    document.getElementById("mode").textContent = mode || "-";
    let recordingText = "No active run.";
    if (rec.active) {
      recordingText = `Recording ${rec.sample_count} samples | ${rec.run_dir}`;
    } else if (rec.archive_dir) {
      recordingText = `Archived: ${rec.archive_dir} | source: ${rec.run_dir}`;
    } else if (rec.archive_error) {
      recordingText = `Saved in tmp: ${rec.run_dir} | archive failed: ${rec.archive_error}`;
    } else if (rec.run_dir) {
      recordingText = `Saved: ${rec.run_dir}`;
    }
    const video = rec.video || {};
    if (video.error) {
      recordingText += ` | video failed: ${video.error}`;
    } else if (video.enabled && video.path) {
      recordingText += ` | video ${video.frame_count || 0} frames`;
    }
    document.getElementById("recording").textContent = recordingText;
    updateControls(status);
    document.getElementById("raw").textContent = JSON.stringify(status, null, 2);
    drawHistory((data.trajectory || {}).points || []);
  } catch (err) {
    document.getElementById("message").textContent = String(err);
  }
}
setInterval(refresh, 250);
refresh();
</script>
</body>
</html>
"""


def _json_from_msg(msg: String) -> dict:
    try:
        return json.loads(msg.data)
    except (TypeError, ValueError, json.JSONDecodeError):
        return {}


def _rgb_cb(msg: CompressedImage) -> None:
    global latest_rgb, latest_rgb_stamp_sec
    with lock:
        latest_rgb = bytes(msg.data)
        latest_rgb_stamp_sec = time.time()
    new_frame.set()


def _stats_cb(msg: String) -> None:
    with lock:
        apriltag_stats.clear()
        apriltag_stats.update(_json_from_msg(msg))


def _detections_cb(msg: String) -> None:
    with lock:
        apriltag_detections.clear()
        apriltag_detections.update(_json_from_msg(msg))


def _status_cb(msg: String) -> None:
    with lock:
        servo_status.clear()
        servo_status.update(_json_from_msg(msg))


def _trajectory_cb(msg: String) -> None:
    with lock:
        trajectory.clear()
        trajectory.update(_json_from_msg(msg))


def generate_rgb_stream():
    while True:
        new_frame.wait(timeout=1.0)
        with lock:
            frame = latest_rgb
        if frame is None:
            continue
        yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"


@app.route("/")
def index():
    return render_template_string(
        HTML_PAGE,
        camera_rotate_180=EGOCENTRIC_CAMERA_ROTATE_180,
    )


@app.route("/feed/rgb")
def feed_rgb():
    return Response(generate_rgb_stream(), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/status")
def api_status():
    with lock:
        return jsonify(
            {
                "server_time_sec": time.time(),
                "rgb_age_sec": time.time() - latest_rgb_stamp_sec if latest_rgb_stamp_sec else None,
                "apriltag_stats": dict(apriltag_stats),
                "apriltag_detections": dict(apriltag_detections),
                "servo_status": dict(servo_status),
                "trajectory": dict(trajectory),
            }
        )


@app.route("/api/command", methods=["POST"])
def api_command():
    if command_pub is None:
        return jsonify({"ok": False, "error": "ROS command publisher unavailable"}), 503
    payload = request.get_json(silent=True) or {}
    command = payload.get("command")
    if not isinstance(command, str):
        return jsonify({"ok": False, "error": "Missing command"}), 400
    deadline = time.time() + 1.0
    while command_pub.get_num_connections() < 1 and time.time() < deadline and not rospy.is_shutdown():
        time.sleep(0.02)
    if command_pub.get_num_connections() < 1:
        return jsonify({"ok": False, "error": "No servo node subscriber on command topic"}), 503
    command_pub.publish(String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True)))
    return jsonify({"ok": True})


def start_ros() -> None:
    global command_pub
    command_pub = rospy.Publisher(COMMAND_TOPIC, String, queue_size=10)
    rospy.Subscriber(RGB_COMPRESSED_TOPIC, CompressedImage, _rgb_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(APRILTAG_STATS_TOPIC, String, _stats_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(APRILTAG_DETECTIONS_TOPIC, String, _detections_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(STATUS_TOPIC, String, _status_cb, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber(TRAJECTORY_TOPIC, String, _trajectory_cb, queue_size=1, tcp_nodelay=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="ANYmal egocentric servo field-test console")
    parser.add_argument("--port", type=int, default=5004)
    args = parser.parse_args(rospy.myargv()[1:])

    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    rospy.init_node("anymal_egocentric_servo_console", anonymous=False)
    start_ros()

    hostname = socket.gethostname()
    try:
        local_ip = socket.gethostbyname(hostname)
    except socket.gaierror:
        local_ip = "127.0.0.1"
    print("ANYmal egocentric servo console available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")
    app.run(host="0.0.0.0", port=args.port, threaded=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
