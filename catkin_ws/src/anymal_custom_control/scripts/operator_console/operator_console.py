#!/usr/bin/env python3
"""Unified operator console with OAK-D perception and GIRAF arm telemetry."""

from __future__ import annotations

import argparse
import json
import math
import socket
import threading
import time

import cv2
import depthai as dai
import numpy as np
from flask import Flask, Response, jsonify, render_template_string

try:
    import rospy
    from std_msgs.msg import String
except ImportError:
    rospy = None
    String = None

try:
    from pupil_apriltags import Detector
except ImportError:
    Detector = None


app = Flask(__name__)

FRAME_WIDTH = 640
FRAME_HEIGHT = 360
RGB_FPS = 30
DEPTH_MIN_MM = 10
DEPTH_MAX_MM = 1000
APRILTAG_FAMILY = "tag16h5"
APRILTAG_DECISION_MARGIN = 50
APRILTAG_QUAD_DECIMATE = 1.0
APRILTAG_THREADS = 2
DEFAULT_TAG_SIZE_M = 0.0956
TAG_SIZE_M = DEFAULT_TAG_SIZE_M
MAX_EVENT_LOG = 40

lock = threading.Lock()
new_frame_events = {
    "rgb": threading.Event(),
    "depth": threading.Event(),
}
latest_frames = {
    "rgb": None,
    "depth": None,
}
stream_stats = {
    "rgb": {"fps": 0.0, "width": 0, "height": 0},
    "depth": {"fps": 0.0, "width": 0, "height": 0},
}
apriltag_stats = {
    "enabled": Detector is not None,
    "fps": 0.0,
    "detections": 0,
    "rgb_summary": "No detections",
    "depth_summary": "No detections",
}
arm_state = {
    "connected": False,
    "last_update_sec": 0.0,
    "data": None,
    "error": None,
}
event_log = []
anymal_status = {
    "connected": False,
    "summary": "Telemetry integration pending",
    "details": "ANYmal D base telemetry will be integrated here in a later pass.",
}


def _sanitize_json_value(value):
    if isinstance(value, dict):
        return {key: _sanitize_json_value(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_sanitize_json_value(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
    <title>ANYmal Operator Console</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <style>
        :root {
            --bg-0: #0d1114;
            --bg-1: #12181d;
            --panel: #1a232b;
            --line: #2f404d;
            --line-strong: #4b6275;
            --text: #eef4f8;
            --muted: #9eb0bd;
            --accent: #76e0d6;
            --success: #66d17a;
            --warn: #ffb454;
            --danger: #ff6b6b;
            --shadow: rgba(0, 0, 0, 0.24);
        }
        * {
            box-sizing: border-box;
        }
        body {
            margin: 0;
            background:
                radial-gradient(circle at top left, rgba(118, 224, 214, 0.08), transparent 28%),
                linear-gradient(180deg, var(--bg-0), #0a0f12 72%);
            color: var(--text);
            font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
        }
        .shell {
            max-width: 1680px;
            margin: 0 auto;
            padding: 24px;
        }
        .topbar {
            display: flex;
            flex-wrap: wrap;
            gap: 18px;
            justify-content: space-between;
            align-items: flex-start;
            margin-bottom: 20px;
            padding: 18px 20px;
            border: 1px solid rgba(118, 224, 214, 0.18);
            background: linear-gradient(180deg, rgba(26, 35, 43, 0.96), rgba(18, 24, 29, 0.92));
            box-shadow: 0 16px 40px var(--shadow);
        }
        .branding {
            display: flex;
            flex-direction: column;
            gap: 8px;
        }
        .eyebrow {
            color: var(--accent);
            letter-spacing: 0.18em;
            font-size: 11px;
            font-weight: 700;
            text-transform: uppercase;
        }
        .title {
            margin: 0;
            font-size: clamp(28px, 3vw, 40px);
            line-height: 1.05;
            letter-spacing: -0.03em;
            font-weight: 700;
        }
        .subtitle {
            margin: 0;
            color: var(--muted);
            font-size: 14px;
            max-width: 680px;
        }
        .jumpbar {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 6px;
        }
        .jumpbar a {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            min-width: 112px;
            padding: 10px 14px;
            border: 1px solid var(--line);
            background: rgba(255, 255, 255, 0.02);
            color: var(--text);
            text-decoration: none;
            font-size: 12px;
            font-weight: 700;
            letter-spacing: 0.08em;
            text-transform: uppercase;
            transition: background 0.15s ease, border-color 0.15s ease, transform 0.15s ease;
        }
        .jumpbar a:hover {
            background: rgba(118, 224, 214, 0.1);
            border-color: rgba(118, 224, 214, 0.45);
            transform: translateY(-1px);
        }
        .status-pills {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            align-items: center;
            justify-content: flex-end;
            max-width: 560px;
        }
        .pill {
            min-width: 120px;
            padding: 12px 14px;
            border: 1px solid var(--line);
            background: rgba(255, 255, 255, 0.03);
        }
        .pill-label {
            display: block;
            font-size: 10px;
            letter-spacing: 0.12em;
            text-transform: uppercase;
            color: var(--muted);
            margin-bottom: 6px;
        }
        .pill-value {
            display: block;
            font-size: 15px;
            font-weight: 700;
        }
        .pill.healthy {
            border-color: rgba(102, 209, 122, 0.5);
        }
        .pill.warn {
            border-color: rgba(255, 180, 84, 0.5);
        }
        .pill.fault {
            border-color: rgba(255, 107, 107, 0.58);
        }
        .layout {
            display: grid;
            grid-template-columns: minmax(340px, 0.9fr) minmax(440px, 1.25fr) minmax(320px, 0.75fr);
            gap: 18px;
            align-items: start;
        }
        .column {
            display: flex;
            flex-direction: column;
            gap: 18px;
        }
        .panel {
            border: 1px solid var(--line);
            background: linear-gradient(180deg, rgba(26, 35, 43, 0.98), rgba(18, 24, 29, 0.98));
            box-shadow: 0 14px 34px var(--shadow);
        }
        .panel-header {
            display: flex;
            justify-content: space-between;
            align-items: flex-start;
            gap: 12px;
            padding: 16px 18px 12px;
            border-bottom: 1px solid rgba(255, 255, 255, 0.06);
        }
        .panel-title-group h2 {
            margin: 0;
            font-size: 20px;
            font-weight: 700;
            letter-spacing: -0.02em;
        }
        .panel-title-group p {
            margin: 6px 0 0;
            color: var(--muted);
            font-size: 13px;
        }
        .panel-tag {
            padding: 6px 10px;
            border: 1px solid rgba(118, 224, 214, 0.28);
            color: var(--accent);
            font-size: 11px;
            font-weight: 700;
            letter-spacing: 0.12em;
            text-transform: uppercase;
            white-space: nowrap;
        }
        .panel-body {
            padding: 18px;
        }
        .telemetry-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 12px;
        }
        .telemetry-card {
            padding: 14px;
            border: 1px solid rgba(255, 255, 255, 0.06);
            background: rgba(255, 255, 255, 0.03);
        }
        .telemetry-card.wide {
            grid-column: 1 / -1;
        }
        .telemetry-label {
            color: var(--muted);
            font-size: 11px;
            letter-spacing: 0.12em;
            text-transform: uppercase;
            margin-bottom: 8px;
        }
        .telemetry-value {
            font-size: 28px;
            font-weight: 700;
            line-height: 1.05;
            letter-spacing: -0.03em;
        }
        .telemetry-note {
            margin-top: 8px;
            color: var(--muted);
            font-size: 12px;
            line-height: 1.4;
        }
        .mini-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 10px;
        }
        .mini-metric {
            padding: 10px 12px;
            border: 1px solid rgba(255, 255, 255, 0.06);
            background: rgba(255, 255, 255, 0.02);
        }
        .mini-metric span {
            display: block;
        }
        .mini-metric .name {
            color: var(--muted);
            font-size: 11px;
            letter-spacing: 0.1em;
            text-transform: uppercase;
        }
        .mini-metric .value {
            margin-top: 6px;
            font-size: 18px;
            font-weight: 700;
        }
        .vector-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 10px;
        }
        .vector-item {
            padding: 10px 12px;
            border: 1px solid rgba(255, 255, 255, 0.05);
            background: rgba(255, 255, 255, 0.025);
        }
        .vector-item .axis {
            color: var(--muted);
            font-size: 11px;
            letter-spacing: 0.12em;
            text-transform: uppercase;
        }
        .vector-item .value {
            margin-top: 6px;
            font-size: 18px;
            font-weight: 700;
        }
        .camera-grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 16px;
        }
        .stream-panel {
            padding: 12px;
            border: 1px solid rgba(255, 255, 255, 0.06);
            background: rgba(255, 255, 255, 0.02);
        }
        .stream-panel h3 {
            margin: 0 0 12px;
            font-size: 15px;
            letter-spacing: 0.02em;
        }
        .stream-panel img {
            width: 100%;
            border: 1px solid var(--line-strong);
            display: block;
            background: #0b0f13;
        }
        .stream-meta {
            margin-top: 10px;
            color: var(--muted);
            font-size: 13px;
            line-height: 1.45;
        }
        .summary-strip {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 12px;
            margin-top: 14px;
        }
        .summary-card {
            padding: 12px 14px;
            border: 1px solid rgba(255, 255, 255, 0.06);
            background: rgba(255, 255, 255, 0.025);
        }
        .summary-card .label {
            color: var(--muted);
            font-size: 11px;
            letter-spacing: 0.12em;
            text-transform: uppercase;
        }
        .summary-card .value {
            margin-top: 8px;
            font-size: 15px;
            line-height: 1.4;
        }
        .event-log {
            display: flex;
            flex-direction: column;
            gap: 10px;
            max-height: 560px;
            overflow: auto;
            padding-right: 4px;
        }
        .event-item {
            padding: 12px 14px;
            border: 1px solid rgba(255, 255, 255, 0.06);
            background: rgba(255, 255, 255, 0.025);
        }
        .event-item.warn {
            border-color: rgba(255, 180, 84, 0.4);
        }
        .event-item.err {
            border-color: rgba(255, 107, 107, 0.48);
        }
        .event-meta {
            color: var(--muted);
            font-size: 11px;
            letter-spacing: 0.1em;
            text-transform: uppercase;
            margin-bottom: 6px;
        }
        .event-message {
            font-size: 14px;
            line-height: 1.45;
        }
        .placeholder-body {
            padding: 36px 18px 30px;
            text-align: center;
        }
        .placeholder-title {
            font-size: 18px;
            font-weight: 700;
        }
        .placeholder-text {
            margin: 10px auto 0;
            color: var(--muted);
            max-width: 320px;
            line-height: 1.5;
        }
        .muted {
            color: var(--muted);
        }
        @media (max-width: 1240px) {
            .layout {
                grid-template-columns: 1fr;
            }
            .camera-grid,
            .summary-strip,
            .telemetry-grid,
            .vector-grid,
            .mini-grid {
                grid-template-columns: 1fr;
            }
            .status-pills {
                justify-content: flex-start;
            }
        }
    </style>
</head>
<body>
    <div class="shell">
        <section class="topbar">
            <div class="branding">
                <div class="eyebrow">Operator Station</div>
                <h1 class="title">ANYmal D / GIRAF Console</h1>
                <p class="subtitle">
                    Unified camera and manipulator console. Perception remains local to the OAK-D process;
                    arm telemetry is read live from the ROS controller.
                </p>
                <div class="jumpbar">
                    <a href="#arm">Arm</a>
                    <a href="#camera">Camera</a>
                    <a href="#base">ANYmal</a>
                    <a href="#events">Events</a>
                </div>
            </div>
            <div class="status-pills">
                <div class="pill" id="pill-arm">
                    <span class="pill-label">Arm</span>
                    <span class="pill-value" id="pill-arm-value">Waiting</span>
                </div>
                <div class="pill" id="pill-source">
                    <span class="pill-label">Command</span>
                    <span class="pill-value" id="pill-source-value">Unknown</span>
                </div>
                <div class="pill" id="pill-stop">
                    <span class="pill-label">Stop</span>
                    <span class="pill-value" id="pill-stop-value">Unknown</span>
                </div>
                <div class="pill healthy" id="pill-camera">
                    <span class="pill-label">Perception</span>
                    <span class="pill-value" id="pill-camera-value">Waiting</span>
                </div>
            </div>
        </section>

        <div class="layout">
            <div class="column">
                <section class="panel" id="arm">
                    <div class="panel-header">
                        <div class="panel-title-group">
                            <h2>Manipulator</h2>
                            <p>Live GIRAF arm controller state from `/giraf_arm/state`.</p>
                        </div>
                        <div class="panel-tag">Robot</div>
                    </div>
                    <div class="panel-body">
                        <div class="telemetry-grid">
                            <div class="telemetry-card">
                                <div class="telemetry-label">Controller Mode</div>
                                <div class="telemetry-value" id="arm-mode">Waiting</div>
                                <div class="telemetry-note" id="arm-source-note">No controller data yet.</div>
                            </div>
                            <div class="telemetry-card">
                                <div class="telemetry-label">End Effector</div>
                                <div class="telemetry-value" id="arm-ee">-.--- -.--- -.---</div>
                                <div class="telemetry-note">Meters in controller kinematic frame.</div>
                            </div>
                            <div class="telemetry-card wide">
                                <div class="telemetry-label">Selected Task Velocity</div>
                                <div class="vector-grid">
                                    <div class="vector-item"><div class="axis">vx</div><div class="value" id="cmd-vx">0.000</div></div>
                                    <div class="vector-item"><div class="axis">vy</div><div class="value" id="cmd-vy">0.000</div></div>
                                    <div class="vector-item"><div class="axis">vz</div><div class="value" id="cmd-vz">0.000</div></div>
                                    <div class="vector-item"><div class="axis">wx</div><div class="value" id="cmd-wx">0.000</div></div>
                                    <div class="vector-item"><div class="axis">wy</div><div class="value" id="cmd-wy">0.000</div></div>
                                    <div class="vector-item"><div class="axis">wz</div><div class="value" id="cmd-wz">0.000</div></div>
                                </div>
                            </div>
                            <div class="telemetry-card wide">
                                <div class="telemetry-label">Joint / Mechanism State</div>
                                <div class="mini-grid">
                                    <div class="mini-metric"><span class="name">Roll</span><span class="value" id="arm-roll">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Pitch</span><span class="value" id="arm-pitch">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Boom</span><span class="value" id="arm-boom">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Grip</span><span class="value" id="arm-grip">0.000</span></div>
                                    <div class="mini-metric"><span class="name">th4</span><span class="value" id="arm-th4">0.000</span></div>
                                    <div class="mini-metric"><span class="name">th5</span><span class="value" id="arm-th5">0.000</span></div>
                                    <div class="mini-metric"><span class="name">th6</span><span class="value" id="arm-th6">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Grip Cmd</span><span class="value" id="arm-grip-cmd">0.000</span></div>
                                </div>
                            </div>
                            <div class="telemetry-card wide">
                                <div class="telemetry-label">Control Health</div>
                                <div class="mini-grid">
                                    <div class="mini-metric"><span class="name">Manipulability</span><span class="value" id="arm-w">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Inv. Cond.</span><span class="value" id="arm-icn">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Sigma Min</span><span class="value" id="arm-smin">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Sigma Max</span><span class="value" id="arm-smax">0.000</span></div>
                                    <div class="mini-metric"><span class="name">Teleop Age</span><span class="value" id="teleop-age">-.---</span></div>
                                    <div class="mini-metric"><span class="name">Auto Age</span><span class="value" id="auto-age">-.---</span></div>
                                </div>
                            </div>
                        </div>
                    </div>
                </section>
            </div>

            <div class="column">
                <section class="panel" id="camera">
                    <div class="panel-header">
                        <div class="panel-title-group">
                            <h2>Perception</h2>
                            <p>OAK-D RGB, aligned depth, and AprilTag overlays.</p>
                        </div>
                        <div class="panel-tag">Camera</div>
                    </div>
                    <div class="panel-body">
                        <div class="camera-grid">
                            <div class="stream-panel">
                                <h3>RGB</h3>
                                <img src="/feed/rgb" />
                                <div class="stream-meta" id="stats-rgb">Waiting for frames...</div>
                            </div>
                            <div class="stream-panel">
                                <h3>Aligned Depth</h3>
                                <img src="/feed/depth" />
                                <div class="stream-meta" id="stats-depth">Waiting for frames...</div>
                            </div>
                        </div>
                        <div class="summary-strip">
                            <div class="summary-card">
                                <div class="label">AprilTag Summary</div>
                                <div class="value" id="stats-apriltag">Waiting for AprilTag stats...</div>
                            </div>
                            <div class="summary-card">
                                <div class="label">RGB Pose Depth</div>
                                <div class="value" id="stats-rgb-depth">Waiting for AprilTag pose depth...</div>
                            </div>
                            <div class="summary-card">
                                <div class="label">Depth Mask Summary</div>
                                <div class="value" id="stats-depth-region">Waiting for masked depth stats...</div>
                            </div>
                            <div class="summary-card">
                                <div class="label">Perception Status</div>
                                <div class="value" id="perception-health">Waiting for stream stats...</div>
                            </div>
                        </div>
                    </div>
                </section>

                <section class="panel" id="base">
                    <div class="panel-header">
                        <div class="panel-title-group">
                            <h2>ANYmal D</h2>
                            <p>Base telemetry space reserved for the next integration pass.</p>
                        </div>
                        <div class="panel-tag">Placeholder</div>
                    </div>
                    <div class="placeholder-body">
                        <div class="placeholder-title" id="anymal-summary">Telemetry integration pending</div>
                        <p class="placeholder-text" id="anymal-details">
                            This panel will host quadruped state, locomotion mode, and health data when the ANYmal telemetry interface is wired in.
                        </p>
                    </div>
                </section>
            </div>

            <div class="column">
                <section class="panel">
                    <div class="panel-header">
                        <div class="panel-title-group">
                            <h2>Control Status</h2>
                            <p>High-visibility controller readiness and command freshness.</p>
                        </div>
                        <div class="panel-tag">Status</div>
                    </div>
                    <div class="panel-body">
                        <div class="telemetry-grid">
                            <div class="telemetry-card">
                                <div class="telemetry-label">Controller</div>
                                <div class="telemetry-value" id="controller-health">Waiting</div>
                                <div class="telemetry-note" id="controller-health-note">No ROS arm state received yet.</div>
                            </div>
                            <div class="telemetry-card">
                                <div class="telemetry-label">Stop State</div>
                                <div class="telemetry-value" id="controller-stop">Unknown</div>
                                <div class="telemetry-note">`X` in joystick teleop requests a latched stop.</div>
                            </div>
                            <div class="telemetry-card wide">
                                <div class="telemetry-label">Status Snapshot</div>
                                <div class="mini-grid">
                                    <div class="mini-metric"><span class="name">Active Source</span><span class="value" id="status-source">Unknown</span></div>
                                    <div class="mini-metric"><span class="name">Cmd Source Param</span><span class="value" id="status-param-source">Unknown</span></div>
                                    <div class="mini-metric"><span class="name">Arm Freshness</span><span class="value" id="status-freshness">-.---</span></div>
                                    <div class="mini-metric"><span class="name">Detect FPS</span><span class="value" id="status-detect-fps">0.0</span></div>
                                </div>
                            </div>
                        </div>
                    </div>
                </section>

                <section class="panel" id="events">
                    <div class="panel-header">
                        <div class="panel-title-group">
                            <h2>Events</h2>
                            <p>Recent arm-controller debug events from `/giraf_arm/debug`.</p>
                        </div>
                        <div class="panel-tag">Log</div>
                    </div>
                    <div class="panel-body">
                        <div class="event-log" id="event-log">
                            <div class="event-item">
                                <div class="event-meta">Waiting</div>
                                <div class="event-message muted">No events received yet.</div>
                            </div>
                        </div>
                    </div>
                </section>
            </div>
        </div>
    </div>
    <script>
        function fmt(value, digits = 3) {
            if (value === null || value === undefined || Number.isNaN(Number(value))) {
                return '-';
            }
            return Number(value).toFixed(digits);
        }

        function escapeHtml(value) {
            return String(value)
                .replace(/&/g, '&amp;')
                .replace(/</g, '&lt;')
                .replace(/>/g, '&gt;')
                .replace(/"/g, '&quot;')
                .replace(/'/g, '&#39;');
        }

        function classifyAge(ageSec, healthyThreshold = 0.2, warnThreshold = 1.0) {
            if (!Number.isFinite(ageSec)) return 'fault';
            if (ageSec <= healthyThreshold) return 'healthy';
            if (ageSec <= warnThreshold) return 'warn';
            return 'fault';
        }

        function renderArm(stats) {
            const arm = stats.arm_state || {};
            const data = arm.data || null;
            const connected = !!arm.connected && !!data;
            const armPill = document.getElementById('pill-arm');
            const sourcePill = document.getElementById('pill-source');
            const stopPill = document.getElementById('pill-stop');

            if (!connected) {
                armPill.className = 'pill fault';
                sourcePill.className = 'pill warn';
                stopPill.className = 'pill warn';
                document.getElementById('pill-arm-value').textContent = arm.error ? 'No ROS' : 'Waiting';
                document.getElementById('pill-source-value').textContent = 'Unknown';
                document.getElementById('pill-stop-value').textContent = 'Unknown';
                document.getElementById('arm-mode').textContent = 'Waiting';
                document.getElementById('arm-source-note').textContent = arm.error || 'No controller data yet.';
                document.getElementById('controller-health').textContent = arm.error ? 'Disconnected' : 'Waiting';
                document.getElementById('controller-health-note').textContent = arm.error || 'No ROS arm state received yet.';
                document.getElementById('controller-stop').textContent = 'Unknown';
                return;
            }

            const freshness = Math.max(0.0, stats.server_time_sec - Number(data.stamp_sec || 0.0));
            const freshnessClass = classifyAge(freshness);
            const activeSource = data.active_source || 'unknown';
            const stopLatched = !!data.stop_latched;
            const teleopAge = Number(data.teleop_cmd_age_sec ?? NaN);
            const autoAge = Number(data.auto_cmd_age_sec ?? NaN);
            const metrics = data.metrics || {};
            const ee = data.end_effector || {};
            const armData = data.arm || {};
            const cmd = data.selected_task_velocity || {};

            armPill.className = `pill ${stopLatched ? 'fault' : freshnessClass}`;
            sourcePill.className = `pill ${activeSource === 'teleop' ? 'healthy' : 'warn'}`;
            stopPill.className = `pill ${stopLatched ? 'fault' : 'healthy'}`;

            document.getElementById('pill-arm-value').textContent = stopLatched ? 'Stopped' : (freshnessClass === 'healthy' ? 'Live' : freshnessClass === 'warn' ? 'Stale' : 'Lost');
            document.getElementById('pill-source-value').textContent = activeSource.toUpperCase();
            document.getElementById('pill-stop-value').textContent = stopLatched ? 'Latched' : 'Clear';

            document.getElementById('arm-mode').textContent = metrics.mode || 'Unknown';
            document.getElementById('arm-source-note').textContent =
                `Source ${activeSource} | freshness ${fmt(freshness, 3)} s | stop ${stopLatched ? 'latched' : 'clear'}`;
            document.getElementById('arm-ee').textContent = `${fmt(ee.x)} ${fmt(ee.y)} ${fmt(ee.z)}`;

            document.getElementById('cmd-vx').textContent = fmt(cmd.vx);
            document.getElementById('cmd-vy').textContent = fmt(cmd.vy);
            document.getElementById('cmd-vz').textContent = fmt(cmd.vz);
            document.getElementById('cmd-wx').textContent = fmt(cmd.wx);
            document.getElementById('cmd-wy').textContent = fmt(cmd.wy);
            document.getElementById('cmd-wz').textContent = fmt(cmd.wz);

            document.getElementById('arm-roll').textContent = fmt(armData.roll);
            document.getElementById('arm-pitch').textContent = fmt(armData.pitch);
            document.getElementById('arm-boom').textContent = fmt(armData.boom);
            document.getElementById('arm-grip').textContent = fmt(armData.grip);
            document.getElementById('arm-th4').textContent = fmt(armData.th4);
            document.getElementById('arm-th5').textContent = fmt(armData.th5);
            document.getElementById('arm-th6').textContent = fmt(armData.th6);
            document.getElementById('arm-grip-cmd').textContent = fmt(data.selected_gripper_velocity);

            document.getElementById('arm-w').textContent = fmt(metrics.manipulability);
            document.getElementById('arm-icn').textContent = fmt(metrics.inverse_condition_number);
            document.getElementById('arm-smin').textContent = fmt(metrics.sigma_min);
            document.getElementById('arm-smax').textContent = fmt(metrics.sigma_max);
            document.getElementById('teleop-age').textContent = fmt(teleopAge);
            document.getElementById('auto-age').textContent = fmt(autoAge);

            document.getElementById('controller-health').textContent = stopLatched ? 'Stopped' : (freshnessClass === 'healthy' ? 'Ready' : freshnessClass === 'warn' ? 'Degraded' : 'Lost');
            document.getElementById('controller-health-note').textContent =
                `Teleop age ${fmt(teleopAge)} s | Auto age ${fmt(autoAge)} s`;
            document.getElementById('controller-stop').textContent = stopLatched ? 'Latched' : 'Clear';
            document.getElementById('status-source').textContent = activeSource.toUpperCase();
            document.getElementById('status-param-source').textContent = (data.command_source_param || 'unknown').toUpperCase();
            document.getElementById('status-freshness').textContent = `${fmt(freshness)} s`;
        }

        function renderEvents(stats) {
            const events = stats.event_log || [];
            const root = document.getElementById('event-log');
            if (!events.length) {
                root.innerHTML = `
                    <div class="event-item">
                        <div class="event-meta">Waiting</div>
                        <div class="event-message muted">No events received yet.</div>
                    </div>
                `;
                return;
            }

            root.innerHTML = events.map((event) => {
                const level = escapeHtml(event.level || 'info');
                const stamp = escapeHtml(event.stamp || 'Unknown');
                const message = escapeHtml(event.message || 'No message');
                return `
                    <div class="event-item ${level}">
                        <div class="event-meta">${stamp} · ${level.toUpperCase()}</div>
                        <div class="event-message">${message}</div>
                    </div>
                `;
            }).join('');
        }

        function renderAnymal(stats) {
            const anymal = stats.anymal_status || {};
            document.getElementById('anymal-summary').textContent = anymal.summary || 'Telemetry integration pending';
            document.getElementById('anymal-details').textContent = anymal.details || 'ANYmal telemetry will appear here later.';
        }

        async function refreshStats() {
            try {
                const res = await fetch('/stats');
                const stats = await res.json();
                const cameraPill = document.getElementById('pill-camera');
                cameraPill.className = stats.rgb.fps > 0.1 && stats.depth.fps > 0.1 ? 'pill healthy' : 'pill warn';
                document.getElementById('pill-camera-value').textContent =
                    stats.apriltag.enabled ? `${stats.apriltag.detections} tags` : 'Detector Off';
                document.getElementById('stats-apriltag').textContent = stats.apriltag.enabled
                    ? `AprilTag ${stats.apriltag.detections} tags | Detect FPS: ${stats.apriltag.fps.toFixed(1)}`
                    : 'AprilTag detector unavailable (install pupil_apriltags)';
                document.getElementById('stats-rgb').textContent =
                    `FPS: ${stats.rgb.fps.toFixed(1)} | Size: ${stats.rgb.width}x${stats.rgb.height}`;
                document.getElementById('stats-depth').textContent =
                    `FPS: ${stats.depth.fps.toFixed(1)} | Size: ${stats.depth.width}x${stats.depth.height}`;
                document.getElementById('stats-rgb-depth').textContent =
                    `RGB pose depth: ${stats.apriltag.rgb_summary}`;
                document.getElementById('stats-depth-region').textContent =
                    `Depth mask: ${stats.apriltag.depth_summary}`;
                document.getElementById('perception-health').textContent =
                    `RGB ${stats.rgb.fps.toFixed(1)} fps | Depth ${stats.depth.fps.toFixed(1)} fps | AprilTag ${stats.apriltag.fps.toFixed(1)} fps`;
                document.getElementById('status-detect-fps').textContent = stats.apriltag.fps.toFixed(1);
                renderArm(stats);
                renderEvents(stats);
                renderAnymal(stats);
            } catch (err) {
                document.getElementById('pill-camera').className = 'pill fault';
                document.getElementById('pill-camera-value').textContent = 'Unavailable';
                document.getElementById('stats-apriltag').textContent = 'Stats unavailable';
                document.getElementById('stats-rgb').textContent = 'Stats unavailable';
                document.getElementById('stats-depth').textContent = 'Stats unavailable';
                document.getElementById('stats-rgb-depth').textContent = 'Stats unavailable';
                document.getElementById('stats-depth-region').textContent = 'Stats unavailable';
                document.getElementById('perception-health').textContent = 'Stats unavailable';
            }
        }

        refreshStats();
        setInterval(refreshStats, 250);
    </script>
</body>
</html>
"""


def _append_event(level: str, message: str, stamp_sec: float | None = None) -> None:
    timestamp = time.strftime("%H:%M:%S", time.localtime(stamp_sec or time.time()))
    with lock:
        event_log.insert(
            0,
            {
                "level": level,
                "message": message,
                "stamp": timestamp,
            },
        )
        del event_log[MAX_EVENT_LOG:]


def _arm_state_cb(msg):
    try:
        payload = json.loads(msg.data)
    except json.JSONDecodeError as exc:
        with lock:
            arm_state["error"] = f"Invalid JSON from /giraf_arm/state: {exc}"
            arm_state["connected"] = False
        _append_event("err", "Failed to decode /giraf_arm/state payload")
        return

    with lock:
        arm_state["connected"] = True
        arm_state["last_update_sec"] = time.time()
        arm_state["data"] = payload
        arm_state["error"] = None


def _arm_debug_cb(msg):
    try:
        payload = json.loads(msg.data)
        level = str(payload.get("level", "info")).lower()
        message = str(payload.get("message", "No message"))
        stamp_sec = float(payload.get("stamp_sec", time.time()))
    except Exception:
        level = "info"
        message = str(msg.data)
        stamp_sec = time.time()
    _append_event(level, message, stamp_sec=stamp_sec)


def ros_monitor_loop():
    if rospy is None or String is None:
        with lock:
            arm_state["error"] = "rospy is unavailable in this environment."
        _append_event("warn", "ROS monitor unavailable: rospy import failed")
        return

    try:
        rospy.init_node("anymal_operator_console", anonymous=True, disable_signals=True)
        rospy.Subscriber("/giraf_arm/state", String, _arm_state_cb, queue_size=1)
        rospy.Subscriber("/giraf_arm/debug", String, _arm_debug_cb, queue_size=20)
        _append_event("info", "ROS monitor connected; waiting for /giraf_arm/state")
        rospy.spin()
    except Exception as exc:
        with lock:
            arm_state["error"] = f"ROS monitor failed: {exc}"
            arm_state["connected"] = False
        _append_event("err", f"ROS monitor failed: {exc}")


def create_pipeline():
    pipeline = dai.Pipeline()

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_depth = pipeline.create(dai.node.XLinkOut)

    xout_rgb.setStreamName("rgb")
    xout_depth.setStreamName("depth")

    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setPreviewSize(FRAME_WIDTH, FRAME_HEIGHT)
    cam_rgb.setPreviewKeepAspectRatio(False)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(RGB_FPS)

    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_left.setFps(RGB_FPS)
    mono_right.setFps(RGB_FPS)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DETAIL)
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(FRAME_WIDTH, FRAME_HEIGHT)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)
    stereo.initialConfig.setConfidenceThreshold(220)
    config = stereo.initialConfig.get()
    config.postProcessing.speckleFilter.enable = True
    config.postProcessing.speckleFilter.speckleRange = 100
    config.postProcessing.temporalFilter.enable = True
    config.postProcessing.temporalFilter.alpha = 0.6
    config.postProcessing.temporalFilter.delta = 40
    config.postProcessing.spatialFilter.enable = True
    config.postProcessing.spatialFilter.holeFillingRadius = 4
    config.postProcessing.spatialFilter.numIterations = 2
    config.postProcessing.spatialFilter.alpha = 0.6
    config.postProcessing.spatialFilter.delta = 40
    config.postProcessing.thresholdFilter.minRange = DEPTH_MIN_MM
    config.postProcessing.thresholdFilter.maxRange = DEPTH_MAX_MM
    stereo.initialConfig.set(config)

    cam_rgb.preview.link(xout_rgb.input)
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)

    return pipeline


def colorize_depth(depth_frame):
    clipped = np.clip(depth_frame, DEPTH_MIN_MM, DEPTH_MAX_MM)
    normalized = ((clipped - DEPTH_MIN_MM) * 255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM)).astype(np.uint8)
    normalized[depth_frame == 0] = 0
    colored = cv2.applyColorMap(255 - normalized, cv2.COLORMAP_JET)
    colored[depth_frame == 0] = (0, 0, 0)
    return colored


def detect_apriltags(detector, frame):
    if detector is None:
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if TAG_SIZE_M is not None:
        detections = detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=CAMERA_PARAMS,
            tag_size=TAG_SIZE_M,
        )
    else:
        detections = detector.detect(gray)
    return [det for det in detections if det.decision_margin > APRILTAG_DECISION_MARGIN]


def compute_tag_corner_depths(det):
    if TAG_SIZE_M is None:
        return None

    pose_R = getattr(det, "pose_R", None)
    pose_t = getattr(det, "pose_t", None)
    if pose_R is None or pose_t is None:
        return None

    half = TAG_SIZE_M / 2.0
    tag_corners = np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )
    camera_corners = (pose_R @ tag_corners.T).T + pose_t.reshape(1, 3)
    return camera_corners[:, 2]


def compute_masked_depth_mm(depth_frame, det):
    mask = np.zeros(depth_frame.shape, dtype=np.uint8)
    polygon = np.round(det.corners).astype(np.int32)
    cv2.fillConvexPoly(mask, polygon, 255)

    region = depth_frame[mask == 255]
    valid = region[(region > 0) & (region >= DEPTH_MIN_MM) & (region <= DEPTH_MAX_MM)]
    if valid.size == 0:
        return None

    median = float(np.median(valid))
    deviations = np.abs(valid - median)
    mad = float(np.median(deviations))
    if mad > 0.0:
        valid = valid[deviations <= 3.0 * mad]
    if valid.size == 0:
        return None

    return {
        "mean_mm": float(np.mean(valid)),
        "median_mm": float(np.median(valid)),
        "count": int(valid.size),
    }


def format_rgb_summary(detections):
    if not detections:
        return "No detections"
    if TAG_SIZE_M is None:
        return "Set --tag-size-m to enable metric pose depth"

    parts = []
    for det in detections:
        corner_depths = compute_tag_corner_depths(det)
        if corner_depths is None:
            parts.append(f"ID{det.tag_id}: unavailable")
            continue
        avg_z_m = float(np.mean(corner_depths))
        parts.append(f"ID{det.tag_id}: avg Z {avg_z_m:.3f} m")
    return " | ".join(parts)


def format_depth_summary(depth_frame, detections):
    if not detections:
        return "No detections"

    parts = []
    for det in detections:
        stats = compute_masked_depth_mm(depth_frame, det)
        if stats is None:
            parts.append(f"ID{det.tag_id}: unavailable")
            continue
        parts.append(
            f"ID{det.tag_id}: median {stats['median_mm'] / 1000.0:.3f} m "
            f"mean {stats['mean_mm'] / 1000.0:.3f} m"
        )
    return " | ".join(parts)


def draw_apriltags(frame, detections):
    for det in detections:
        corners = det.corners.astype(int)

        for i in range(4):
            cv2.line(frame, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2)

        cx, cy = int(det.center[0]), int(det.center[1])
        size = 15
        cv2.line(frame, (cx - size, cy), (cx + size, cy), (0, 255, 0), 2)
        cv2.line(frame, (cx, cy - size), (cx, cy + size), (0, 255, 0), 2)

        label = f"ID:{det.tag_id} ({cx},{cy}) m:{det.decision_margin:.0f}"
        cv2.putText(
            frame,
            label,
            (corners[0][0], corners[0][1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )


def draw_apriltag_status(frame, detections, detect_fps):
    if Detector is None:
        status = "AprilTag detector unavailable"
        color = (0, 0, 255)
    else:
        status = f"AprilTag {APRILTAG_FAMILY} {len(detections)} tags {detect_fps:.1f} fps"
        color = (255, 255, 255)

    cv2.putText(
        frame,
        status,
        (10, frame.shape[0] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
    )


def update_stream(kind, frame, fps):
    with lock:
        latest_frames[kind] = frame
        stream_stats[kind]["fps"] = fps
        stream_stats[kind]["height"], stream_stats[kind]["width"] = frame.shape[:2]
    new_frame_events[kind].set()


def capture_loop():
    detector = None
    if Detector is not None:
        detector = Detector(
            families=APRILTAG_FAMILY,
            nthreads=APRILTAG_THREADS,
            quad_decimate=APRILTAG_QUAD_DECIMATE,
        )

    pipeline = create_pipeline()
    with dai.Device(pipeline) as device:
        calib = device.readFactoryCalibration()
        intrinsics = np.array(
            calib.getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A,
                FRAME_WIDTH,
                FRAME_HEIGHT,
            ),
            dtype=np.float64,
        )
        camera_params = (
            float(intrinsics[0, 0]),
            float(intrinsics[1, 1]),
            float(intrinsics[0, 2]),
            float(intrinsics[1, 2]),
        )
        global CAMERA_PARAMS
        CAMERA_PARAMS = camera_params
        queues = {
            "rgb": device.getOutputQueue(name="rgb", maxSize=4, blocking=False),
            "depth": device.getOutputQueue(name="depth", maxSize=4, blocking=False),
        }
        frame_counts = {"rgb": 0, "depth": 0}
        fps_timers = {"rgb": time.perf_counter(), "depth": time.perf_counter()}
        fps_values = {"rgb": 0.0, "depth": 0.0}
        detect_count = 0
        detect_timer = time.perf_counter()
        detect_fps = 0.0
        latest_detections = []

        while True:
            rgb_msg = queues["rgb"].tryGet()
            if rgb_msg is not None:
                rgb_frame = rgb_msg.getCvFrame()
                latest_detections = detect_apriltags(detector, rgb_frame)
                draw_apriltags(rgb_frame, latest_detections)
                detect_count += 1
                now = time.perf_counter()
                detect_elapsed = now - detect_timer
                if detect_elapsed >= 1.0:
                    detect_fps = detect_count / detect_elapsed
                    detect_count = 0
                    detect_timer = now
                frame_counts["rgb"] += 1
                elapsed = now - fps_timers["rgb"]
                if elapsed >= 1.0:
                    fps_values["rgb"] = frame_counts["rgb"] / elapsed
                    frame_counts["rgb"] = 0
                    fps_timers["rgb"] = now
                draw_apriltag_status(rgb_frame, latest_detections, detect_fps)
                with lock:
                    apriltag_stats["fps"] = detect_fps
                    apriltag_stats["detections"] = len(latest_detections)
                    apriltag_stats["rgb_summary"] = format_rgb_summary(latest_detections)
                update_stream("rgb", rgb_frame, fps_values["rgb"])

            depth_msg = queues["depth"].tryGet()
            if depth_msg is not None:
                depth_frame = depth_msg.getFrame()
                depth_color = colorize_depth(depth_frame)
                draw_apriltags(depth_color, latest_detections)
                frame_counts["depth"] += 1
                now = time.perf_counter()
                elapsed = now - fps_timers["depth"]
                if elapsed >= 1.0:
                    fps_values["depth"] = frame_counts["depth"] / elapsed
                    frame_counts["depth"] = 0
                    fps_timers["depth"] = now
                draw_apriltag_status(depth_color, latest_detections, detect_fps)
                with lock:
                    apriltag_stats["depth_summary"] = format_depth_summary(depth_frame, latest_detections)
                update_stream("depth", depth_color, fps_values["depth"])

            if rgb_msg is None and depth_msg is None:
                time.sleep(0.001)


def make_placeholder(label):
    frame = np.zeros((FRAME_HEIGHT, FRAME_WIDTH, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        label,
        (110, FRAME_HEIGHT // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (80, 80, 80),
        2,
    )
    return frame


def generate_stream(kind):
    placeholder = make_placeholder(f"Waiting for {kind} frame")

    while True:
        new_frame_events[kind].wait(timeout=1.0)
        new_frame_events[kind].clear()

        with lock:
            frame = latest_frames[kind].copy() if latest_frames[kind] is not None else placeholder

        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n"
        )


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/feed/<kind>")
def feed(kind):
    if kind not in latest_frames:
        return ("Unknown stream", 404)
    return Response(generate_stream(kind), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/stats")
def stats():
    with lock:
        arm_snapshot = {
            "connected": arm_state["connected"],
            "last_update_sec": arm_state["last_update_sec"],
            "data": arm_state["data"],
            "error": arm_state["error"],
        }
        return jsonify(
            _sanitize_json_value(
                {
                "server_time_sec": time.time(),
                "rgb": stream_stats["rgb"],
                "depth": stream_stats["depth"],
                "apriltag": apriltag_stats,
                "arm_state": arm_snapshot,
                "event_log": list(event_log),
                "anymal_status": anymal_status,
                }
            )
        )


def main():
    parser = argparse.ArgumentParser(description="Operator console with OAK-D RGB, aligned depth, AprilTag, and arm telemetry.")
    parser.add_argument("--port", type=int, default=5004, help="HTTP port")
    parser.add_argument(
        "--tag-size-m",
        type=float,
        default=DEFAULT_TAG_SIZE_M,
        help="Physical AprilTag edge length in meters for RGB pose depth estimation",
    )
    args = parser.parse_args()
    global TAG_SIZE_M
    TAG_SIZE_M = args.tag_size_m

    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    perception_thread = threading.Thread(target=capture_loop, daemon=True)
    perception_thread.start()
    ros_thread = threading.Thread(target=ros_monitor_loop, daemon=True)
    ros_thread.start()

    print("Operator console available at:")
    print(f"  http://localhost:{args.port}")
    print(f"  http://{local_ip}:{args.port}")
    print(f"Depth range colorized over {DEPTH_MIN_MM}mm to {DEPTH_MAX_MM}mm")
    if Detector is not None:
        print(
            f"AprilTag detector enabled: {APRILTAG_FAMILY}, "
            f"quad_decimate={APRILTAG_QUAD_DECIMATE}, nthreads={APRILTAG_THREADS}, "
            f"margin>{APRILTAG_DECISION_MARGIN}"
        )
        print(f"RGB pose depth enabled with tag size {TAG_SIZE_M:.4f} m")
    else:
        print("AprilTag detector disabled: install pupil_apriltags")
    if rospy is None:
        print("ROS arm telemetry disabled: rospy not importable")
    else:
        print("ROS arm telemetry enabled: listening to /giraf_arm/state and /giraf_arm/debug")

    app.run(host="0.0.0.0", port=args.port, threaded=True)


if __name__ == "__main__":
    main()
