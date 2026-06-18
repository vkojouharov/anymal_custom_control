# `scripts/operator_console`

Unified browser-facing operator console apps live here.

These scripts are distinct from:

- `scripts/camera/` for camera-specific perception apps
- `scripts/run/` for user-facing launchers

Current app:

- `operator_console.py`

## Future Work

- Reduce web-stream latency when OAK-D RGB plus ANYmal front/rear RGB are all
  visible. With all three MJPEG streams active over robot WiFi, observed browser
  latency can reach roughly 200-500 ms. Add per-stream output throttling,
  downscaling, and lower JPEG quality in the Flask MJPEG layer before changing
  ROS camera topics or adding new nodes.
