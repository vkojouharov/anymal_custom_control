#!/usr/bin/env python3
"""Launch the ANYmal egocentric AprilTag servo stack."""

from __future__ import annotations

import argparse
import sys

from anymal_custom_control.egocentric_servo.constants import DEFAULT_ARCHIVE_DIR, DEFAULT_RECORD_DIR, DEFAULT_TAG_LOSS_PAUSE_SEC, RGB_COMPRESSED_TOPIC
from anymal_custom_control.runtime import LaunchSpec, ProcessManager, run_script_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Launch ANYmal egocentric visual servo stack")
    parser.add_argument("--port", type=int, default=5004, help="HTTP port for the egocentric servo console")
    parser.add_argument("--target-tag-id", type=int, default=None, help="Target tag ID; default uses best visible tag")
    parser.add_argument("--target-distance-m", type=float, default=0.5, help="Target standoff distance in meters")
    parser.add_argument("--tag-loss-pause-sec", type=float, default=DEFAULT_TAG_LOSS_PAUSE_SEC, help="Pause only after this many seconds without a fresh target tag")
    parser.add_argument("--record-dir", default=DEFAULT_RECORD_DIR)
    parser.add_argument("--archive-dir", default=DEFAULT_ARCHIVE_DIR)
    parser.add_argument("--no-record-video", dest="record_video", action="store_false", help="Disable RGB MP4 recording")
    parser.set_defaults(record_video=True)
    parser.add_argument("--video-fps", type=float, default=30.0, help="RGB MP4 recording frame rate")
    parser.add_argument("--video-topic", default=RGB_COMPRESSED_TOPIC, help="Compressed RGB topic to record")
    parser.add_argument("--no-oakd", action="store_true", help="Do not start run_oakd_sensor_node.py")
    parser.add_argument("--mono-resolution", default="400p", choices=("400p", "720p", "800p"))
    parser.add_argument("--depth-fps", type=float, default=30.0)
    parser.add_argument("--rgb-fps", type=float, default=30.0)
    args = parser.parse_args()

    specs: list[LaunchSpec] = []
    if not args.no_oakd:
        specs.append(
            LaunchSpec(
                name="oakd_sensor",
                command=[
                    sys.executable,
                    str(run_script_path("run_oakd_sensor_node.py")),
                    "--mono-resolution",
                    args.mono_resolution,
                    "--depth-fps",
                    str(args.depth_fps),
                    "--rgb-fps",
                    str(args.rgb_fps),
                ],
            )
        )

    servo_command = [
        sys.executable,
        str(run_script_path("run_egocentric_servo_node.py")),
        "--target-distance-m",
        str(args.target_distance_m),
        "--tag-loss-pause-sec",
        str(args.tag_loss_pause_sec),
        "--record-dir",
        args.record_dir,
        "--archive-dir",
        args.archive_dir,
        "--video-fps",
        str(args.video_fps),
        "--video-topic",
        args.video_topic,
    ]
    if not args.record_video:
        servo_command.append("--no-record-video")
    if args.target_tag_id is not None:
        servo_command.extend(["--target-tag-id", str(args.target_tag_id)])
    specs.append(LaunchSpec(name="egocentric_servo", command=servo_command))
    specs.append(
        LaunchSpec(
            name="egocentric_console",
            command=[
                sys.executable,
                str(run_script_path("run_egocentric_servo_console.py")),
                "--port",
                str(args.port),
            ],
        )
    )

    manager = ProcessManager()
    for spec in specs:
        print(f"Starting {spec.name}: {' '.join(spec.command)}")
        manager.start(spec)
    print(f"Console: http://localhost:{args.port}")

    try:
        name, code = manager.wait_until_any_exit()
        print(f"{name} exited with code {code}")
        return code
    except KeyboardInterrupt:
        print("\nShutting down ANYmal egocentric servo stack...")
        return 0
    finally:
        manager.terminate_all()


if __name__ == "__main__":
    raise SystemExit(main())
