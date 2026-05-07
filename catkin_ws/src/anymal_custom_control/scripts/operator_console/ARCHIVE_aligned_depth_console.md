# Archived Operator Console Aligned Depth Notes

Aligned depth was removed from the active operator console to keep the
perception panel focused on RGB AprilTag overlays and structured tag poses.

The implementation that existed before this cleanup had three active pieces in
`operator_console.py`:

- ROS-backed full mode subscribed to `/oakd/depth/image_colorized/compressed`
  and exposed it through `/feed/depth`.
- Direct-camera mode built a `StereoDepth` pipeline from `CAM_B` / `CAM_C`,
  colorized the depth frame, drew AprilTag outlines on it, and exposed it as
  the depth stream.
- The UI showed an `Aligned Depth` stream plus `RGB Pose Depth` and `Depth Mask
  Summary` cards populated from `/oakd/apriltag/stats_json`.

To restore it later, reintroduce those pieces together:

1. Add `depth` back to `new_frame_events`, `latest_frames`, `stream_stats`, and
   `ros_image_stats`.
2. Subscribe to `/oakd/depth/image_colorized/compressed` in `ros_monitor_loop`.
3. Restore the direct-camera `StereoDepth` pipeline and `colorize_depth`.
4. Add the `Aligned Depth` stream and depth summary cards back to `HTML_PAGE`.

The production OAK-D node still publishes aligned colorized depth, so this
archive is only for the operator-console display path.
