# AprilTag Visual Servoing Approach

This note documents the current prototype autonomous visual servoing path:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_demo.py
```

The goal is to make the behavior reproducible from the document alone: what
happens when `YB` is pressed, which frames are used, how the state machine
advances, and what gains and thresholds are active.

The main implementation is:

- `catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_demo.py`
- `catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_teleop.py`
- `catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_autonomy.py`
- `catkin_ws/src/anymal_custom_control/scripts/run/run_oakd_sensor_node.py`
- `catkin_ws/src/anymal_custom_control/src/anymal_custom_control/control/giraf_arm_controller.py`

## Launch Stack

The prototype launcher starts four processes:

```text
giraf_arm_controller
oakd_sensor
visual_servo_autonomy
visual_servo_teleop
```

Run options:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_demo.py
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_demo.py --diagnostics
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_demo.py --no-auto-y-stabilization
```

Default visual-servo dashboard:

```text
http://localhost:5010
```

The `--diagnostics` option prints `VISUAL_SERVO_DIAG {...}` JSON lines at
10 Hz while autonomous mode is active. This is the best terminal log to paste
back for debugging frame/sign issues.

## ROS Interfaces

Autonomy input topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/giraf_arm/state` | `std_msgs/String` JSON | Current arm joint state and timestamp |
| `/giraf_arm/command_source` | `std_msgs/String` | Active source: `teleop` or `auto` |
| `/oakd/apriltag/detections_json` | `std_msgs/String` JSON | AprilTag detections and PnP poses |
| `/oakd/rgb/image_color/compressed` | `sensor_msgs/CompressedImage` | RGB stream for Flask dashboard |
| `/oakd/camera_y_level_error` | `std_msgs/Float64` | Camera-Y level error for stabilization |

Autonomy output topics:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/giraf_arm/auto_task_velocity_cmd` | `geometry_msgs/TwistStamped` | Autonomous task-space velocity |
| `/giraf_arm/auto_gripper_velocity_cmd` | `std_msgs/Float64` | Autonomous gripper velocity |
| `/giraf_arm/command_source` | `std_msgs/String` | Autonomy publishes `teleop` after sequence completion |
| `/giraf_arm/visual_servo_status_json` | `std_msgs/String` JSON | Status for teleop gating, dashboard, diagnostics |

The controller subscribes to both teleop and auto command topics. It selects one
command source based on `/giraf_arm/command_source`. This avoids teleop and
autonomy racing on a shared topic.

## AprilTag Perception

The OAK-D node publishes AprilTag detections on:

```text
/oakd/apriltag/detections_json
```

Current AprilTag settings:

| Parameter | Value |
| --- | --- |
| Family | `tag16h5` |
| Detector decision margin filter | `> 35` |
| Known cube tag IDs for object | `10, 11, 12, 13, 14, 15` |
| Cube tag size | `0.020 m` |

For each known-size tag, the OAK-D node solves a PnP pose from the RGB camera
intrinsics. Stereo depth is not used for the autonomous visual servoing policy.

The useful tag fields are:

```json
{
  "id": 10,
  "known_size": true,
  "tag_size_m": 0.02,
  "decision_margin": 65.0,
  "center_px": [cx, cy],
  "corners_px": [[x0, y0], [x1, y1], [x2, y2], [x3, y3]],
  "pose_t_camera_m": [x, y, z],
  "pose_R_camera_tag": [[...], [...], [...]]
}
```

`pose_t_camera_m` is the tag origin position in the OpenCV camera frame.
`pose_R_camera_tag` maps tag-frame axes into the camera frame.

The tag object points used by PnP are:

```text
[-half,  half, 0]
[ half,  half, 0]
[ half, -half, 0]
[-half, -half, 0]
```

So tag local `+X` and `+Y` come directly from this square definition. The yaw
alignment behavior assumes the tag's local `X/Y` axes are meaningful box/grasp
axes. If the physical tag is mounted rotated on the object, add a per-tag static
yaw offset later.

## Frame Conventions

### Global / FK frame

The autonomy code treats task velocity components as the FK/global frame used by
`num_forward_transform(...)` and the GIRAF arm controller.

Current practical convention:

```text
global +X: forward from the robot home pose
global +Y: left
global +Z: up, aligned with gravity
```

At the home pose described during testing:

```text
tool/gripper +Z: physical gripper forward, approximately global +X
tool/gripper +Y: physical gripper up, approximately global +Z
tool/gripper -X: physical gripper right, approximately global -Y
```

### Camera frame

The camera uses the OpenCV convention:

```text
camera +X: image right
camera +Y: image down
camera +Z: optical axis forward
```

The current rigid camera mount is modeled as a 15 degree downward pitch from the
gripper/tool frame.

`R_tool_camera` maps camera-frame vectors into the FK tool frame:

```text
theta = 15 deg

x_camera_in_tool = [-1, 0, 0]
y_camera_in_tool = [0, -cos(theta), -sin(theta)]
z_camera_in_tool = [0, -sin(theta),  cos(theta)]

R_tool_camera = columns(x_camera_in_tool, y_camera_in_tool, z_camera_in_tool)
```

At home this implies:

```text
camera +X ~= global -Y
camera +Z ~= cos(15 deg) * global +X - sin(15 deg) * global +Z
camera +Y ~= -sin(15 deg) * global +X - cos(15 deg) * global +Z
```

To transform a camera-frame vector to global:

```text
R_global_camera = R_global_tool * R_tool_camera
v_global = R_global_camera * v_camera
```

To transform the tag ray:

```text
ray_camera = pose_t_camera_m / norm(pose_t_camera_m)
ray_tool = R_tool_camera * ray_camera
ray_global = R_global_tool * ray_tool
ray_global = ray_global / norm(ray_global)
```

## Pressing YB

The `run_visual_servo_teleop.py` process owns the joystick.

`YB` is rising-edge toggled:

1. If the current command source is `auto`, pressing `YB` switches back to
   `teleop` and publishes zero motion.
2. If the current command source is `teleop`, pressing `YB` switches to `auto`
   only if `/giraf_arm/visual_servo_status_json` says `ready=true` and that
   status is fresh within `0.5 s`.
3. If no vertical cube tag target is ready, `YB` is ignored and a warning is
   logged:
   `YB ignored; no vertical cube tag target is currently ready for autonomous mode`.

Ready means the autonomy node currently has:

```text
at least one cube tag detection exists for IDs 10-15
tag age <= 0.25 s
decision margin >= 35.0
tag pose rotation is available
tag normal is within 45 deg of global +/-Z after FK/camera projection
norm(pose_t_camera_m) is finite and > 1e-6
```

When teleop publishes `auto` to `/giraf_arm/command_source`, the autonomy node:

```text
resets PID state
clears current waypoint and debug vectors
clears centering metrics
clears previous selected target tag ID
sets mode_state = APPROACH
sets message = "Autonomous mode active"
```

The next autonomy loop tick then decides whether it must wait, center, approach,
grasp, or lift.

## Cube Target Selection

The cube object uses six 20 mm tags:

```text
10, 11, 12, 13, 14, 15
```

While the command source is `teleop`, the policy does not keep a locked target
ID. The status publisher continuously reevaluates all currently detected cube
tags and reports `ready=true` if at least one candidate passes the gate below.
When `YB` switches to `auto`, the policy selects from that current candidate set.

For each candidate:

```text
R_global_camera = R_global_tool * R_tool_camera
tag_normal_global = R_global_camera * pose_R_camera_tag[:, 2]
gravity_alignment = dot(normalize(tag_normal_global), [0, 0, 1])
gravity_alignment_abs = abs(gravity_alignment)
```

The selected manipulation target is the candidate with the largest
`gravity_alignment_abs`, provided it is at least:

```text
cos(45 deg) ~= 0.707
```

The absolute value is intentional: it handles normal sign ambiguity and selects
the face whose tag normal is most vertical, whether it points with or against
global `+Z`. Once selected, the policy locks onto that tag ID for the autonomous
run. If the selected tag is temporarily lost, the robot pauses instead of
silently switching to another cube face.

## State Machine

The autonomy loop runs at:

```text
50 Hz
```

Top-level states used in status:

```text
IDLE
WAITING_FOR_STATE
PAUSED_LOST_TAG
CENTER
APPROACH
WAYPOINT_REACHED
GRASPING
LIFT
DONE
```

The effective state machine is:

```text
teleop command source
  -> IDLE

YB switches command source to auto
  -> APPROACH initial state

each auto tick:
  if no fresh arm state:
    WAITING_FOR_STATE, publish zero

  else if FK unavailable:
    WAITING_FOR_STATE, publish zero

  else if state == LIFT:
    drive lift waypoint, then switch to teleop

  else if state == GRASPING:
    close gripper for fixed duration, then LIFT

  else if no target cube tag has been selected:
    choose detected cube tag whose normal is most vertical in global frame
    if no candidate passes the vertical-alignment gate:
      PAUSED_LOST_TAG, clear waypoint/PID, publish zero

  else if selected cube tag is not ready:
    PAUSED_LOST_TAG, clear waypoint/PID, publish zero

  else if tag distance <= success distance:
    GRASPING

  else:
    update tag/ray/centering metrics
    if any centering metric is outside threshold:
      CENTER
    else if no current waypoint:
      compute new approach waypoint
    else:
      drive current approach waypoint with task-space PID
      if waypoint reached:
        recompute next waypoint
```

Important detail: `GRASPING` and `LIFT` are checked before tag readiness. Once
the grasp starts, the tag is no longer required. This handles the hardware case
where the tag becomes too close or out of focus after the gripper closes.

## State Details

### IDLE

Condition:

```text
command_source != "auto"
```

Action:

```text
mode_state = IDLE
message = "Waiting for teleop"
publish zero task velocity
publish zero gripper velocity
```

### WAITING_FOR_STATE

Conditions:

```text
no /giraf_arm/state received
or state age > 0.5 s
or FK transform computation fails
```

Action:

```text
mode_state = WAITING_FOR_STATE
message = "Waiting for fresh arm state" or "FK unavailable"
publish zero task velocity
publish zero gripper velocity
```

### PAUSED_LOST_TAG

Conditions:

```text
selected cube tag missing
or tag age > 0.25 s
or decision margin < 35.0
or tag distance is invalid
```

Action:

```text
mode_state = PAUSED_LOST_TAG
message = "Waiting for fresh selected cube tag pose"
clear waypoint
clear ray/debug metrics
reset PID
publish zero task velocity
publish zero gripper velocity
```

### CENTER

Purpose:

Before driving toward the object, the policy aligns several local visual metrics
so the approach ray is safer:

```text
lateral tag position in camera frame
vertical tag position in camera frame
camera/tag face angle
global yaw alignment with tag in-plane axes
```

The code re-evaluates these metrics every auto tick, including during approach.
So yes: the system can re-enter `CENTER` after every approach phase if the tag
drifts outside the release thresholds.

Centering thresholds use hysteresis:

| Metric | Enter/re-enter CENTER outside | Stay in CENTER until within |
| --- | ---: | ---: |
| Lateral camera X | `0.015 m` | `0.010 m` |
| Vertical camera Y | `0.015 m` | `0.010 m` |
| Tag face angle error | `7 deg` | `5 deg` |
| Yaw alignment error | `8 deg` | `5 deg` |

When any metric is outside threshold:

```text
mode_state = CENTER
message = "Centering tag before approach"
clear waypoint
waypoint_distance_m = current tag distance
waypoint_tolerance_m = max(0.010, 0.010) = 0.010 m
reset PID
publish centering task velocity
```

#### Lateral centering

Metric:

```text
lateral_error = pose_t_camera_m[0]
```

Axis:

```text
camera_x_global = R_global_tool * R_tool_camera[:, 0]
camera_x_global = camera_x_global / norm(camera_x_global)
```

Command:

```text
lateral_speed = clip(0.7 * lateral_error, -0.035, 0.035) m/s
linear += lateral_speed * camera_x_global
```

Interpretation:

If the tag appears offset in camera `+X`, the robot commands motion along global
camera `+X` to reduce that offset. With the current mount, camera `+X` is roughly
physical gripper right at home.

#### Vertical image centering

Metric:

```text
vertical_error = pose_t_camera_m[1]
vertical_angle_error = atan2(vertical_error, max(pose_t_camera_m[2], 0.001))
```

Axis:

```text
angular_axis_global = camera_x_global
```

Command:

```text
angular_speed = clip(-1.0 * vertical_angle_error, -0.25, 0.25) rad/s
angular += angular_speed * angular_axis_global
```

This is an image-plane pitch correction: it rotates around the globalized
camera `+X` axis to drive the tag toward the center vertically.

#### Tag face angle centering

Metric:

```text
ray_camera = pose_t_camera_m / norm(pose_t_camera_m)
tag_normal_camera = pose_R_camera_tag[:, 2] / norm(pose_R_camera_tag[:, 2])
tag_face_angle = acos(abs(dot(ray_camera, tag_normal_camera)))
tag_face_angle_error = 45 deg - tag_face_angle
```

Target:

```text
tag_face_angle = 45 deg
```

The `abs(dot(...))` makes the metric insensitive to the sign of the tag normal.

Command axis:

```text
global +Z translation
```

The implementation estimates how moving the camera in global `+Z` changes the
tag face angle. It uses a small finite difference:

```text
R_global_camera = R_global_tool * R_tool_camera
global_z_in_camera = transpose(R_global_camera) * [0, 0, 1]
eps = 0.002 m

pose_plus  = pose_t_camera_m - eps * global_z_in_camera
pose_minus = pose_t_camera_m + eps * global_z_in_camera

dangle_dz = (angle(pose_plus) - angle(pose_minus)) / (2 * eps)
```

Then:

```text
z_speed = clip(0.08 * tag_face_angle_error * dangle_dz,
               -0.025, 0.025) m/s
linear[2] += z_speed
```

If `pose_R_camera_tag` is unavailable, this metric and command are skipped for
that tick.

#### Yaw alignment centering

Purpose:

Rotate around global `Z` so the gripper can slide into the box along a useful
box/tag axis.

Gripper forward axis:

```text
gripper_forward_global = R_global_tool * [0, 0, 1]
gripper_forward_xy = normalize([x, y, 0])
```

Candidate tag axes:

```text
R_global_camera = R_global_tool * R_tool_camera
tag_x_global = R_global_camera * pose_R_camera_tag[:, 0]
tag_y_global = R_global_camera * pose_R_camera_tag[:, 1]

candidates = +tag_x, -tag_x, +tag_y, -tag_y
```

Each candidate is projected into horizontal `XY` and normalized. The selected
axis is whichever candidate has the largest dot product with
`gripper_forward_xy`, i.e. whichever one is already closest.

Signed yaw error:

```text
dot = dot(gripper_forward_xy, selected_tag_axis_xy)
cross_z = gripper_forward_xy.x * selected_tag_axis_xy.y
          - gripper_forward_xy.y * selected_tag_axis_xy.x

yaw_error = atan2(cross_z, dot)
```

Command:

```text
wz = clip(1.0 * yaw_error, -0.25, 0.25) rad/s
angular[2] += wz
```

If the tag rotation is missing or the horizontal projection is degenerate, yaw
alignment is skipped for that tick.

### APPROACH

The approach phase moves to a waypoint halfway from the current gripper/tool
position toward the current AprilTag ray.

Waypoint computation:

```text
distance = norm(pose_t_camera_m)
ray_camera = pose_t_camera_m / distance
ray_tool = R_tool_camera * ray_camera
ray_global = R_global_tool * ray_tool
ray_global = ray_global / norm(ray_global)

current_global = T_global_tool[:3, 3]
waypoint_global = current_global + 0.50 * distance * ray_global
waypoint_distance_m = distance
waypoint_tolerance_m = max(0.005, 0.10 * distance)
```

Example:

```text
tag distance = 0.20 m
waypoint step = 0.10 m toward tag
waypoint tolerance = 0.020 m
```

The robot does not drive all the way to the tag in one shot. It repeatedly:

```text
center if needed
compute waypoint halfway to current tag
PID to waypoint
when waypoint is reached, recompute from the new view
```

This makes the servoing iterative and allows tag measurements to correct the
trajectory as the arm moves.

### WAYPOINT_REACHED

This is mostly a transient status. When the current global position is within
the waypoint tolerance:

```text
if norm(waypoint_global - current_global) <= waypoint_tolerance_m:
    mode_state = WAYPOINT_REACHED
    message = "Waypoint reached; recomputing"
    compute a fresh waypoint from the current tag pose
```

If the waypoint recomputation succeeds in the same tick, the status often moves
back to `APPROACH` immediately.

### GRASPING

Success condition:

```text
norm(pose_t_camera_m) <= 0.08 m
```

Once this condition is met:

```text
mode_state = GRASPING
message = "Tag within grasp distance; closing gripper"
clear waypoint and debug vectors
reset PID
publish gripper close velocity
```

Gripper close command:

```text
velocity = -GRIPPER_SPEED = -2.0
duration = 1.5 s
```

The close duration is fixed. The PWM/current limit on hardware is expected to
prevent grasp overload.

Important detail:

`GRASPING` does not require the tag to remain visible. This is intentional,
because at close range the tag may leave the field of view or become out of
focus.

### LIFT

After the fixed close duration:

```text
mode_state = LIFT
message = "Grasp command complete; starting lift"
```

The lift waypoint is:

```text
lift_waypoint_global = current_global + [0, 0, 0.30]
waypoint_distance_m = 0.30
waypoint_tolerance_m = 0.02
```

The same task-space PID used for approach drives the lift.

When the lift error is within tolerance:

```text
if norm(lift_waypoint_global - current_global) <= 0.02:
    command_source = teleop
    mode_state = DONE
    message = "Lift complete; returning to teleop"
    publish zero
    publish /giraf_arm/command_source = "teleop"
```

As with `GRASPING`, `LIFT` does not require tag visibility.

## Task-Space PID

The approach and lift phases use the same PID function on global position error:

```text
error_global = waypoint_global - current_global
velocity = KP * error_global
         + KI * integral_error
         + KD * derivative_error
```

Current PID constants:

| Parameter | Value |
| --- | ---: |
| `KP` | `0.8` |
| `KI` | `0.08` |
| `KD` | `0.03` |
| Integral accumulator clamp | `+/-0.04` per axis |

Derivative:

```text
derivative_error = (error_global - previous_error) / dt
```

The PID output is speed-limited by waypoint distance:

```text
max_speed = min(0.06, max(0.012, 0.35 * waypoint_distance_m))

if norm(velocity) > max_speed:
    velocity = velocity * max_speed / norm(velocity)
```

So:

```text
maximum approach/lift speed = 0.06 m/s
minimum nonzero PID speed limit = 0.012 m/s
closer waypoints reduce the allowed speed
```

The PID resets when:

```text
command source changes
tag is lost
entering CENTER
computing a new waypoint
entering grasp success logic
setting lift waypoint
```

## Camera-Y Stabilization During Autonomy

By default, autonomous task commands also receive the same camera-Y
stabilization correction used in teleop.

Disable it for testing with:

```bash
python3 catkin_ws/src/anymal_custom_control/scripts/run/run_visual_servo_demo.py --no-auto-y-stabilization
```

The stabilization input is:

```text
/oakd/camera_y_level_error
```

Freshness timeout:

```text
0.25 s
```

Gains from `run_teleop_stabilized.py`:

| Parameter | Value |
| --- | ---: |
| `STABILIZE_KP` | `10.0` |
| `STABILIZE_KI` | `0.0` |
| `STABILIZE_KD` | `0.5` |
| Deadband | `0.75 deg` |
| Max angular speed | `1.0 rad/s` |
| Integral limit | `0.25 rad` |
| Sign | `-1.0` |

The correction is added after the autonomy command is converted into a
`TwistStamped`. The `command_angular` diagnostic field is the autonomy angular
command before this extra stabilization correction.

Safety zero commands use stabilization disabled. The gripper close command
publishes zero task velocity through the normal twist path, so if auto
stabilization is enabled and fresh it can still add a small angular correction
while the gripper is closing.

## Controller Limits And Source Selection

The visual-servo policy publishes conservative velocities. The arm controller
still clamps task velocity to:

| Component | Limit |
| --- | ---: |
| `vx` | `+/-0.2 m/s` |
| `vy` | `+/-0.2 m/s` |
| `vz` | `+/-0.1 m/s` |
| `wx` | `+/-1.0 rad/s` |
| `wy` | `+/-1.0 rad/s` |
| `wz` | `+/-1.0 rad/s` |

Controller command timeout:

```text
0.15 s
```

If the selected command source is stale, the controller commands zero task
velocity and zero gripper velocity.

The active source is selected by:

```text
/giraf_arm/command_source
```

Accepted values:

```text
teleop
auto
```

When selected source is `auto`, the controller uses:

```text
/giraf_arm/auto_task_velocity_cmd
/giraf_arm/auto_gripper_velocity_cmd
```

When selected source is `teleop`, the controller uses:

```text
/giraf_arm/teleop_task_velocity_cmd
/giraf_arm/teleop_gripper_velocity_cmd
```

## Status And Diagnostics

The autonomy node publishes compact status JSON on:

```text
/giraf_arm/visual_servo_status_json
```

Important fields:

```text
mode
state
ready
active
auto_y_stabilization
target_tag_ids
selected_target_tag_id
target_gravity_alignment_min
target_candidates
tag.id
tag.visible
tag.age_sec
tag.decision_margin
tag.pose_t_camera_m
tag.distance_m
waypoint_global
lift_waypoint_global
waypoint_distance_m
waypoint_tolerance_m
lateral_error_camera_m
vertical_error_camera_m
tag_face_angle_deg
tag_face_angle_error_deg
yaw_error_deg
yaw_target_axis
yaw_command_rad_s
gripper_forward_global
selected_tag_axis_global
ray_camera
ray_tool
ray_global
camera_axes_global
R_tool_camera
current_global
error_global
command_linear
command_angular
command_components
message
```

With `--diagnostics`, a subset of this is printed at 10 Hz as:

```text
VISUAL_SERVO_DIAG {...}
```

This is the best place to debug:

```text
wrong camera/tool frame mapping
wrong sign on lateral centering
wrong sign on vertical angular correction
wrong yaw sign
tag axis selection flipping
whether approach re-entered CENTER
whether tag loss caused PAUSED_LOST_TAG
whether GRASPING/LIFT are bypassing tag readiness as intended
```

## Implementation Pseudocode

This is a condensed form of the current `step()` logic:

```python
def step():
    now = rospy.get_time()
    command_linear = zeros(3)
    command_angular = zeros(3)

    if command_source != "auto":
        state = "IDLE"
        publish_status()
        publish_zero()
        return

    if not fresh_arm_state(now):
        state = "WAITING_FOR_STATE"
        publish_status()
        publish_zero()
        return

    T = current_fk_transform()
    if T is None:
        state = "WAITING_FOR_STATE"
        publish_status()
        publish_zero()
        return

    current_global = T[:3, 3]

    if state == "LIFT":
        if lift_waypoint is None:
            set_lift_waypoint(current_global)
        error = lift_waypoint - current_global
        if norm(error) <= POST_GRASP_LIFT_TOLERANCE_M:
            command_source = "teleop"
            state = "DONE"
            publish_zero()
            publish_command_source("teleop")
        else:
            command_linear = pid(error)
            publish_auto_task(command_linear, command_angular)
        return

    if state == "GRASPING":
        if elapsed_grasp_time <= GRASP_CLOSE_SEC:
            publish_close_gripper()
        else:
            state = "LIFT"
            set_lift_waypoint(current_global)
        return

    if selected_target_tag_id is None:
        if not select_cube_tag_with_most_vertical_normal(T):
            state = "PAUSED_LOST_TAG"
            clear_waypoint_and_pid()
            publish_zero()
            return

    if not tag_ready(now):
        state = "PAUSED_LOST_TAG"
        clear_waypoint_and_pid()
        publish_zero()
        return

    tag_distance = norm(tag.pose_t_camera_m)

    if tag_distance <= SUCCESS_DISTANCE_M:
        state = "GRASPING"
        start_or_continue_grasp_timer()
        publish_close_gripper()
        return

    update_ray_and_centering_metrics(T)

    if needs_centering():
        state = "CENTER"
        clear_waypoint()
        reset_pid()
        command_linear, command_angular = centering_command(T)
        publish_auto_task(command_linear, command_angular)
        return

    if waypoint is None:
        if not compute_waypoint(T):
            state = "PAUSED_LOST_TAG"
            publish_zero()
            return

    error = waypoint - current_global
    if norm(error) <= waypoint_tolerance:
        state = "WAYPOINT_REACHED"
        if not compute_waypoint(T):
            publish_zero()
            return
        error = waypoint - current_global

    state = "APPROACH"
    command_linear = pid(error)
    publish_auto_task(command_linear, command_angular)
```

## Current Constant Cheat Sheet

| Name | Value |
| --- | ---: |
| `TARGET_TAG_IDS` | `10, 11, 12, 13, 14, 15` |
| `TARGET_MARGIN_MIN` | `35.0` |
| `TARGET_GRAVITY_ALIGNMENT_MIN` | `cos(45 deg) ~= 0.707` |
| `TAG_TIMEOUT_SEC` | `0.25` |
| `STATE_TIMEOUT_SEC` | `0.5` |
| `SUCCESS_DISTANCE_M` | `0.08` |
| `WAYPOINT_FRACTION` | `0.50` |
| `WAYPOINT_TOLERANCE_FRACTION` | `0.10` |
| `WAYPOINT_TOLERANCE_MIN_M` | `0.005` |
| `CENTER_LATERAL_TOLERANCE_M` | `0.010` |
| `CENTER_LATERAL_RELEASE_M` | `0.015` |
| `CENTER_LATERAL_KP` | `0.7` |
| `CENTER_LATERAL_MAX_SPEED_M_S` | `0.035` |
| `CENTER_VERTICAL_TOLERANCE_M` | `0.010` |
| `CENTER_VERTICAL_RELEASE_M` | `0.015` |
| `CENTER_VERTICAL_ANG_KP` | `1.0` |
| `CENTER_VERTICAL_MAX_ANG_SPEED_RAD_S` | `0.25` |
| `CENTER_TAG_FACE_ANGLE_DEG` | `45.0` |
| `CENTER_TAG_FACE_ANGLE_TOLERANCE_DEG` | `5.0` |
| `CENTER_TAG_FACE_ANGLE_RELEASE_DEG` | `7.0` |
| `CENTER_TAG_FACE_ANGLE_KP` | `0.08` |
| `CENTER_TAG_FACE_ANGLE_MAX_Z_SPEED_M_S` | `0.025` |
| `CENTER_YAW_TOLERANCE_DEG` | `5.0` |
| `CENTER_YAW_RELEASE_DEG` | `8.0` |
| `CENTER_YAW_KP` | `1.0` |
| `CENTER_YAW_MAX_SPEED_RAD_S` | `0.25` |
| `KP` | `0.8` |
| `KI` | `0.08` |
| `KD` | `0.03` |
| `INTEGRAL_LIMIT_M_S` | `0.04` |
| `MAX_PID_SPEED_M_S` | `0.06` |
| `MIN_PID_SPEED_M_S` | `0.012` |
| `DISTANCE_SPEED_GAIN` | `0.35` |
| `GRASP_CLOSE_SEC` | `1.5` |
| `POST_GRASP_LIFT_M` | `0.30` |
| `POST_GRASP_LIFT_TOLERANCE_M` | `0.02` |
| `CAMERA_DOWNWARD_PITCH_DEG` | `15.0` |
| `DIAGNOSTICS_PRINT_HZ` | `10.0` |

## Replication Notes

To replicate the current behavior in another node:

1. Subscribe to the same arm state, command source, and AprilTag detection
   topics.
2. Use the same FK offsets before calling `num_forward_transform`.
3. Use the same `R_tool_camera` mount model.
4. Treat `pose_t_camera_m` as metric camera-frame translation from PnP, not
   stereo depth.
5. Gate autonomous entry on a fresh cube tag pose with margin >= 35 and vertical normal alignment.
6. Re-evaluate centering every control tick before PID approach.
7. Use the iterative half-distance waypoint rule.
8. Stop requiring tag visibility after entering `GRASPING`.
9. Lift in global `+Z`, then publish command source `teleop`.

The most fragile assumptions are:

```text
camera-to-tool extrinsics
FK terminal/tool frame convention
tag local X/Y alignment with the physical box
the sign conventions of lateral, vertical, and yaw commands
```

When those change, run with `--diagnostics` and inspect:

```text
R_tool_camera
camera_axes_global
ray_camera / ray_tool / ray_global
lateral_error_camera_m
vertical_error_camera_m
tag_face_angle_error_deg
yaw_error_deg
yaw_target_axis
command_linear
command_angular
```
