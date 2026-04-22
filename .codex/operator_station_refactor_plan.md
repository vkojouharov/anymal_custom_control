# Operator Station Refactor Plan

This file is an implementation brief for a later execution pass. It is not a
record of completed work. The repository should still be treated as using the
current mixed `scripts/` layout until a future change explicitly performs this
refactor.

## Intent

Refactor `anymal_custom_control` into a cleaner operator-station architecture
that separates:

- robot control / teleop
- OAK-D perception and diagnostics
- browser dashboard / operator UI
- low-level debugging tools

The goal is to preserve the current working capabilities while making the
system easier to launch, safer to evolve, and less cluttered for day-to-day
use.

The operator-facing experience should become:

`one obvious launcher -> predictable background services -> one dashboard URL`

## Current Problems

The current structure is functional but mixed:

- `scripts/` contains production teleop entrypoints, one-off debug tools,
  experiments, and trajectory demos at the same level.
- `scripts/camera/` contains both active OAK-D web interfaces and ad hoc probe
  utilities.
- control logic and web/perception logic are both exposed through top-level
  runnable scripts, but the reusable code is not yet organized around those
  concerns.
- there is no single operator-station launcher that starts the intended stack.
- the OAK-D scripts are evolving into a coherent subsystem, but they are still
  structured as stand-alone apps instead of a shared perception service.
- the teleop entrypoints are safety-critical but currently look similar to
  every other helper script in the package.

## Desired Outcome

After the refactor:

- there is one clear place for operator launchers
- there is one clear place for debug / probe tools
- perception code is reusable and not duplicated across scripts
- teleop remains isolated from Flask / MJPEG / AprilTag / IMU processing
- the repo has one clear path toward a combined operator station
- legacy scripts can remain temporarily, but they are not the preferred public
  entrypoints

## Locked Decisions

These decisions should guide the implementation unless a concrete blocker is
found:

- Teleop control must remain separate from the web dashboard / perception
  runtime.
- OAK-D ownership should converge to one perception service, not multiple
  competing camera scripts grabbing the device independently in production.
- Browser dashboards should be treated as operator-facing interfaces, not as
  the canonical implementation home of the underlying logic.
- Debug tools should remain terminal-first and minimal.
- New operator-facing runnables should stop using the `RUN_*` naming pattern.
- The refactor should add structure first and behavior changes second.
- Existing working scripts should remain available during migration.

## Proposed Runtime Model

The target runtime is a small set of cooperating processes, not one giant
Python script.

### 1. Control process

Responsibilities:

- joystick input
- motor communication
- J-PARSE teleop loop
- robot state publication
- clean shutdown behavior

Non-responsibilities:

- Flask
- MJPEG encoding
- AprilTag detection
- OAK-D IMU handling

The current seed for this process is:

- `scripts/RUN_arm_wrist_JPARSE_teleop.py`

### 2. Perception process

Responsibilities:

- OAK-D pipeline ownership
- RGB and aligned depth acquisition
- AprilTag detection and summaries
- IMU acquisition and summaries
- publication of a shared perception state

Non-responsibilities:

- direct motor control
- teleop loop timing
- browser templating beyond what is needed for API serialization

The current seeds for this process are:

- `scripts/camera/depthai_rgb_stream.py`
- `scripts/camera/depthai_depth_stream.py`
- `scripts/camera/depthai_apriltag_stream.py`
- `scripts/camera/depthai_accel_gyro_test.py`

### 3. Dashboard process

Responsibilities:

- render operator-facing browser UI
- present robot status, camera views, tag summaries, and later IMU summaries
- poll or subscribe to structured state from control and perception

Non-responsibilities:

- direct device access to motors or OAK-D
- teleop logic
- core estimation logic

### 4. Launcher process

Responsibilities:

- start the intended stack
- select modes such as teleop-only / perception-only / full operator station
- manage startup order and shutdown
- provide one obvious command for the user

## Target Layout

New code should stop accumulating directly in flat `scripts/`.

Implement toward this structure:

```text
catkin_ws/src/anymal_custom_control/
  scripts/
    run/
      run_operator_station.py
      run_arm_jparse_teleop.py
      run_perception_dashboard.py
    tools/
      depthai_probe.py
      depthai_imu_terminal_debug.py
      depthai_accel_gyro_test.py
      discover_topics.py
      probe_msg.py
      monitor.py
    legacy/
      RUN_arm_wrist_JPARSE_teleop.py
      RUN_arm_wrist_CBF_teleop.py
      RUN_arm_wrist_RMRC_teleop.py
      RUN_anymal_teleop_joystick.py
      RUN_anymal_teleop_keyboard.py
      ...
    trajectories/
      ...
    dynamixel/
      ...
  src/anymal_custom_control/
    control/
      __init__.py
      teleop_jparse.py
      joystick_session.py
      motor_session.py
      robot_state.py
    perception/
      __init__.py
      depthai_device.py
      rgb_depth_pipeline.py
      apriltag_pipeline.py
      imu_pipeline.py
      perception_state.py
    dashboard/
      __init__.py
      web_app.py
      pages.py
      serializers.py
    runtime/
      __init__.py
      operator_station.py
      process_manager.py
      config.py
```

Notes:

- `scripts/run/` should contain only thin user-facing launchers.
- `scripts/tools/` should contain diagnostics, probes, and validation tools.
- `scripts/legacy/` should preserve the current entrypoints during migration.
- reusable logic belongs under `src/anymal_custom_control/`, not in the
  launcher scripts.

## Public Entry Points

The desired public run commands after migration are:

- `scripts/run/run_operator_station.py`
- `scripts/run/run_arm_jparse_teleop.py`
- `scripts/run/run_perception_dashboard.py`

The desired debug / bring-up tools are:

- `scripts/tools/depthai_probe.py`
- `scripts/tools/depthai_imu_terminal_debug.py`
- `scripts/tools/depthai_accel_gyro_test.py`

The current `RUN_*` scripts should remain callable during transition but should
be documented as legacy wrappers.

## Intended User Workflow

### Operator workflow

The target operator workflow should be:

1. start the full system with one command
2. see the dashboard URL printed in the terminal
3. use the browser for situational awareness
4. use joystick / teleop terminal for robot control
5. stop everything cleanly with one Ctrl-C path

Target command:

```bash
python3 scripts/run/run_operator_station.py
```

### Teleop-only workflow

For pure control work without the OAK-D stack:

```bash
python3 scripts/run/run_arm_jparse_teleop.py
```

### Perception-only workflow

For camera and dashboard work without robot motion:

```bash
python3 scripts/run/run_perception_dashboard.py
```

### Diagnostics workflow

For hardware checks:

```bash
python3 scripts/tools/depthai_probe.py
python3 scripts/tools/depthai_imu_terminal_debug.py
python3 scripts/tools/depthai_accel_gyro_test.py
```

## Combined Operator Station Scope

The first combined operator station does not need to solve every integration
problem. It should focus on one coherent, useful stack:

- J-PARSE wrist teleop
- OAK-D RGB + aligned depth
- AprilTag overlay and depth summaries
- optional IMU status
- one browser dashboard

It does not need to include every historical teleop mode or every existing
camera script on day one.

## Perception Refactor Direction

The current OAK-D scripts are valuable prototypes, but they should converge
toward one shared implementation.

### Phase-1 combined perception service

One OAK-D process should be able to expose:

- RGB frame
- aligned depth frame
- AprilTag detections
- RGB pose-depth summary
- depth-mask summary
- optional IMU summary

The dashboard can consume those through one structured state/API rather than
starting separate Flask apps for each feature.

### Existing scripts that should become wrappers or tools

- `depthai_rgb_stream.py`
- `depthai_depth_stream.py`
- `depthai_apriltag_stream.py`

Recommended direction:

- keep them temporarily
- gradually turn them into thin wrappers around shared perception modules
- stop treating them as the long-term architecture

## Control Refactor Direction

The current teleop script is already doing real work and should be treated as
the reference control implementation during migration.

Immediate refactor target:

- extract the core J-PARSE teleop logic from
  `RUN_arm_wrist_JPARSE_teleop.py` into a reusable control module
- keep the current script as a wrapper around that module

Longer term:

- unify controller configuration and state reporting
- expose a small status interface that the dashboard can read safely

## Interface Boundary Recommendation

There are two viable boundaries between control, perception, and dashboard:

- local Python state / subprocess boundary
- ROS topics / messages

Near-term recommendation:

- use local subprocess management for the first operator station launcher
- keep state exchange simple and explicit

Long-term recommendation:

- converge important robot/perception outputs onto ROS topics where that
  naturally improves observability and reuse

The first refactor should not block on designing a perfect ROS API.

## Migration Strategy

Implement this in phases.

### Phase 0: documentation and naming

- add this plan
- document the intended public entrypoints
- stop adding new production scripts to flat `scripts/`

### Phase 1: structural cleanup without behavior changes

- create `scripts/run/`, `scripts/tools/`, and `scripts/legacy/`
- move or wrap existing scripts into those buckets
- keep old legacy paths working where practical
- create placeholder modules under `src/anymal_custom_control/control`,
  `perception`, `dashboard`, and `runtime`

Success criteria:

- repo is easier to scan
- new code has an obvious home
- existing workflows still work

### Phase 2: extract reusable modules

- move J-PARSE teleop internals into `control/`
- move shared OAK-D logic into `perception/`
- move Flask page construction and API serialization into `dashboard/`

Success criteria:

- launcher scripts become short
- multiple runnables can share the same underlying logic

### Phase 3: add a real operator-station launcher

- implement `run_operator_station.py`
- support:
  - full stack
  - teleop-only
  - perception-only
- manage child process lifecycle and shutdown

Success criteria:

- user can start the intended stack with one obvious command

### Phase 4: unify perception dashboard

- converge RGB/depth/AprilTag/IMU dashboard features into one app
- de-emphasize stand-alone experimental web scripts

Success criteria:

- one OAK-D perception app
- one dashboard URL

## Documentation Requirements

When the refactor is implemented, add or update docs covering:

- new folder purpose
- public run commands
- legacy entrypoints and their replacement
- expected startup order
- dashboard URLs
- teleop safety notes
- OAK-D bring-up / probe path

The new structure should be understandable without reading code first.

## Risks

Primary risks:

- accidentally coupling teleop timing to Flask/perception load
- breaking current working entrypoints during migration
- letting the launcher become a giant ball of ad hoc orchestration logic
- duplicating OAK-D logic instead of truly consolidating it

Mitigations:

- keep control separate from perception
- keep wrappers for old scripts during migration
- keep debug tools terminal-first
- extract reusable modules before building a large launcher

## Immediate Next Steps

The first implementation pass should:

1. create the new `scripts/run`, `scripts/tools`, and `scripts/legacy`
   structure
2. add thin wrapper entrypoints with the desired future names
3. move diagnostic scripts into `scripts/tools`
4. preserve current script behavior through wrappers or legacy copies
5. identify the first shared `control/` and `perception/` modules to extract

## Non-Goals

This refactor does not require:

- rewriting all historical teleop variants immediately
- converting everything to ROS topics before cleanup
- redesigning the J-PARSE controller
- finalizing the IMU browser interface before the structure is cleaned up
- deleting old scripts early in the migration

The first goal is clarity and organization, not maximal consolidation in one
pass.
