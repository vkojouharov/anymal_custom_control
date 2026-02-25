# ANYmal Custom Control

Python API for programmatic control of ANYmal D robot movement and camera access via ROS1.

Injects commands through the existing joy_manager pipeline at `/anyjoy/operator`,
preserving all safety mechanisms (velocity limits, timeout auto-stop).

## Quick Start

### 1. Build

```bash
cd ~/anymal_custom_control
docker compose build
```

### 2. Connect to robot


Turn on the robot, connect with the robot wifi, run the ANYmal launcher in OPC mode and connect the robot. Take control of the robot via button, set it to walking mode, then keep ANYmal GUI open.

This docker container utilizes the open connection to the robot to inject control signals.

```bash
# Start ANYmal launcher first (brings up roscore, locomotion, GUI, etc.)
# Then start our container:
cd ~/anymal_custom_control
docker compose up -d
```

### 3. Discover topics (read-only, safe)

```bash
docker exec -it anymal_custom_control bash

# Inside the container:
python3 /catkin_ws/src/anymal_custom_control/scripts/discover_topics.py # unnecessary
python3 /catkin_ws/src/anymal_custom_control/scripts/monitor.py # good shit 
python3 /catkin_ws/src/anymal_custom_control/scripts/teleop.py # better shit?
```

### 4. Use the API

```python
import rospy
from anymal_custom_control.movement import MovementController
from anymal_custom_control.camera import CameraReceiver

rospy.init_node('my_controller')

# Camera (use topic from discover_topics.py)
cam = CameraReceiver('/depth_camera_front/color/image_raw')

# Movement (requires locomotion active in GUI first!)
with MovementController() as mc:
    mc.forward(0.2)
    rospy.sleep(2.0)
    mc.stop()

frame = cam.get_frame()  # numpy array (H, W, 3) BGR
```

---

## API Reference

### MovementController

```python
mc = MovementController(topic='/anyjoy/operator', rate_hz=10)
mc.start()                  # start 10 Hz publish thread

# Simple commands (speed in [0, 1.0], fraction of max velocity):
mc.forward(0.3)
mc.backward(0.3)
mc.strafe_left(0.3)
mc.strafe_right(0.3)
mc.turn_left(0.3)
mc.turn_right(0.3)

# Full control (all values in [-1.0, 1.0]):
mc.set_velocity(heading=0.2, lateral=0.1, turning=0.05)

mc.stop()                   # zero all axes
mc.get_current_velocity()   # returns dict of current axes
mc.shutdown()               # stop publish thread
```

### CameraReceiver

```python
cam = CameraReceiver('/depth_camera_front/color/image_raw')

frame = cam.get_frame()               # numpy array or None
frame, stamp = cam.get_frame_with_timestamp()  # frame + ROS time
cam.is_receiving()                     # True if frames arriving
cam.frame_count                        # total frames received
cam.shutdown()                         # unsubscribe
```

---

## Axis Mapping

| Index | Name    | Effect              | Positive = |
|-------|---------|---------------------|------------|
| 0     | LATERAL | Strafe left/right   | Left       |
| 1     | HEADING | Forward/backward    | Forward    |
| 2     | ROLL    | Body roll           |            |
| 3     | TURNING | Yaw rotation        | CCW        |
| 4     | VERTICAL| Body height         |            |
| 5     | PITCH   | Body pitch          |            |

Values are scale factors `[-1.0, 1.0]`. The joy_manager multiplies them by the
active motion controller's max velocity.

---

## Operational Workflow

```
1. Start ANYmal launcher
2. Use GUI: Stand up robot
3. Use GUI: Activate locomotion (e.g., trotting)
4. Start our container: docker compose up -d
5. Use API: mc.forward(0.1)  # start slow!
```

**The robot must be in locomotion mode** for movement commands to work.
The joy_manager reads velocity limits from the active controller —
if no controller is active, all limits are 0 and commands have no effect.

---

## Safety

- **Auto-stop on timeout**: The joy_manager has a 0.5s timeout. If our script
  crashes or we stop publishing, the robot halts automatically.
- **Velocity limits**: Commands go through the joy_manager which applies the
  active controller's max velocity. You cannot exceed these limits.
- **Start slow**: Begin with `speed=0.1` and increase gradually.
- **E-stop**: Always have the hardware e-stop ready.

---

## Troubleshooting

### "rostopic list" hangs or returns nothing
```bash
ping anymal-d181-lpc              # check robot hostname resolves
echo $ROS_MASTER_URI              # should be http://anymal-d181-lpc:11311
echo $ROS_IP                      # should be 192.168.0.225
```

### "No module named joy_manager_msgs"
```bash
source /catkin_ws/devel/setup.bash
```

### Camera topic returns None
```bash
rostopic hz <camera_topic>        # is anything being published?
rostopic list | grep image_raw    # find correct topic name
```

### Movement commands have no effect
```bash
rostopic echo -n 1 /motion_control_manager/active_motion
# If motion_controller is empty, activate locomotion in the ANYmal GUI first.
```

### Robot doesn't stop
1. Use hardware e-stop (physical button)
2. Or publish zeros: `rostopic pub --once /anyjoy/operator joy_manager_msgs/AnyJoy "{joy: {axes: [0,0,0,0,0,0], buttons: []}, modules: [], commands: []}"`

---

## Cleanup

```bash
cd ~/anymal_custom_control
docker compose down

# To remove everything:
rm -rf ~/anymal_custom_control
docker rmi anymal-custom-control
```
