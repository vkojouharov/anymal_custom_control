# GIRAF robot-side OptiTrack teleoperation

This stack keeps all actuator ownership on the robot NUC. The operator desktop
publishes only `/giraf_arm/teleop_task_velocity_cmd`; it must never run CANdle,
the MD80 services, or the Dynamixel driver.

## Network modes

Field/default mode is unchanged:

```bash
docker compose up -d
```

The default compose file uses the robot-facing field address
`192.168.0.231`. It remains independent of SRC.

Lab/SRC mode applies the checked-in override:

```bash
docker compose -f docker-compose.yml -f docker-compose.lab.yml up -d
```

This sets both `ROS_MASTER_URI=http://172.24.68.55:11311` and
`ROS_IP=172.24.68.55` for the NUC. The operator desktop must advertise its own
reachable SRC address (`172.24.69.1`), not the NUC address. ROS1 requires
bidirectional TCP connectivity and dynamic TCPROS ports in addition to port
11311.

Only one compose mode may be running at a time. Both modes use the same
`anymal-control` service and the same hardware-control container.

## Dry-run commissioning

Start the lab container, then run:

```bash
docker exec -it anymal_custom_control \
  python3 /catkin_ws/src/anymal_custom_control/scripts/run/run_giraf_robot_side.py \
  --backend dry-run --start-master
```

Dry-run is the launcher's default. It opens no CAN, USB, U2D2, Dynamixel, or
motor service. It uses the authoritative kinematics and the same J-PARSE,
integration, task-clamp, and joint-clamp logic in a separate dry-run node. The
proven hardware controller is intentionally not refactored or wrapped by this
backend.

For compatibility, older joystick/operator-station launchers that invoke
`run_giraf_arm_controller.py` without ROS parameters retain their historical
hardware behavior. Use `run_giraf_robot_side.py` for new remote workflows so
backend selection and physical-home confirmation are explicit.

It publishes:

- `/giraf_arm/state` (`std_msgs/String`) at 10 Hz;
- `/giraf_arm/readiness` (`std_msgs/String`) at 10 Hz;
- `/md80/joint_states` (`sensor_msgs/JointState`) at 10 Hz.

The controller begins at nominal home but reports `active_source` as
`initializing` until it receives a fresh all-zero command from the selected
source. It then reports both source fields as `teleop`. A stale command always
becomes zero task and gripper velocity and therefore holds the integrated
position target.

`/giraf_arm/state.arm` is the controller's integrated **command state**, not a
complete measured six-joint state. Dry-run MD80 feedback mirrors the simulated
base command state. Hardware `/md80/joint_states` remains measured CANdle
feedback.

## Supervised hardware startup

Before running the following command:

1. Test and hold the physical estop.
2. Ensure no other controller, joystick teleop, or CANdle owner is running.
3. Manually place the physical arm in its established joystick-startup home.
4. Verify the MAB roll, pitch, and retracted-boom home; these will be encoder
   zero (`0 rad`) when the MD80 startup service is called.
5. Verify the metal wrist is safe to hold/command to the checked-in home ticks.
6. Keep the operator desktop at Space released so it is publishing zero.

Then run:

```bash
docker exec -it anymal_custom_control \
  python3 /catkin_ws/src/anymal_custom_control/scripts/run/run_giraf_robot_side.py \
  --backend hardware --start-master \
  --confirm-home ARM_PHYSICALLY_AT_HOME
```

Hardware mode starts exactly one `candle_ros_node` and the original
`giraf_arm_controller`. The explicit launcher confirmation protects entry into
the existing home convention:

- add and zero MD80 IDs 11/12/13 at the physically staged MAB home;
- configure impedance mode and enable them;
- connect/configure the Dynamixels using the established driver;
- immediately command MAB targets `[0, 0, 0]` and the checked-in
  wrist/gripper home;
- enter the original 200 Hz hardware control loop.

The checked-in metal-wrist home is:

| Coordinate | Home | Source |
|---|---:|---|
| roll | 0 rad | MD80 encoder zero at staged home |
| pitch | 0 rad | MD80 encoder zero at staged home |
| boom spool | approximately 0 rad | retracted `d3=0.31 m` cubic mapping |
| th4 / Dynamixel 11 | 2030 ticks | `dynamixel/control_table.py` |
| th5 / Dynamixel 12 | 2003 ticks | `dynamixel/control_table.py` |
| th6 / Dynamixel 13 | 2068 ticks | `dynamixel/control_table.py` |
| gripper / Dynamixel 14 | 680 ticks, open | `dynamixel/control_table.py` |

The active kinematic coordinate offsets remain
`[0, +pi/2, 0, +pi/2, -pi/2, 0]` from `giraf_arm_common.py` and the controller
uses only `RRPRRR_kinematic_model.py` for full-arm FK/Jacobians.

## Readiness contract

The dry-run backend publishes `/giraf_arm/readiness` JSON containing:

- `backend`, `ready`, `hardware_connected`, `feedback_valid`, and `homed`;
- `command_source` and `watchdog_state`;
- `md80_enabled` and `dynamixel_torque_enabled`;
- `stop_latched`, `estop_state`, `active_faults`;
- `last_command_receipt_age_sec`.

The production hardware controller does not publish this optional readiness
topic; the desktop continues to use its existing state and MD80 interlocks.
The software has no estop input or complete MD80/Dynamixel fault topic, so
`estop_state` is explicitly reported as `not_available_in_software`. Do not
treat the readiness topic as a replacement for the physical estop.

## Remaining commissioning limitations

- MD80 feedback is unavailable before the existing enable path, so the
  software cannot independently prove physical MAB home before encoder zero.
  The explicit physical-home confirmation remains mandatory.
- The authoritative metal-wrist values above come from executable code, but
  the older Dynamixel README contains conflicting historical calibration.
  Verify the metal wrist physically before the first supervised run.
- Hardware `/giraf_arm/state` remains integrated command coordinates. Only the
  three MD80 positions are continuously published as measured ROS feedback;
  there is no unified measured wrist/gripper `JointState` yet.
- Acceleration/profile limits and an electronic estop/fault status are not
  exposed by the current combined hardware interface.
- `/giraf_arm/stop` is a latched intentional shutdown. Space release should
  continue publishing zero velocity and use controlled position hold.
