"""MD80 motor driver via candle_ros — matches joystick_driver.py pattern."""
import warnings
import rospy
from candle_ros.srv import AddMd80s, GenericMd80Msg, SetModeMd80s
from candle_ros.msg import ImpedanceCommand, MotionCommand
from sensor_msgs.msg import JointState

IDS = {"ROLL": 11, "PITCH": 12, "BOOM": 13}
_ALL_IDS = list(IDS.values())
_GAIN_KEYS = ("kp", "kd", "max_torque")


def _normalize_gain_overrides(gain_overrides):
    if not gain_overrides:
        return {}

    normalized = {}
    for drive_id, gains in gain_overrides.items():
        drive_id = int(drive_id)
        if drive_id not in _ALL_IDS:
            warnings.warn(f"Ignoring gain override for unknown motor ID {drive_id}", RuntimeWarning)
            continue

        drive_gains = {}
        for key in _GAIN_KEYS:
            value = gains.get(key)
            if value is not None:
                drive_gains[key] = float(value)
        if drive_gains:
            normalized[drive_id] = drive_gains
    return normalized


def _gain_values(default_value, gain_overrides, gain_name):
    return [
        gain_overrides.get(drive_id, {}).get(gain_name, default_value)
        for drive_id in _ALL_IDS
    ]


def motor_connect(kp=1000.0, kd=50.0, max_torque=25.0, gain_overrides={11: {"kp": 1000.0, "kd": 50.0, "max_torque": 25.0}}):
    """Add, zero, configure impedance mode, and enable all motors.

    ``kp``, ``kd``, and ``max_torque`` set the baseline for every MD80.
    ``gain_overrides`` can override individual IDs, for example::

        {11: {"kp": 100.0, "kd": 5.0, "max_torque": 6.0}}

    Returns a dict with 'motion_pub' and 'latest_joint_state'.
    """
    rospy.wait_for_service('/add_md80s', timeout=10)
    srv_add = rospy.ServiceProxy('/add_md80s', AddMd80s)
    srv_zero = rospy.ServiceProxy('/zero_md80s', GenericMd80Msg)
    srv_mode = rospy.ServiceProxy('/set_mode_md80s', SetModeMd80s)
    srv_enable = rospy.ServiceProxy('/enable_md80s', GenericMd80Msg)

    # Add motors
    resp = srv_add(drive_ids=_ALL_IDS)
    for i, ok in enumerate(resp.drives_success):
        if not ok:
            raise RuntimeError(f"Failed to add motor ID {_ALL_IDS[i]}")

    # Zero encoders
    resp = srv_zero(drive_ids=_ALL_IDS)
    for i, ok in enumerate(resp.drives_success):
        if not ok:
            warnings.warn(f"Failed to zero motor ID {_ALL_IDS[i]}", RuntimeWarning)

    # Set impedance mode
    modes = ["IMPEDANCE"] * len(_ALL_IDS)
    resp = srv_mode(drive_ids=_ALL_IDS, mode=modes)
    for i, ok in enumerate(resp.drives_success):
        if not ok:
            raise RuntimeError(f"Failed to set mode for motor ID {_ALL_IDS[i]}")

    # Set impedance params
    gain_overrides = _normalize_gain_overrides(gain_overrides)
    imp_pub = rospy.Publisher('md80/impedance_command', ImpedanceCommand, queue_size=1, latch=True)
    imp_msg = ImpedanceCommand()
    imp_msg.drive_ids = _ALL_IDS
    imp_msg.kp = _gain_values(float(kp), gain_overrides, "kp")
    imp_msg.kd = _gain_values(float(kd), gain_overrides, "kd")
    imp_msg.max_output = _gain_values(float(max_torque), gain_overrides, "max_torque")
    imp_pub.publish(imp_msg)

    # Enable motors (also starts candle.begin() on the node)
    resp = srv_enable(drive_ids=_ALL_IDS)
    for i, ok in enumerate(resp.drives_success):
        if not ok:
            raise RuntimeError(f"Failed to enable motor ID {_ALL_IDS[i]}")

    # Publisher for motion commands
    motion_pub = rospy.Publisher('md80/motion_command', MotionCommand, queue_size=1)

    # Subscribe to joint states for feedback
    state = {'joint_state': None}

    def _joint_state_cb(msg):
        state['joint_state'] = msg

    rospy.Subscriber('md80/joint_states', JointState, _joint_state_cb)

    return {'motion_pub': motion_pub, 'state': state}


def motor_status(ctx):
    """Print latest joint state feedback (position, velocity, torque)."""
    js = ctx['state']['joint_state']
    if js is None:
        print("No joint state received yet.")
        return
    for i, name in enumerate(js.name):
        pos = js.position[i] if i < len(js.position) else 0.0
        vel = js.velocity[i] if i < len(js.velocity) else 0.0
        eff = js.effort[i] if i < len(js.effort) else 0.0
        print(f"  {name}: pos={pos:.3f}  vel={vel:.3f}  torque={eff:.3f}")


def motor_drive(ctx, roll, pitch, boom):
    """Command target positions for all three motors."""
    msg = MotionCommand()
    msg.drive_ids = [IDS["ROLL"], IDS["PITCH"], IDS["BOOM"]]
    msg.target_position = [roll, pitch, boom]
    msg.target_velocity = [0.0, 0.0, 0.0]
    msg.target_torque = [0.0, 0.0, 0.0]
    ctx['motion_pub'].publish(msg)


def motor_disconnect():
    """Disable all motors."""
    try:
        srv_disable = rospy.ServiceProxy('/disable_md80s', GenericMd80Msg)
        srv_disable(drive_ids=_ALL_IDS)
    except rospy.ServiceException as e:
        warnings.warn(f"Failed to disable motors: {e}", RuntimeWarning)
