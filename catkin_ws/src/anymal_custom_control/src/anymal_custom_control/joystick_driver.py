"""Gamepad driver using evdev — works headless in Docker containers."""
import sys
import time
import evdev
from evdev import ecodes

# Xbox Series X controller axis codes
_AXIS_CODES = {
    ecodes.ABS_X:  'LX',
    ecodes.ABS_Y:  'LY',
    ecodes.ABS_RX: 'RX',
    ecodes.ABS_RY: 'RY',
    ecodes.ABS_Z:  'LT',
    ecodes.ABS_RZ: 'RT',
    ecodes.ABS_HAT0X: 'DPAD_X',
    ecodes.ABS_HAT0Y: 'DPAD_Y',
}

# Xbox Series X controller button codes
_BUTTON_CODES = {
    ecodes.BTN_SOUTH: 'AB',
    ecodes.BTN_EAST:  'BB',
    ecodes.BTN_NORTH: 'XB',
    ecodes.BTN_WEST:  'YB',
    ecodes.BTN_TL:    'LB',
    ecodes.BTN_TR:    'RB',
    ecodes.BTN_SELECT: 'MENULEFT',
    ecodes.BTN_START:  'MENURIGHT',
    ecodes.BTN_THUMBL: 'LSB',
    ecodes.BTN_THUMBR: 'RSB',
}

# Axes that should be inverted (push-up = positive)
_INVERT_AXES = {'LY', 'RY'}

# Axes that are triggers (0 to 1 instead of -1 to 1)
_TRIGGER_AXES = {'LT', 'RT'}


def joystick_connect(device_index=0):
    """Find and return a gamepad device.

    Returns a dict with 'device' (evdev.InputDevice), 'absinfo' (axis calibration),
    and 'state' (current axis/button values, normalized).
    """
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    gamepads = []
    for dev in devices:
        caps = dev.capabilities()
        if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
            gamepads.append(dev)

    if not gamepads:
        raise RuntimeError("No gamepad found. Is the controller plugged in?")

    if device_index >= len(gamepads):
        raise RuntimeError(f"Gamepad index {device_index} out of range ({len(gamepads)} found)")

    dev = gamepads[device_index]

    # Build axis calibration lookup: code -> (min, max)
    absinfo = {}
    for code, info in dev.capabilities().get(ecodes.EV_ABS, []):
        absinfo[code] = (info.min, info.max)

    # Initialize state with current values
    state = {
        'LX': 0.0, 'LY': 0.0, 'RX': 0.0, 'RY': 0.0,
        'LT': 0.0, 'RT': 0.0,
        'DPAD_X': 0.0, 'DPAD_Y': 0.0,
        'AB': 0, 'BB': 0, 'XB': 0, 'YB': 0,
        'LB': 0, 'RB': 0,
        'MENULEFT': 0, 'MENURIGHT': 0,
        'LSB': 0, 'RSB': 0,
    }

    return {'device': dev, 'absinfo': absinfo, 'state': state}


def _normalize_axis(value, min_val, max_val, name):
    """Normalize raw axis value to [-1, 1] or [0, 1] for triggers."""
    if max_val == min_val:
        return 0.0
    if name in _TRIGGER_AXES:
        # Triggers: map [min, max] -> [0, 1]
        normalized = (value - min_val) / (max_val - min_val)
    else:
        # Sticks: map [min, max] -> [-1, 1]
        center = (min_val + max_val) / 2.0
        half_range = (max_val - min_val) / 2.0
        normalized = (value - center) / half_range
        if name in _INVERT_AXES:
            normalized = -normalized
    return max(-1.0, min(1.0, normalized))


def joystick_read(js, deadzone=0.1):
    """Read and process all pending events, return current state dict.

    Values: axes [-1, 1], triggers [0, 1], buttons 0 or 1.
    """
    dev = js['device']
    absinfo = js['absinfo']
    state = js['state']

    # Process all pending events (non-blocking)
    try:
        while True:
            event = dev.read_one()
            if event is None:
                break
            if event.type == ecodes.EV_ABS:
                name = _AXIS_CODES.get(event.code)
                if name and event.code in absinfo:
                    min_val, max_val = absinfo[event.code]
                    state[name] = _normalize_axis(event.value, min_val, max_val, name)
            elif event.type == ecodes.EV_KEY:
                name = _BUTTON_CODES.get(event.code)
                if name:
                    state[name] = 1 if event.value else 0
    except BlockingIOError:
        pass

    # Apply deadzone to stick axes
    result = dict(state)
    for axis in ('LX', 'LY', 'RX', 'RY'):
        if -deadzone < result[axis] < deadzone:
            result[axis] = 0.0

    return result


def joystick_disconnect(js):
    """Close the gamepad device."""
    js['device'].close()


def main():
    debug = "-debug" in sys.argv
    js = joystick_connect()
    print(f"Connected: {js['device'].name}")
    try:
        while True:
            data = joystick_read(js)
            if debug:
                axes = "  ".join(f"{k}:{data[k]:+.2f}" for k in ('LX', 'LY', 'RX', 'RY', 'LT', 'RT'))
                btns = "  ".join(f"{k}:{'ON' if data[k] else 'off'}" for k in ('AB', 'BB', 'XB', 'YB', 'LB', 'RB', 'MENULEFT', 'MENURIGHT'))
                sys.stdout.write(f"\r{axes}  {btns}  ")
                sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        joystick_disconnect(js)
        print()


if __name__ == "__main__":
    main()
