"""ANYmal Custom Control public API.

The exports are lazy so control-only nodes do not require camera/OpenCV or
ANYmal locomotion dependencies merely to import this package.
"""

__all__ = ["CameraReceiver", "ModeController", "MovementController"]


def __getattr__(name):
    if name == "CameraReceiver":
        from anymal_custom_control.camera import CameraReceiver

        return CameraReceiver
    if name == "ModeController":
        from anymal_custom_control.modes import ModeController

        return ModeController
    if name == "MovementController":
        from anymal_custom_control.movement import MovementController

        return MovementController
    raise AttributeError(name)
