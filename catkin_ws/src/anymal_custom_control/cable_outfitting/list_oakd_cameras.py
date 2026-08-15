#!/usr/bin/env python3
"""Print stable identifiers and link information for connected OAK cameras."""

import depthai as dai


def value(obj, name, default="unknown"):
    item = getattr(obj, name, default)
    return item() if callable(item) else item


def main() -> int:
    print(f"DepthAI {dai.__version__}")
    devices = dai.Device.getAllAvailableDevices()
    print(f"Connected cameras: {len(devices)}")
    if not devices:
        return 1
    for index, info in enumerate(devices):
        print(f"\n[{index}] name={value(info, 'name')} mxid={info.getMxId()} state={value(info, 'state')}")
        try:
            with dai.Device(info) as device:
                calibration = device.readCalibration()
                eeprom = calibration.getEepromData()
                speed = str(device.getUsbSpeed()).rsplit(".", 1)[-1]
                cameras = ", ".join(str(item).rsplit(".", 1)[-1] for item in device.getConnectedCameras())
                print(f"    usb_speed={speed} cameras={cameras or 'none'} imu={device.getConnectedIMU()}")
                print(f"    product={value(eeprom, 'productName')} board={value(eeprom, 'boardName')} revision={value(eeprom, 'boardRev')}")
        except Exception as exc:
            print(f"    open_error={exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
