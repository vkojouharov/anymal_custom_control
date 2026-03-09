# CANdle ROS in Docker — Setup Guide

## Overview

The MAB Robotics CANdle (USB-TO-CAN adapter) communicates with MD80 brushless servo motors over CAN bus. The `candle_ros` package wraps the `candle` C++ library as a ROS Noetic node.

## Hardware

- **Adapter:** MAB Robotics MD USB-TO-CAN (vendor `0069`, product `1000`)
- **Host device:** `/dev/ttyACM0` (registered via kernel `cdc_acm` driver)
- **Motors:** IDs 11 (ROLL), 12 (PITCH), 13 (BOOM) at **1M CAN baud rate**

## Host Prerequisites

1. Install mdtool (for diagnostics, not required at runtime):
   ```bash
   # Download amd64 .deb from https://github.com/mabrobotics/mdtool/releases
   sudo dpkg -i mdtool-amd64-<version>-Linux.deb
   sudo adduser $USER dialout
   # Log out and back in for group to take effect
   ```

2. Verify hardware on host:
   ```bash
   lsusb | grep -i mab
   # Expected: Bus 003 Device XXX: ID 0069:1000 MAB Robotics MD USB-TO-CAN

   mdtool ping all
   # Expected: Found drives at 1M — IDs 11, 12, 13
   ```

## Docker Configuration

### Dockerfile — Required packages

```dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev \
    udev \
    setserial \
    && rm -rf /var/lib/apt/lists/*
```

- **`udev`** — Critical. The candle library's USB device detection calls `udevadm info /dev/ttyACMx` to verify vendor/product identity. Without udev installed, the command returns empty and device detection silently fails with `[USB] No devices found!`.
- **`setserial`** — Used by candle library to set low-latency mode on the serial device.
- **`libusb-1.0-0-dev`** — USB library dependency for candle.

### docker-compose.yml — Required devices and volumes

```yaml
services:
  anymal-control:
    privileged: true
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
    volumes:
      - /run/udev:/run/udev:ro
```

- **`/dev/ttyACM0`** — The CANdle serial device.
- **`/run/udev:ro`** — The host's udev database. Required so `udevadm info` inside the container can resolve device metadata (vendor, product, serial).
- **`privileged: true`** — Required for USB device access.

### candle submodule version

The `candle_ros` package includes the `candle` C++ library as a git submodule at `catkin_ws/src/candle_ros/candle`. Current working version: **v3.5.3** (matches host mdtool).

To update:
```bash
cd catkin_ws/src/candle_ros
git -C candle checkout v3.5.3
# Then rebuild Docker image
```

## Running the Node

### Launch candle_ros_node

```bash
docker exec -it anymal_custom_control bash
rosrun candle_ros candle_ros_node USB 1M
```

Expected output:
```
[CANDLE] CANdle library version: v3.5.3
[CANDLE] Device firmware version: v2.2.1
[CANDLE] CANdle at /dev/ttyACM0, ID: 0xac12a3d6ae741d0f ready (USB)
[CANDLE] Found CANdle with ID: 12399152866549570831
[INFO] [...]: candle_ros_node v1.3.2 has started.
```

**Bus type must be `USB`** (not `UART`). The candle library handles the `/dev/ttyACM0` serial path internally when using USB mode.

### Test motor connectivity (in a second terminal)

```bash
docker exec -it anymal_custom_control bash

# Add motors
rosservice call /add_md80s "drive_ids: [11, 12, 13]"

# Zero encoders
rosservice call /zero_md80s "drive_ids: [11, 12, 13]"

# Set impedance mode
rosservice call /set_mode_md80s "drive_ids: [11, 12, 13]" "mode: ['IMPEDANCE', 'IMPEDANCE', 'IMPEDANCE']"

# Enable motors (starts real-time comms + 10 Hz joint state publishing)
rosservice call /enable_md80s "drive_ids: [11, 12, 13]"

# Verify feedback
rostopic echo /md80/joint_states

# Disable when done
rosservice call /disable_md80s "drive_ids: [11, 12, 13]"
```

## ROS Interface Summary

### Services (setup)
| Service | Type | Purpose |
|---------|------|---------|
| `/add_md80s` | `AddMd80s` | Register motors by CAN ID |
| `/zero_md80s` | `GenericMd80Msg` | Zero encoders at current position |
| `/set_mode_md80s` | `SetModeMd80s` | Set mode: `IMPEDANCE`, `POSITION_PID`, `VELOCITY_PID`, `RAW_TORQUE` |
| `/enable_md80s` | `GenericMd80Msg` | Enable motors + start comms |
| `/disable_md80s` | `GenericMd80Msg` | Stop comms + disable motors |

### Topics (runtime)
| Topic | Type | Direction | Purpose |
|-------|------|-----------|---------|
| `md80/motion_command` | `MotionCommand` | Subscribe | Target position/velocity/torque |
| `md80/impedance_command` | `ImpedanceCommand` | Subscribe | Set kp, kd, max_torque |
| `md80/joint_states` | `sensor_msgs/JointState` | Publish (10 Hz) | Position, velocity, torque feedback |

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `[USB] No devices found!` | `udevadm` not installed or udev DB not mounted | Install `udev` package + mount `/run/udev:ro` |
| Node hangs on startup (no output) | ROS master unreachable | Verify `ping anymal-d181-lpc` and `rostopic list` work first |
| `[UART] ERROR CRC!` | Wrong bus type | Use `USB 1M`, not `UART` |
