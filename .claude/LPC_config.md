# ANYmal LPC Configuration Notes

## LPC Access
- SSH: `ssh integration@anymal-d181-lpc`
- The LPC runs the ANYmal software stack entirely in Docker containers
- Containers are **recreated from images on every reboot** — filesystem changes inside containers do NOT persist

## Docker Containers on LPC

| Container | Image | Purpose |
|---|---|---|
| any-locomotion-1 | anymal-robot-production:release-25.09 | Locomotion controller (computes joint torques, uses URDF for mass model) |
| any-inspection-1 | anymal-robot-production:release-25.09 | Inspection stack |
| any-lpc_drivers-1 | anymal-robot-production:release-25.09 | LPC hardware drivers |
| any-lpc_background-1 | anymal-robot-production:release-25.09 | LPC background services |
| any-inspection_drivers-1 | anymal-robot-production:release-25.09 | Inspection payload drivers |
| any-roscore-1 | anymal-robot-production:release-25.09 | ROS master |
| any-livekit-bridge-lpc-1 | livekit-bridge:25.136.102 | LiveKit bridge |
| any-edgeBroker-1 | eclipse-mosquitto:2.0-openssl | MQTT broker |

All main containers use the same image: `docker.anymal.com/anymal/downloads/anymal-robot-production:release-25.09`

## URDF Location

- **Active URDF used by locomotion controller**: `/opt/ros/noetic/share/loco_anymal/test/resources/anymal_d_20240220.urdf` (inside `any-locomotion-1` container)
- This URDF was auto-generated from `anymal_d.urdf.xacro`
- The current URDF does **NOT** include the inspection payload — no `inspection_payload` links/joints
- It has 76 link definitions

## Inspection Payload URDF Data

Source: `~/ANYmal/anymal-research-software/anymal/anymal/anymal_simulation/anymal_gazebo/urdf/anymal_d.urdf` on dev machine (lines 2208-2330)

The simulation URDF includes the full inspection payload with three mass-bearing components:

| Component | Mass (kg) | CoM xyz | Attached to |
|---|---|---|---|
| inspection_payload_mount | 0.21580 | 0.00290, 0.00, 0.01050 | base (fixed joint at xyz 0.14253, 0.0, 0.092) |
| inspection_payload_pan | 3.18886 | 0.00068, -0.00349, 0.07578 | mount (offset xyz 0, 0, 0.023) |
| inspection_payload_tilt | 2.084 | 0.00534, -0.00379, -0.00091 | pan (offset xyz 0.03, 0, 0.152) |
| **Total** | **~5.49 kg** | | |

Full inertia tensors are in the simulation URDF file above.

## Feature Toggle (did NOT work)

- `~/.ros/config.yaml` on LPC has `inspection_payload: disabled/enabled` toggle
- Setting to `enabled` and rebooting did NOT add the payload to the locomotion URDF
- The toggle may only affect other aspects of the stack (drivers, inspection software), not the URDF itself

## What We Tried

1. Copied URDF out of container: `docker cp any-locomotion-1:/opt/ros/.../anymal_d_20240220.urdf ~/anymal_d_current.urdf`
2. Backup created: `~/anymal_d_current.urdf.backup` (on LPC home dir)
3. Planned to add payload links/joints (mass/inertia only, no visuals) before `</robot>` tag
4. Would copy back: `docker cp ~/anymal_d_current.urdf any-locomotion-1:/opt/ros/.../anymal_d_20240220.urdf`
5. **Problem**: Containers are recreated on reboot, so changes are lost

## TODO: Persistent URDF Modification

Need to find one of:
1. **Docker compose file** on LPC that defines the containers — could add a volume mount for the modified URDF
2. **Config rendering pipeline** in `~/.anymal/rendered_configs/` — may need to modify templates so the URDF is regenerated with payload
3. **ANYbotics-supported method** from their email: custom xacro + extension-config-file pointing to it, with `.ros/config.yaml` override for `xacro_file` path

### Validate URDF before deploying (no xmllint on LPC)
```bash
python3 -c "import xml.etree.ElementTree as ET; ET.parse('anymal_d_current.urdf'); print('URDF is valid XML')"
```

### Recovery if URDF breaks locomotion
```bash
docker cp ~/anymal_d_current.urdf.backup any-locomotion-1:/opt/ros/noetic/share/loco_anymal/test/resources/anymal_d_20240220.urdf
```
Or just reboot — container recreation restores the original.
