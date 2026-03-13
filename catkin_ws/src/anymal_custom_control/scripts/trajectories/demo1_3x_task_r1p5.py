# ANYmal waypoints: (dx, dy, dyaw) displacements from start pose [meters, rad]
ANYMAL_WAYPOINTS = [
    (-2.053, -0.501, 0.0),     # waypoint 1: 0.5m forward
    (-3.093, -0.324, 0.0),     # waypoint 2: hold x, 0.3m left
    (-3.521, -0.251, 0.0),     # waypoint 3: another 0.5m forward
]

# Arm EE waypoints: (x, y, z) target positions in arm base frame [meters]
ARM_WAYPOINTS = [
    (-0.053, 1.499, 0.5),     # waypoint 1
    (-0.093, -1.324, 0.5),     # waypoint 2
    (1.479, -0.251, 0.5),     # waypoint 3
]