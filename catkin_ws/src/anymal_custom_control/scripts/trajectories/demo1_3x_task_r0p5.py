# ANYmal waypoints: (dx, dy, dyaw) displacements from start pose [meters, rad]
ANYMAL_WAYPOINTS = [
    (-0.895, -0.511, 0.0),     # waypoint 1: 0.5m forward
    (-1.642,  0.521, 0.0),     # waypoint 2: hold x, 0.3m left
    (-2.533,  0.179, 0.0),     # waypoint 3: another 0.5m forward
]

# Arm EE waypoints: (x, y, z) target positions in arm base frame [meters]
ARM_WAYPOINTS = [
    (0.105, 0.489, 0.5),     # waypoint 1
    (-0.142, -0.479, 0.5),     # waypoint 2
    (0.467, 0.179, 0.5),     # waypoint 3
]