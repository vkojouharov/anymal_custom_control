# ANYmal waypoints: (dx, dy, dyaw) displacements from start pose [meters, rad]
ANYMAL_WAYPOINTS = [
    (-1.900, -1.257, 0.0),     # waypoint 1: 0.5m forward
    (-3.252,  0.294, 0.0),     # waypoint 2: hold x, 0.3m left
    (-4.260,  0.124, 0.0),     # waypoint 3: another 0.5m forward
]

# Arm EE waypoints: (x, y, z) target positions in arm base frame [meters]
ARM_WAYPOINTS = [
    (0.100, 0.743, 0.5),     # waypoint 1
    (-0.252, -0.706, 0.5),     # waypoint 2
    (0.740, 0.124, 0.5),     # waypoint 3
]