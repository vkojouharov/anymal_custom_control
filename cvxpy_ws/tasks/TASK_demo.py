import numpy as np

TASK = {
    "start": np.array([0.0, 0.0], dtype=float),
    "targets": [
        {"center": np.array([1.0, 1.0], dtype=float), "radius": 0.5},
        {"center": np.array([3.0, -1.0], dtype=float), "radius": 0.5},
        {"center": np.array([5.0, 0.0], dtype=float), "radius": 0.5},
    ],
    "obstacles": [
        {"center": np.array([2.0, 0.25], dtype=float), "radius": 0.75},
        {"center": np.array([4.0, -0.25], dtype=float), "radius": 0.75},
    ]
}
