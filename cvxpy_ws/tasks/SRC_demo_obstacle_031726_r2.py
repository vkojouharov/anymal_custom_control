import numpy as np

TASK = {
    "start": np.array([0.0, 0.0], dtype=float),
    "targets": [
        {"center": np.array([0.5, -2.0], dtype=float), "radius": 1.75},
        {"center": np.array([3.5, 1.0], dtype=float), "radius": 1.75},
        {"center": np.array([5.0, -0.5], dtype=float), "radius": 1.75},
    ],
    "obstacles": [
        {"center": np.array([3.0, 0.1], dtype=float), "radius": 1.0}
    ]
}
