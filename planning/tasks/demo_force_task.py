import numpy as np

TASK = {
    "start": np.array([0.0, 0.0], dtype=float),
    "targets": [
        {"center": np.array([1.5, 1.0, 1.0], dtype=float), "radius": 1.5, "force": np.array([1.0, 0.0, 0.0], dtype=float)},
        {"center": np.array([4.0, -1.5, 1.0], dtype=float), "radius": 1.5, "force": np.array([0.0, 0.0, 1.0], dtype=float)},
        {"center": np.array([6.0, 0.0, 1.0], dtype=float), "radius": 1.5, "force": np.array([-1.0, 0.0, -1.0], dtype=float)},
    ],
    # "obstacles": [
    #     {"center": np.array([2.0, 0.25], dtype=float), "radius": 0.75},
    #     {"center": np.array([4.0, -0.25], dtype=float), "radius": 0.75},
    # ]
}
