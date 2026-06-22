# Tape Spring Shear Center Simulator

Interactive Matplotlib tool for visualizing the centroid and shear center of a
thin-walled circular tape-spring cross section.

Run from the repository root:

```bash
python3 experiments/shear_center_sim/shear_center_gui.py
```

Export the 360-to-90-to-360 degree sweep animation:

```bash
python3 experiments/shear_center_sim/animate_shear_center.py
```

This writes `experiments/shear_center_sim/shear_center_sweep.mp4`.

## Model

- The cross section is a circular arc cut from a tube.
- `alpha = 360 deg` is treated as a closed circular tube.
- `alpha < 360 deg` is treated as an open thin-walled circular arc with its
  free opening oriented upward.
- The fixed `50 mm` diameter is the wall midline diameter.
- The nominal `1.0 mm` thickness is used only for drawing the wall band.
- The material is assumed homogeneous and isotropic, approximating fiberglass.
  In this geometric thin-wall model, material constants do not affect the
  centroid or shear-center location.

For open sections, the plot uses the thin-wall circular-arc formulas with
`beta = alpha / 2`:

```text
centroid_y = -R sin(beta) / beta
shear_center_y = -2R (sin(beta) - beta cos(beta)) / (beta - sin(beta) cos(beta))
```

The exact `360 deg` state intentionally jumps to the closed-tube result:
centroid and shear center both at the circle center.
