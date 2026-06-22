# Boom Compliance Notes

This folder contains the current workflow for converting OptiTrack/load-cell
captures into shear-center wrench/displacement data, fitting a 6x6 compliance
matrix, and visualizing the result.

## Geometry And Mechanics

- The boom is modeled as an open circular thin-walled section.
- Current default geometry:
  - midline diameter: `40 mm`
  - subtended angle: `270 deg`
  - nominal wall thickness for visualization: `1 mm`
  - force magnitude for current dataset: `5 N`
  - boom length for residual weighting: `1.5 m`
- The cross-section opening is in `+Y` of the beam frame.
- The shear center is offset from the circle center in `-Y`.

For an open circular arc with subtended angle `2 alpha` and midline radius `R`:

```text
y = opening direction
y_sc = -2R (sin(alpha) - alpha cos(alpha)) / (alpha - sin(alpha) cos(alpha))
```

For `D = 40 mm` and `2 alpha = 270 deg`:

```text
y_sc = -33.236 mm
```

Important interpretation:

- The circle center is the center of curvature, not the centroid.
- The geometric centroid and circle center are not generally no-twist points.
- A transverse force through the shear center produces no induced longitudinal
  twist in the ideal beam-section model.
- A force through the circle center or centroid generally produces a torsional
  moment about the shear center.
- The shear center is not an instantaneous pivot. The endpoint can translate
  substantially at the shear center.

## Frame Convention

Raw `id34` rigid-body frame from the capture:

```text
raw +Z = boom longitudinal direction
raw -Y = open side of the cross section
raw +X = remaining right-handed transverse direction
```

Beam frame used by converted CSVs and fitting:

```text
beam +X = raw +Z      longitudinal
beam +Y = raw -Y      opening direction
beam +Z = raw +X      completes right-handed frame
```

In code:

```python
R_BEAM_FROM_ID34 = np.array([
    [0.0,  0.0, 1.0],
    [0.0, -1.0, 0.0],
    [1.0,  0.0, 0.0],
])
```

All converted wrench and displacement values are expressed in the undeformed
endpoint frame, defined by `id34_t1`.

## Raw Data Meaning

The current raw file is:

```text
data_1p5m_5N.csv
```

Each row contains:

```text
id34_t1_*  undeformed boom endpoint pose
id34_t2_*  deformed boom endpoint pose
id35_t2_*  load-cell pose
```

The applied force direction is:

```text
id35_t2 position - id34_t2 position
```

The force is treated as applied at the deformed circle center. The current
dataset uses constant force magnitude `5 N`.

## Wrench And Displacement Transform

The converter writes:

```text
data_1p5m_5N_shear_center.csv
```

with 12 columns:

```text
Fx_N,Fy_N,Fz_N,Mx_Nm,My_Nm,Mz_Nm,
ux_m,uy_m,uz_m,theta_x_rad,theta_y_rad,theta_z_rad
```

The force is first computed at the circle center. It is then converted to an
equivalent wrench about the shear center:

```text
r_sc_to_circle_center = [0, -y_sc, 0]
M_sc = r_sc_to_circle_center x F
```

With signed `y_sc < 0`:

```text
r_sc_to_circle_center = [0, positive distance, 0]
```

The measured circle-center displacement is converted to shear-center
displacement with the small-angle relation:

```text
u_sc = u_cc - theta x r_sc_to_circle_center
```

The rotation vector is computed from the relative orientation:

```text
R_rel = R_id34_t1.T @ R_id34_t2
theta = log(R_rel)
```

then mapped from raw `id34` coordinates into the beam frame.

## Fitting Model

The compliance model is:

```text
X = C W
```

where:

```text
X = [ux, uy, uz, theta_x, theta_y, theta_z]^T
W = [Fx, Fy, Fz, Mx, My, Mz]^T
```

The fit script is:

```text
fitting/fit_compliance_cvx.py
```

It solves the convex problem:

```text
minimize_C || H^(1/2) (X - C W) ||_F^2
subject to C = C^T
           C >= 0
```

Current weighting:

```text
H^(1/2) = diag(1, 1, 1, L, L, L)
L = 1.5 m
```

This makes angular residuals comparable to translational residuals through the
endpoint scale `L theta`.

Current compliance output:

```text
data_1p5m_5N_compliance.csv
```

The fitted matrix is symmetric positive definite. Earlier checks showed it is
physically plausible in scale, but fairly ill-conditioned, so validation plots
are important.

## Scripts

Convert raw capture to shear-center wrench/displacement CSV:

```bash
python3 experiments/boom_compliance/convert_raw_data.py
```

Fit compliance:

```bash
python3 experiments/boom_compliance/fitting/fit_compliance_cvx.py
```

Plot weighted residual boxplot:

```bash
python3 experiments/boom_compliance/fitting/plot_compliance_errors.py
```

Visualize one raw pose transform without shear-center/beam conversions:

```bash
python3 experiments/boom_compliance/visualize/visualize_raw_pose_relative.py --row 1
```

Visualize one converted reference-frame row:

```bash
python3 experiments/boom_compliance/visualize/visualize_reference_frame.py --row 1
```

Export one PNG per row:

```bash
python3 experiments/boom_compliance/visualize/export_reference_frame_rows.py
```

Visualize force directions colored by fit quality:

```bash
python3 experiments/boom_compliance/visualize/visualize_compliance_fit.py
```

## Visualization Details

`visualize_reference_frame.py` shows:

- gray undeformed boom shell
- dashed RGB undeformed frame at the circle center
- solid RGB measured deformed frame
- thicker 50%-opacity RGB predicted frame from `C @ W`
- black dot for circle center
- black `x` for shear center
- magenta applied force arrow, starting at the deformed circle center

The boom shell is drawn as:

```text
40 mm diameter
270 deg subtended angle
1 mm visual wall thickness
opening in +Y
extending from X = 0 to negative X
```

The exported row images are in:

```text
visualize/data_1p5m_5N/row1.png ... row50.png
```

Plot bounds are fixed so the boom stays in the same place:

```text
X:  0.05 to -0.15
Y: -0.10 to  0.10
Z: -0.10 to  0.10
```

In the interactive reference-frame visualizer, press `v` after rotating the plot
to print the current Matplotlib camera:

```python
ax.view_init(elev=..., azim=..., roll=..., vertical_axis="y")
```

## Debugging Findings

- The first apparent "compression force along the boom" came from using the
  wrong raw-axis mapping. The corrected mapping is:

```text
beam +X = raw +Z
beam +Y = raw -Y
beam +Z = raw +X
```

- With the corrected mapping, row 1 had positive translational and rotational
  work checks:

```text
force dot displacement > 0
moment dot theta > 0
```

- The raw-pose visualizer was useful because it avoids all shear-center and
  beam-frame transforms. Use it first if pose conventions look suspicious.

- Do not flip rotation signs based only on an energy check. First validate the
  raw `id34_t1` and `id34_t2` frames visually. The current convention uses
  `R_t1.T @ R_t2`, expressed in the undeformed endpoint frame.

## Current File Naming

Use the `data_1p5m_5N` stem consistently:

```text
data_1p5m_5N.csv
data_1p5m_5N_shear_center.csv
data_1p5m_5N_compliance.csv
visualize/data_1p5m_5N/
```

