# Penalty Model Notes

## Domain

All `penalty_model.py` experiments were defined on the wedge inside the disk

```text
x^2 + y^2 <= R^2,   R = 2
```

with angular bounds

```text
-45 deg <= theta <= 45 deg
```

which is equivalent to

```text
x >= 0,   |y| <= x.
```

The plot shows:

- the radius-2 circle
- dashed black rays at `theta = +/- 45 deg`
- a heatmap only inside the wedge

## Line Distances

Let:

```text
L_45   : y = x
L_-45  : y = -x
```

Then the Euclidean distances to those lines are

```text
d_+(x,y) = |y - x| / sqrt(2)
d_-(x,y) = |y + x| / sqrt(2).
```

## Model 1: Sum of Both Distances

First model:

```text
C_1(x,y) = -(d_+(x,y) + d_-(x,y)).
```

Inside the wedge:

- `|y - x| = x - y`
- `|y + x| = x + y`

so this simplifies to

```text
C_1(x,y) = -sqrt(2) x.
```

Therefore, on the wedge, `C_1` is affine, hence both convex and concave.

## Model 2: Upper/Lower Single-Line Distance

Next model:

```text
C_2(x,y) = -d_+(x,y)   for y >= 0
C_2(x,y) = -d_-(x,y)   for y <= 0.
```

Inside the wedge this simplifies to

```text
C_2(x,y) = (-x + |y|) / sqrt(2).
```

Properties:

- convex
- not smooth at `y = 0`

The non-smoothness comes from the `|y|` term.

## Model 3: Smoothed Convex Version

To smooth the kink at `y = 0`, replace `|y|` with

```text
sqrt(y^2 + eps^2).
```

The current implemented model is

```text
C_3(x,y) = (-x + sqrt(y^2 + eps^2)) / sqrt(2)
```

with

```text
eps = 0.05.
```

Properties:

- smooth
- convex
- reduces toward `(-x + |y|) / sqrt(2)` as `eps -> 0`

Convexity follows because:

- `-x` is affine
- `sqrt(y^2 + eps^2)` is convex in `y`
- affine plus convex is convex

The Hessian is diagonal with nonnegative entries, so the function is convex on
the wedge, and in fact on all of `R^2`.

## Current Script State

`penalty_model.py` currently visualizes `C_3`:

```text
(-x + sqrt(y^2 + 0.05^2)) / sqrt(2)
```

inside the wedge `-45 deg <= theta <= 45 deg`, masked outside that region.
