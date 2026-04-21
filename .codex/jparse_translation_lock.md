# J-PARSE Wrist Teleop With Translation Lock

This note summarizes the functional control approach now implemented in
`RUN_arm_wrist_JPARSE_teleop.py`.

## Goal

The wrist teleop problem has two related failure modes near singularity:

1. Standard resolved-rate control (`pinv` or DLS / RMRC) can jump to the
   wrong kinematic branch.
2. Even when branch handling improves, a commanded pure end-effector
   rotation can leak into translation because the wrist loses mobility in
   some angular directions near singularity.

The current J-PARSE teleop addresses these as follows:

- Use a J-PARSE-style inverse instead of raw pseudoinverse / DLS for better
  directional singularity handling.
- When the command is effectively "pure rotation", prioritize zero
  translation first and allow angular tracking to degrade if needed.

## High-Level Idea

There are now two control modes:

### 1. Full 6D Mode

Used when the user commands a general task-space twist:

- desired linear velocity `v_des`
- desired angular velocity `omega_des`

The controller computes joint velocity from the full 6x6 Jacobian:

`qdot = J_jparse * [v_des; omega_des]`

This behaves like a singularity-aware resolved-rate controller.

### 2. Rotation-Lock Mode

Used when the command is effectively pure rotation:

- `||v_des||` is near zero
- `||omega_des||` is nonzero

In this case the controller changes the task hierarchy:

1. Enforce "do not translate"
2. Within that subspace, realize as much of the requested rotation as possible

This is what removed the translation drift during pure rotational commands
near wrist singularity.

## J-PARSE Concept

J-PARSE does not only add global damping. It works direction-by-direction.

Given Jacobian `J`, compute its SVD:

`J = U * Sigma * V^T`

Then:

- classify singular values relative to `gamma * sigma_max`
- preserve non-singular directions normally
- clamp very small singular values in a "safety Jacobian"
- attenuate only the collapsing task-space directions

This is better than uniform DLS when only some task directions are becoming
ill-conditioned.

## Translation-Lock Algorithm

For pure rotation commands, split the full Jacobian into:

- `Jv = J[:3, :]`  -> translational Jacobian
- `Jw = J[3:, :]`  -> angular Jacobian

### Step 1. Build the translation nullspace

Compute a damped pseudoinverse of `Jv`:

`Jv^+ = dls(Jv, damping=lambda_v)`

Then form the nullspace projector:

`N_v = I - Jv^+ * Jv`

Interpretation:

- `N_v` keeps only joint motions that do not create tool-point translation
- if `qdot = N_v * z`, then ideally `Jv * qdot = 0`

### Step 2. Restrict rotation control to that subspace

Project the angular Jacobian into the translation-nullspace:

`Jw_locked = Jw * N_v`

This is the angular map available while translation is locked.

### Step 3. Solve reduced angular IK with J-PARSE

Use J-PARSE on `Jw_locked`:

`z = Jparse(Jw_locked) * omega_des`

Then project back:

`qdot = N_v * z`

This yields a joint velocity that:

- approximately preserves zero translation
- still produces as much desired angular motion as the kinematics allow

## Why This Works Better

Near wrist singularity, the robot cannot always satisfy both:

- zero translation at the task point
- arbitrary angular motion

A single least-squares solve on the full Jacobian tends to compromise by
allowing translational leakage.

The translation-lock mode changes the priority:

- translation is treated as the primary constraint
- angular tracking becomes secondary

So when the wrist loses an angular DOF near singularity:

- angular tracking may momentarily degrade
- but the tool point no longer drifts in translation

## Why a Small Wiggle Can Still Happen

A brief wiggle near singularity is expected for several reasons:

- `N_v` is approximate because `Jv^+` uses damping
- joint velocity clipping happens after the solve
- the feasible angular subspace can change quickly near singularity
- any model mismatch shows up more strongly near singular regions

This is usually a better failure mode than translation drift.

## Parameters That Control Behavior

These live in `RUN_arm_wrist_JPARSE_teleop.py`.

### J-PARSE parameters

- `JPARSE_GAMMA`
  - singularity threshold relative to `sigma_max`
  - smaller values: less aggressive singularity classification
  - larger values: more directions treated as singular sooner

- `JPARSE_POS_GAIN`
  - singular-direction gain for translational rows in full 6D mode

- `JPARSE_ANG_GAIN`
  - singular-direction gain for angular rows
  - in practice this matters most in the rotation-lock mode

### Joint motion limits

- `QDOT_LIMITS`
  - final per-joint velocity clipping
  - lower values make the controller calmer but can reduce responsiveness
  - clipping after solving can slightly break the ideal nullspace condition

### Rotation-lock trigger

- `PURE_ROTATION_LINEAR_EPS`
  - how close linear command magnitude must be to zero before translation lock engages

- `PURE_ROTATION_ANGULAR_EPS`
  - minimum angular command magnitude required before treating the command as meaningful pure rotation

### Translation-lock nullspace stability

- `TRANSLATION_LOCK_DAMPING`
  - damping used in the pseudoinverse of `Jv`
  - larger values: more stable / smoother nullspace estimate, less exact translation lock
  - smaller values: tighter nullspace, but more sensitivity near translational ill-conditioning

## Practical Tuning Guidance

If pure rotation still causes visible translation:

- decrease `TRANSLATION_LOCK_DAMPING` slightly
- reduce `QDOT_LIMITS`
- reduce commanded wrist angular speeds

If rotation becomes too sluggish in rotation-lock mode:

- increase `JPARSE_ANG_GAIN`
- reduce `JPARSE_GAMMA` slightly
- increase wrist command scaling carefully

If mode switching feels abrupt:

- add hysteresis or blending between full mode and rotation-lock mode

## Current Limitations

The current implementation is intentionally simple.

It does not yet include:

- explicit branch/posture continuity inside the J-PARSE teleop
- blending/hysteresis at the mode switch
- a QP that enforces translation lock and joint bounds simultaneously
- re-projection after joint clipping

Those are the most likely next steps if the remaining wiggle becomes worth
addressing.

## Short Summary

The current controller is:

- J-PARSE for general singularity-aware inverse kinematics
- plus a translation-nullspace projection for pure rotation commands

The key object is:

`N_v = I - Jv^+ * Jv`

which means:

"Only allow joint motions that do not move the task point linearly."

That is the core reason pure rotation now behaves much better near wrist
singularity.
