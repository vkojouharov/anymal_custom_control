# Task Pose Optimization

## Definition of Symbols

**Manipulation task w/ *N* locations:** $M=\{m_1,m_2,\ldots,m_N\}$

**Position of each task location:** $m_i=\begin{bmatrix}x_m&y_m&z_m\end{bmatrix}$

**Wrench at each task location:** $w_i=\begin{bmatrix}F_x&F_y&F_z&\tau_x&\tau_y&\tau_z\end{bmatrix}$ ****

**Mobile base position:** $\mathbf{x}_b=\begin{bmatrix}x_b & y_b & \theta_b\end{bmatrix}$

**Global boom yaw angle: $\psi$**

**Global boom pitch angle: $\alpha$**

## Overview

Large workspace manipulation task *M* represented by a discrete set of locations. 

$$
M=\{m_1,m_2,\ldots,m_N\}
$$

At each location, define the manipulation task by a 3D point and a static 6-axis wrench (force/torque), resulting in 9D state. 

$$
m_i= \begin{bmatrix}x_m&y_m&z_m&F_x&F_y&F_z&\tau_x&\tau_y&\tau_z\end{bmatrix}
$$

## Task Metric / Boom Compliance

Define a “task optimality” metric for the quality of a particular pose (mobile base location + boom reaching up to task) as the weighted total deflection of the boom from an endpoint compliance matrix.

Compliance Matrix: 

$$
\delta\mathbf{x} = C\mathbf{w} \qquad C \in \mathbb{R}^{6\times 6}
$$

With load and deflections in 3D as:

$$
\mathbf{w} = \begin{bmatrix}F_x&F_y&F_z&\tau_x&\tau_y&\tau_z\end{bmatrix}^T
$$

$$
\delta\mathbf{x}=\begin{bmatrix}\delta x & \delta y & \delta z & \delta \theta_x & \delta \theta_y & \delta \theta_z\end{bmatrix}^T
$$

Weighted total deflection as a quadratic cost (*V* is a diagonal weight matrix)

$$
D=\delta\mathbf{x}^T V \delta\mathbf{x}
$$

## Compliance Transformation

Consider a reference configuration (expressed in a fixed global frame) where we measure the boom’s compliance $C_{ref}$. Here, the boom is horizontal and the boom axis is aligned with the global (+x) direction, and gravity is along global (-z) and points down into the boom. 

For a general configuration (w/ our shoulder DoFs), the boom orientation is then obtained by first applying a yaw angle rotation $\psi$ about the fixed global (+z) axis, followed by a pitch rotation $\alpha$ upward about the boom’s rotated local (+y) axis.

Define the rotation matrix describing the boom’s orientation as

$$
R = R_z(\psi)R_y(-\alpha)
$$

We can then transform the compliance from the reference boom frame (compliance of unrotated boom) to global frame (general boom pose after rotation) as

$$
T=\begin{bmatrix}R & 0 \\ 0 & R\end{bmatrix}
$$

$$
C_{global}=T C_{ref} T^T
$$

$$
\delta \mathbf{x}_{global} = TC_{ref}T^T \mathbf{w}_{global}
$$

Given $\mathbf{w}_{global}$ from the defined task wrench and known boom pose $(T)$, we can then compute the endpoint deflections $\delta\mathbf{x}_{global}$ and weighted total deflection metric.

## Pose Optimization

Consider a task location $m_i=[x_m, y_m, z_m]$. The mobile base has pose $\mathbf{x}_b=\begin{bmatrix}x_b & y_b & \theta_b\end{bmatrix}$. The height of the mobile base can be represented by a constant $h_b$. Assuming the endpoint gripper/wrist are (approximately) located at the end of the boom (neglect offsets for initial analysis), the length of the boom and global orientation are found as:

$$
L=\sqrt{(x_m-x_b)^2+(y_m-y_b)^2+(z_m-h_b)^2}
$$

$$
\psi=\arctan\left(\dfrac{y_m-y_b}{x_m-x_b}\right)
$$

$$
\alpha=\arctan\left(\frac{z_m-h_b}{\sqrt{(x_m-x_b)^2+(y_m-y_b)^2}}\right)
$$

For a given mobile base position, we find the boom length $L$ and associated compliance $C_{ref}(L)$. We can then find the boom rotation $R(\psi, \alpha)$ and transform the boom compliance to global frame $C_{global}$. Then, applying the known task wrench $w_i$, we can find the total deflection $\delta\mathbf{x}$ and associated weighted metric.

The problem as defined here is NOT convex. Thus, an initial offline approach for pose optimization can rely on a sampling-based grid search. Iterate through many poses (in a grid) and evaluate the weighted total deflection at each. Then, subject to feasibility (workspace obstacles, etc), we can pick the optimal pose!

## [Ex.] Cylindrical Rod

Consider a simple cylindrical tube (isotropic, uniform cross section)

$$
\begin{bmatrix}\Delta x\\\Delta y\\\Delta z\\\theta_x\\\theta_y\\\theta_z\end{bmatrix}=C(L) \begin{bmatrix}F_x\\F_y\\F_z\\M_x\\M_y\\M_z\end{bmatrix}
$$

$$
C(L)=\begin{bmatrix}
\frac{L}{EA} & 0 & 0 & 0 & 0 & 0\\
0 & \frac{L^3}{3EI}+\frac{L}{\kappa GA} & 0 & 0 & 0 & \frac{L^2}{2EI}\\
0 & 0 & \frac{L^3}{3EI}+\frac{L}{\kappa GA} & 0 & -\frac{L^2}{2EI} & 0\\
0 & 0 & 0 & \frac{L}{GJ} & 0 & 0\\
0 & 0 & -\frac{L^2}{2EI} & 0 & \frac{L}{EI} & 0\\
0 & \frac{L^2}{2EI} & 0 & 0 & 0 & \frac{L}{EI}
\end{bmatrix}
$$

As an example, let’s consider a fiberglass tube with outer diameter of 40 mm and wall thickness of 1.5 mm. Some approximate values for parameters are then:

| Property | Value |
| --- | --- |
| Cross section area | A = 181.4 mm² |
| Second moment of area | I = 33,670 mm⁴ |
| Polar moment of area | J = 67,330 mm⁴ |
| Young’s modulus | 10-30 GPa |
| Shear modulus | 3-5 GPa |

Assume no workspace obstacles and take $h_b=0.8 \text{ m}$ (approximate value for payload on ANYmal D). The following plots show several example task locations/wrenches and the associated metric in a 3 m. radius around each task.