#!/usr/bin/env python3
"""Visualize raw boom compliance data in the undeformed endpoint frame."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D

EXPERIMENT_DIR = Path(__file__).resolve().parents[1]
if str(EXPERIMENT_DIR) not in sys.path:
    sys.path.insert(0, str(EXPERIMENT_DIR))

from convert_raw_data import R_BEAM_FROM_ID34, pose_from_row, quat_to_rotmat, rotmat_to_rotvec, shear_center_y_m


DEFAULT_INPUT_CSV = EXPERIMENT_DIR / "data_2m_5N.csv"
DEFAULT_COMPLIANCE_CSV = EXPERIMENT_DIR / "data_2m_5N_compliance_constrained.csv"
DEFAULT_FORCE_N = 5.0
DEFAULT_DIAMETER_MM = 40.0
DEFAULT_SUBTENDED_ANGLE_DEG = 270.0
DEFAULT_THICKNESS_MM = 1.0
BOOM_DRAW_LENGTH_M = 0.14


def draw_vector(
    ax,
    start: np.ndarray,
    vector: np.ndarray,
    *,
    color: str,
    label: str,
    scale: float = 1.0,
    linestyle: str = "-",
    linewidth: float = 2.5,
    alpha: float = 1.0,
    add_label: bool = True,
) -> None:
    shown = vector * scale
    ax.quiver(
        start[0],
        start[1],
        start[2],
        shown[0],
        shown[1],
        shown[2],
        color=color,
        linewidth=linewidth,
        linestyle=linestyle,
        alpha=alpha,
        arrow_length_ratio=0.12,
        label=label if add_label else "_nolegend_",
    )


def draw_frame(
    ax,
    origin: np.ndarray,
    r_frame_from_beam: np.ndarray,
    *,
    label_prefix: str,
    linestyle: str,
    linewidth: float,
    alpha: float = 1.0,
    add_labels: bool = False,
) -> None:
    axis_len = 0.03
    specs = [
        ("X", np.array([axis_len, 0.0, 0.0]), "red"),
        ("Y", np.array([0.0, axis_len, 0.0]), "green"),
        ("Z", np.array([0.0, 0.0, axis_len]), "blue"),
    ]
    for axis_name, axis_beam, color in specs:
        draw_vector(
            ax,
            origin,
            r_frame_from_beam @ axis_beam,
            color=color,
            label=f"{label_prefix} {axis_name}",
            linestyle=linestyle,
            linewidth=linewidth,
            alpha=alpha,
            add_label=add_labels,
        )


def draw_boom_shell(
    ax,
    *,
    diameter_mm: float,
    thickness_mm: float,
    subtended_angle_deg: float,
    length_m: float,
) -> list[np.ndarray]:
    radius_outer = (diameter_mm + thickness_mm) / 2000.0
    radius_inner = (diameter_mm - thickness_mm) / 2000.0
    opening_angle = np.deg2rad(360.0 - subtended_angle_deg)
    phi = np.linspace(opening_angle / 2.0, 2.0 * np.pi - opening_angle / 2.0, 80)
    x = np.linspace(-length_m, 0.0, 14)
    phi_grid, x_grid = np.meshgrid(phi, x)

    for radius, alpha in ((radius_outer, 0.42), (radius_inner, 0.26)):
        y_grid = radius * np.cos(phi_grid)
        z_grid = radius * np.sin(phi_grid)
        ax.plot_surface(
            x_grid,
            y_grid,
            z_grid,
            color="#8a8a8a",
            alpha=alpha,
            linewidth=0,
            shade=False,
            antialiased=True,
        )

    for phi_edge in (opening_angle / 2.0, 2.0 * np.pi - opening_angle / 2.0):
        y_outer = radius_outer * np.cos(phi_edge)
        z_outer = radius_outer * np.sin(phi_edge)
        y_inner = radius_inner * np.cos(phi_edge)
        z_inner = radius_inner * np.sin(phi_edge)
        ax.plot(
            [-length_m, 0.0],
            [y_outer, y_outer],
            [z_outer, z_outer],
            color="#5f5f5f",
            linewidth=1.2,
            alpha=0.65,
        )
        ax.plot(
            [-length_m, 0.0],
            [y_inner, y_inner],
            [z_inner, z_inner],
            color="#5f5f5f",
            linewidth=1.0,
            alpha=0.55,
        )

    return [
        np.array([-length_m, -radius_outer, -radius_outer]),
        np.array([0.0, radius_outer, radius_outer]),
    ]


def rotvec_to_rotmat(rotvec: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return np.eye(3)
    axis = rotvec / angle
    kx, ky, kz = axis
    k = np.array(
        [
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0],
        ],
        dtype=float,
    )
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def load_compliance(path: Path) -> np.ndarray:
    try:
        matrix = np.loadtxt(path, delimiter=",")
    except ValueError:
        matrix = np.loadtxt(path, delimiter=",", skiprows=1)
    if matrix.shape != (6, 6):
        raise ValueError(f"expected 6x6 compliance matrix, got {matrix.shape} from {path}")
    return matrix


def set_equal_bounds(ax, points: list[np.ndarray]) -> None:
    stacked = np.vstack(points)
    center = 0.5 * (stacked.min(axis=0) + stacked.max(axis=0))
    span = float(np.max(stacked.max(axis=0) - stacked.min(axis=0)))
    radius = max(span * 0.65, 0.08)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def set_fixed_bounds(ax) -> None:
    ax.set_xlim(-0.15, 0.05)
    ax.set_ylim(-0.10, 0.10)
    ax.set_zlim(-0.10, 0.10)
    try:
        ax.set_box_aspect((1, 1, 1))
    except AttributeError:
        pass


def plot_reference_frame_row(
    row: dict[str, str],
    row_number: int,
    *,
    force_n: float = DEFAULT_FORCE_N,
    diameter_mm: float = DEFAULT_DIAMETER_MM,
    subtended_angle_deg: float = DEFAULT_SUBTENDED_ANGLE_DEG,
    thickness_mm: float = DEFAULT_THICKNESS_MM,
    y_sc_mm: float | None = None,
    compliance_matrix: np.ndarray | None = None,
    diagnose_rotation: bool = False,
    print_debug: bool = True,
):
    p34_t1, q34_t1 = pose_from_row(row, "id34_t1")
    p34_t2, q34_t2 = pose_from_row(row, "id34_t2")
    p35_t2, _q35_t2 = pose_from_row(row, "id35_t2")

    r_world_from_ref = quat_to_rotmat(q34_t1)
    r_ref_from_world = r_world_from_ref.T
    r_world_from_deformed = quat_to_rotmat(q34_t2)

    u_circle_center_id34 = r_ref_from_world @ (p34_t2 - p34_t1)
    theta_id34 = rotmat_to_rotvec(r_ref_from_world @ r_world_from_deformed)
    u_circle_center = R_BEAM_FROM_ID34 @ u_circle_center_id34
    theta = R_BEAM_FROM_ID34 @ theta_id34

    if y_sc_mm is None:
        y_sc_m = shear_center_y_m(diameter_mm, subtended_angle_deg)
    else:
        y_sc_m = y_sc_mm / 1000.0
    circle_center = np.zeros(3)
    shear_center = np.array([0.0, y_sc_m, 0.0])
    r_sc_to_circle_center = circle_center - shear_center

    force_direction_world = p35_t2 - p34_t2
    force_direction_world /= np.linalg.norm(force_direction_world)
    force_id34 = r_ref_from_world @ (force_n * force_direction_world)
    force = R_BEAM_FROM_ID34 @ force_id34
    moment_about_shear_center = np.cross(r_sc_to_circle_center, force)
    predicted_circle_center = None
    predicted_rotation = None
    if compliance_matrix is not None:
        wrench = np.concatenate([force, moment_about_shear_center])
        predicted_x = compliance_matrix @ wrench
        predicted_shear_center = predicted_x[:3]
        predicted_theta = predicted_x[3:]
        predicted_circle_center = predicted_shear_center + np.cross(predicted_theta, r_sc_to_circle_center)
        predicted_rotation = rotvec_to_rotmat(predicted_theta)

    r_deformed_id34_from_ref_id34 = r_ref_from_world @ r_world_from_deformed
    r_deformed_beam_from_ref_beam = R_BEAM_FROM_ID34 @ r_deformed_id34_from_ref_id34 @ R_BEAM_FROM_ID34.T
    r_inverse_beam = r_deformed_beam_from_ref_beam.T
    theta_inverse = rotmat_to_rotvec(r_inverse_beam)

    fig = plt.figure(f"Boom Compliance Row {row_number}", figsize=(12.75, 10.5))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_title(f"Row {row_number} in beam frame derived from id34_t1")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    boom_points = draw_boom_shell(
        ax,
        diameter_mm=diameter_mm,
        thickness_mm=thickness_mm,
        subtended_angle_deg=subtended_angle_deg,
        length_m=BOOM_DRAW_LENGTH_M,
    )

    draw_frame(
        ax,
        circle_center,
        np.eye(3),
        label_prefix="t1 dashed",
        linestyle="--",
        linewidth=1.6,
    )
    draw_frame(
        ax,
        u_circle_center,
        r_deformed_beam_from_ref_beam,
        label_prefix="t2 solid",
        linestyle="-",
        linewidth=2.8,
    )
    if predicted_circle_center is not None and predicted_rotation is not None:
        draw_frame(
            ax,
            predicted_circle_center,
            predicted_rotation,
            label_prefix="predicted",
            linestyle="-",
            linewidth=5.6,
            alpha=0.50,
        )
    if diagnose_rotation:
        draw_frame(
            ax,
            u_circle_center,
            r_inverse_beam,
            label_prefix="t2 inverse dotted",
            linestyle=":",
            linewidth=1.8,
            alpha=0.55,
        )

    ax.scatter(*circle_center, color="black", marker="o", s=42, depthshade=False, label="circle center")
    ax.scatter(*shear_center, color="black", marker="x", s=70, linewidths=2.0, label="shear center")

    force_scale = 0.012
    draw_vector(ax, u_circle_center, force, color="#d000ff", label="applied force (5N)", scale=force_scale)

    points = [
        circle_center,
        shear_center,
        u_circle_center + force * force_scale,
        u_circle_center,
        *boom_points,
    ]
    set_fixed_bounds(ax)
    try:
        ax.view_init(elev=8, azim=100, roll=0, vertical_axis="y")
    except TypeError:
        ax.view_init(elev=12, azim=112)

    def print_current_view(event) -> None:
        if event.key != "v":
            return
        roll = getattr(ax, "roll", 0.0)
        print(f'ax.view_init(elev={ax.elev:.1f}, azim={ax.azim:.1f}, roll={roll:.1f}, vertical_axis="y")')

    fig.canvas.mpl_connect("key_press_event", print_current_view)

    legend_handles = [
        Line2D([0], [0], color="0.25", linestyle="--", linewidth=1.8, label="undeformed frame"),
        Line2D([0], [0], color="0.25", linestyle="-", linewidth=2.8, label="deformed frame"),
        Line2D([0], [0], color="black", marker="o", linestyle="None", markersize=7, label="circle center"),
        Line2D([0], [0], color="black", marker="x", linestyle="None", markersize=8, markeredgewidth=2.0, label="shear center"),
        Line2D([0], [0], color="#d000ff", linestyle="-", linewidth=2.8, label="applied force (5N)"),
    ]
    if compliance_matrix is not None:
        legend_handles.insert(
            2,
            Line2D([0], [0], color="0.25", linestyle="-", linewidth=5.6, alpha=0.50, label="predicted frame"),
        )
    ax.legend(handles=legend_handles, loc="upper left", fontsize=10)

    if print_debug:
        print(f"row: {row_number}")
        print(f"u_circle_center [m]: {u_circle_center}")
        print(f"theta [rad]: {theta}")
        print(f"force [N]: {force}")
        print(f"moment about shear center [Nm]: {moment_about_shear_center}")
        print(f"y_sc [mm]: {1000.0 * y_sc_m:+.3f}")
        print(f"force dot u_circle_center [J approx]: {float(force @ u_circle_center):+.6e}")
        print(f"moment dot theta_current [J approx]: {float(moment_about_shear_center @ theta):+.6e}")
        print(f"moment dot theta_inverse [J approx]: {float(moment_about_shear_center @ theta_inverse):+.6e}")
        print(f"theta_inverse [rad]: {theta_inverse}")

    return fig, ax


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT_CSV)
    parser.add_argument("--compliance", type=Path, default=DEFAULT_COMPLIANCE_CSV)
    parser.add_argument("--row", type=int, default=1, help="1-based data row to visualize")
    parser.add_argument("--force-n", type=float, default=DEFAULT_FORCE_N)
    parser.add_argument("--diameter-mm", type=float, default=DEFAULT_DIAMETER_MM)
    parser.add_argument("--subtended-angle-deg", type=float, default=DEFAULT_SUBTENDED_ANGLE_DEG)
    parser.add_argument("--thickness-mm", type=float, default=DEFAULT_THICKNESS_MM)
    parser.add_argument("--y-sc-mm", type=float, default=None)
    parser.add_argument("--no-predicted", action="store_true")
    parser.add_argument(
        "--diagnose-rotation",
        action="store_true",
        help="Overlay the inverse relative frame and print sign checks for rotation convention debugging.",
    )
    args = parser.parse_args()

    with args.input.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if args.row < 1 or args.row > len(rows):
        raise IndexError(f"--row must be between 1 and {len(rows)}")
    compliance_matrix = None if args.no_predicted else load_compliance(args.compliance)

    plot_reference_frame_row(
        rows[args.row - 1],
        args.row,
        force_n=args.force_n,
        diameter_mm=args.diameter_mm,
        subtended_angle_deg=args.subtended_angle_deg,
        thickness_mm=args.thickness_mm,
        y_sc_mm=args.y_sc_mm,
        compliance_matrix=compliance_matrix,
        diagnose_rotation=args.diagnose_rotation,
    )

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
