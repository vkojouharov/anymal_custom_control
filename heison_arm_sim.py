"""
2D Robotic Arm Simulator
========================
Robot configuration (from whiteboard):
  - Joint 1: revolute (θ₁) at base (0,0)
  - Joint 2: prismatic (d₂) — extensible rod
  - Joint 3: revolute (θ₃) at end of prismatic link
  - End-effector at distance L from joint 3
Generalized coordinates: q = [θ₁, d₂, θ₃]
Forward kinematics:
  x = d₂·cos(θ₁) + L·cos(θ₁ + θ₃)
  y = d₂·sin(θ₁) + L·sin(θ₁ + θ₃)
Control:
  WASD → Vref (end-effector velocity)
  q̇ = J⁺ · Vref  (pseudoinverse resolved-rate control)
"""

import numpy as np
import pygame
import sys

# ── Robot Parameters ──────────────────────────────────────────────
L = 0.25  # length of the last link (from joint 3 to end-effector)

# Joint limits
D2_MIN, D2_MAX = 0.3, 2.5
def wrap_angle(a):
    """Normalize angle to [-π, π]."""
    return np.arctan2(np.sin(a), np.cos(a))

# Initial configuration
theta1_0 = np.deg2rad(45)
d2_0 = 1.2
theta3_0 = np.deg2rad(-30)

# Control
V_SPEED = 1.5  # end-effector speed (m/s in sim units)
ROTATE_SPEED = 1.2  # θ₁ rotation speed for Q/E (rad/s)
DT = 1 / 60.0

# Trajectory tracking
TRAJ_KP = 5.0            # proportional feedback gain
WAYPOINT_SPEED = 1.2     # EE travel speed toward waypoints (m/s)
WAYPOINT_ARRIVE_R = 0.03 # arrival threshold (m)

# ── Display ───────────────────────────────────────────────────────
WIDTH, HEIGHT = 1000, 750
SCALE = 120  # pixels per sim-unit
ORIGIN = np.array([WIDTH // 2, HEIGHT // 2 + 100])

BG_COLOR = (15, 15, 25)
GRID_COLOR = (35, 35, 50)
LINK_COLOR_1 = (80, 200, 255)
LINK_COLOR_2 = (255, 160, 60)
JOINT_COLOR = (255, 255, 255)
EE_COLOR = (255, 80, 80)
TARGET_COLOR = (80, 255, 120)
TRAIL_COLOR = (255, 80, 80, 80)
TEXT_COLOR = (200, 200, 220)
ACCENT_COLOR = (120, 180, 255)


def forward_kinematics(theta1, d2, theta3):
    """Compute joint positions and end-effector position."""
    base = np.array([0.0, 0.0])
    joint2 = base + d2 * np.array([np.cos(theta1), np.sin(theta1)])
    ee = joint2 + L * np.array([np.cos(theta1 + theta3), np.sin(theta1 + theta3)])
    return base, joint2, ee


def jacobian(theta1, d2, theta3):
    """
    J = ∂x/∂q,  q = [θ₁, d₂, θ₃],  x = [x_ee, y_ee]
    Returns 2×3 matrix.
    """
    s1 = np.sin(theta1)
    c1 = np.cos(theta1)
    s13 = np.sin(theta1 + theta3)
    c13 = np.cos(theta1 + theta3)

    J = np.array([
        [-d2 * s1 - L * s13, c1, -L * s13],
        [ d2 * c1 + L * c13, s1,  L * c13],
        [1, 0, 1]
    ])
    return J


def rotate_joint2_around_ee(theta1, d2, theta3, delta_angle):
    """Rotate joint2 around EE by delta_angle (rad). EE stays fixed.
    joint2 sits on a circle of radius L centered at EE.
    We rotate joint2's position on that circle, then recover θ₁, d₂, θ₃.
    Returns (theta1_new, d2_new, theta3_new) or None if infeasible.
    """
    _, joint2, ee = forward_kinematics(theta1, d2, theta3)

    # Current angle of joint2 relative to EE
    beta = np.arctan2(joint2[1] - ee[1], joint2[0] - ee[0])

    # Rotate
    beta_new = beta + delta_angle

    # New joint2 position (still on circle of radius L around EE)
    j2_new = ee + L * np.array([np.cos(beta_new), np.sin(beta_new)])

    # Recover θ₁: direction from base(0,0) to joint2
    theta1_new = np.arctan2(j2_new[1], j2_new[0])

    # Recover d₂: distance from base to joint2
    d2_new = np.linalg.norm(j2_new)

    # Recover θ₃: angle of link2 (from joint2 to EE) minus θ₁
    link2_angle = np.arctan2(ee[1] - j2_new[1], ee[0] - j2_new[0])
    theta3_new = link2_angle - theta1_new
    theta3_new = np.arctan2(np.sin(theta3_new), np.cos(theta3_new))

    # Check d₂ limit
    if not (D2_MIN <= d2_new <= D2_MAX):
        return None

    return wrap_angle(theta1_new), d2_new, wrap_angle(theta3_new)


def screen_to_world(sp):
    """Convert screen coords to simulation coords."""
    x = (sp[0] - ORIGIN[0]) / SCALE
    y = (ORIGIN[1] - sp[1]) / SCALE
    return np.array([x, y])


def draw_waypoints(surface, font, waypoints, current_idx):
    """Draw all waypoints, path lines, and labels."""
    if not waypoints:
        return

    # Draw path lines between consecutive waypoints
    for i in range(len(waypoints) - 1):
        s1 = world_to_screen(waypoints[i])
        s2 = world_to_screen(waypoints[i + 1])
        color = (40, 100, 50) if i < current_idx else (60, 180, 100)
        pygame.draw.line(surface, color, s1, s2, 2)

    # Draw each waypoint
    for i, wp in enumerate(waypoints):
        sp = world_to_screen(wp)
        if i < current_idx:
            # Already visited
            pygame.draw.circle(surface, (60, 120, 70), sp, 6, 2)
        elif i == current_idx:
            # Current target — pulsing
            pulse = int(3 * abs(np.sin(pygame.time.get_ticks() * 0.005)))
            pygame.draw.circle(surface, (80, 255, 120), sp, 10 + pulse, 2)
            pygame.draw.circle(surface, (80, 255, 120), sp, 4)
        else:
            # Future waypoint
            pygame.draw.circle(surface, (80, 255, 120), sp, 6)

        # Number label
        txt = font.render(str(i + 1), True, (80, 255, 120) if i >= current_idx else (60, 120, 70))
        surface.blit(txt, (sp[0] + 10, sp[1] - 14))


def world_to_screen(p):
    """Convert simulation coords (x right, y up) to screen coords (y down)."""
    return (int(ORIGIN[0] + p[0] * SCALE),
            int(ORIGIN[1] - p[1] * SCALE))


def draw_grid(surface):
    for i in range(-10, 11):
        px = ORIGIN[0] + i * SCALE
        pygame.draw.line(surface, GRID_COLOR, (px, 0), (px, HEIGHT))
        py = ORIGIN[1] - i * SCALE
        pygame.draw.line(surface, GRID_COLOR, (0, py), (WIDTH, py))


def draw_dashed_line(surface, color, start, end, dash_len=8, gap_len=5, width=1):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dist = max((dx**2 + dy**2) ** 0.5, 1e-6)
    ux, uy = dx / dist, dy / dist
    drawn = 0
    while drawn < dist:
        seg_start = (start[0] + ux * drawn, start[1] + uy * drawn)
        seg_end_d = min(drawn + dash_len, dist)
        seg_end = (start[0] + ux * seg_end_d, start[1] + uy * seg_end_d)
        pygame.draw.line(surface, color, seg_start, seg_end, width)
        drawn += dash_len + gap_len


def draw_arc(surface, color, center, radius, start_angle, end_angle, width=2):
    """Draw an arc from start_angle to end_angle (in radians, screen coords where y is flipped)."""
    if abs(end_angle - start_angle) < 1e-4:
        return
    n_segments = max(int(abs(end_angle - start_angle) / 0.05), 8)
    points = []
    for i in range(n_segments + 1):
        t = start_angle + (end_angle - start_angle) * i / n_segments
        px = center[0] + radius * np.cos(t)
        py = center[1] - radius * np.sin(t)  # screen y is flipped
        points.append((px, py))
    if len(points) >= 2:
        pygame.draw.lines(surface, color, False, points, width)


def draw_annotations(surface, font, theta1, d2, theta3, base, joint2, ee):
    """Draw reference lines, angle arcs, and labels for θ₁, d₂, θ₃."""
    sb = world_to_screen(base)
    sj = world_to_screen(joint2)
    se = world_to_screen(ee)

    ARC_COLOR_1 = (120, 220, 255, 180)
    ARC_COLOR_3 = (255, 200, 80, 180)
    REF_COLOR = (80, 80, 110)
    LABEL_COLOR_1 = (120, 220, 255)
    LABEL_COLOR_2 = (80, 200, 255)
    LABEL_COLOR_3 = (255, 200, 80)
    D2_LABEL_COLOR = (160, 230, 255)

    # ── θ₁: angle from +x axis to link 1 at base ──
    ref_len = 55
    ref_end = (sb[0] + ref_len, sb[1])
    draw_dashed_line(surface, REF_COLOR, sb, ref_end, dash_len=5, gap_len=4, width=1)

    arc_r1 = 40
    draw_arc(surface, LABEL_COLOR_1, sb, arc_r1, 0, theta1, width=2)

    # Arrow tip on the arc
    tip_angle = theta1
    tip_x = sb[0] + arc_r1 * np.cos(tip_angle)
    tip_y = sb[1] - arc_r1 * np.sin(tip_angle)
    pygame.draw.circle(surface, LABEL_COLOR_1, (int(tip_x), int(tip_y)), 3)

    # θ₁ label
    label_angle = theta1 / 2
    lx = sb[0] + (arc_r1 + 16) * np.cos(label_angle)
    ly = sb[1] - (arc_r1 + 16) * np.sin(label_angle)
    txt = font.render("θ₁", True, LABEL_COLOR_1)
    surface.blit(txt, (int(lx) - txt.get_width() // 2, int(ly) - txt.get_height() // 2))

    # ── d₂: dimension line along link 1 ──
    if d2 > 0.4:
        dx = sj[0] - sb[0]
        dy = sj[1] - sb[1]
        link_len = max((dx**2 + dy**2) ** 0.5, 1e-6)
        nx, ny = -dy / link_len, dx / link_len  # perpendicular (screen coords)
        offset = 18

        p1 = (sb[0] + nx * offset, sb[1] + ny * offset)
        p2 = (sj[0] + nx * offset, sj[1] + ny * offset)

        # Tick marks at ends
        tick = 8
        pygame.draw.line(surface, D2_LABEL_COLOR,
                         (p1[0] - nx * tick, p1[1] - ny * tick),
                         (p1[0] + nx * tick, p1[1] + ny * tick), 1)
        pygame.draw.line(surface, D2_LABEL_COLOR,
                         (p2[0] - nx * tick, p2[1] - ny * tick),
                         (p2[0] + nx * tick, p2[1] + ny * tick), 1)

        # Connecting line
        pygame.draw.line(surface, D2_LABEL_COLOR, p1, p2, 1)

        # d₂ label at midpoint
        mx = (p1[0] + p2[0]) / 2 + nx * 12
        my = (p1[1] + p2[1]) / 2 + ny * 12
        txt = font.render("d₂", True, D2_LABEL_COLOR)
        surface.blit(txt, (int(mx) - txt.get_width() // 2, int(my) - txt.get_height() // 2))

    # ── θ₃: angle from link1-extension to link2 at joint2 ──
    ref_len2 = 45
    link1_dir_x = np.cos(theta1)
    link1_dir_y = np.sin(theta1)
    ref_end2 = (sj[0] + ref_len2 * link1_dir_x,
                sj[1] - ref_len2 * link1_dir_y)  # screen y flipped
    draw_dashed_line(surface, REF_COLOR,
                     sj, (int(ref_end2[0]), int(ref_end2[1])),
                     dash_len=5, gap_len=4, width=1)

    arc_r3 = 32
    draw_arc(surface, LABEL_COLOR_3, sj, arc_r3, theta1, theta1 + theta3, width=2)

    # Arrow tip on the arc
    tip_angle3 = theta1 + theta3
    tip_x3 = sj[0] + arc_r3 * np.cos(tip_angle3)
    tip_y3 = sj[1] - arc_r3 * np.sin(tip_angle3)
    pygame.draw.circle(surface, LABEL_COLOR_3, (int(tip_x3), int(tip_y3)), 3)

    # θ₃ label
    label_angle3 = theta1 + theta3 / 2
    lx3 = sj[0] + (arc_r3 + 16) * np.cos(label_angle3)
    ly3 = sj[1] - (arc_r3 + 16) * np.sin(label_angle3)
    txt = font.render("θ₃", True, LABEL_COLOR_3)
    surface.blit(txt, (int(lx3) - txt.get_width() // 2, int(ly3) - txt.get_height() // 2))

    # ── L label on link 2 ──
    if L > 0.3:
        dx2 = se[0] - sj[0]
        dy2 = se[1] - sj[1]
        l2_len = max((dx2**2 + dy2**2) ** 0.5, 1e-6)
        nx2, ny2 = -dy2 / l2_len, dx2 / l2_len
        mx2 = (sj[0] + se[0]) / 2 + nx2 * 14
        my2 = (sj[1] + se[1]) / 2 + ny2 * 14
        txt = font.render("L", True, LINK_COLOR_2)
        surface.blit(txt, (int(mx2) - txt.get_width() // 2, int(my2) - txt.get_height() // 2))

    # ── End-effector label ──
    txt = font.render("x(EE)", True, EE_COLOR)
    surface.blit(txt, (se[0] + 12, se[1] - 20))


def draw_arm(surface, base, joint2, ee):
    sb = world_to_screen(base)
    sj = world_to_screen(joint2)
    se = world_to_screen(ee)

    # Link 1 (prismatic — dashed to indicate extensible)
    draw_dashed_line(surface, LINK_COLOR_1, sb, sj, dash_len=10, gap_len=6, width=5)

    # Link 2 (rigid)
    pygame.draw.line(surface, LINK_COLOR_2, sj, se, 5)

    # Joints
    pygame.draw.circle(surface, JOINT_COLOR, sb, 10)
    pygame.draw.circle(surface, (40, 40, 60), sb, 6)
    pygame.draw.circle(surface, JOINT_COLOR, sj, 8)
    pygame.draw.circle(surface, (40, 40, 60), sj, 5)

    # End-effector
    pygame.draw.circle(surface, EE_COLOR, se, 7)


def draw_hud(surface, font, font_sm, theta1, d2, theta3, ee, vref, trail,
             tracking_active=False, tracking_error=0.0, waypoints=None, wp_index=0):
    y = 15
    lines = [
        f"θ₁ = {np.rad2deg(theta1):+7.1f}°",
        f"d₂ = {d2:6.3f}",
        f"θ₃ = {np.rad2deg(theta3):+7.1f}°",
        f"EE = ({ee[0]:+.2f}, {ee[1]:+.2f})",
        f"Vref = ({vref[0]:+.1f}, {vref[1]:+.1f})",
    ]
    if tracking_active:
        lines.append(f"error = {tracking_error:.4f}")
        lines.append(f"waypoint {wp_index + 1}/{len(waypoints or [])}")
    elif waypoints:
        lines.append(f"waypoints: {len(waypoints)}")

    # Panel background
    pygame.draw.rect(surface, (20, 20, 35, 200), (10, 8, 230, len(lines) * 26 + 12), border_radius=8)
    for line in lines:
        color = TEXT_COLOR
        if "error" in line:
            color = (255, 255, 80) if tracking_error > 0.05 else (80, 255, 120)
        txt = font.render(line, True, color)
        surface.blit(txt, (20, y))
        y += 26

    # Mode indicator
    if tracking_active:
        mode_txt = font.render("TRACKING", True, (80, 255, 120))
        bw = mode_txt.get_width() + 20
        bx = WIDTH - bw - 15
        pygame.draw.rect(surface, (20, 50, 30), (bx, 12, bw, 30), border_radius=8)
        pygame.draw.rect(surface, (80, 255, 120), (bx, 12, bw, 30), width=2, border_radius=8)
        surface.blit(mode_txt, (bx + 10, 16))
    elif waypoints:
        mode_txt = font.render("PLACING", True, (255, 200, 80))
        bw = mode_txt.get_width() + 20
        bx = WIDTH - bw - 15
        pygame.draw.rect(surface, (40, 35, 15), (bx, 12, bw, 30), border_radius=8)
        pygame.draw.rect(surface, (255, 200, 80), (bx, 12, bw, 30), width=2, border_radius=8)
        surface.blit(mode_txt, (bx + 10, 16))
    else:
        mode_txt = font.render("MANUAL", True, (180, 180, 200))
        bw = mode_txt.get_width() + 20
        bx = WIDTH - bw - 15
        pygame.draw.rect(surface, (20, 20, 35), (bx, 12, bw, 30), border_radius=8)
        pygame.draw.rect(surface, (100, 100, 120), (bx, 12, bw, 30), width=1, border_radius=8)
        surface.blit(mode_txt, (bx + 10, 16))

    # Controls hint
    hint_lines = [
        "WASD — Up, Left, Down, Right",
        "Q/E — Clockwise, Anti-clockwise rotation while EE unmoved",
        "Cursor click — setting task points in task space",
        "T — start/end trajectory tracking",
        "C — clear all task points",
        "R — reset all",
        "ESC — logout",
    ]
    hy = HEIGHT - 20 - len(hint_lines) * 22
    pygame.draw.rect(surface, (20, 20, 35, 200),
                     (10, hy - 8, 260, len(hint_lines) * 22 + 16), border_radius=8)
    for hl in hint_lines:
        txt = font_sm.render(hl, True, (140, 140, 160))
        surface.blit(txt, (20, hy))
        hy += 22


def draw_trail(surface, trail):
    if len(trail) < 2:
        return
    pts = [world_to_screen(p) for p in trail]
    for i in range(1, len(pts)):
        alpha = int(60 * i / len(pts))
        color = (255, 80, 80, alpha)
        pygame.draw.line(surface, (min(255, 80 + alpha), 40, 40), pts[i - 1], pts[i], 2)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Robotic Arm Simulator — WASD Control")
    clock = pygame.time.Clock()

    try:
        font = pygame.font.SysFont("menlo", 18)
        font_sm = pygame.font.SysFont("menlo", 15)
    except Exception:
        font = pygame.font.Font(None, 22)
        font_sm = pygame.font.Font(None, 18)

    theta1 = theta1_0
    d2 = d2_0
    theta3 = theta3_0

    trail = []
    MAX_TRAIL = 500

    # Waypoint tracking state
    waypoints = []       # list of np.array([x, y]) in world coords
    wp_index = 0         # index of current target waypoint
    tracking_active = False
    tracking_error = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    theta1, d2, theta3 = theta1_0, d2_0, theta3_0
                    trail.clear()
                    waypoints.clear()
                    wp_index = 0
                    tracking_active = False
                    tracking_error = 0.0
                elif event.key == pygame.K_t:
                    if waypoints and not tracking_active:
                        tracking_active = True
                        wp_index = 0
                        tracking_error = 0.0
                        trail.clear()
                    elif tracking_active:
                        tracking_active = False
                elif event.key == pygame.K_c:
                    waypoints.clear()
                    wp_index = 0
                    tracking_active = False
                    tracking_error = 0.0
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if not tracking_active:
                    wp_world = screen_to_world(event.pos)
                    waypoints.append(wp_world)

        keys = pygame.key.get_pressed()

        if tracking_active and wp_index < len(waypoints):
            # ── Waypoint tracking mode ──
            _, _, ee_current = forward_kinematics(theta1, d2, theta3)
            p_target = waypoints[wp_index]

            error_vec = p_target - ee_current
            tracking_error = np.linalg.norm(error_vec)

            # Move toward target at fixed speed, with proportional correction
            if tracking_error > WAYPOINT_ARRIVE_R:
                direction = error_vec / tracking_error
                v_feedforward = direction * WAYPOINT_SPEED
                vref = v_feedforward + TRAJ_KP * error_vec
                vref = np.append(vref, 0.0)
            else:
                vref = TRAJ_KP * error_vec
                vref = np.append(vref, 0.0)

            J = jacobian(theta1, d2, theta3)
            try:
                J_pinv = np.linalg.pinv(J)
                q_dot = J_pinv @ vref
            except np.linalg.LinAlgError:
                q_dot = np.zeros(3)

            theta1 += q_dot[0] * DT
            d2 += q_dot[1] * DT
            theta3 += q_dot[2] * DT

            theta1 = wrap_angle(theta1)
            d2 = np.clip(d2, D2_MIN, D2_MAX)
            theta3 = wrap_angle(theta3)

            # Check arrival
            if tracking_error < WAYPOINT_ARRIVE_R:
                wp_index += 1
                if wp_index >= len(waypoints):
                    tracking_active = False

        elif tracking_active and wp_index >= len(waypoints):
            tracking_active = False
            vref = np.array([0.0, 0.0, 0.0])
        else:
            # ── Manual mode ──
            vx, vy, wz = 0.0, 0.0, 0.0
            if keys[pygame.K_d]:
                vx += V_SPEED
            if keys[pygame.K_a]:
                vx -= V_SPEED
            if keys[pygame.K_w]:
                vy += V_SPEED
            if keys[pygame.K_s]:
                vy -= V_SPEED
            if keys[pygame.K_q]:
                wz = ROTATE_SPEED
            if keys[pygame.K_e]:
                wz = -ROTATE_SPEED
            vref = np.array([vx, vy, wz])

            if np.linalg.norm(vref) > 1e-8:
                J = jacobian(theta1, d2, theta3)
                try:
                    J_pinv = np.linalg.pinv(J)
                    q_dot = J_pinv @ vref
                except np.linalg.LinAlgError:
                    q_dot = np.zeros(3)

                theta1 += q_dot[0] * DT
                d2 += q_dot[1] * DT
                theta3 += q_dot[2] * DT

                theta1 = wrap_angle(theta1)
                d2 = np.clip(d2, D2_MIN, D2_MAX)
                theta3 = wrap_angle(theta3)

        # Forward kinematics
        base, joint2, ee = forward_kinematics(theta1, d2, theta3)

        # Trail
        trail.append(ee.copy())
        if len(trail) > MAX_TRAIL:
            trail.pop(0)

        # ── Draw ──────────────────────────────────────────────
        screen.fill(BG_COLOR)
        draw_grid(screen)

        # Axes labels
        ax_font = font_sm
        screen.blit(ax_font.render("x", True, (80, 80, 100)),
                     (ORIGIN[0] + 5 * SCALE + 5, ORIGIN[1] - 12))
        screen.blit(ax_font.render("y", True, (80, 80, 100)),
                     (ORIGIN[0] + 5, ORIGIN[1] - 4 * SCALE - 20))

        # Axis lines
        pygame.draw.line(screen, (60, 60, 80),
                         (ORIGIN[0] - 5 * SCALE, ORIGIN[1]),
                         (ORIGIN[0] + 5 * SCALE, ORIGIN[1]), 1)
        pygame.draw.line(screen, (60, 60, 80),
                         (ORIGIN[0], ORIGIN[1] + 3 * SCALE),
                         (ORIGIN[0], ORIGIN[1] - 4 * SCALE), 1)

        # Draw waypoints and path
        draw_waypoints(screen, font_sm, waypoints, wp_index)

        # Draw error line to current target
        if tracking_active and wp_index < len(waypoints):
            se = world_to_screen(ee)
            st = world_to_screen(waypoints[wp_index])
            pygame.draw.line(screen, (255, 255, 80), se, st, 1)

        draw_trail(screen, trail)
        draw_arm(screen, base, joint2, ee)
        draw_annotations(screen, font_sm, theta1, d2, theta3, base, joint2, ee)

        # Velocity arrow at end-effector
        if np.linalg.norm(vref) > 1e-8:
            se = world_to_screen(ee)
            arrow_end = (se[0] + int(vref[0] * 30), se[1] - int(vref[1] * 30))
            pygame.draw.line(screen, TARGET_COLOR, se, arrow_end, 3)
            pygame.draw.circle(screen, TARGET_COLOR, arrow_end, 4)

        draw_hud(screen, font, font_sm, theta1, d2, theta3, ee, vref, trail,
                 tracking_active, tracking_error, waypoints, wp_index)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
