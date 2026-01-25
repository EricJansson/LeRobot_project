# /IK_calculations.py

import math
from typing import List, Tuple, Optional
import utils.FK_calculations as forward_kinematics

# ============================================================
# 3-DOF PLANAR IK (ALL SOLUTIONS)

# UNITS CONTRACT
# ------------------------------------------------------------
# All linear quantities are in centimeters (cm).
# All angles are in degrees.
# Trigonometric functions use radians internally.

# ============================================================

def solve_base_plus_planar_ik(
    links,
    shoulder_z,                 # cm
    base_offset,                # cm (4)
    target_xyz,                 # (x, y, z) in cm
    phi_deg,
    prev_solution=None,
    yaw_step_deg=5.0,
    lateral_tol=0.5,
):
    """
    Solve IK for a yawing base + planar arm.

    Returns:
        (base_yaw_deg, θ1_deg, θ2_deg, θ3_deg)
        or None if unreachable
    """

    tx, ty, tz = target_xyz
    candidates = []

    # ------------------------------------------------------------
    # 1. Find candidate base yaw angles
    # ------------------------------------------------------------
    yaw_candidates = find_base_yaw_candidates(
        base_offset=base_offset,
        target_xy=(tx, ty),
        yaw_step_deg=yaw_step_deg,
        lateral_tol=lateral_tol,
    )

    if not yaw_candidates:
        return None

    # ------------------------------------------------------------
    # 2. Try planar IK for each yaw candidate
    # ------------------------------------------------------------
    for yaw_deg in yaw_candidates:
        yaw = math.radians(yaw_deg)

        # Shoulder position in world frame
        sx = base_offset * math.cos(yaw)
        sy = base_offset * math.sin(yaw)
        sz = shoulder_z

        # Vector from shoulder to target
        vx = tx - sx
        vy = ty - sy
        vz = tz - sz

        # Arm plane axes
        ex = math.cos(yaw)
        ey = math.sin(yaw)

        # Project into planar frame
        x_planar = vx * ex + vy * ey
        y_planar = vz

        # --------------------------------------------------------
        # Planar IK (unchanged)
        # --------------------------------------------------------
        planar_solutions = ik_3dof_planar_all_deg(
            links,
            x_planar,
            y_planar,
            phi_deg,
        )

        if not planar_solutions:
            continue

        # Continuity hint
        prev_planar = None
        if prev_solution is not None:
            _, t1, t2, t3 = prev_solution
            prev_planar = (t1, t2, t3)

        chosen = select_ik_solution_deg(
            planar_solutions,
            prev_angles_deg=prev_planar,
        )

        if chosen is None:
            continue

        candidates.append((yaw_deg, *chosen))

    # ------------------------------------------------------------
    # 3. Choose best full solution
    # ------------------------------------------------------------
    if not candidates:
        return None

    if prev_solution is None:
        return candidates[0]

    prev_yaw = prev_solution[0]

    def yaw_dist(a, b):
        return abs(((a - b + 180) % 360) - 180)

    candidates.sort(key=lambda c: yaw_dist(c[0], prev_yaw))
    return candidates[0]


def find_base_yaw_candidates(
    base_offset: float,            # cm
    target_xy: Tuple[float, float],
    yaw_step_deg: float = 5.0,
    lateral_tol: float = 0.5       # cm
) -> List[float]:
    """
    Find base yaw angles (deg) for which the target lies
    approximately in the arm's plane.
    """

    tx, ty = target_xy
    candidates = []

    for yaw_deg in range(-180, 181, int(yaw_step_deg)):
        yaw = math.radians(yaw_deg)

        # Shoulder XY position
        sx = base_offset * math.cos(yaw)
        sy = base_offset * math.sin(yaw)

        vx = tx - sx
        vy = ty - sy

        # Lateral distance to the arm plane
        lateral = (-math.sin(yaw) * vx) + (math.cos(yaw) * vy)

        if abs(lateral) <= lateral_tol:
            candidates.append(float(yaw_deg))

    return candidates



def ik_3dof_planar_all_deg(
    links: Tuple[float, float, float],
    x: float,
    y: float,
    phi_deg: float
) -> List[Tuple[float, float, float]]:
    """
    Analytic inverse kinematics for a 3-DOF planar RRR robot arm.

    PARAMETERS
    ----------
    links : (l1, l2, l3)
        Link lengths (any linear unit, e.g. meters or mm)
    x, y : float
        Desired end-effector position (same unit as links)
    phi_deg : float
        Desired end-effector orientation in DEGREES
        (phi = theta1 + theta2 + theta3)

    RETURNS
    -------
    List of (theta1_deg, theta2_deg, theta3_deg)
        All valid IK solutions in DEGREES.
        Returns [] if no solution exists.
    """

# yaw calculations
# guardrails/restrictions for motor degrees rather than IK calculations


    TOL = 1e-6  # cm-scale numeric tolerance


    l1, l2, l3 = links
    phi = math.radians(phi_deg)  # convert once

    solutions: List[Tuple[float, float, float]] = []

    # ------------------------------------------------------------
    # Step 1: Compute wrist center (remove last link contribution)
    # ------------------------------------------------------------
    wx = x - l3 * math.cos(phi)
    wy = y - l3 * math.sin(phi)


    r2 = wx * wx + wy * wy
    l1pl2 = l1 + l2
    l1ml2 = abs(l1 - l2)

    # ------------------------------------------------------------
    # Step 2: Reachability check for 2-link subproblem (use squared distances)
    # ------------------------------------------------------------
    if r2 > (l1pl2 + TOL) ** 2 or r2 < (l1ml2 - TOL) ** 2:
        return []

    # ------------------------------------------------------------
    # Step 3: Solve elbow angle (two solutions)
    # ------------------------------------------------------------
    cos_t2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)

    if abs(cos_t2) > 1.0 + TOL:
        return []

    cos_t2 = max(-1.0, min(1.0, cos_t2))

    # ------------------------------------------------------------
    # Step 4: Solve shoulder and wrist for both elbow configs
    # ------------------------------------------------------------
    for sign in (+1.0, -1.0):  # elbow-up / elbow-down
        t2 = sign * math.acos(cos_t2)

        k1 = l1 + l2 * math.cos(t2)
        k2 = l2 * math.sin(t2)

        t1 = math.atan2(wy, wx) - math.atan2(k2, k1)
        t3 = phi - t1 - t2

        solutions.append((
            math.degrees(t1),
            math.degrees(t2),
            math.degrees(t3),
        ))

    return solutions


# ============================================================
# IK SOLUTION SELECTION (POLICY / CONTINUITY)
# ============================================================

def _wrap_deg(a: float) -> float:
    """Wrap angle to [-180, 180) degrees."""
    return (a + 180.0) % 360.0 - 180.0


def _angle_dist_deg(a: float, b: float) -> float:
    """Shortest angular distance in degrees."""
    return abs(_wrap_deg(a - b))


def select_ik_solution_deg(
    solutions: List[Tuple[float, float, float]],
    prev_angles_deg: Optional[Tuple[float, float, float]] = None,
    joint_limits_deg: Optional[List[Tuple[float, float]]] = None,
    elbow_preference: Optional[str] = None,  # "up", "down", or None
    hysteresis: float = 0.85
) -> Optional[Tuple[float, float, float]]:
    """
    Selects ONE solution from multiple IK solutions.

    PARAMETERS
    ----------
    solutions : list of (θ1, θ2, θ3)
        Output from ik_3dof_planar_all_deg
    prev_angles_deg : (θ1, θ2, θ3), optional
        Previous robot joint angles (DEGREES).
        Used to avoid sudden jumps.
    joint_limits_deg : [(min, max), ...], optional
        Joint limits in DEGREES.
    elbow_preference : "up" | "down" | None
        Optional elbow bias (based on sign of θ2).
    hysteresis : float
        Prevents elbow flipping unless clearly better.

    RETURNS
    -------
    (θ1, θ2, θ3) in DEGREES, or None if no valid solution remains.
    """

    if not solutions:
        return None

    candidates = solutions[:]

    # ------------------------------------------------------------
    # Step 1: Joint limit filtering
    # ------------------------------------------------------------
    if joint_limits_deg is not None:
        def in_limits(sol):
            return all(lo <= a <= hi for a, (lo, hi) in zip(sol, joint_limits_deg))
        candidates = [s for s in candidates if in_limits(s)]
        if not candidates:
            return None

    # ------------------------------------------------------------
    # Step 2: Elbow preference
    # ------------------------------------------------------------
    if elbow_preference in ("up", "down"):
        sign = 1 if elbow_preference == "up" else -1
        preferred = [s for s in candidates if math.copysign(1, s[1]) == sign]
        if preferred:
            candidates = preferred

    # ------------------------------------------------------------
    # Step 3: Continuity / hysteresis
    # ------------------------------------------------------------
    if prev_angles_deg is not None and len(candidates) > 1:
        def score(sol):
            return sum(
                _angle_dist_deg(a, p)
                for a, p in zip(sol, prev_angles_deg)
            )

        scored = sorted((score(s), s) for s in candidates)
        best_score, best = scored[0]

        if len(scored) > 1:
            second_score, _ = scored[1]
            if best_score < hysteresis * second_score:
                return best

        return best

    # ------------------------------------------------------------
    # Step 4: Deterministic fallback
    # ------------------------------------------------------------
    return candidates[0]
