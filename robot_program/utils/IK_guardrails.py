# /IK_guardrails.py

# ============================================================
# SOLUTION VALIDATION / GUARDRAILS

# UNITS CONTRACT
# ------------------------------------------------------------
# All linear quantities are in centimeters (cm).
# All angles are in degrees.
# Trigonometric functions use radians internally.

# ============================================================

import math
from typing import List, Optional, Tuple
from utils.FK_calculations import fk_points_3dof_planar_deg

def is_solution_valid(
    solution: Tuple[float, float, float],
    links: Tuple[float, float, float],
    joint_limits_deg: Optional[List[Tuple[float, float]]] = None,
    check_finite: bool = True
) -> bool:
    """
    Validate whether a computed IK solution is legal/feasible.

    PARAMETERS
    ----------
    solution : (θ1, θ2, θ3)
        Joint angles in DEGREES from IK solver
    links : (l1, l2, l3)
        Link lengths (must be positive)
    joint_limits_deg : [(min, max), (min, max), (min, max)], optional
        Joint angle limits in DEGREES. If provided, solution angles
        must fall within these bounds.
    check_finite : bool
        If True, rejects solutions with NaN or Inf values (default: True)

    RETURNS
    -------
    bool
        True if solution passes all checks, False otherwise
    """

    t1, t2, t3 = solution
    l1, l2, l3 = links

    # Check 1: All angles must be finite (no NaN, no Inf)
    if check_finite:
        finite_mask = [math.isfinite(a) for a in solution]
        if not all(finite_mask):
            bad = [(i, a) for i, (a, ok) in enumerate(zip(solution, finite_mask)) if not ok]
            print(f"[IK][invalid] Non-finite angle(s): {bad}  solution={solution}")
            return False

    # Check 2: Link lengths must be positive
    link_ok = [l > 0 for l in links]
    if not all(link_ok):
        bad = [(i, l) for i, (l, ok) in enumerate(zip(links, link_ok)) if not ok]
        print(f"[IK][invalid] Non-positive link length(s): {bad}  links={links}")
        return False

    # Check 3: Joint limits (if specified)
    if joint_limits_deg is not None:
        if len(joint_limits_deg) != 3:
            print(f"[IK][invalid] joint_limits_deg must have length 3, got {len(joint_limits_deg)}: {joint_limits_deg}")
            return False

        for i, (angle, (lo, hi)) in enumerate(zip(solution, joint_limits_deg)):
            if angle < lo or angle > hi:
                print(
                    f"[IK][invalid] Joint {i} out of limits: angle={angle:.6f} "
                    f"limits=[{lo:.6f}, {hi:.6f}]  solution={solution}"
                )
                return False

    return True



def filter_solutions(
    solutions: List[Tuple[float, float, float]],
    links: Tuple[float, float, float],
    joint_limits_deg: Optional[List[Tuple[float, float]]] = None,
    check_finite: bool = True
) -> List[Tuple[float, float, float]]:
    """
    Filter a list of IK solutions, keeping only valid/legal ones.

    PARAMETERS
    ----------
    solutions : list of (θ1, θ2, θ3)
        Raw solutions from ik_3dof_planar_all_deg
    links : (l1, l2, l3)
        Link lengths
    joint_limits_deg : [(min, max), ...], optional
        Joint limits in DEGREES
    check_finite : bool
        If True, reject solutions with NaN/Inf

    RETURNS
    -------
    List of valid solutions (may be empty if none pass validation)
    """
    return [
        sol for sol in solutions
        if is_solution_valid(sol, links, joint_limits_deg, check_finite)
    ]



