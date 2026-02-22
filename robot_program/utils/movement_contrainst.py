# /movement_contrainst.py

# ============================================================
# ARM CONSTRAINTS
# ============================================================
    
from .FK_calculations import fk_points_3dof_planar_deg


def check_workspace_limits(
    solution,
    links,
    x_min=None,
    x_max=None,
    y_min=None,
    y_max=None,
):
    """
    Reject solutions whose joints leave a rectangular workspace.
    """
    _, p1, p2, p3 = fk_points_3dof_planar_deg(links, solution)

    points = [p1, p2, p3]

    for x, y in points:
        if x_min is not None and x < x_min:
            return False
        if x_max is not None and x > x_max:
            return False
        if y_min is not None and y < y_min:
            return False
        if y_max is not None and y > y_max:
            return False

    return True


def check_not_behind_base(solution, links):
    _, p1, p2, p3 = fk_points_3dof_planar_deg(links, solution)

    for x, _ in (p1, p2, p3):
        if x < 0:
            return False
    return True


def check_elbow_above_table(solution, links, shoulder_y, table_y=0.0):
    _, _, p2, _ = fk_points_3dof_planar_deg(links, solution, shoulder_y)
    return p2[1] >= table_y
