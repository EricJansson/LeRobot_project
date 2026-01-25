import math

# ============================================================
# UNITS CONTRACT

# ------------------------------------------------------------
# All linear quantities are in centimeters (cm).
# All angles are in degrees.
# Trigonometric functions use radians internally.

# ============================================================


def fk_points_3dof_planar_deg(links, angles_deg, shoulder_y):
    l1, l2, l3 = links
    t1, t2, t3 = map(math.radians, angles_deg)

    p0 = (0.0, shoulder_y)  # shoulder pivot in world coords

    p1 = (p0[0] + l1*math.cos(t1),            p0[1] + l1*math.sin(t1))
    p2 = (p1[0] + l2*math.cos(t1+t2),         p1[1] + l2*math.sin(t1+t2))
    p3 = (p2[0] + l3*math.cos(t1+t2+t3),      p2[1] + l3*math.sin(t1+t2+t3))
    return p0, p1, p2, p3

