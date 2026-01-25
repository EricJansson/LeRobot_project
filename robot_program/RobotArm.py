# /RobotArm.py

from dataclasses import dataclass
from typing import Optional, Tuple, List
import math
from utils.IK_calculations import find_base_yaw_candidates, ik_3dof_planar_all_deg, select_ik_solution_deg


@dataclass
class RobotState:
    """All angles are in degrees."""
    base_yaw_deg: float = 0.0
    theta1_deg: float = 1.0
    theta2_deg: float = 160.0
    theta3_deg: float = 40.0

    @property
    def planar_angles(self) -> Tuple[float, float, float]:
        return (self.theta1_deg, self.theta2_deg, self.theta3_deg)

@dataclass(frozen=True)
class ArmModel:
    """
    Immutable robot geometry/config.

    UNITS CONTRACT
    - All linear quantities are centimeters (cm).
    """
    links: Tuple[float, float, float]   # (l1, l2, l3)
    shoulder_z: float                   # cm (height above table)
    base_offset: float                  # cm (XY offset from yaw axis)


class RobotArm:
    """
    Robot model + current state.

    UNITS CONTRACT
    - All linear quantities are centimeters (cm).
    - All angles are degrees.
    """

    def __init__(
        self,
        model: ArmModel,
        joint_limits_deg: Optional[List[Tuple[float, float]]] = None
    ):
        self.model = model
        self.joint_limits_deg = joint_limits_deg

        # Current commanded/estimated state (degrees)
        self.state = RobotState()


    def solve_base_plus_planar_ik(
        self,
        target_xyz: Tuple[float, float, float],  # (x, y, z) in cm
        phi_deg: float,
        yaw_step_deg: float = 5.0,
        lateral_tol: float = 0.5
    ) -> Optional[RobotState]:
        """
        Compute a new IK solution (does NOT update self.state).
        Returns RobotState or None.
        """

        tx, ty, tz = target_xyz
        candidates: List[Tuple[float, float, float, float]] = []

        # 1) get yaw candidates (deg)
        yaw_candidates = find_base_yaw_candidates(
            base_offset=self.model.base_offset,
            target_xy=(tx, ty),
            yaw_step_deg=yaw_step_deg,
            lateral_tol=lateral_tol,
        )
        if not yaw_candidates:
            return None

        # continuity hint from current state
        prev_planar = self.state.planar_angles

        for yaw_deg in yaw_candidates:
            yaw = math.radians(yaw_deg)

            # Shoulder position in world frame (cm)
            sx = self.model.base_offset * math.cos(yaw)
            sy = self.model.base_offset * math.sin(yaw)
            sz = self.model.shoulder_z

            # Vector from shoulder to target
            vx = tx - sx
            vy = ty - sy
            vz = tz - sz

            # Arm plane axes (in world XY)
            ex = math.cos(yaw)
            ey = math.sin(yaw)

            # Project into planar frame (cm)
            x_planar = vx * ex + vy * ey
            y_planar = vz

            planar_solutions = ik_3dof_planar_all_deg(
                self.model.links,
                x_planar,
                y_planar,
                phi_deg,
            )
            if not planar_solutions:
                continue

            # Optional: filter by joint limits (if you want selection to respect them)
            if self.joint_limits_deg is not None:
                planar_solutions = [
                    s for s in planar_solutions
                    if all(lo <= a <= hi for a, (lo, hi) in zip(s, self.joint_limits_deg))
                ]
                if not planar_solutions:
                    continue

            chosen = select_ik_solution_deg(
                planar_solutions,
                prev_angles_deg=prev_planar,
                joint_limits_deg=None,   # already filtered above
            )
            if chosen is None:
                continue

            candidates.append((yaw_deg, *chosen))

        if not candidates:
            return None

        # Choose best candidate: smallest yaw change from current state
        prev_yaw = self.state.base_yaw_deg

        def yaw_dist(a, b):
            return abs(((a - b + 180) % 360) - 180)

        candidates.sort(key=lambda c: yaw_dist(c[0], prev_yaw))

        yaw_deg, t1, t2, t3 = candidates[0]
        return RobotState(yaw_deg, t1, t2, t3)

    def move_end_effector(
        self,
        target_xyz: Tuple[float, float, float],
        phi_deg: float,
        yaw_step_deg: float = 5.0,
        lateral_tol: float = 0.5
    ) -> bool:
        """
        High-level command: solve IK and update internal state if solvable.
        Returns True on success, False if unreachable.
        """
        sol = self.solve_base_plus_planar_ik(
            target_xyz=target_xyz,
            phi_deg=phi_deg,
            yaw_step_deg=yaw_step_deg,
            lateral_tol=lateral_tol,
        )
        if sol is None:
            return False

        # Update state (this is where you'd also send commands to motors)
        self.state = sol
        return True


if __name__ == "__main__":
    model = ArmModel(
        links=(12.0, 10.0, 6.0),  # cm
        shoulder_z=10.0,          # cm
        base_offset=4.0           # cm
    )

    arm = RobotArm(
        model=model,
        joint_limits_deg=[(-180, 180), (-120, 120), (-120, 120)]
    )

    ok = arm.move_end_effector(
        target_xyz=(18.0, 5.0, 12.0),
        phi_deg=0.0,
    )

    print("Move OK?", ok)
    print("New state:", arm.state)
