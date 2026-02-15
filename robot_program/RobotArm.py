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
    
    Default link lengths based on robot specifications:
    - L0: First link (calculated from motor offset geometry)
      ARM_A = 11.3 cm, ARM_B = 3.8 cm (perpendicular)
      L0 = hypot(ARM_A, ARM_B) ≈ 11.92 cm
    - L1: Second link = 13.5 cm
    - L2: Third link (end-effector) = 17.0 cm
    """
    links: Tuple[float, float, float] = (11.92, 13.5, 17.0)   # (l1, l2, l3) in cm
    shoulder_z: float = 11.0                                   # cm (height above table)
    base_offset: float = 4.0                                   # cm (XY offset from yaw axis)


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
        lateral_tol: float = 0.5,
        allow_phi_adaptation: bool = True,
        phi_adaptation_range: float = 45.0,
        phi_adaptation_step: float = 5.0
    ) -> Optional[RobotState]:
        """
        Compute a new IK solution (does NOT update self.state).
        Returns RobotState or None.
        
        Args:
            target_xyz: Target position (x, y, z) in cm
            phi_deg: Desired end-effector orientation in degrees
            lateral_tol: Tolerance for lateral positioning
            allow_phi_adaptation: If True, search for alternative phi if target unreachable
            phi_adaptation_range: Max phi deviation to search (degrees)
            phi_adaptation_step: Step size when searching phi values (degrees)
        """

        tx, ty, tz = target_xyz
        
        # Try with desired phi first
        result = self._solve_ik_for_phi(
            target_xyz, phi_deg, lateral_tol
        )
        if result is not None:
            return result
        
        # If allow_phi_adaptation, try alternative orientations
        if allow_phi_adaptation:
            # Search range: phi_deg ± phi_adaptation_range
            min_phi = phi_deg - phi_adaptation_range
            max_phi = phi_deg + phi_adaptation_range
            
            # Search outward from desired phi (prefer solutions close to target phi)
            search_phis = []
            for offset in range(
                int(phi_adaptation_step),
                int(phi_adaptation_range) + int(phi_adaptation_step),
                int(phi_adaptation_step)
            ):
                search_phis.append(phi_deg + offset)
                search_phis.append(phi_deg - offset)
            
            best_result = None
            best_phi_error = float('inf')
            
            for try_phi in search_phis:
                result = self._solve_ik_for_phi(target_xyz, try_phi, lateral_tol)
                if result is not None:
                    phi_error = abs(try_phi - phi_deg)
                    if phi_error < best_phi_error:
                        best_result = result
                        best_phi_error = phi_error
            
            return best_result
        
        return None

    def _solve_ik_for_phi(
        self,
        target_xyz: Tuple[float, float, float],
        phi_deg: float,
        lateral_tol: float = 0.5
    ) -> Optional[RobotState]:
        """
        Internal helper: solve IK for a specific phi_deg value.
        """
        tx, ty, tz = target_xyz
        candidates: List[Tuple[float, float, float, float]] = []

        # 1) get yaw candidates (deg)
        yaw_candidates = find_base_yaw_candidates(
            base_offset=self.model.base_offset,
            target_xy=(tx, ty),
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
        lateral_tol: float = 0.5,
        allow_phi_adaptation: bool = True,
        phi_adaptation_range: float = 45.0,
        phi_adaptation_step: float = 5.0
    ) -> bool:
        """
        High-level command: solve IK and update internal state if solvable.
        Returns True on success, False if unreachable.
        
        Args:
            target_xyz: Target position (x, y, z) in cm
            phi_deg: Desired end-effector orientation in degrees
            lateral_tol: Tolerance for lateral positioning
            allow_phi_adaptation: If True, search for alternative phi if target unreachable at desired phi
            phi_adaptation_range: Max phi deviation to search (degrees)
            phi_adaptation_step: Step size when searching phi values (degrees)
        """
        sol = self.solve_base_plus_planar_ik(
            target_xyz=target_xyz,
            phi_deg=phi_deg,
            lateral_tol=lateral_tol,
            allow_phi_adaptation=allow_phi_adaptation,
            phi_adaptation_range=phi_adaptation_range,
            phi_adaptation_step=phi_adaptation_step,
        )
        if sol is None:
            return False

        # Update state (this is where you'd also send commands to motors)
        self.state = sol
        return True


if __name__ == "__main__":
    # Use default arm model (or customize as needed)
    model = ArmModel()

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
