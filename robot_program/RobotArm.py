# /RobotArm.py

from dataclasses import dataclass
from typing import Optional, Tuple, List
import math
import json
from pathlib import Path
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


def load_joint_limits_from_calibration(
    calibration_path: Optional[Path] = None
) -> List[Tuple[float, float]]:
    """
    Load joint limits from the calibration JSON file.
    
    MAPPING:
    - Motor 1 (shoulder_pan): base_yaw_deg (NOT included in returned limits)
    - Motor 2 (shoulder_lift): theta1_deg -> limits[0]
    - Motor 3 (elbow_flex): theta2_deg -> limits[1]
    - Motor 4 (wrist_flex): theta3_deg -> limits[2]
    
    Args:
        calibration_path: Path to calibration JSON. 
                         If None, uses default location relative to this file.
    
    Returns:
        List of (min_deg, max_deg) tuples for the first 3 planar joints.
        Format: [(theta1_min, theta1_max), (theta2_min, theta2_max), (theta3_min, theta3_max)]
    """
    if calibration_path is None:
        # Default location relative to this file
        root = Path(__file__).parent.parent  # robot_program -> repo root
        calibration_path = root / "calibration" / "lerobot_arm_with_degrees.json"
    elif not isinstance(calibration_path, Path):
        calibration_path = Path(calibration_path)
    
    if not calibration_path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")
    
    with open(calibration_path, 'r') as f:
        config = json.load(f)
    
    motors = config.get("motors", [])
    
    # Extract limits for motors 2, 3, 4 (indices 1, 2, 3)
    # These map to theta1, theta2, theta3 in the planar arm
    joint_limits = []
    for motor_id in [2, 3, 4]:  # Motor IDs are 1-indexed, we want 2, 3, 4
        motor = next((m for m in motors if m["id"] == motor_id), None)
        if motor is None:
            raise ValueError(f"Motor {motor_id} not found in calibration file")
        
        degree_min = motor["degree_min"]
        degree_max = motor["degree_max"]
        joint_limits.append((degree_min, degree_max))
    
    return joint_limits


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
        joint_limits_deg: Optional[List[Tuple[float, float]]] = None,
        calibration_path: Optional[Path] = None,
        auto_load_limits: bool = True
    ):
        """
        Initialize RobotArm with model and optional joint limits.
        
        Args:
            model: ArmModel geometry specification
            joint_limits_deg: Manual joint limits [(min, max), ...] for theta1, theta2, theta3.
                            If None and auto_load_limits=True, loads from calibration file.
            calibration_path: Path to calibration JSON file. Used if auto_load_limits=True.
            auto_load_limits: If True and joint_limits_deg=None, automatically load limits from calibration.
        """
        self.model = model
        
        # Load or use provided joint limits
        if joint_limits_deg is not None:
            self.joint_limits_deg = joint_limits_deg
        elif auto_load_limits:
            try:
                self.joint_limits_deg = load_joint_limits_from_calibration(calibration_path)
            except (FileNotFoundError, ValueError, KeyError) as e:
                # If auto-load fails, warn but continue without limits
                print(f"Warning: Could not auto-load joint limits: {e}")
                self.joint_limits_deg = None
        else:
            self.joint_limits_deg = None

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
        
        # Try with desired phi first
        result = self._solve_ik_for_phi(
            target_xyz, phi_deg, lateral_tol
        )
        if result is not None:
            return result
        
        # If allow_phi_adaptation, try alternative orientations
        if allow_phi_adaptation:
            # Search outward from desired phi (prefer solutions close to target phi)
            # Range: phi_deg ± phi_adaptation_range
            search_phis = []
            for offset in range(
                int(phi_adaptation_step),
                int(phi_adaptation_range) + 1,
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

            chosen = select_ik_solution_deg(
                planar_solutions,
                prev_angles_deg=prev_planar,
                joint_limits_deg=self.joint_limits_deg,
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

    # Automatically loads joint limits from calibration file
    arm = RobotArm(model=model)
    
    print(f"Joint limits loaded: {arm.joint_limits_deg}")

    ok = arm.move_end_effector(
        target_xyz=(18.0, 5.0, 12.0),
        phi_deg=0.0,
    )

    print("Move OK?", ok)
    print("New state:", arm.state)
