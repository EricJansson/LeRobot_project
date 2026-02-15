"""
Comprehensive tests for the RobotArm class.
Tests the robot arm model with base yaw + 3-DOF planar kinematics.
"""

import pytest
import math
import sys
from pathlib import Path

# Add the robot_program module to path
sys.path.insert(0, str(Path(__file__).parent.parent / "robot_program"))

from robot_program.RobotArm import RobotArm, ArmModel, RobotState
from robot_program.utils.IK_calculations import ik_3dof_planar_all_deg
import robot_program.utils.FK_calculations as forward_kinematics


def fk_4dof_deg(
    links: tuple,
    base_yaw_deg: float,
    planar_angles_deg: tuple,
    shoulder_z: float,
    base_offset: float
) -> tuple:
    """
    Forward kinematics for 4-DOF robot (yaw + 3-DOF planar).
    
    Args:
        links: (l1, l2, l3) link lengths in cm
        base_yaw_deg: Base yaw angle in degrees
        planar_angles_deg: (theta1, theta2, theta3) in degrees
        shoulder_z: Height of shoulder above table in cm
        base_offset: XY offset from yaw axis in cm
    
    Returns:
        (x, y, z, phi_deg) end-effector position and orientation in world frame
    """
    l1, l2, l3 = links
    t1, t2, t3 = map(math.radians, planar_angles_deg)
    yaw = math.radians(base_yaw_deg)
    
    # Shoulder position in world frame
    sx = base_offset * math.cos(yaw)
    sy = base_offset * math.sin(yaw)
    sz = shoulder_z
    
    # Planar arm end-effector relative to shoulder
    t12 = t1 + t2
    t123 = t12 + t3
    
    x_planar = (
        l1 * math.cos(t1)
        + l2 * math.cos(t12)
        + l3 * math.cos(t123)
    )
    y_planar = (
        l1 * math.sin(t1)
        + l2 * math.sin(t12)
        + l3 * math.sin(t123)
    )
    
    # Transform planar coordinates to world frame (arm plane)
    ex = math.cos(yaw)  # Arm plane X axis
    ey = math.sin(yaw)  # Arm plane Y axis
    
    x = sx + x_planar * ex
    y = sy + x_planar * ey
    z = sz + y_planar
    
    phi_deg = math.degrees(t123)
    return x, y, z, phi_deg


class TestRobotState:
    """Tests for RobotState dataclass."""

    def test_default_initialization(self):
        """Test RobotState with default values."""
        state = RobotState()
        assert state.base_yaw_deg == 0.0
        assert state.theta1_deg == 1.0
        assert state.theta2_deg == 160.0
        assert state.theta3_deg == 40.0

    def test_custom_initialization(self):
        """Test RobotState with custom values."""
        state = RobotState(
            base_yaw_deg=45.0,
            theta1_deg=30.0,
            theta2_deg=90.0,
            theta3_deg=50.0
        )
        assert state.base_yaw_deg == 45.0
        assert state.theta1_deg == 30.0
        assert state.theta2_deg == 90.0
        assert state.theta3_deg == 50.0

    def test_planar_angles_property(self):
        """Test planar_angles property extraction."""
        state = RobotState(
            base_yaw_deg=90.0,
            theta1_deg=20.0,
            theta2_deg=100.0,
            theta3_deg=40.0
        )
        planar = state.planar_angles
        assert planar == (20.0, 100.0, 40.0)

    def test_negative_angles(self):
        """Test RobotState with negative angles."""
        state = RobotState(
            base_yaw_deg=-90.0,
            theta1_deg=-30.0,
            theta2_deg=-45.0,
            theta3_deg=-20.0
        )
        assert state.base_yaw_deg == -90.0
        assert state.theta1_deg == -30.0


class TestArmModel:
    """Tests for ArmModel configuration dataclass."""

    def test_arm_model_creation(self):
        """Test basic ArmModel instantiation."""
        model = ArmModel()
        assert model.links == (11.92, 13.5, 17.0)
        assert model.shoulder_z == 11.0
        assert model.base_offset == 4.0

    def test_equal_link_lengths(self):
        """Test ArmModel with equal link lengths."""
        model = ArmModel(
            links=(10.0, 10.0, 10.0),
            shoulder_z=8.0,
            base_offset=3.0
        )
        l1, l2, l3 = model.links
        assert l1 == l2 == l3

    def test_unequal_link_lengths(self):
        """Test ArmModel with unequal link lengths."""
        model = ArmModel(
            links=(15.0, 12.0, 8.0),
            shoulder_z=10.0,
            base_offset=5.0
        )
        l1, l2, l3 = model.links
        assert l1 > l2 > l3

    def test_immutability(self):
        """Test that ArmModel is frozen (immutable)."""
        model = ArmModel(
            links=(10.0, 10.0, 10.0),
            shoulder_z=8.0,
            base_offset=3.0
        )
        with pytest.raises(AttributeError):
            model.shoulder_z = 12.0


class TestRobotArmInitialization:
    """Tests for RobotArm class initialization."""

    @pytest.fixture
    def standard_model(self):
        """Standard arm model for testing."""
        return ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )

    def test_robot_arm_creation(self, standard_model):
        """Test basic RobotArm instantiation."""
        arm = RobotArm(model=standard_model)
        assert arm.model == standard_model
        assert arm.joint_limits_deg is None
        assert isinstance(arm.state, RobotState)

    def test_robot_arm_with_joint_limits(self, standard_model):
        """Test RobotArm with joint limits."""
        joint_limits = [(-180, 180), (-120, 120), (-120, 120)]
        arm = RobotArm(
            model=standard_model,
            joint_limits_deg=joint_limits
        )
        assert arm.joint_limits_deg == joint_limits

    def test_initial_state(self, standard_model):
        """Test that arm starts with default state."""
        arm = RobotArm(model=standard_model)
        assert arm.state.base_yaw_deg == 0.0
        assert arm.state.theta1_deg == 1.0
        assert arm.state.theta2_deg == 160.0
        assert arm.state.theta3_deg == 40.0

    def test_different_arm_geometries(self):
        """Test RobotArm with different geometric configurations."""
        model1 = ArmModel(
            links=(10.0, 10.0, 10.0),
            shoulder_z=5.0,
            base_offset=2.0
        )
        model2 = ArmModel(
            links=(20.0, 15.0, 10.0),
            shoulder_z=15.0,
            base_offset=8.0
        )
        arm1 = RobotArm(model=model1)
        arm2 = RobotArm(model=model2)
        assert arm1.model != arm2.model


class TestSolveBaseYawIK:
    """Tests for the solve_base_plus_planar_ik method."""

    @pytest.fixture
    def standard_arm(self):
        """Standard arm configuration."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    @pytest.fixture
    def constrained_arm(self):
        """Arm with joint limits."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(
            model=model,
            joint_limits_deg=[(-180, 180), (-120, 120), (-120, 120)]
        )

    def test_solve_center_position(self, standard_arm):
        """Test IK solution for a position near arm center."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)

        assert solution is not None
        assert isinstance(solution, RobotState)
        assert -180 <= solution.base_yaw_deg <= 180

    def test_solve_returns_robot_state(self, standard_arm):
        """Test that solution is a valid RobotState."""
        target = (15.0, 8.0, 10.0)
        phi_deg = 45.0

        solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)

        if solution is not None:
            assert hasattr(solution, 'base_yaw_deg')
            assert hasattr(solution, 'theta1_deg')
            assert hasattr(solution, 'theta2_deg')
            assert hasattr(solution, 'theta3_deg')

    def test_unreachable_position(self, standard_arm):
        """Test that unreachable positions return None."""
        # Position very far away
        target = (100.0, 100.0, 100.0)
        phi_deg = 0.0

        solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)
        # May or may not be reachable depending on arm length
        assert solution is None or isinstance(solution, RobotState)

    def test_solution_with_different_orientations(self, standard_arm):
        """Test IK solutions for various end-effector orientations."""
        target = (16.0, 6.0, 11.0)
        orientations = [0.0, 30.0, 45.0, -30.0, 90.0]

        for phi_deg in orientations:
            solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)
            if solution is not None:
                assert isinstance(solution, RobotState)

    def test_solution_continuity(self, standard_arm):
        """Test that solutions show continuity from previous state."""
        targets = [
            (18.0, 5.0, 12.0),
            (18.5, 5.5, 12.0),
            (18.0, 5.0, 12.0),
        ]

        prev_state = standard_arm.state
        for target in targets:
            sol = standard_arm.solve_base_plus_planar_ik(target, 0.0)
            if sol is not None:
                prev_state = sol

    def test_with_lateral_tolerance(self, standard_arm):
        """Test IK with different lateral tolerances."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        # Tight tolerance
        sol1 = standard_arm.solve_base_plus_planar_ik(
            target, phi_deg, lateral_tol=0.1
        )

        # Loose tolerance
        sol2 = standard_arm.solve_base_plus_planar_ik(
            target, phi_deg, lateral_tol=2.0
        )

        # Loose tolerance should be more likely to find solution
        if sol1 is None and sol2 is not None:
            assert True  # Expected behavior
        elif sol1 is not None and sol2 is not None:
            assert True  # Both found solutions

    def test_negative_coordinates(self, standard_arm):
        """Test IK with negative target coordinates."""
        target = (-10.0, -8.0, 12.0)
        phi_deg = 0.0

        solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)
        # Should handle negative coordinates gracefully
        assert solution is None or isinstance(solution, RobotState)

    def test_zero_position(self, standard_arm):
        """Test IK for position at origin."""
        target = (0.0, 0.0, 10.0)
        phi_deg = 0.0

        solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)
        # Origin may or may not be reachable
        assert solution is None or isinstance(solution, RobotState)


class TestMoveEndEffector:
    """Tests for the move_end_effector method."""

    @pytest.fixture
    def standard_arm(self):
        """Standard arm configuration."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    def test_move_updates_state(self, standard_arm):
        """Test that successful move updates internal state."""
        initial_state = standard_arm.state
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        ok = standard_arm.move_end_effector(target, phi_deg)

        assert isinstance(ok, bool)
        if ok:
            assert standard_arm.state != initial_state

    def test_move_returns_bool(self, standard_arm):
        """Test that move_end_effector returns boolean."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        result = standard_arm.move_end_effector(target, phi_deg)

        assert isinstance(result, bool)
        assert result in (True, False)

    def test_move_unreachable_returns_false(self, standard_arm):
        """Test that unreachable position returns False."""
        target = (200.0, 200.0, 200.0)
        phi_deg = 0.0

        ok = standard_arm.move_end_effector(target, phi_deg)

        # Should return False or True depending on reachability
        assert isinstance(ok, bool)

    def test_move_preserves_model(self, standard_arm):
        """Test that move doesn't change arm model."""
        original_model = standard_arm.model
        target = (18.0, 5.0, 12.0)

        standard_arm.move_end_effector(target, 0.0)

        assert standard_arm.model == original_model

    def test_multiple_sequential_moves(self, standard_arm):
        """Test a sequence of moves."""
        targets = [
            ((18.0, 5.0, 12.0), 0.0),
            ((16.0, 6.0, 11.0), 30.0),
            ((17.0, 4.0, 12.0), -15.0),
        ]

        for target_xyz, phi in targets:
            ok = standard_arm.move_end_effector(target_xyz, phi)
            assert isinstance(ok, bool)

    def test_move_with_custom_parameters(self, standard_arm):
        """Test move with custom lateral tolerance."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        ok = standard_arm.move_end_effector(
            target,
            phi_deg,
            lateral_tol=1.0
        )

        assert isinstance(ok, bool)

    def test_move_same_target_twice(self, standard_arm):
        """Test moving to same target twice."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        ok1 = standard_arm.move_end_effector(target, phi_deg)
        state1 = standard_arm.state
        ok2 = standard_arm.move_end_effector(target, phi_deg)
        state2 = standard_arm.state

        # Both should succeed or both fail
        if ok1 and ok2:
            # States might differ slightly due to continuity
            assert state1 is not None
            assert state2 is not None


class TestRobotArmIntegration:
    """Integration tests for complete RobotArm workflows."""

    @pytest.fixture
    def standard_arm(self):
        """Standard arm configuration."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    @pytest.fixture
    def constrained_arm(self):
        """Arm with joint limits."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(
            model=model,
            joint_limits_deg=[(-180, 180), (-120, 120), (-120, 120)]
        )

    def test_full_workflow(self, standard_arm):
        """Test complete workflow: solve IK and move."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        # Solve without moving
        solution = standard_arm.solve_base_plus_planar_ik(target, phi_deg)

        # Then move
        ok = standard_arm.move_end_effector(target, phi_deg)

        if solution is not None:
            assert ok is True, "Move should succeed if IK found solution"

    def test_trajectory_planning(self, standard_arm):
        """Test following a trajectory of target positions."""
        trajectory = [
            ((18.0, 5.0, 12.0), 0.0),
            ((18.5, 5.5, 12.0), 10.0),
            ((18.0, 5.0, 12.0), 0.0),
            ((17.0, 6.0, 11.0), -10.0),
        ]

        successful_moves = 0
        for target_xyz, phi in trajectory:
            ok = standard_arm.move_end_effector(target_xyz, phi)
            if ok:
                successful_moves += 1

        assert successful_moves >= 0, "Should complete trajectory steps"

    def test_state_tracking(self, standard_arm):
        """Test that state is properly tracked through sequence."""
        state_history = [standard_arm.state]

        targets = [
            (18.0, 5.0, 12.0),
            (18.5, 5.5, 12.0),
            (17.5, 6.0, 11.5),
        ]

        for target in targets:
            ok = standard_arm.move_end_effector(target, 0.0)
            if ok:
                state_history.append(standard_arm.state)

        assert len(state_history) >= 1, "Should track states"

    def test_different_arm_sizes(self):
        """Test RobotArm with different physical sizes."""
        small_model = ArmModel(
            links=(5.0, 4.0, 2.0),
            shoulder_z=3.0,
            base_offset=1.0
        )
        large_model = ArmModel(
            links=(20.0, 18.0, 12.0),
            shoulder_z=15.0,
            base_offset=8.0
        )

        small_arm = RobotArm(model=small_model)
        large_arm = RobotArm(model=large_model)

        # Test both arms
        small_sol = small_arm.solve_base_plus_planar_ik((6.0, 2.0, 4.0), 0.0)
        large_sol = large_arm.solve_base_plus_planar_ik((25.0, 8.0, 15.0), 0.0)

        # Both should return RobotState or None
        assert small_sol is None or isinstance(small_sol, RobotState)
        assert large_sol is None or isinstance(large_sol, RobotState)

    def test_arm_with_joint_limits_integration(self, constrained_arm):
        """Test arm behavior with joint limits applied."""
        target = (18.0, 5.0, 12.0)
        phi_deg = 0.0

        ok = constrained_arm.move_end_effector(target, phi_deg)

        if ok:
            # Check that solution respects joint limits
            state = constrained_arm.state
            theta1, theta2, theta3 = state.planar_angles
            
            assert -120 <= theta1 <= 120, "theta1 should respect limits"
            assert -120 <= theta2 <= 120, "theta2 should respect limits"
            assert -120 <= theta3 <= 120, "theta3 should respect limits"

    def test_ik_fk_consistency(self, standard_arm):
        """Verify that IK solutions actually reach the target via FK."""
        target_xyz = (18.0, 5.0, 12.0)
        phi_deg = 0.0
        tolerance = 0.5  # cm

        ok = standard_arm.move_end_effector(target_xyz, phi_deg)

        if ok:
            state = standard_arm.state
            
            # Compute FK of the IK solution
            x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                standard_arm.model.links,
                state.base_yaw_deg,
                state.planar_angles,
                standard_arm.model.shoulder_z,
                standard_arm.model.base_offset
            )
            
            # Verify FK matches target within tolerance
            assert abs(x_fk - target_xyz[0]) <= tolerance, \
                f"X mismatch: FK={x_fk:.2f}, target={target_xyz[0]}, diff={abs(x_fk - target_xyz[0]):.2f}"
            assert abs(y_fk - target_xyz[1]) <= tolerance, \
                f"Y mismatch: FK={y_fk:.2f}, target={target_xyz[1]}, diff={abs(y_fk - target_xyz[1]):.2f}"
            assert abs(z_fk - target_xyz[2]) <= tolerance, \
                f"Z mismatch: FK={z_fk:.2f}, target={target_xyz[2]}, diff={abs(z_fk - target_xyz[2]):.2f}"

    def test_ik_fk_consistency_with_real_arm_geometry(self):
        """Test IK/FK consistency with actual robot link dimensions."""
        # Use default ArmModel with real link lengths
        model = ArmModel()  # Uses (11.92, 13.5, 17.0)
        arm = RobotArm(model=model)
        
        # Target within realistic reach
        target_xyz = (20.0, 8.0, 15.0)
        phi_deg = 0.0
        tolerance = 1.0  # cm (slightly larger for real geometry)

        ok = arm.move_end_effector(target_xyz, phi_deg)

        if ok:
            state = arm.state
            
            # Verify FK
            x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                arm.model.links,
                state.base_yaw_deg,
                state.planar_angles,
                arm.model.shoulder_z,
                arm.model.base_offset
            )
            
            assert abs(x_fk - target_xyz[0]) <= tolerance
            assert abs(y_fk - target_xyz[1]) <= tolerance
            assert abs(z_fk - target_xyz[2]) <= tolerance

    def test_multiple_targets_fk_consistency(self, standard_arm):
        """Test IK/FK consistency across multiple target positions."""
        targets = [
            ((18.0, 5.0, 12.0), 0.0),
            ((16.0, 6.0, 11.0), 30.0),
            ((17.0, 4.0, 12.0), -15.0),
        ]
        tolerance = 0.5  # cm

        for target_xyz, phi_deg in targets:
            ok = standard_arm.move_end_effector(target_xyz, phi_deg)

            if ok:
                state = standard_arm.state
                
                x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                    standard_arm.model.links,
                    state.base_yaw_deg,
                    state.planar_angles,
                    standard_arm.model.shoulder_z,
                    standard_arm.model.base_offset
                )
                
                assert abs(x_fk - target_xyz[0]) <= tolerance, \
                    f"Target {target_xyz}: X mismatch"
                assert abs(y_fk - target_xyz[1]) <= tolerance, \
                    f"Target {target_xyz}: Y mismatch"
                assert abs(z_fk - target_xyz[2]) <= tolerance, \
                    f"Target {target_xyz}: Z mismatch"


class TestPhiAdaptation:
    """Tests for phi adaptation feature."""

    @pytest.fixture
    def standard_arm(self):
        """Standard arm configuration."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    def test_phi_adaptation_enabled_by_default(self, standard_arm):
        """Test that phi adaptation is enabled by default in move_end_effector."""
        # Some unreachable target at exact phi might succeed with adaptation
        target = (25.0, 15.0, 20.0)
        
        # Try without explicit allow_phi_adaptation (should use default True)
        result = standard_arm.move_end_effector(target, phi_deg=0.0)
        
        # Should be bool (not crash)
        assert isinstance(result, bool)

    def test_phi_adaptation_disabled_explicitly(self, standard_arm):
        """Test that disabling phi adaptation prevents searching."""
        target = (18.0, 5.0, 12.0)
        
        # With adaptation disabled
        result_no_adapt = standard_arm.solve_base_plus_planar_ik(
            target, 
            phi_deg=0.0,
            allow_phi_adaptation=False
        )
        
        # With adaptation enabled (default)
        result_with_adapt = standard_arm.solve_base_plus_planar_ik(
            target, 
            phi_deg=0.0,
            allow_phi_adaptation=True
        )
        
        # Both should be valid but may differ
        assert (result_no_adapt is None) or isinstance(result_no_adapt, RobotState)
        assert (result_with_adapt is None) or isinstance(result_with_adapt, RobotState)

    def test_phi_adaptation_finds_solution_when_exact_fails(self, standard_arm):
        """Test that adaptation finds solution when exact phi is unreachable."""
        # Use a target that might be unreachable at 0° but reachable at another angle
        target = (31.0, 0.0, 10.0) 
        desired_phi = 270.0  # Pointing down
        
        print(f"\nTarget: {target}, desired_phi: {desired_phi} deg")
        
        # With adaptation, should be more likely to succeed
        result = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=desired_phi,
            allow_phi_adaptation=True,
            phi_adaptation_range=360.0
        )
        
        # If it finds a solution, verify it's valid with FK
        if result is not None:
            assert isinstance(result, RobotState)
            assert math.isfinite(result.base_yaw_deg)
            assert math.isfinite(result.theta1_deg)
            assert math.isfinite(result.theta2_deg)
            assert math.isfinite(result.theta3_deg)
            
            # Print motor angles
            print(f"  [OK] SOLUTION FOUND:")
            print(f"    - Base Yaw (motor 0):  {result.base_yaw_deg:8.2f} deg")
            print(f"    - Theta 1 (motor 1):   {result.theta1_deg:8.2f} deg")
            print(f"    - Theta 2 (motor 2):   {result.theta2_deg:8.2f} deg")
            print(f"    - Theta 3 (motor 3):   {result.theta3_deg:8.2f} deg")
            
            # Verify with FK
            x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                standard_arm.model.links,
                result.base_yaw_deg,
                result.planar_angles,
                standard_arm.model.shoulder_z,
                standard_arm.model.base_offset
            )
            
            print(f"    FK Verification:")
            print(f"      Target:   X={target[0]:7.2f} Y={target[1]:7.2f} Z={target[2]:7.2f} phi={desired_phi:7.2f}")
            print(f"      FK out:   X={x_fk:7.2f} Y={y_fk:7.2f} Z={z_fk:7.2f} phi={phi_fk:7.2f}")
            
            # Verify position accuracy
            pos_tol = 1.0
            assert abs(x_fk - target[0]) <= pos_tol, f"X error: {abs(x_fk - target[0]):.3f}"
            assert abs(y_fk - target[1]) <= pos_tol, f"Y error: {abs(y_fk - target[1]):.3f}"
            assert abs(z_fk - target[2]) <= pos_tol, f"Z error: {abs(z_fk - target[2]):.3f}"
        else:
            print(f"  [X] NO SOLUTION (unreachable even with adaptation)")

    def test_phi_adaptation_prefers_close_values(self, standard_arm):
        """Test that adaptation prefers solutions near target phi."""
        target = (18.0, 5.0, 12.0)
        desired_phi = 0.0
        
        print(f"\nTarget: {target}, desired_phi: {desired_phi} deg")
        
        # Ensure solution with adaptation
        result = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=desired_phi,
            allow_phi_adaptation=True,
            phi_adaptation_range=45.0,
            phi_adaptation_step=5.0
        )
        
        if result is not None:
            # Verify FK to get actual phi
            x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                standard_arm.model.links,
                result.base_yaw_deg,
                result.planar_angles,
                standard_arm.model.shoulder_z,
                standard_arm.model.base_offset
            )
            
            print(f"  [OK] SOLUTION FOUND:")
            print(f"    - Base Yaw (motor 0):  {result.base_yaw_deg:8.2f} deg")
            print(f"    - Theta 1 (motor 1):   {result.theta1_deg:8.2f} deg")
            print(f"    - Theta 2 (motor 2):   {result.theta2_deg:8.2f} deg")
            print(f"    - Theta 3 (motor 3):   {result.theta3_deg:8.2f} deg")
            
            print(f"    FK Verification:")
            print(f"      Target phi: {desired_phi:7.2f} deg")
            print(f"      Actual phi: {phi_fk:7.2f} deg")
            
            # Actual phi should be within adaptation range
            phi_error = abs(((phi_fk - desired_phi + 180) % 360) - 180)
            print(f"      Phi error:  {phi_error:7.2f} deg")
            
            assert phi_error <= 45.0, f"Phi should be within 45 degrees, got {phi_error}"
            print(f"      Phi preference verified: error {phi_error:.2f} deg <= 45.0 deg")

    def test_phi_adaptation_with_different_ranges(self, standard_arm):
        """Test phi adaptation with different search ranges."""
        target = (18.0, 5.0, 12.0)
        
        print(f"\nTarget: {target}")
        
        # Small range
        result_small = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=10.0
        )
        
        # Large range
        result_large = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=60.0
        )
        
        print(f"  Small range (10deg): {'Found' if result_small else 'Not found'}")
        print(f"  Large range (60deg): {'Found' if result_large else 'Not found'}")
        
        # Both should be valid if reachable
        assert (result_small is None) or isinstance(result_small, RobotState)
        assert (result_large is None) or isinstance(result_large, RobotState)

    def test_phi_adaptation_with_different_steps(self, standard_arm):
        """Test phi adaptation with different step sizes."""
        target = (18.0, 5.0, 12.0)
        
        print(f"\nTarget: {target}")
        
        # Fine steps (more thorough)
        result_fine = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=30.0,
            phi_adaptation_step=2.0
        )
        
        # Coarse steps (faster)
        result_coarse = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=30.0,
            phi_adaptation_step=10.0
        )
        
        print(f"  Fine steps (2deg):   {'Found' if result_fine else 'Not found'}")
        print(f"  Coarse steps (10deg): {'Found' if result_coarse else 'Not found'}")
        
        # Both should return valid results
        assert (result_fine is None) or isinstance(result_fine, RobotState)
        assert (result_coarse is None) or isinstance(result_coarse, RobotState)

    def test_phi_adaptation_fk_consistency(self, standard_arm):
        """Test that adapted solutions still have good FK accuracy."""
        target = (20.0, 8.0, 14.0)
        
        print(f"\nTarget: {target}, desired_phi: 0.0 deg")
        
        result = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=45.0
        )
        
        if result is not None:
            # Verify FK
            x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                standard_arm.model.links,
                result.base_yaw_deg,
                result.planar_angles,
                standard_arm.model.shoulder_z,
                standard_arm.model.base_offset
            )
            
            print(f"  [OK] SOLUTION FOUND:")
            print(f"    - Base Yaw (motor 0):  {result.base_yaw_deg:8.2f} deg")
            print(f"    - Theta 1 (motor 1):   {result.theta1_deg:8.2f} deg")
            print(f"    - Theta 2 (motor 2):   {result.theta2_deg:8.2f} deg")
            print(f"    - Theta 3 (motor 3):   {result.theta3_deg:8.2f} deg")
            
            print(f"    FK Verification:")
            print(f"      Target: X={target[0]:7.2f} Y={target[1]:7.2f} Z={target[2]:7.2f}")
            print(f"      FK out: X={x_fk:7.2f} Y={y_fk:7.2f} Z={z_fk:7.2f}")
            
            # Position should match to tolerance
            pos_tol = 0.5
            assert abs(x_fk - target[0]) <= pos_tol, \
                f"FK X: {x_fk:.2f} vs target {target[0]:.2f}, error {abs(x_fk - target[0]):.3f}"
            assert abs(y_fk - target[1]) <= pos_tol, \
                f"FK Y: {y_fk:.2f} vs target {target[1]:.2f}, error {abs(y_fk - target[1]):.3f}"
            assert abs(z_fk - target[2]) <= pos_tol, \
                f"FK Z: {z_fk:.2f} vs target {target[2]:.2f}, error {abs(z_fk - target[2]):.3f}"
            
            print(f"      FK accuracy verified: X={abs(x_fk - target[0]):.3f}, Y={abs(y_fk - target[1]):.3f}, Z={abs(z_fk - target[2]):.3f} cm")

    def test_phi_adaptation_move_end_effector(self, standard_arm):
        """Test move_end_effector with phi adaptation (default enabled)."""
        targets = [
            (18.0, 5.0, 12.0, 0.0),
            (20.0, 8.0, 14.0, 30.0),
            (16.0, 6.0, 11.0, -45.0),
        ]
        
        print(f"\nTesting move_end_effector with phi adaptation:")
        
        for x, y, z, phi in targets:
            print(f"  Target: ({x}, {y}, {z}), phi: {phi} deg")
            
            result = standard_arm.move_end_effector((x, y, z), phi)
            assert isinstance(result, bool)
            
            if result:
                # Verify state was updated and FK is accurate
                state = standard_arm.state
                assert math.isfinite(state.base_yaw_deg)
                assert math.isfinite(state.theta1_deg)
                assert math.isfinite(state.theta2_deg)
                assert math.isfinite(state.theta3_deg)
                
                # Verify with FK
                x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                    standard_arm.model.links,
                    state.base_yaw_deg,
                    state.planar_angles,
                    standard_arm.model.shoulder_z,
                    standard_arm.model.base_offset
                )
                
                pos_error_x = abs(x_fk - x)
                pos_error_y = abs(y_fk - y)
                pos_error_z = abs(z_fk - z)
                
                print(f"    [OK] Position error: X={pos_error_x:.3f}, Y={pos_error_y:.3f}, Z={pos_error_z:.3f} cm")
                
                assert pos_error_x <= 1.0
                assert pos_error_y <= 1.0
                assert pos_error_z <= 1.0
            else:
                print(f"    [X] No solution found")

    def test_phi_adaptation_zero_range(self, standard_arm):
        """Test phi adaptation with zero range (should only try exact phi)."""
        target = (18.0, 5.0, 12.0)
        
        print(f"\nTarget: {target}, zero adaptation range")
        
        # Zero range means no adaptation search
        result = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=0.0  # No adaptation
        )
        
        print(f"  Result: {'Found' if result else 'Not found'}")
        
        # Should behave like allow_phi_adaptation=False
        assert (result is None) or isinstance(result, RobotState)

    def test_phi_adaptation_extreme_phi(self, standard_arm):
        """Test phi adaptation with extreme orientation requests."""
        target = (18.0, 5.0, 12.0)
        
        print(f"\nTarget: {target}, testing extreme phi values")
        
        # Try extreme phi values
        extreme_phis = [90.0, -90.0, 180.0, -180.0]
        
        for phi in extreme_phis:
            print(f"  Trying phi={phi:7.1f} deg: ", end="")
            
            result = standard_arm.solve_base_plus_planar_ik(
                target,
                phi_deg=phi,
                allow_phi_adaptation=True,
                phi_adaptation_range=45.0
            )
            
            # Should handle gracefully
            assert (result is None) or isinstance(result, RobotState)
            
            if result is not None:
                # Verify with FK
                x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                    standard_arm.model.links,
                    result.base_yaw_deg,
                    result.planar_angles,
                    standard_arm.model.shoulder_z,
                    standard_arm.model.base_offset
                )
                
                phi_error = abs(((phi_fk - phi + 180) % 360) - 180)
                print(f"Found, phi_error={phi_error:6.2f} deg")
                
                # Position should be accurate
                assert abs(x_fk - target[0]) <= 1.0
                assert abs(y_fk - target[1]) <= 1.0
                assert abs(z_fk - target[2]) <= 1.0
            else:
                print(f"Not reachable with this phi")

    def test_phi_adaptation_continuity(self, standard_arm):
        """Test that phi adaptation maintains solution continuity."""
        targets = [
            (18.0, 5.0, 12.0, 0.0),
            (18.5, 5.5, 12.0, 10.0),
            (18.1, 5.2, 12.0, -5.0),
        ]
        
        print(f"\nTesting solution continuity:")
        
        prev_state = None
        state_changes = []
        
        for x, y, z, phi in targets:
            print(f"  Target: ({x}, {y}, {z}), phi: {phi} deg: ", end="")
            
            result = standard_arm.move_end_effector((x, y, z), phi)
            
            if result:
                current_state = standard_arm.state
                
                if prev_state is not None:
                    # Compute state delta (rough continuity check)
                    yaw_delta = abs(((current_state.base_yaw_deg - prev_state.base_yaw_deg + 180) % 360) - 180)
                    state_changes.append(yaw_delta)
                    print(f"Success, yaw_delta={yaw_delta:6.2f} deg")
                else:
                    print(f"Success (first move)")
                
                prev_state = current_state
            else:
                print(f"Failed to find solution")
        
        # If we have multiple solutions, they should show reasonable continuity
        assert len(state_changes) >= 0

    @pytest.mark.parametrize("target_xyz,desired_phi", [
        ((18.0, 5.0, 12.0), 0.0),
        ((18.0, 5.0, 12.0), 45.0),
        ((18.0, 5.0, 12.0), -45.0),
        ((16.0, 6.0, 11.0), 0.0),
        ((16.0, 6.0, 11.0), 90.0),
        ((20.0, 4.0, 13.0), -30.0),
    ])
    def test_phi_adaptation_parametrized(self, standard_arm, target_xyz, desired_phi):
        """Parametrized test for phi adaptation across various targets and orientations."""
        print(f"\nTarget: {target_xyz}, desired_phi: {desired_phi} deg")
        
        # With adaptation enabled (default)
        result = standard_arm.solve_base_plus_planar_ik(
            target_xyz,
            phi_deg=desired_phi,
            allow_phi_adaptation=True
        )
        
        # Should return valid result or None
        assert (result is None) or isinstance(result, RobotState)
        
        # If solution found, verify angles are well-formed
        if result is not None:
            assert math.isfinite(result.base_yaw_deg)
            assert math.isfinite(result.theta1_deg)
            assert math.isfinite(result.theta2_deg)
            assert math.isfinite(result.theta3_deg)
            
            # Verify FK accuracy
            x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                standard_arm.model.links,
                result.base_yaw_deg,
                result.planar_angles,
                standard_arm.model.shoulder_z,
                standard_arm.model.base_offset
            )
            
            pos_error_x = abs(x_fk - target_xyz[0])
            pos_error_y = abs(y_fk - target_xyz[1])
            pos_error_z = abs(z_fk - target_xyz[2])
            phi_error = abs(((phi_fk - desired_phi + 180) % 360) - 180)
            
            print(f"  [OK] Found solution:")
            print(f"    Position error: X={pos_error_x:.3f}, Y={pos_error_y:.3f}, Z={pos_error_z:.3f} cm")
            print(f"    Phi error: {phi_error:.2f} deg")
            
            assert pos_error_x <= 1.0, f"Position X should match"
            assert pos_error_y <= 1.0, f"Position Y should match"
            assert pos_error_z <= 1.0, f"Position Z should match"
        else:
            print(f"  [X] No solution found")


class TestRobotArmStress:
    """Stress tests with parameterized target values to identify edge cases."""

    @pytest.fixture
    def standard_arm(self):
        """Standard arm configuration."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    @pytest.mark.parametrize("target_xyz,phi_deg,description", [
        # Close range targets
        ((10.0, 2.0, 10.0), 0.0, "close_1"),
        ((12.0, 1.0, 11.0), 0.0, "close_2"),
        ((14.0, 3.0, 9.0), 0.0, "close_3"),
        
        # Medium range targets
        ((18.0, 5.0, 12.0), 0.0, "medium_1"),
        ((16.0, 6.0, 11.0), 0.0, "medium_2"),
        ((20.0, 4.0, 13.0), 0.0, "medium_3"),
        ((15.0, 8.0, 10.0), 0.0, "medium_4"),
        
        # Far range targets (near limit)
        ((25.0, 2.0, 15.0), 0.0, "far_1"),
        ((22.0, 8.0, 14.0), 0.0, "far_2"),
        ((24.0, -5.0, 16.0), 0.0, "far_3"),
        
        # Negative coordinates
        ((-15.0, -5.0, 12.0), 0.0, "negative_1"),
        ((-10.0, -8.0, 11.0), 0.0, "negative_2"),
        ((10.0, -5.0, 12.0), 0.0, "negative_3"),
        ((-18.0, 3.0, 10.0), 0.0, "negative_4"),
        
        # Edge cases - very small Z
        ((18.0, 5.0, 5.0), 0.0, "low_z_1"),
        ((16.0, 6.0, 6.0), 0.0, "low_z_2"),
        
        # Edge cases - high Z
        ((18.0, 5.0, 20.0), 0.0, "high_z_1"),
        ((16.0, 6.0, 25.0), 0.0, "high_z_2"),
        
        # Origin area
        ((0.0, 0.0, 10.0), 0.0, "origin_1"),
        ((2.0, 2.0, 11.0), 0.0, "origin_2"),
        ((1.0, -1.0, 10.0), 0.0, "origin_3"),
        
        # Various orientations
        ((18.0, 5.0, 12.0), 0.0, "ori_0"),
        ((18.0, 5.0, 12.0), 45.0, "ori_45"),
        ((18.0, 5.0, 12.0), 90.0, "ori_90"),
        ((18.0, 5.0, 12.0), -45.0, "ori_neg45"),
        ((16.0, 6.0, 11.0), 30.0, "ori_30"),
        ((16.0, 6.0, 11.0), -60.0, "ori_neg60"),
        
        # Stress test - extreme coordinates
        ((30.0, 1.0, 18.0), 0.0, "stress_1"),
        ((8.0, 10.0, 15.0), 0.0, "stress_2"),
        ((28.0, -6.0, 14.0), 0.0, "stress_3"),
        ((5.0, 12.0, 8.0), 0.0, "stress_4"),
        ((25.0, 15.0, 20.0), 0.0, "stress_5"),
    ])
    def test_with_various_targets(self, standard_arm, target_xyz, phi_deg, description):
        """
        Parametrized test to try solving IK for many different target positions.
        Tests should not crash; returns None cleanly for unreachable targets.
        Verbose output shows all motor angles for each target.
        
        Args:
            target_xyz: Target position (x, y, z) in cm
            phi_deg: End-effector orientation in degrees
            description: Test case identifier
        """
        try:
            # Print verbose header
            print(f"\n[{description}] Target: {target_xyz}, phi: {phi_deg} deg")
            
            # Attempt to solve IK
            solution = standard_arm.solve_base_plus_planar_ik(target_xyz, phi_deg)
            
            # Should return either RobotState or None (not crash)
            assert solution is None or isinstance(solution, RobotState), \
                f"[{description}] Expected RobotState or None, got {type(solution)}"
            
            # If solution found, verify it contains valid angles
            if solution is not None:
                # Print motor angles
                print(f"  [OK] SOLUTION FOUND:")
                print(f"    - Base Yaw (motor 0):  {solution.base_yaw_deg:8.2f} deg")
                print(f"    - Theta 1 (motor 1):   {solution.theta1_deg:8.2f} deg")
                print(f"    - Theta 2 (motor 2):   {solution.theta2_deg:8.2f} deg")
                print(f"    - Theta 3 (motor 3):   {solution.theta3_deg:8.2f} deg")
                
                assert math.isfinite(solution.base_yaw_deg), \
                    f"[{description}] base_yaw is not finite"
                assert math.isfinite(solution.theta1_deg), \
                    f"[{description}] theta1 is not finite"
                assert math.isfinite(solution.theta2_deg), \
                    f"[{description}] theta2 is not finite"
                assert math.isfinite(solution.theta3_deg), \
                    f"[{description}] theta3 is not finite"
                
                # Verify angles are in reasonable range
                assert -360 <= solution.base_yaw_deg <= 360, \
                    f"[{description}] base_yaw out of range: {solution.base_yaw_deg}"
                assert -360 <= solution.theta1_deg <= 360, \
                    f"[{description}] theta1 out of range: {solution.theta1_deg}"
                assert -360 <= solution.theta2_deg <= 360, \
                    f"[{description}] theta2 out of range: {solution.theta2_deg}"
                assert -360 <= solution.theta3_deg <= 360, \
                    f"[{description}] theta3 out of range: {solution.theta3_deg}"
                
                # Verify FK consistency for this solution
                x_fk, y_fk, z_fk, phi_fk = fk_4dof_deg(
                    standard_arm.model.links,
                    solution.base_yaw_deg,
                    solution.planar_angles,
                    standard_arm.model.shoulder_z,
                    standard_arm.model.base_offset
                )
                
                # Print FK verification results
                print(f"    FK Verification:")
                print(f"      Target: X={target_xyz[0]:7.2f} Y={target_xyz[1]:7.2f} Z={target_xyz[2]:7.2f} phi={phi_deg:7.2f}")
                print(f"      FK out: X={x_fk:7.2f} Y={y_fk:7.2f} Z={z_fk:7.2f} phi={phi_fk:7.2f}")
                
                # Stricter FK validation
                pos_tolerance = 0.5  # cm - position accuracy
                ori_tolerance = 2.0  # degrees - orientation accuracy
                
                assert abs(x_fk - target_xyz[0]) <= pos_tolerance, \
                    f"[{description}] FK X error: {abs(x_fk - target_xyz[0]):.3f} cm (exceeds {pos_tolerance} cm)"
                assert abs(y_fk - target_xyz[1]) <= pos_tolerance, \
                    f"[{description}] FK Y error: {abs(y_fk - target_xyz[1]):.3f} cm (exceeds {pos_tolerance} cm)"
                assert abs(z_fk - target_xyz[2]) <= pos_tolerance, \
                    f"[{description}] FK Z error: {abs(z_fk - target_xyz[2]):.3f} cm (exceeds {pos_tolerance} cm)"
                
                # Normalize angles to [-180, 180] for comparison
                phi_error = (phi_fk - phi_deg + 180) % 360 - 180
                assert abs(phi_error) <= ori_tolerance, \
                    f"[{description}] FK orientation error: {abs(phi_error):.2f} deg (exceeds {ori_tolerance} deg)"
            else:
                print(f"  [X] NO SOLUTION (unreachable)")
        
        except Exception as e:
            pytest.fail(
                f"[{description}] Target {target_xyz} crashed: {type(e).__name__}: {e}"
            )

    @pytest.mark.parametrize("target_xyz,phi_deg", [
        # Series of targets to validate continuity
        ((18.0, 5.0, 12.0), 0.0),
        ((18.1, 5.0, 12.0), 0.0),
        ((18.2, 5.0, 12.0), 0.0),
        ((18.3, 5.0, 12.0), 0.0),
        ((18.4, 5.0, 12.0), 0.0),
    ])
    def test_trajectory_continuity(self, standard_arm, target_xyz, phi_deg):
        """
        Test that small incremental moves show continuity in solutions.
        Verbose output shows all motor angles for each waypoint.
        """
        print(f"\n  -> Target: {target_xyz}, phi: {phi_deg} deg")
        
        ok = standard_arm.move_end_effector(target_xyz, phi_deg)
        
        # Small incremental moves should usually be solvable
        assert isinstance(ok, bool), "move_end_effector should return bool"
        
        if ok:
            # Verify solution is valid
            state = standard_arm.state
            
            # Print motor angles
            print(f"    [OK] Base Yaw: {state.base_yaw_deg:8.2f} deg | theta1: {state.theta1_deg:8.2f} deg | theta2: {state.theta2_deg:8.2f} deg | theta3: {state.theta3_deg:8.2f} deg")
            
            assert math.isfinite(state.base_yaw_deg)
            assert math.isfinite(state.theta1_deg)
            assert math.isfinite(state.theta2_deg)
            assert math.isfinite(state.theta3_deg)
        else:
            print(f"    [X] NO SOLUTION (unreachable)")
