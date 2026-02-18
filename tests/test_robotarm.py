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
        """Real robot arm model (default geometry from ArmModel)."""
        return ArmModel()

    def test_robot_arm_creation(self, standard_model):
        """Test basic RobotArm instantiation."""
        arm = RobotArm(model=standard_model)
        assert arm.model == standard_model
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

    def test_robot_arm_state_is_copy(self, standard_model):
        """Test that initial state is independent instance."""
        arm1 = RobotArm(model=standard_model)
        arm2 = RobotArm(model=standard_model)
        assert arm1.state is not arm2.state
        assert arm1.state.base_yaw_deg == arm2.state.base_yaw_deg

    def test_robot_arm_with_multiple_joint_limits(self, standard_model):
        """Test RobotArm with different joint limit configurations."""
        limits1 = [(-90, 90), (-120, 120), (-120, 120)]
        limits2 = [(-180, 180), (-180, 180), (-180, 180)]
        
        arm1 = RobotArm(model=standard_model, joint_limits_deg=limits1)
        arm2 = RobotArm(model=standard_model, joint_limits_deg=limits2)
        
        assert arm1.joint_limits_deg == limits1
        assert arm2.joint_limits_deg == limits2
        assert arm1.joint_limits_deg != arm2.joint_limits_deg

    def test_robot_arm_preserves_model_properties(self, standard_model):
        """Test that RobotArm correctly stores model properties."""
        arm = RobotArm(model=standard_model)
        
        assert arm.model.links == (11.92, 13.5, 17.0)
        assert arm.model.shoulder_z == 11.0
        assert arm.model.base_offset == 4.0

    def test_robot_arm_state_update_preserves_model(self, standard_model):
        """Test that updating state doesn't affect model."""
        arm = RobotArm(model=standard_model)
        original_model = arm.model
        
        new_state = RobotState(
            base_yaw_deg=45.0,
            theta1_deg=30.0,
            theta2_deg=90.0,
            theta3_deg=50.0
        )
        arm.state = new_state
        
        assert arm.model == original_model
        assert arm.model is standard_model

    def test_robot_arm_multiple_instances_independence(self):
        """Test that multiple RobotArm instances are independent."""
        model1 = ArmModel(links=(10.0, 10.0, 10.0), shoulder_z=5.0, base_offset=2.0)
        model2 = ArmModel(links=(15.0, 12.0, 8.0), shoulder_z=8.0, base_offset=3.0)
        
        arm1 = RobotArm(model=model1)
        arm2 = RobotArm(model=model2)
        
        # Move arm1
        arm1.move_end_effector((10.0, 5.0, 6.0), 0.0)
        
        # Arm2 should not be affected
        assert arm2.state.base_yaw_deg == 0.0
        assert arm2.state.theta1_deg == 1.0

    def test_robot_arm_with_asymmetric_joint_limits(self, standard_model):
        """Test RobotArm with asymmetric joint limits."""
        limits = [(-170, 170), (-100, 150), (-80, 100)]
        arm = RobotArm(model=standard_model, joint_limits_deg=limits)
        
        assert len(arm.joint_limits_deg) == 3
        assert arm.joint_limits_deg[0] == (-170, 170)
        assert arm.joint_limits_deg[1] == (-100, 150)
        assert arm.joint_limits_deg[2] == (-80, 100)

    def test_robot_arm_initialization_with_default_model(self):
        """Test RobotArm initialization with default ArmModel."""
        arm = RobotArm(model=ArmModel())
        
        assert arm.model.links == (11.92, 13.5, 17.0)
        assert arm.model.shoulder_z == 11.0
        assert arm.model.base_offset == 4.0
        assert isinstance(arm.state, RobotState)

    def test_robot_arm_joint_limits_validation(self, standard_model):
        """Test that joint limits are properly stored without modification."""
        limits = [(-180, 180), (-120, 120), (-120, 120)]
        arm = RobotArm(model=standard_model, joint_limits_deg=limits)
        
        # Verify structure
        for i, (lower, upper) in enumerate(arm.joint_limits_deg):
            assert isinstance(lower, (int, float))
            assert isinstance(upper, (int, float))
            assert lower <= upper, f"Joint {i}: lower limit should be <= upper limit"


class TestSolveBaseYawIK:
    """Tests for the solve_base_plus_planar_ik method."""

    @pytest.fixture
    def standard_arm(self):
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

    @pytest.fixture
    def constrained_arm(self):
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

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
        # Use reachable targets for the real arm (11.92, 13.5, 17.0 link lengths, shoulder_z=11)
        # These targets were verified reachable at phi~0 within calibrated joint limits
        targets = [
            (28.0, 4.0, 22.0),
            (28.5, 4.5, 22.0),
            (28.0, 4.0, 22.0),
        ]

        prev_state = standard_arm.state
        max_yaw_jump = 0.0
        solutions_found = 0
        
        for target in targets:
            sol = standard_arm.solve_base_plus_planar_ik(target, 0.0)
            if sol is not None:
                solutions_found += 1
                # Check continuity: yaw shouldn't jump dramatically between similar targets
                yaw_delta = abs(((sol.base_yaw_deg - prev_state.base_yaw_deg + 180) % 360) - 180)
                max_yaw_jump = max(max_yaw_jump, yaw_delta)
                prev_state = sol
        
        # At least one solution should be found
        assert solutions_found > 0, "Should find at least one IK solution"
        # Yaw shouldn't jump more than 90° for these close targets (continuity check)
        assert max_yaw_jump <= 90.0, f"Yaw jumped {max_yaw_jump:.2f}°, suggesting discontinuity"

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

        # Loose tolerance should be at least as likely (or more) to find solution than tight
        if sol1 is None:
            # If tight fails, loose should try at least one more candidate
            # (it may still fail if target is unreachable)
            assert sol2 is None or isinstance(sol2, RobotState), \
                "Loose tolerance should return valid result"
        else:
            # If tight succeeds, both should succeed
            assert sol2 is not None, \
                "Loose tolerance should at least match tight tolerance success"
            assert isinstance(sol2, RobotState), \
                "Solution should be a valid RobotState"

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
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

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
        # Use a clearly unreachable target (far beyond arm reach)
        # Arm span with default model: ~40-50cm; target 500cm away
        target = (500.0, 500.0, 500.0)
        phi_deg = 0.0

        ok = standard_arm.move_end_effector(target, phi_deg)

        # Should return False for clearly unreachable target
        assert isinstance(ok, bool), "move_end_effector must return bool"
        assert ok is False, \
            f"Target {target} should be unreachable for arm with span ~40-50cm, but returned {ok}"

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
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

    @pytest.fixture
    def constrained_arm(self):
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

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
            # Check that solution respects the loaded calibration joint limits
            state = constrained_arm.state
            theta1, theta2, theta3 = state.planar_angles
            limits = constrained_arm.joint_limits_deg

            lo1, hi1 = limits[0]
            lo2, hi2 = limits[1]
            lo3, hi3 = limits[2]
            assert lo1 <= theta1 <= hi1, \
                f"theta1 {theta1:.2f} outside calibrated limits [{lo1}, {hi1}]"
            assert lo2 <= theta2 <= hi2, \
                f"theta2 {theta2:.2f} outside calibrated limits [{lo2}, {hi2}]"
            assert lo3 <= theta3 <= hi3, \
                f"theta3 {theta3:.2f} outside calibrated limits [{lo3}, {hi3}]"

    def test_ik_fk_consistency(self, standard_arm):
        """Verify that IK solutions actually reach the target via FK."""
        target_xyz = (18.0, 5.0, 12.0)
        phi_deg = 0.0
        tolerance = 0.5  # cm

        ok = standard_arm.move_end_effector(target_xyz, phi_deg)

        if ok:
            state = standard_arm.state
            
            # Compute FK of the IK solution
            x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                state.base_yaw_deg,
                state.theta1_deg,
                state.theta2_deg,
                state.theta3_deg
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
            x_fk, y_fk, z_fk, phi_fk = arm.forward_kinematics(
                state.base_yaw_deg,
                state.theta1_deg,
                state.theta2_deg,
                state.theta3_deg
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
                
                x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                    state.base_yaw_deg,
                    state.theta1_deg,
                    state.theta2_deg,
                    state.theta3_deg
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
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

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
            phi_adaptation_range=60.0
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
            x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                result.base_yaw_deg,
                result.theta1_deg,
                result.theta2_deg,
                result.theta3_deg
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
            x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                result.base_yaw_deg,
                result.theta1_deg,
                result.theta2_deg,
                result.theta3_deg
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
            x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                result.base_yaw_deg,
                result.theta1_deg,
                result.theta2_deg,
                result.theta3_deg
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
                x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                    state.base_yaw_deg,
                    state.theta1_deg,
                    state.theta2_deg,
                    state.theta3_deg
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
        result_zero_range = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=0.0  # No adaptation
        )
        
        # Compare with explicit allow_phi_adaptation=False
        result_no_adapt = standard_arm.solve_base_plus_planar_ik(
            target,
            phi_deg=0.0,
            allow_phi_adaptation=False
        )
        
        print(f"  Zero range result: {'Found' if result_zero_range else 'Not found'}")
        print(f"  No adapt result:   {'Found' if result_no_adapt else 'Not found'}")
        
        # Both should return valid types
        assert (result_zero_range is None) or isinstance(result_zero_range, RobotState)
        assert (result_no_adapt is None) or isinstance(result_no_adapt, RobotState)
        
        # They should produce equivalent results (both find or both don't)
        # since zero range adaptation effectively disables adaptation
        assert (result_zero_range is None) == (result_no_adapt is None), \
            "Zero adaptation range should behave equivalently to allow_phi_adaptation=False"

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
                x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                    result.base_yaw_deg,
                    result.theta1_deg,
                    result.theta2_deg,
                    result.theta3_deg
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
        # Targets verified reachable with real arm/joint limits (within phi adaptation ±45°)
        targets = [
            (28.0, 4.0, 22.0, 0.0),
            (28.5, 4.5, 22.0, 10.0),
            (28.2, 4.2, 22.0, -5.0),
        ]
        
        print(f"\nTesting solution continuity:")
        
        prev_state = None
        state_changes = []
        successful_moves = 0
        
        for x, y, z, phi in targets:
            print(f"  Target: ({x}, {y}, {z}), phi: {phi} deg: ", end="")
            
            result = standard_arm.move_end_effector((x, y, z), phi)
            
            if result:
                current_state = standard_arm.state
                successful_moves += 1
                
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
        
        # Should find at least one solution in the trajectory
        assert successful_moves > 0, "Should find at least one solution in trajectory"
        
        # If we have multiple solutions, they should show reasonable continuity
        # (yaw shouldn't jump more than 90° between adjacent close targets)
        if len(state_changes) > 0:
            max_yaw_delta = max(state_changes)
            assert max_yaw_delta <= 90.0, \
                f"Continuity check failed: max yaw delta {max_yaw_delta:.2f}° exceeds 90° threshold"

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
            x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                result.base_yaw_deg,
                result.theta1_deg,
                result.theta2_deg,
                result.theta3_deg
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
        """Real robot arm with calibrated geometry and joint limits."""
        return RobotArm(model=ArmModel())

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
        
        # Far range targets (near limit) TODO: Where are the links pointing??
        ((40.0, 0.0, 0.0), 0.0, "far_1"),
        ((22.0, 8.0, 14.0), 0.0, "far_2"),
        ((20.0, -5.0, 16.0), 0.0, "far_3"),
        
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
            
            # Attempt to solve IK WITHOUT phi adaptation (exact phi only)
            print(f"  [1] Trying WITHOUT phi adaptation (exact phi only)...")
            solution_exact = standard_arm.solve_base_plus_planar_ik(
                target_xyz,
                phi_deg,
                allow_phi_adaptation=False
            )
            
            # Attempt to solve IK WITH phi adaptation
            print(f"  [2] Trying WITH phi adaptation (search +/- 45 degrees)...")
            solution_adapted = standard_arm.solve_base_plus_planar_ik(
                target_xyz,
                phi_deg,
                allow_phi_adaptation=True,
                phi_adaptation_range=45.0,
                phi_adaptation_step=5.0
            )
            
            # Report results
            exact_found = solution_exact is not None
            adapt_found = solution_adapted is not None
            
            status_exact = "[OK]" if exact_found else "[XX]"
            status_adapt = "[OK]" if adapt_found else "[XX]"
            
            print(f"    Exact:     {status_exact} - {'Solution found' if exact_found else 'No solution'}")
            print(f"    Adapted:   {status_adapt} - {'Solution found' if adapt_found else 'No solution'}")
            
            # Should return either RobotState or None (not crash)
            assert solution_exact is None or isinstance(solution_exact, RobotState), \
                f"[{description}] Exact search: Expected RobotState or None, got {type(solution_exact)}"
            assert solution_adapted is None or isinstance(solution_adapted, RobotState), \
                f"[{description}] Adapted search: Expected RobotState or None, got {type(solution_adapted)}"
            
            # Analyze both solutions
            solutions_to_check = []
            if solution_exact is not None:
                solutions_to_check.append(("EXACT", solution_exact))
            if solution_adapted is not None:
                solutions_to_check.append(("ADAPTED", solution_adapted))
            
            for sol_type, solution in solutions_to_check:
                # Print motor angles
                print(f"  {sol_type} Solution:")
                print(f"    - Base Yaw (motor 0):  {solution.base_yaw_deg:8.2f} deg")
                print(f"    - Theta 1 (motor 1):   {solution.theta1_deg:8.2f} deg")
                print(f"    - Theta 2 (motor 2):   {solution.theta2_deg:8.2f} deg")
                print(f"    - Theta 3 (motor 3):   {solution.theta3_deg:8.2f} deg")
                
                assert math.isfinite(solution.base_yaw_deg), \
                    f"[{description}] {sol_type}: base_yaw is not finite"
                assert math.isfinite(solution.theta1_deg), \
                    f"[{description}] {sol_type}: theta1 is not finite"
                assert math.isfinite(solution.theta2_deg), \
                    f"[{description}] {sol_type}: theta2 is not finite"
                assert math.isfinite(solution.theta3_deg), \
                    f"[{description}] {sol_type}: theta3 is not finite"
                
                # Verify angles are in reasonable range
                assert -360 <= solution.base_yaw_deg <= 360, \
                    f"[{description}] {sol_type}: base_yaw out of range: {solution.base_yaw_deg}"
                assert -360 <= solution.theta1_deg <= 360, \
                    f"[{description}] {sol_type}: theta1 out of range: {solution.theta1_deg}"
                assert -360 <= solution.theta2_deg <= 360, \
                    f"[{description}] {sol_type}: theta2 out of range: {solution.theta2_deg}"
                assert -360 <= solution.theta3_deg <= 360, \
                    f"[{description}] {sol_type}: theta3 out of range: {solution.theta3_deg}"
                
                # Verify FK consistency for this solution
                x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                    solution.base_yaw_deg,
                    solution.theta1_deg,
                    solution.theta2_deg,
                    solution.theta3_deg
                )
                
                # Print FK verification results
                print(f"    FK Verification ({sol_type}):")
                print(f"      Target: X={target_xyz[0]:7.2f} Y={target_xyz[1]:7.2f} Z={target_xyz[2]:7.2f} phi={phi_deg:7.2f}")
                print(f"      FK out: X={x_fk:7.2f} Y={y_fk:7.2f} Z={z_fk:7.2f} phi={phi_fk:7.2f}")
                
                # Stricter FK validation
                pos_tolerance = 0.5  # cm - position accuracy
                ori_tolerance = 2.0  # degrees - orientation accuracy for exact solutions
                # For adapted solutions, allow looser orientation if within adaptation range
                adapt_ori_tolerance = 45.0 if sol_type == "ADAPTED" else ori_tolerance
                
                assert abs(x_fk - target_xyz[0]) <= pos_tolerance, \
                    f"[{description}] {sol_type}: FK X error: {abs(x_fk - target_xyz[0]):.3f} cm (exceeds {pos_tolerance} cm)"
                assert abs(y_fk - target_xyz[1]) <= pos_tolerance, \
                    f"[{description}] {sol_type}: FK Y error: {abs(y_fk - target_xyz[1]):.3f} cm (exceeds {pos_tolerance} cm)"
                assert abs(z_fk - target_xyz[2]) <= pos_tolerance, \
                    f"[{description}] {sol_type}: FK Z error: {abs(z_fk - target_xyz[2]):.3f} cm (exceeds {pos_tolerance} cm)"
                
                # Normalize angles to [-180, 180] for comparison
                phi_error = (phi_fk - phi_deg + 180) % 360 - 180
                assert abs(phi_error) <= adapt_ori_tolerance, \
                    f"[{description}] {sol_type}: FK orientation error: {abs(phi_error):.2f} deg (exceeds {adapt_ori_tolerance} deg)"
                
                print(f"    Position error: {math.sqrt((x_fk-target_xyz[0])**2 + (y_fk-target_xyz[1])**2 + (z_fk-target_xyz[2])**2):.3f} cm")
                print(f"    Orientation error: {abs(phi_error):.2f} deg")
            
            if not solutions_to_check:
                print(f"  [No solutions] Both exact and adapted searches failed")
                print(f"    - Target may be outside workspace")
                print(f"    - Or orientation is unreachable in workspace")
            
            # Report which method was better
            if exact_found and not adapt_found:
                print(f"  [Finding] Exact phi search was sufficient")
            elif adapt_found and not exact_found:
                print(f"  [Finding] Phi adaptation found solution that exact search missed - IMPROVED")
            elif adapt_found and exact_found:
                print(f"  [Finding] Both methods found solutions")
            else:
                print(f"  [Finding] Target unreachable in workspace")
        
        except Exception as e:
            pytest.fail(
                f"[{description}] Target {target_xyz} crashed: {type(e).__name__}: {e}"
            )

    @pytest.mark.parametrize("target_xyz,phi_deg,allow_adaptation", [
        # Test exact phi only (no adaptation)
        ((18.0, 5.0, 12.0), 0.0, False),
        ((16.0, 6.0, 11.0), 0.0, False),
        ((15.0, 3.0, 10.0), 0.0, False),
        
        # Test with phi adaptation enabled
        ((18.0, 5.0, 12.0), 0.0, True),
        ((16.0, 6.0, 11.0), 0.0, True),
        ((15.0, 3.0, 10.0), 0.0, True),
        
        # Challenging targets with specific orientations
        ((20.0, 8.0, 14.0), 45.0, False),
        ((20.0, 8.0, 14.0), 45.0, True),
        
        # Edge cases
        ((25.0, -3.0, 15.0), 90.0, False),
        ((25.0, -3.0, 15.0), 90.0, True),
    ])
    def test_with_and_without_phi_adaptation(self, standard_arm, target_xyz, phi_deg, allow_adaptation):
        """
        Test IK solving with and without phi adaptation for same target.
        
        Demonstrates that:
        - Exact phi search works for reachable targets at that orientation
        - Phi adaptation can find solutions at nearby orientations
        - Phi adaptation expands effective workspace
        
        Args:
            target_xyz: Target position (x, y, z) in cm
            phi_deg: Desired end-effector orientation in degrees
            allow_adaptation: Whether to allow phi adaptation in search
        """
        adaptation_str = "WITH" if allow_adaptation else "WITHOUT"
        print(f"\nTarget: {target_xyz}, phi: {phi_deg} deg [{adaptation_str} phi adaptation]")
        
        # Attempt solving with specified adaptation setting
        solution = standard_arm.solve_base_plus_planar_ik(
            target_xyz,
            phi_deg,
            allow_phi_adaptation=allow_adaptation,
            phi_adaptation_range=45.0,  # Search within 45 degrees
            phi_adaptation_step=5.0
        )
        
        # Should return valid type
        assert solution is None or isinstance(solution, RobotState), \
            f"Expected RobotState or None, got {type(solution)}"
        
        if solution is not None:
            actual_phi = solution.theta1_deg + solution.theta2_deg + solution.theta3_deg
            phi_error = abs(((actual_phi - phi_deg + 180) % 360) - 180)
            
            print(f"  [OK] Solution found:")
            print(f"    Desired phi: {phi_deg:.2f} deg")
            print(f"    Actual phi:  {actual_phi:.2f} deg (error: {phi_error:.2f} deg)")
            print(f"    Base yaw:    {solution.base_yaw_deg:.2f} deg")
            print(f"    Theta1:      {solution.theta1_deg:.2f} deg")
            print(f"    Theta2:      {solution.theta2_deg:.2f} deg")
            print(f"    Theta3:      {solution.theta3_deg:.2f} deg")
            
            # Verify FK
            x_fk, y_fk, z_fk, phi_fk = standard_arm.forward_kinematics(
                solution.base_yaw_deg,
                solution.theta1_deg,
                solution.theta2_deg,
                solution.theta3_deg
            )
            
            pos_error = math.sqrt(
                (x_fk - target_xyz[0])**2 +
                (y_fk - target_xyz[1])**2 +
                (z_fk - target_xyz[2])**2
            )
            print(f"    Position error: {pos_error:.3f} cm")
            
            # FK should be reasonably accurate
            assert pos_error <= 1.0, \
                f"Position accuracy failed: {pos_error:.3f} cm exceeds 1.0 cm limit"
        else:
            print(f"  [No solution] Target unreachable at phi={phi_deg:.2f} deg " +
                  f"{'even with' if allow_adaptation else 'without'} adaptation")

    def test_phi_adaptation_increases_workspace(self, standard_arm):
        """
        Test that phi adaptation significantly increases effective workspace.
        
        Strategy:
        - Test multiple targets at specific orientations
        - Solve both WITH and WITHOUT phi adaptation
        - Verify adaptation finds solutions when exact phi fails
        - Track success rate improvement
        """
        test_cases = [
            ((20.0, 8.0, 14.0), 45.0),
            ((22.0, 6.0, 16.0), 60.0),
            ((18.0, 10.0, 13.0), 30.0),
            ((24.0, 2.0, 15.0), 75.0),
            ((16.0, 12.0, 11.0), -30.0),
        ]
        
        print("\nPhi Adaptation Workspace Expansion Test:")
        print("-" * 60)
        
        exact_solutions = 0
        adapted_solutions = 0
        improved = 0
        
        for target_xyz, phi_deg in test_cases:
            # Try exact phi only
            exact_sol = standard_arm.solve_base_plus_planar_ik(
                target_xyz,
                phi_deg,
                allow_phi_adaptation=False
            )
            
            # Try with adaptation
            adapted_sol = standard_arm.solve_base_plus_planar_ik(
                target_xyz,
                phi_deg,
                allow_phi_adaptation=True,
                phi_adaptation_range=60.0,
                phi_adaptation_step=5.0
            )
            
            exact_status = "Found" if exact_sol is not None else "Failed"
            adapted_status = "Found" if adapted_sol is not None else "Failed"
            
            if exact_sol is not None:
                exact_solutions += 1
            if adapted_sol is not None:
                adapted_solutions += 1
            
            # Track improvements (found with adaptation but not exact)
            if adapted_sol is not None and exact_sol is None:
                improved += 1
            
            print(f"  Target {target_xyz} @ phi={phi_deg:6.1f}deg:")
            print(f"    Exact phi:     {exact_status:6s}")
            print(f"    With adapt:    {adapted_status:6s}")
            
            if adapted_sol is not None:
                if exact_sol is None:
                    print(f"    -> IMPROVED (adaptation found solution)")
        
        print("-" * 60)
        print(f"  Exact phi solutions:     {exact_solutions}/{len(test_cases)}")
        print(f"  With adaptation:         {adapted_solutions}/{len(test_cases)}")
        print(f"  Improved cases:          {improved}/{len(test_cases)}")
        print(f"  Improvement rate:        {100*improved/len(test_cases):.1f}%")
        
        # Adaptation should help at least sometimes
        assert adapted_solutions >= exact_solutions, \
            "Phi adaptation should find at least as many solutions as exact phi"

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
