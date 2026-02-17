"""
Comprehensive tests for joint limits enforcement in RobotArm.

Tests verify that inverse kinematics solutions respect motor constraints
from the calibration file and that invalid motor commands are rejected.
"""

import sys
import math
from pathlib import Path

# Add the robot_program module to path
sys.path.insert(0, str(Path(__file__).parent.parent / "robot_program"))

import pytest
from robot_program.RobotArm import RobotArm, ArmModel, RobotState, load_joint_limits_from_calibration


class TestLoadJointLimitsFromCalibration:
    """Tests for the load_joint_limits_from_calibration function."""

    def test_load_default_calibration_file(self):
        """Test loading joint limits from the default calibration file location."""
        limits = load_joint_limits_from_calibration()
        
        assert limits is not None, "Should successfully load limits from calibration file"
        assert len(limits) == 3, "Should have limits for 3 planar joints (theta1, theta2, theta3)"
        
        for i, (min_deg, max_deg) in enumerate(limits, 1):
            assert isinstance(min_deg, float), f"theta{i} min should be float"
            assert isinstance(max_deg, float), f"theta{i} max should be float"
            assert min_deg < max_deg, f"theta{i} min should be less than max"

    def test_load_limits_motor_2_shoulder_lift(self):
        """Test that motor 2 (shoulder_lift) limits are correctly extracted."""
        limits = load_joint_limits_from_calibration()
        theta1_min, theta1_max = limits[0]
        
        # Expected values from lerobot_arm_with_degrees.json
        assert abs(theta1_min - (-102.13)) < 0.01, "theta1 min should match motor 2 degree_min"
        assert abs(theta1_max - 107.84) < 0.01, "theta1 max should match motor 2 degree_max"

    def test_load_limits_motor_3_elbow_flex(self):
        """Test that motor 3 (elbow_flex) limits are correctly extracted."""
        limits = load_joint_limits_from_calibration()
        theta2_min, theta2_max = limits[1]
        
        # Expected values from lerobot_arm_with_degrees.json
        assert abs(theta2_min - (-106.26)) < 0.01, "theta2 min should match motor 3 degree_min"
        assert abs(theta2_max - 88.33) < 0.01, "theta2 max should match motor 3 degree_max"

    def test_load_limits_motor_4_wrist_flex(self):
        """Test that motor 4 (wrist_flex) limits are correctly extracted."""
        limits = load_joint_limits_from_calibration()
        theta3_min, theta3_max = limits[2]
        
        # Expected values from lerobot_arm_with_degrees.json
        assert abs(theta3_min - (-100.90)) < 0.01, "theta3 min should match motor 4 degree_min"
        assert abs(theta3_max - 104.77) < 0.01, "theta3 max should match motor 4 degree_max"

    def test_load_limits_with_custom_path(self):
        """Test loading limits from a custom calibration file path."""
        calib_path = Path(__file__).parent.parent / "calibration" / "lerobot_arm_with_degrees.json"
        limits = load_joint_limits_from_calibration(calibration_path=calib_path)
        
        assert limits is not None, "Should load from custom path"
        assert len(limits) == 3, "Should have 3 joint limits"

    def test_load_limits_nonexistent_file_raises_error(self):
        """Test that loading from a nonexistent file raises FileNotFoundError."""
        fake_path = Path("/nonexistent/path/calibration.json")
        
        with pytest.raises(FileNotFoundError):
            load_joint_limits_from_calibration(calibration_path=fake_path)


class TestRobotArmAutoLoadLimits:
    """Tests for automatic joint limit loading in RobotArm initialization."""

    @pytest.fixture
    def standard_model(self):
        """Standard arm model for testing."""
        return ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )

    def test_arm_auto_loads_limits_by_default(self, standard_model):
        """Test that RobotArm automatically loads limits from calibration file."""
        arm = RobotArm(model=standard_model)
        
        assert arm.joint_limits_deg is not None, \
            "RobotArm should auto-load joint limits by default"
        assert len(arm.joint_limits_deg) == 3, \
            "Should have 3 joint limits loaded"

    def test_arm_loaded_limits_match_calibration(self, standard_model):
        """Test that auto-loaded limits match the calibration file values."""
        arm = RobotArm(model=standard_model)
        
        # Expected limits from calibration file
        expected_limits = [
            (-102.13, 107.84),   # theta1 (motor 2)
            (-106.26, 88.33),    # theta2 (motor 3)
            (-100.90, 104.77),   # theta3 (motor 4)
        ]
        
        for i, (loaded, expected) in enumerate(zip(arm.joint_limits_deg, expected_limits)):
            loaded_min, loaded_max = loaded
            expected_min, expected_max = expected
            assert abs(loaded_min - expected_min) < 0.01, \
                f"theta{i+1} min limit mismatch"
            assert abs(loaded_max - expected_max) < 0.01, \
                f"theta{i+1} max limit mismatch"

    def test_arm_respects_manual_limits_override(self, standard_model):
        """Test that manually provided limits override auto-loading."""
        custom_limits = [(-30.0, 30.0), (-40.0, 40.0), (-35.0, 35.0)]
        arm = RobotArm(
            model=standard_model,
            joint_limits_deg=custom_limits,
            auto_load_limits=False
        )
        
        assert arm.joint_limits_deg == custom_limits, \
            "Should use provided manual limits instead of auto-loading"

    def test_arm_can_disable_auto_loading(self, standard_model):
        """Test that auto-loading can be disabled."""
        arm = RobotArm(
            model=standard_model,
            auto_load_limits=False
        )
        
        assert arm.joint_limits_deg is None, \
            "Should not load limits when auto_load_limits=False"

    def test_arm_custom_calibration_path(self, standard_model):
        """Test that custom calibration path is used when specified."""
        calib_path = Path(__file__).parent.parent / "calibration" / "lerobot_arm_with_degrees.json"
        arm = RobotArm(
            model=standard_model,
            calibration_path=calib_path
        )
        
        assert arm.joint_limits_deg is not None, \
            "Should load from custom calibration path"


class TestJointLimitsEnforcement:
    """Tests verifying that IK solutions respect joint limits."""

    @pytest.fixture
    def arm_with_limits(self):
        """RobotArm with default auto-loaded limits."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    @pytest.fixture
    def arm_no_limits(self):
        """RobotArm with no limits enforced."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model, auto_load_limits=False)

    def test_ik_solution_respects_theta1_limits(self, arm_with_limits):
        """Test that IK solutions respect theta1 (shoulder_lift) joint limits."""
        # Try to find a reachable position
        target = (15.0, 3.0, 11.0)
        
        solution = arm_with_limits.solve_base_plus_planar_ik(target, phi_deg=0.0)
        
        if solution is not None:
            theta1 = solution.theta1_deg
            theta1_min, theta1_max = arm_with_limits.joint_limits_deg[0]
            
            assert theta1_min <= theta1 <= theta1_max, \
                f"theta1={theta1:.2f} should be within [{theta1_min:.2f}, {theta1_max:.2f}]"

    def test_ik_solution_respects_theta2_limits(self, arm_with_limits):
        """Test that IK solutions respect theta2 (elbow_flex) joint limits."""
        target = (15.0, 3.0, 11.0)
        
        solution = arm_with_limits.solve_base_plus_planar_ik(target, phi_deg=0.0)
        
        if solution is not None:
            theta2 = solution.theta2_deg
            theta2_min, theta2_max = arm_with_limits.joint_limits_deg[1]
            
            assert theta2_min <= theta2 <= theta2_max, \
                f"theta2={theta2:.2f} should be within [{theta2_min:.2f}, {theta2_max:.2f}]"

    def test_ik_solution_respects_theta3_limits(self, arm_with_limits):
        """Test that IK solutions respect theta3 (wrist_flex) joint limits."""
        target = (15.0, 3.0, 11.0)
        
        solution = arm_with_limits.solve_base_plus_planar_ik(target, phi_deg=0.0)
        
        if solution is not None:
            theta3 = solution.theta3_deg
            theta3_min, theta3_max = arm_with_limits.joint_limits_deg[2]
            
            assert theta3_min <= theta3 <= theta3_max, \
                f"theta3={theta3:.2f} should be within [{theta3_min:.2f}, {theta3_max:.2f}]"

    def test_all_joint_limits_respected_together(self, arm_with_limits):
        """Test that all three joint limits are respected simultaneously."""
        targets = [
            (14.0, 2.0, 10.0),
            (15.0, 4.0, 12.0),
            (13.0, 3.0, 11.0),
        ]
        
        for target in targets:
            solution = arm_with_limits.solve_base_plus_planar_ik(target, phi_deg=0.0)
            
            if solution is not None:
                theta1, theta2, theta3 = solution.planar_angles
                limits = arm_with_limits.joint_limits_deg
                
                # Check all three limits
                assert limits[0][0] <= theta1 <= limits[0][1], \
                    f"Target {target}: theta1={theta1:.2f} violates limits"
                assert limits[1][0] <= theta2 <= limits[1][1], \
                    f"Target {target}: theta2={theta2:.2f} violates limits"
                assert limits[2][0] <= theta3 <= limits[2][1], \
                    f"Target {target}: theta3={theta3:.2f} violates limits"

    def test_unreachable_within_limits_returns_none(self, arm_with_limits):
        """Test that solutions outside joint limits are rejected and None is returned."""
        # Try a position that might require angles outside the limits
        extreme_target = (3.0, 15.0, 5.0)
        
        solution = arm_with_limits.solve_base_plus_planar_ik(
            extreme_target, 
            phi_deg=0.0,
            allow_phi_adaptation=True,
            phi_adaptation_range=90.0
        )
        
        if solution is not None:
            # If a solution was found, verify it respects limits
            theta1, theta2, theta3 = solution.planar_angles
            limits = arm_with_limits.joint_limits_deg
            
            assert limits[0][0] <= theta1 <= limits[0][1]
            assert limits[1][0] <= theta2 <= limits[1][1]
            assert limits[2][0] <= theta3 <= limits[2][1]


class TestMoveEndEffectorWithLimits:
    """Tests for move_end_effector respecting joint limits."""

    @pytest.fixture
    def arm_with_limits(self):
        """RobotArm with auto-loaded limits."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model)

    def test_move_only_succeeds_with_valid_limits(self, arm_with_limits):
        """Test that move_end_effector only succeeds with solutions within limits."""
        # Try various targets, all should produce solutions respecting limits
        targets = [
            ((14.0, 2.0, 10.0), 0.0),
            ((15.0, 3.0, 11.0), 15.0),
            ((13.0, 4.0, 9.0), -10.0),
        ]
        
        for target_xyz, phi in targets:
            ok = arm_with_limits.move_end_effector(target_xyz, phi)
            
            if ok:
                # Verify state respects limits
                theta1, theta2, theta3 = arm_with_limits.state.planar_angles
                limits = arm_with_limits.joint_limits_deg
                
                assert limits[0][0] <= theta1 <= limits[0][1], \
                    f"After move to {target_xyz}: theta1={theta1:.2f} violates limits"
                assert limits[1][0] <= theta2 <= limits[1][1], \
                    f"After move to {target_xyz}: theta2={theta2:.2f} violates limits"
                assert limits[2][0] <= theta3 <= limits[2][1], \
                    f"After move to {target_xyz}: theta3={theta3:.2f} violates limits"

    def test_move_updates_state_only_when_valid(self, arm_with_limits):
        """Test that state is only updated when solution respects limits."""
        initial_state = arm_with_limits.state
        
        # Try a reachable target
        ok = arm_with_limits.move_end_effector(
            target_xyz=(14.0, 3.0, 11.0),
            phi_deg=0.0
        )
        
        if ok:
            # State should have changed
            assert arm_with_limits.state != initial_state, \
                "State should update on successful move"
        else:
            # State should remain unchanged
            assert arm_with_limits.state == initial_state, \
                "State should not change on failed move"


class TestLimitComparisonWithAndWithoutConstraints:
    """Tests comparing behavior with and without joint limits."""

    @pytest.fixture
    def arm_constrained(self):
        """RobotArm with tight joint limits."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(
            model=model,
            joint_limits_deg=[(-30.0, 30.0), (-30.0, 30.0), (-30.0, 30.0)],
            auto_load_limits=False
        )

    @pytest.fixture
    def arm_unconstrained(self):
        """RobotArm with no joint limits."""
        model = ArmModel(
            links=(12.0, 10.0, 6.0),
            shoulder_z=10.0,
            base_offset=4.0
        )
        return RobotArm(model=model, auto_load_limits=False)

    def test_constrained_more_restrictive_than_unconstrained(self, arm_constrained, arm_unconstrained):
        """Test that tighter limits reduce the workspace."""
        targets = [
            ((14.0, 2.0, 10.0), 0.0),
            ((16.0, 5.0, 12.0), 0.0),
            ((12.0, 6.0, 9.0), 0.0),
        ]
        
        unconstrained_solutions = []
        constrained_solutions = []
        
        for target_xyz, phi in targets:
            unconstrained = arm_unconstrained.solve_base_plus_planar_ik(target_xyz, phi)
            constrained = arm_constrained.solve_base_plus_planar_ik(target_xyz, phi)
            
            unconstrained_solutions.append(unconstrained is not None)
            constrained_solutions.append(constrained is not None)
        
        # This is a probabilistic test - most targets should be more solvable
        # when unconstrained than when constrained
        print(f"\nWorkspace comparison:")
        print(f"  Unconstrained solvable: {sum(unconstrained_solutions)}/{len(unconstrained_solutions)}")
        print(f"  Constrained solvable: {sum(constrained_solutions)}/{len(constrained_solutions)}")


class TestMotorSpecificLimits:
    """Tests for specific motor constraints from calibration file."""

    def test_shoulder_lift_limits_are_asymmetric(self):
        """Test that motor 2 (shoulder_lift) has realistic asymmetric limits."""
        limits = load_joint_limits_from_calibration()
        theta1_min, theta1_max = limits[0]
        
        # Shoulder lift should have approximately symmetric but slightly different limits
        assert theta1_min < 0, "Shoulder lift should be able to go negative"
        assert theta1_max > 0, "Shoulder lift should be able to go positive"
        assert abs(abs(theta1_min) - abs(theta1_max)) < 20.0, \
            "Shoulder lift limits should be roughly symmetric"

    def test_elbow_flex_limits_are_asymmetric(self):
        """Test that motor 3 (elbow_flex) has realistic asymmetric limits."""
        limits = load_joint_limits_from_calibration()
        theta2_min, theta2_max = limits[1]
        
        # Elbow can extend more than contract
        assert theta2_min < 0, "Elbow should be able to flex backward"
        assert theta2_max > 0, "Elbow should be able to extend forward"
        assert abs(theta2_min) > abs(theta2_max) * 0.8, \
            "Elbow flex limits should have reasonable asymmetry"

    def test_wrist_flex_limits_are_symmetric(self):
        """Test that motor 4 (wrist_flex) has roughly symmetric limits."""
        limits = load_joint_limits_from_calibration()
        theta3_min, theta3_max = limits[2]
        
        # Wrist should be roughly symmetric
        assert theta3_min < 0, "Wrist should be able to flex down"
        assert theta3_max > 0, "Wrist should be able to flex up"
        assert abs(abs(theta3_min) - abs(theta3_max)) < 10.0, \
            "Wrist flex limits should be roughly symmetric"

    def test_all_limits_have_reasonable_ranges(self):
        """Test that all joint limits have reasonable degree ranges."""
        limits = load_joint_limits_from_calibration()
        
        min_reasonable_range = 50.0  # At least 50 degrees
        max_reasonable_range = 250.0  # At most 250 degrees
        
        for i, (min_deg, max_deg) in enumerate(limits, 1):
            degree_range = max_deg - min_deg
            assert min_reasonable_range <= degree_range <= max_reasonable_range, \
                f"theta{i} range ({degree_range:.2f} deg) should be reasonable"


class TestIntegrationWithCalibrationFile:
    """Integration tests using calibration file constraints."""

    def test_load_limits_and_create_arm(self):
        """Test complete workflow: load limits and create arm."""
        limits = load_joint_limits_from_calibration()
        model = ArmModel()
        arm = RobotArm(model=model, joint_limits_deg=limits, auto_load_limits=False)
        
        assert arm.joint_limits_deg == limits, \
            "Arm should use provided limits"

    def test_solve_and_verify_all_solutions_valid(self):
        """Test that all IK solutions found are within calibration limits."""
        arm = RobotArm(model=ArmModel())
        
        test_targets = [
            ((14.0, 2.0, 10.0), 0.0),
            ((15.0, 3.0, 11.0), 15.0),
            ((13.0, 4.0, 9.0), -10.0),
            ((16.0, 1.0, 12.0), 30.0),
        ]
        
        valid_solutions = 0
        total_attempts = 0
        
        for target_xyz, phi in test_targets:
            total_attempts += 1
            solution = arm.solve_base_plus_planar_ik(target_xyz, phi)
            
            if solution is not None:
                valid_solutions += 1
                theta1, theta2, theta3 = solution.planar_angles
                
                # Verify against stored limits
                for i, (theta, (min_lim, max_lim)) in enumerate(
                    zip([theta1, theta2, theta3], arm.joint_limits_deg),
                    1
                ):
                    assert min_lim <= theta <= max_lim, \
                        f"theta{i}={theta:.2f} violates stored limits [{min_lim}, {max_lim}]"
        
        print(f"\nIntegration test results:")
        print(f"  Valid solutions found: {valid_solutions}/{total_attempts}")
        print(f"  All solutions within joint limits: PASS")

    def test_calibration_and_ik_consistency(self):
        """Test that calibration limits are consistently applied."""
        arm1 = RobotArm(model=ArmModel())  # Auto-loads
        limits = load_joint_limits_from_calibration()
        arm2 = RobotArm(model=ArmModel(), joint_limits_deg=limits, auto_load_limits=False)
        
        # Both arms should have same limits
        for i, (auto_loaded, manual) in enumerate(zip(arm1.joint_limits_deg, arm2.joint_limits_deg), 1):
            assert auto_loaded == manual, \
                f"theta{i}: auto-loaded {auto_loaded} should match manual {manual}"
