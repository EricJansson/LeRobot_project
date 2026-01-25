"""
Comprehensive tests for the IK_calculations module.
Tests forward and inverse kinematics calculations for 3-DOF planar RRR robot.
"""

import pytest
import math
import sys
from pathlib import Path

# Add the robot_program module to path
sys.path.insert(0, str(Path(__file__).parent.parent / "robot_program"))

from robot_program.utils.IK_calculations import (
    ik_3dof_planar_all_deg,
    select_ik_solution_deg,
    _wrap_deg,
    _angle_dist_deg,
)

def fk_3dof_planar_deg(links, angles_deg):
    """
    Forward kinematics for 3-DOF planar arm (degrees).
    Used ONLY for testing IK correctness.
    """
    l1, l2, l3 = links
    t1, t2, t3 = map(math.radians, angles_deg)

    t12 = t1 + t2
    t123 = t12 + t3

    x = (
        l1 * math.cos(t1)
        + l2 * math.cos(t12)
        + l3 * math.cos(t123)
    )
    y = (
        l1 * math.sin(t1)
        + l2 * math.sin(t12)
        + l3 * math.sin(t123)
    )

    phi_deg = math.degrees(t123)
    return x, y, phi_deg


class TestWrapDeg:
    """Tests for angle wrapping utility."""

    def test_wrap_positive_angle(self):
        """Test wrapping angles > 180 degrees."""
        assert _wrap_deg(270) == -90
        assert _wrap_deg(360) == 0
        assert _wrap_deg(450) == 90

    def test_wrap_negative_angle(self):
        """Test wrapping negative angles."""
        assert _wrap_deg(-270) == 90
        assert _wrap_deg(-180) == -180
        assert _wrap_deg(-190) == 170

    def test_wrap_zero_and_small(self):
        """Test wrapping angles close to zero."""
        assert _wrap_deg(0) == 0
        assert _wrap_deg(45) == 45
        assert _wrap_deg(-45) == -45

    def test_wrap_boundary(self):
        """Test boundary values [-180, 180)."""
        result = _wrap_deg(180)
        assert -180 <= result < 180


class TestAngleDist:
    """Tests for angular distance calculation."""

    def test_same_angle(self):
        """Test distance between identical angles."""
        assert _angle_dist_deg(45, 45) == 0
        assert _angle_dist_deg(0, 0) == 0

    def test_simple_distance(self):
        """Test straightforward angular distances."""
        assert _angle_dist_deg(0, 90) == 90
        assert _angle_dist_deg(45, 135) == 90

    def test_wraparound_distance(self):
        """Test distances that wrap around 180 degrees."""
        assert _angle_dist_deg(170, -170) == pytest.approx(20, abs=1e-6)
        assert _angle_dist_deg(-175, 175) == pytest.approx(10, abs=1e-6)

    def test_symmetric(self):
        """Test that distance is symmetric."""
        assert _angle_dist_deg(10, 50) == _angle_dist_deg(50, 10)
        assert _angle_dist_deg(-90, 90) == _angle_dist_deg(90, -90)


class TestIK3DOFPlanarAllDeg:
    """Tests for the main IK solver."""

    @pytest.fixture
    def standard_arm(self):
        """Standard 3-DOF arm with equal link lengths."""
        return (1.0, 1.0, 0.5)  # l1, l2, l3

    @pytest.fixture
    def unequal_arm(self):
        """Arm with unequal link lengths."""
        return (1.5, 1.0, 0.75)

    def test_reachable_center_position(self, standard_arm):
        """Test IK for a position within reach at center."""
        l1, l2, l3 = standard_arm
        # Target: close to arm reach
        x, y = 1.5, 0.5
        phi_deg = 0

        solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)

        assert len(solutions) > 0, "Should have at least one solution"
        for sol in solutions:
            assert len(sol) == 3, "Each solution should have 3 angles"
            for angle in sol:
                assert isinstance(angle, float), "Angles should be floats"

    def test_unreachable_position(self, standard_arm):
        l1, l2, l3 = standard_arm
        phi_deg = 0

        # Place wrist beyond reach
        wx = l1 + l2 + 0.5
        wy = 0.0

        x = wx + l3
        y = wy

        solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)
        assert solutions == []


    def test_multiple_solutions(self, standard_arm):
        """Test that arm returns multiple solutions (elbow-up/down)."""
        x, y = 1.2, 0.3
        phi_deg = 0

        solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)

        # Typically should have 2 solutions (elbow up and down)
        assert len(solutions) == 2, "Should have 2 solutions for typical reachable position"

    def test_solution_continuity_check(self, standard_arm):
        """Test that solutions are continuous (no NaN or Inf)."""
        x, y = 1.5, 0.2
        phi_deg = 45

        solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)

        for sol in solutions:
            for angle in sol:
                assert math.isfinite(angle), f"Angle should be finite, got {angle}"


    def test_unequal_links(self, unequal_arm):
        solutions = ik_3dof_planar_all_deg(unequal_arm, 1.8, 0.5, 30)
        assert isinstance(solutions, list)


    def test_zero_position_and_orientation(self, standard_arm):
        """Test IK for zero position with zero orientation."""
        solutions = ik_3dof_planar_all_deg(standard_arm, 0, 0, 0)

        # Arm configuration to reach origin
        assert isinstance(solutions, list)

    def test_ik_fk_consistency(self, standard_arm):
        x, y, phi_deg = 1.3, 0.4, 30

        solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)
        assert solutions, "Expected at least one IK solution"

        for sol in solutions:
            x_fk, y_fk, phi_fk = fk_3dof_planar_deg(standard_arm, sol)

            assert x_fk == pytest.approx(x, abs=1e-6)
            assert y_fk == pytest.approx(y, abs=1e-6)
            assert _wrap_deg(phi_fk - phi_deg) == pytest.approx(0, abs=1e-6)


class TestSelectIKSolution:
    """Tests for IK solution selection policy."""

    @pytest.fixture
    def standard_arm(self):
        return (1.0, 1.0, 0.5)

    @pytest.fixture
    def sample_solutions(self):
        """Two sample solutions representing elbow-up and elbow-down."""
        return [
            (30.0, 60.0, 10.0),   # elbow-up (positive θ2)
            (45.0, -50.0, 25.0),  # elbow-down (negative θ2)
        ]

    def test_select_from_empty_solutions(self):
        """Test selection with no solutions."""
        result = select_ik_solution_deg([])
        assert result is None

    def test_select_single_solution(self, sample_solutions):
        """Test selection with only one valid solution."""
        result = select_ik_solution_deg([sample_solutions[0]])
        assert result == sample_solutions[0]

    def test_continuity_preference(self, sample_solutions):
        """Test that solution continues from previous angles."""
        prev_angles = (32.0, 58.0, 12.0)  # Close to first solution

        result = select_ik_solution_deg(sample_solutions, prev_angles_deg=prev_angles)

        # Should prefer solution closer to previous angles
        assert result is not None
        assert result == sample_solutions[0]

    def test_joint_limits_filtering(self, sample_solutions):
        """Test that solutions outside joint limits are filtered."""
        joint_limits = [(-30, 60), (-70, 70), (-30, 30)]

        result = select_ik_solution_deg(
            sample_solutions,
            joint_limits_deg=joint_limits
        )

        # First solution should pass, second might not
        assert result is not None

    def test_elbow_preference_up(self, sample_solutions):
        """Test elbow-up preference (positive θ2)."""
        result = select_ik_solution_deg(
            sample_solutions,
            elbow_preference="up"
        )

        assert result == sample_solutions[0], "Should prefer elbow-up"

    def test_elbow_preference_down(self, sample_solutions):
        """Test elbow-down preference (negative θ2)."""
        result = select_ik_solution_deg(
            sample_solutions,
            elbow_preference="down"
        )

        assert result == sample_solutions[1], "Should prefer elbow-down"

    def test_hysteresis_effect(self, sample_solutions):
        """Test hysteresis prevents unnecessary flipping."""
        prev_angles = (30.0, 65.0, 9.0)  # Very close to first solution

        # With high hysteresis, should stay with first solution
        result = select_ik_solution_deg(
            sample_solutions,
            prev_angles_deg=prev_angles,
            hysteresis=0.85
        )

        assert result == sample_solutions[0]

    def test_no_valid_solutions_after_filtering(self, sample_solutions):
        """Test when all solutions are filtered out."""
        joint_limits = [(0, 10), (0, 10), (0, 10)]  # Very restrictive

        result = select_ik_solution_deg(
            sample_solutions,
            joint_limits_deg=joint_limits
        )

        assert result is None

    def test_deterministic_fallback(self, sample_solutions):
        """Test deterministic selection when continuity unavailable."""
        result1 = select_ik_solution_deg(sample_solutions)
        result2 = select_ik_solution_deg(sample_solutions)

        assert result1 == result2, "Should deterministically select first candidate"

    def test_combined_filters(self, sample_solutions):
        """Test combination of multiple filters."""
        prev_angles = (35.0, 55.0, 15.0)
        joint_limits = [(-60, 60), (-100, 100), (-40, 40)]

        result = select_ik_solution_deg(
            sample_solutions,
            prev_angles_deg=prev_angles,
            joint_limits_deg=joint_limits,
            elbow_preference="up",
            hysteresis=0.8
        )

        assert result is not None


class TestIntegration:
    """Integration tests combining IK solving and solution selection."""

    @pytest.fixture
    def standard_arm(self):
        return (1.0, 1.0, 0.5)

    def test_solve_and_select_workflow(self, standard_arm):
        """Test complete workflow: solve IK, then select solution."""
        x, y = 1.3, 0.3
        phi_deg = 20

        # Step 1: Get all solutions
        solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)

        # Step 2: Select one
        if solutions:
            selected = select_ik_solution_deg(solutions)
            assert selected is not None
            assert len(selected) == 3

    def test_continuous_motion_sequence(self, standard_arm):
        """Test a sequence of positions with continuity."""
        targets = [
            (1.2, 0.3, 0),
            (1.3, 0.4, 15),
            (1.1, 0.2, -10),
        ]

        prev_solution = None

        for x, y, phi_deg in targets:
            solutions = ik_3dof_planar_all_deg(standard_arm, x, y, phi_deg)

            if solutions:
                selected = select_ik_solution_deg(
                    solutions,
                    prev_angles_deg=prev_solution
                )

                assert selected is not None
                prev_solution = selected

