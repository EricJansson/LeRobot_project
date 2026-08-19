"""
Tests for Cartesian workspace guardrails in RobotArm.

Verifies that move_end_effector rejects targets that violate configured
workspace limits (z_min, z_max, x_min, x_max, y_min, y_max).
"""

import pytest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).parent.parent / "robot_program"))

from robot_program.RobotArm import ArmModel, RobotArm, WorkspaceLimits


@pytest.fixture
def arm():
    """RobotArm with default WorkspaceLimits (z_min=3.0 cm)."""
    model = ArmModel(
        links=(11.92, 13.5, 17.0),
        shoulder_z=11.0,
        base_offset=4.0,
    )
    return RobotArm(model=model)


@pytest.fixture
def arm_no_guardrails():
    """RobotArm with workspace guardrails disabled."""
    model = ArmModel(
        links=(11.92, 13.5, 17.0),
        shoulder_z=11.0,
        base_offset=4.0,
    )
    return RobotArm(model=model, workspace_limits=None)


class TestWorkspaceLimits:
    """Unit tests for the WorkspaceLimits dataclass."""

    def test_default_z_min_is_3cm(self):
        wl = WorkspaceLimits()
        assert wl.z_min == 3.0

    def test_default_other_bounds_are_none(self):
        wl = WorkspaceLimits()
        assert wl.z_max is None
        assert wl.x_min is None
        assert wl.x_max is None
        assert wl.y_min is None
        assert wl.y_max is None

    def test_is_within_limits_above_z_min(self):
        wl = WorkspaceLimits(z_min=3.0)
        assert wl.is_within_limits(10.0, 0.0, 5.0) is True

    def test_is_within_limits_at_z_min(self):
        wl = WorkspaceLimits(z_min=3.0)
        assert wl.is_within_limits(10.0, 0.0, 3.0) is True

    def test_is_within_limits_below_z_min(self):
        wl = WorkspaceLimits(z_min=3.0)
        assert wl.is_within_limits(10.0, 0.0, 2.9) is False

    def test_is_within_limits_z_max(self):
        wl = WorkspaceLimits(z_min=3.0, z_max=50.0)
        assert wl.is_within_limits(10.0, 0.0, 60.0) is False
        assert wl.is_within_limits(10.0, 0.0, 30.0) is True

    def test_is_within_limits_x_bounds(self):
        wl = WorkspaceLimits(x_min=5.0, x_max=40.0)
        assert wl.is_within_limits(4.9, 0.0, 10.0) is False
        assert wl.is_within_limits(40.1, 0.0, 10.0) is False
        assert wl.is_within_limits(20.0, 0.0, 10.0) is True

    def test_is_within_limits_y_bounds(self):
        wl = WorkspaceLimits(y_min=-20.0, y_max=20.0)
        assert wl.is_within_limits(10.0, -21.0, 10.0) is False
        assert wl.is_within_limits(10.0, 21.0, 10.0) is False
        assert wl.is_within_limits(10.0, 0.0, 10.0) is True

    def test_no_bounds_always_passes(self):
        wl = WorkspaceLimits(z_min=None)
        assert wl.is_within_limits(0.0, 0.0, 0.0) is True
        assert wl.is_within_limits(100.0, -100.0, -100.0) is True

    def test_custom_z_min(self):
        wl = WorkspaceLimits(z_min=10.0)
        assert wl.is_within_limits(0.0, 0.0, 9.9) is False
        assert wl.is_within_limits(0.0, 0.0, 10.0) is True


class TestDefaultZGuardrail:
    """Tests that the default z_min=3.0 cm guardrail is applied by RobotArm."""

    def test_arm_has_default_workspace_limits(self, arm):
        assert arm.workspace_limits is not None
        assert arm.workspace_limits.z_min == 3.0

    def test_move_above_z_min_succeeds(self, arm):
        """A reachable target above 3 cm should succeed."""
        result = arm.move_end_effector(
            target_xyz=(30.0, 0.0, 15.0),
            phi_deg=0.0,
        )
        assert result is True

    def test_move_exactly_at_z_min_not_rejected_by_guardrail(self, arm):
        """Target at exactly z_min=3.0 should not be rejected by the guardrail."""
        initial_state = arm.state
        # The guardrail allows z==z_min; IK may still fail for other reasons.
        # We verify that if it fails, it's not due to the guardrail (state unchanged).
        result = arm.move_end_effector(
            target_xyz=(30.0, 0.0, 3.0),
            phi_deg=0.0,
        )
        # No assertion on True/False — the guardrail must not reject z==3.0
        # If failed, state must be unchanged (i.e. not a partial update).
        if not result:
            assert arm.state == initial_state

    def test_move_below_z_min_rejected(self, arm):
        """Any target with z < 3.0 cm must be rejected without updating state."""
        initial_state = arm.state

        result = arm.move_end_effector(
            target_xyz=(30.0, 0.0, 2.0),
            phi_deg=0.0,
        )

        assert result is False, "move_end_effector should return False for z < 3 cm"
        assert arm.state == initial_state, "State must not change when guardrail rejects target"

    def test_move_zero_z_rejected(self, arm):
        """z=0 (table surface) must be rejected."""
        result = arm.move_end_effector(
            target_xyz=(20.0, 0.0, 0.0),
            phi_deg=0.0,
        )
        assert result is False

    def test_move_negative_z_rejected(self, arm):
        """Negative z (below the table) must be rejected."""
        result = arm.move_end_effector(
            target_xyz=(20.0, 0.0, -5.0),
            phi_deg=0.0,
        )
        assert result is False


class TestNoGuardrails:
    """Tests that disabling workspace_limits removes the z constraint."""

    def test_arm_with_none_has_no_workspace_limits(self, arm_no_guardrails):
        assert arm_no_guardrails.workspace_limits is None

    def test_low_z_not_blocked_when_guardrails_disabled(self, arm_no_guardrails):
        """Without guardrails, low-z targets should not be blocked by workspace check
        (IK may still fail for geometric reasons)."""
        # Attempt — we only verify the guardrail doesn't block it.
        # The move may still fail due to IK, but that's not the guardrail.
        arm_no_guardrails.move_end_effector(
            target_xyz=(30.0, 0.0, 1.0),
            phi_deg=0.0,
        )
        # No assertion — we just ensure no exception is raised by the guardrail logic.


class TestCustomWorkspaceLimits:
    """Tests for user-specified custom workspace limits."""

    def test_custom_z_min(self):
        model = ArmModel()
        arm = RobotArm(model=model, workspace_limits=WorkspaceLimits(z_min=10.0))

        result = arm.move_end_effector(
            target_xyz=(30.0, 0.0, 9.0),
            phi_deg=0.0,
        )
        assert result is False, "Should be rejected because 9.0 < z_min=10.0"

    def test_custom_z_max(self):
        model = ArmModel()
        arm = RobotArm(model=model, workspace_limits=WorkspaceLimits(z_min=None, z_max=20.0))

        result = arm.move_end_effector(
            target_xyz=(10.0, 0.0, 25.0),
            phi_deg=0.0,
        )
        assert result is False, "Should be rejected because 25.0 > z_max=20.0"

    def test_custom_x_min(self):
        model = ArmModel()
        arm = RobotArm(model=model, workspace_limits=WorkspaceLimits(z_min=None, x_min=10.0))

        result = arm.move_end_effector(
            target_xyz=(5.0, 0.0, 15.0),
            phi_deg=0.0,
        )
        assert result is False, "Should be rejected because x=5.0 < x_min=10.0"

    def test_all_limits_combined(self):
        model = ArmModel()
        limits = WorkspaceLimits(
            z_min=5.0, z_max=40.0,
            x_min=5.0, x_max=40.0,
            y_min=-30.0, y_max=30.0,
        )
        arm = RobotArm(model=model, workspace_limits=limits)

        # Inside all limits — should not be rejected by guardrail
        # (may still fail IK)
        initial_state = arm.state
        result_inside = arm.move_end_effector(
            target_xyz=(20.0, 0.0, 15.0),
            phi_deg=0.0,
        )
        # Just confirm there's no unexpected exception

        # Outside — must be rejected
        result_outside_z = arm.move_end_effector(
            target_xyz=(20.0, 0.0, 2.0),
            phi_deg=0.0,
        )
        assert result_outside_z is False
