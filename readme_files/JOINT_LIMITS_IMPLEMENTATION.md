# Joint limits and calibration

[Home](../README.md) | [Architecture](ARCHITECTURE.md) | [Setup](CONNECT_SETUP.md)

Reviewed 2026-11-02 against the current implementation and [degree-limit export](../calibration/lerobot_arm_with_degrees.json). Project calibration was synchronized from the owner-approved Pi LeRobot calibration, and the degree-limit export regenerated. No motor registers were written.

## Two calibration responsibilities

LeRobot loads hardware calibration separately for SO101Follower, ID lerobot_arm.
The repository follower loads calibration/lerobot_arm.json directly from the
project across LeRobot 0.4 and 0.6; an explicit calibration_dir can override
the directory. No user-cache copy is required. Missing/incomplete calibration fails before connecting;
a hardware mismatch rejects automatic recalibration. This stores motor
homing/range information and is not automatically replaced by the repository export.

The repository export contains degree_min/degree_max plus raw motor data. RobotArm loads the three planar limits; RobotArmController requires all six motor entries and uses their bounds on outgoing commands. Its calibration_path argument selects this export, not LeRobot's own hardware calibration file.

calibration/generate_with_degrees.py reads the project hardware calibration/lerobot_arm.json and writes the degree export. Do not run it as a read-only validation step: it rewrites data. Its formula is:

```text
degrees = ((raw_value - 2048) / 4096) * 360
```

validate_calibration.py checks that arithmetic using export metadata. It does not verify homing, physical geometry, or consistency with URDF conventions.

## Current exported bounds

| Motor ID | Name | Minimum | Maximum | Custom model role |
| --- | --- | ---: | ---: | --- |
| 1 | shoulder_pan | -116.02 | 122.34 | base_yaw_deg |
| 2 | shoulder_lift | -107.31 | 102.92 | theta1_deg |
| 3 | elbow_flex | -104.59 | 89.91 | theta2_deg |
| 4 | wrist_flex | -117.51 | 87.80 | theta3_deg |
| 5 | wrist_roll | -180.00 | 179.91 | Independent controller channel |
| 6 | gripper | -38.06 | 90.26 | Independent controller channel; see unit mismatch below |

The table contains exported degree bounds. Body channels use degrees with use_degrees=True, but LeRobot configures the gripper in normalized 0–100 units. RobotArmController applies these gripper degree bounds directly to normalized values; this is an unresolved mismatch, not an established conversion. Axis mode instead clamps the gripper to normalized 0–50.

## Enforcement behavior

load_all_motor_limits raises FileNotFoundError if the export is absent and ValueError for missing degree bounds or required motor entries. Initialization does **not** silently warn and continue without limits. This is not comprehensive schema/range validation.

RobotArm defaults to auto-loading shoulder_lift, elbow_flex, and wrist_flex limits. Explicit joint_limits_deg overrides auto-loading. auto_load_limits=False without explicit bounds allows an unconstrained mathematical model; this is not the hardware-controller default.

The three planar motors use geometric = motor × sign + offset, signs (-1, -1, -1), offsets (90, -90, 0) degrees. IK converts bounds to the solver frame, sorts transformed endpoints, filters planar solutions, and converts chosen angles back. No solution returns None from solving or False from movement without updating model state.

RobotArmController.send_state clamps all six outgoing targets immediately before calling follower.send_action. set_joint_angles also clamps supplied values. Base yaw is not filtered by the planar solver; later clamping can alter the pose that IK intended. Clamping in send_state does not rewrite all internal state fields.

Direct axis control bypasses RobotArmController and does not load this six-channel limit layer. Standalone IK_guardrails and movement_contrainst helpers are not called by the active teleop chain. None of these bounds supplies collision checking.

## Offline use and tests

This example creates only the mathematical model; it does not connect hardware:

```python
from robot_program.RobotArm import ArmModel, RobotArm

arm = RobotArm(model=ArmModel())
print(arm.joint_limits_deg)
# [(-107.31, 102.92), (-104.59, 89.91), (-117.51, 87.80)]
ok = arm.move_end_effector(target_xyz=(18.0, 5.0, 12.0), phi_deg=0.0)
# ok depends on reachability, limits, and permitted pitch adaptation.
```

From the repository root:

```powershell
python -X utf8 validate_calibration.py
python -m pytest tests/test_joint_limits.py -q -o addopts= --tb=short
```

Tests exercise loading, overrides, transformed limits, and custom IK. Passing them does not validate normalized gripper handling, hardware calibration, or collision clearance.
