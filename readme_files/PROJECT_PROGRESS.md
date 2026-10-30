# Project status and history

[Home](../README.md) | [Architecture](ARCHITECTURE.md) | [TODO](../TODO.md)

Reviewed 2026-10-30. This replaces an outdated phase checklist. Hardware status comes from the owner; implementation status comes from source inspection. No robot commands were executed during the documentation review.

## Historical work

- The owner printed and assembled the SO-101 and explored LeRobot setup/calibration.
- web_sim was an early standalone JavaScript FK/IK visualization. It is now outdated and retained as historical material, not the current simulation or validation workflow.
- A custom Python model was developed around base yaw and three planar joints, with motor-frame signs/offsets, calibration-derived limits, solution selection, and optional pitch adaptation.
- Earlier notes dated 2026-02-18 describe reversed planar motors and frame corrections. Those signs/offsets are present in the current model.
- Earlier notes reported a remaining second-pitch-joint offset and end-effector height error. The owner is uncertain whether it remains; revalidation is required.

## Implemented now

| Area | Evidence / status |
| --- | --- |
| Direct joint gamepad driving | scripts.teleop_axis_control; used on hardware |
| Custom Cartesian IK driving | scripts.main_teleop_ik and reusable teleop_ik_control; used on hardware |
| Plan recording/playback | scripts.main_teleop_plan and reusable teleop_plan_control; used on hardware |
| Dashboard | Separate pygame process, manual joint targets, telemetry, plan editing |
| Hardware adapter | RobotArmController integrates custom model with SO101Follower |
| Offline checks | 159 tests passed under Python 3.12.9 and robot-environment Python 3.10.19 |
| Current runtime | Native Windows/Conda, LeRobot 0.4.0, pygame 2.6.1; external installed-package PID changes |

Plan playback interpolates recorded joint targets and pauses; it is not collision-aware planning. The guardrail/workspace helper files are not wired into active teleop. Tests cover custom mathematics/model behavior and do not establish hardware or end-to-end safety.

## Planned direction

Keep working modes intact and add a separate upstream LeRobot URDF-based FK/IK path. LeRobot's installed RobotKinematics class exists, but placo is absent and no repository control path uses it. The URDF has collision references but its meshes are missing, and its geometry/conventions must be checked against the physical arm.

The next milestones are reproducible Windows venv setup with preserved tuning/calibration, separate upstream IK integration, collision checking across motions, then phone control. See [TODO](../TODO.md) for ordered tasks. Phone transport/UI and collision implementation choices have not yet been made.
