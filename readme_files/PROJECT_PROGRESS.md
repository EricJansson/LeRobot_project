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
| Offline checks | Historical baseline: 159 passed on 2026-10-30; venv verification is recorded in the README |
| Current runtime | Windows: Python 3.11 / LeRobot 0.4.0. Pi: existing Python 3.13 / LeRobot 0.6.1 / CPU PyTorch environment. Shared imports, preserved PID tuning, and project-local hardware calibration |
| Headless Pi control | IK and plan accept --no-dashboard; local gamepad control without dashboard IPC; browser dashboard deferred |

Plan playback interpolates recorded joint targets and pauses; it is not collision-aware planning. The guardrail/workspace helper files are not wired into active teleop. Tests cover custom mathematics/model behavior and do not establish hardware or end-to-end safety.

## Planned direction

Keep working modes intact and add a separate upstream LeRobot URDF-based FK/IK path. LeRobot's installed RobotKinematics class exists, but placo is absent and no repository control path uses it. All 13 URDF-referenced meshes were acquired from a pinned upstream revision with license/checksums; see [asset provenance](../assets/README.md). The local URDF is structurally identical to that upstream model. Its geometry/conventions still need checking against the physical arm.

The Pi migration preserves tuning/calibration and is validated offline; hardware
confirmation remains pending. With the owner's approval, the project calibration
was synchronized from the completed Pi LeRobot calibration and the degree-limit
export regenerated. The controller loads calibration directly from the project. No gamepad was exposed to
the verification session. No motor connections or
physical commands were issued. The next implementation milestones include
separate upstream IK integration, collision checking, and PC/phone web access.
See [TODO](../TODO.md) for tasks; web transport/UI choices remain deferred.
