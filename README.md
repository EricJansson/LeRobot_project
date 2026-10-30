# SO-101 robot arm workspace

This personal project controls a physical **SO-101 follower arm** using Python, a PlayStation-style gamepad, and LeRobot's Feetech hardware interface. It includes direct joint control, custom Cartesian inverse kinematics (IK), recorded motion playback, a desktop dashboard, calibration data, and offline tests.

The next direction is a **separate control path using LeRobot's URDF-based FK/IK**, followed by collision checking and phone control. Preserve the existing implementations while developing that path. Current IK teleoperation uses this repository's custom solver, not LeRobot's `RobotKinematics`.

## Start here

- [Connection and run guide](readme_files/CONNECT_SETUP.md): Windows/Conda setup, alternate WSL instructions, and commands.
- [Architecture and control reference](readme_files/ARCHITECTURE.md): code ownership, controls, units, state, dashboard communication, and extension boundaries.
- [Joint limits and calibration](readme_files/JOINT_LIMITS_IMPLEMENTATION.md): current limits and enforcement.
- [Project status](readme_files/PROJECT_PROGRESS.md), [TODO](TODO.md), and [sources](readme_files/SOURCES.md).
- [Phi adaptation tests](readme_files/PHI_ADAPTATION_TESTS.md) and [historical physical-pose observations](readme_files/helper_files/robot_orientations.md).

## Implementation status

| Capability | Current status |
| --- | --- |
| Direct gamepad joint control | Used successfully on hardware; talks directly to SO101Follower |
| Cartesian gamepad IK | Used successfully on hardware; custom base-yaw + three-joint planar solver |
| Plan recording/playback | Used successfully on hardware; interpolated joint MOVE actions and timed PAUSE actions |
| Desktop dashboard | Gamepad display, manual joint targets, telemetry; plan editing in plan mode |
| Offline tests | 159 passing tests in both inspected Python environments on 2026-10-30 |
| web_sim | Outdated standalone browser experiment; retained for historical reference |
| LeRobot URDF FK/IK integration | Planned; upstream class exists but is not called by this repository |
| Collision avoidance / phone control | Planned; neither is implemented in the active control paths |

Hardware use is confirmed by the owner. The documentation review did not connect to or move the arm. Passing mathematical tests does not validate physical accuracy or collision safety.

## Run an existing mode

From the repository root in the existing environment, choose **one**:

```powershell
conda activate lerobot
python -m scripts.teleop_axis_control --port COM3 --index 0
```

```powershell
python -m scripts.main_teleop_ik --port COM3 --index 0
```

```powershell
python -m scripts.main_teleop_plan --port COM3 --index 0
```

Replace COM3 with the actual adapter port. Windows automatic port selection currently returns COM3; it does not discover the arm. IK and plan launch their dashboard automatically. Only one process should own the robot connection, and IK/plan sessions also share runtime files. Read the [run guide](readme_files/CONNECT_SETUP.md) before operating the arm.

## Architecture

```text
Gamepad -> gamepad_mapper -> teleop loop
  axis mode --------------------------------> SO101Follower -> Feetech motors
  IK mode -> custom RobotArm -> RobotArmController -> SO101Follower
  plan mode -> recorded joints / interpolation -> RobotArmController

Dashboard <-> .runtime JSON files <-> IK or plan teleop process
```

RobotArm is a hardware-independent model: solving IK changes internal state. RobotArmController connects it to LeRobot, reads hardware positions, and sends joint commands. The dashboard never opens the serial port. Plan mode uses custom IK for manual driving but joint interpolation for playback.

| Location | Responsibility |
| --- | --- |
| [scripts/](scripts/) | Mode entry points, reusable teleop loops, dashboard/panels, WSL attachment helper |
| [robot_program/](robot_program/) | Custom model/controller; older keyboard/input experiments; motor and environment diagnostics |
| [robot_program/utils/](robot_program/utils/) | Custom FK/IK, port helpers, standalone guardrail/workspace helpers |
| [controller/input/](controller/input/) | Pygame polling, bindings, input-state profile, example |
| [controller/scripts/run_teleop.py](controller/scripts/run_teleop.py) | Input-profile example; does not send commands to the arm |
| [calibration/](calibration/) | Degree-limit export and conversion utility; distinct from LeRobot's calibration cache |
| [so101_new_calib.urdf](so101_new_calib.urdf) | Description for future integration; referenced assets/*.stl files are missing |
| [tests/](tests/) | Offline custom solver, model, and joint-limit tests |
| [web_sim/](web_sim/) | Early JavaScript FK/IK visualization, disconnected from Python/hardware |
| [readme_files/](readme_files/) | Setup, implementation references, status, sources, historical design material |
| .runtime/ | Ignored dashboard IPC and default saved-plan location |

## Environment and reproducibility

The inspected Windows lerobot Conda environment contains **Python 3.10.19, LeRobot 0.4.0, pygame 2.6.1**. Moving to a Windows venv is future work. There is no complete runtime manifest or lockfile; [requirements-dev.txt](requirements-dev.txt) supplies development tools, not the robot runtime.

The installed LeRobot package contains local motor PID changes outside this repository. Reinstalling the same version will not reproduce that tuning. The [setup guide](readme_files/CONNECT_SETUP.md#external-lerobot-customization) records it for deliberate preservation during migration.

LeRobot 0.4.0 includes `lerobot.model.kinematics.RobotKinematics`, backed by optional placo; placo was absent during inspection. Neither the URDF nor that solver automatically supplies collision avoidance. Missing meshes, model/calibration alignment, and dependency/platform compatibility need resolution before claiming the new path works.

## Tests and diagnostics

Run offline tests from the root:

```powershell
python -m pytest -q -o addopts= --tb=short
```

Baseline on 2026-10-30: **159 passed** under default Python 3.12.9 and the robot environment's Python 3.10.19. These tests cover custom mathematics/model behavior, not hardware, full dashboard/plan integration, upstream IK, or collision checking.

`python -m robot_program.debug_env` reports environment information. `python -X utf8 validate_calibration.py` checks exported raw-to-degree arithmetic, not physical calibration. The motor reader is a hardware operation described in the run guide.

## Boundaries for future work

- Keep existing mode entry points, custom kinematics, calibration values, and dashboard behavior intact while adding the new path.
- Unused guardrail helpers and the old web simulator provide no runtime protection. Current controls have joint/input checks but no self-, table-, or obstacle-collision checking.
- Manual LB gating is not a universal motion stop: presets and plan playback can continue after LB release. Dashboard **Act** submits a direct joint target. See [motion protection boundaries](readme_files/ARCHITECTURE.md#motion-protection-boundaries).
- Verify conversions: custom XYZ uses centimeters, URDF geometry uses meters, joint conventions need alignment, and LeRobot's gripper channel is normalized rather than degrees.
- Follow [TODO](TODO.md). The previously reported second-pitch-joint/height discrepancy needs revalidation; it is not a confirmed current defect.

Last reviewed: 2026-10-30.
