# Project status and history

> The current PC-led remote IK path is verified in simulation. See [remote IK setup](REMOTE_IK_SETUP.md) for commands and commissioning status.


[Home](../README.md) | [Architecture](ARCHITECTURE.md) | [TODO](../TODO.md)

Original implementation review: 2026-10-30. Documentation reconciled with local source and the proposed plans on 2026-11-05. Hardware status comes from the owner; implementation status comes from source inspection. That documentation review performed no runtime tests, installation, robot commands or motor-register operations. The subsequent offline asset synchronization is recorded below.

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
| Headless Pi control | Custom IK and plan accept --no-dashboard; local gamepad control without dashboard IPC |
| Standalone LeRobot Pi IK | scripts.main_teleop_lerobot_ik; LeRobot 0.6.1 / PlaCo URDF FK/IK; owner confirms working fine motion and extended-arm retraction after removing the step gate |
| Pi browser telemetry | Read-only LAN dashboard in standalone LeRobot IK, enabled by default; measured joints/FK/gripper/gamepad state; not a phone control endpoint |

Plan playback interpolates recorded joint targets and pauses; it is not collision-aware planning. The guardrail/workspace helper files are not wired into active teleop. The offline suite covers custom mathematics/model behavior, mocked integration, input mapping and web telemetry, with conditional LeRobot/PlaCo solver regressions. These checks do not establish hardware or end-to-end safety; see [test scope and dependency skips](../README.md#tests-and-diagnostics).

## PC-led remote IK

**2026-11-06 · OFFLINE VERIFIED.** The Windows launcher, WSL2 LeRobot IK worker, Pi user service, and read-only Windows dashboard run together with a simulated bus. The remote launcher uses a mesh-free URDF. Self-collision checks are outside this implementation’s scope. See [remote IK setup and verification](REMOTE_IK_SETUP.md).

**HARDWARE VERIFIED, initial supervised trial.** The owner confirmed a small sideways move from the Windows gamepad and observed that releasing LB stopped advancement. Other axes, gripper, extended-arm retraction, controller loss, and SSH interruption are pending. See the [commissioning settings and results](REMOTE_IK_SETUP.md#verification-status).

## Previous planned direction (historical snapshot; superseded where conflicting)

Earlier plans centered on Pi-local LeRobot IK, collision checking, presets and phone commands through a Pi dashboard. Standalone Pi IK and read-only browser telemetry were implemented. The [November 5 design](PC_PI_ARCHITECTURAL_REFACTOR.md) records the broader PC-led proposal and deferred work.

## Preserved validation and hardware history

- **2026-11-06 · OFFLINE:** A six-second local simulation sustained 50 updates/s. A 20-second SSH simulation delivered 29.2 updates/s with a 50 Hz Pi cycle and 150 ms watchdog. IK p95 was 1.25 ms, Pi tick wait p95 was 19.0 ms, round trip p95 including that wait was 59.0 ms, and command age p99 was 60.0 ms.
- **2026-10-30 — OFFLINE:** 159 tests passed under Python 3.12.9 and the robot environment's Python 3.10.19; a historical custom-model baseline.
- **2026-10-31 — OFFLINE:** 168 tests passed in the Python 3.11.14 Windows venv; `pip check` passed. This migration verification did not connect to hardware.
- **2026-11-02 — OFFLINE:** 204 tests passed on the Pi with Python 3.13.5 / LeRobot 0.6.1; `pip check` and calibration arithmetic passed. Serial connections were blocked and no physical gamepad was exposed. These results do not validate the future PC-led runtime.
- **2026-11-02 — owner-approved calibration synchronization:** The completed Pi LeRobot calibration was copied unchanged into the project and the degree export regenerated. No recalibration or motor-register writes were performed by that synchronization. The controller loads calibration directly from the project.
- **Subsequent owner hardware reports (not newly tested here):** Standalone LeRobot Pi IK fine movement worked. Full IK commands restored extended-arm retraction; both the old measured-joint 5° rejection and coordinated 5° partial commands had stalled. Preserve this evidence separately from passing mocks; see [IK hardware lessons](context/IK_TELEOP_LESSONS.md).
- **2026-11-05 — OFFLINE asset synchronization:** Copied all 13 reduced STLs and `SOURCES.json` from `IK-collision` commit `e836a3ede4df3d8e7d4c022fc47cb25ac46e9032`. Total size fell from 16,129,292 to 1,968,242 bytes; triangle count fell from 322,564 to 39,343. URDF changes were limited to line endings so its bytes match the manifest. File/manifest integrity checks do not validate physical clearance or implement collision avoidance. No hardware was operated.

Physical model alignment, collision validation, gripper conversion and the proposed stop/hold policy remain open. Migration test results and owner motion reports have different scopes; neither proves the target architecture or collision safety. See [README verification history](../README.md#tests-and-diagnostics) and [current priorities](../TODO.md).
