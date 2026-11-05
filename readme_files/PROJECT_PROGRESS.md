# Project status and history

> **New approved direction (2026-11-05): PC-led architectural refactor, not yet implemented.** Existing modes documented here remain the factual legacy baseline. See [target architecture](PC_PI_ARCHITECTURAL_REFACTOR.md) and [phased plan](PC_PI_IMPLEMENTATION_PLAN.md). The new priorities supersede prior phone-through-Pi-dashboard and Pi-only-compute plans; they do not retroactively change implementation status.


[Home](../README.md) | [Architecture](ARCHITECTURE.md) | [TODO](../TODO.md)

Original implementation review: 2026-10-30. Documentation reconciled with local source and the proposed plans on 2026-11-05. Hardware status comes from the owner; implementation status comes from source inspection. This update changed documentation only: no tests, runtime installation, robot commands or motor-register operations were performed.

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

## Approved architectural refactor — priority over older future work

**Current status: NOT STARTED. Development will resume on the Windows PC, not the Raspberry Pi.** No distributed supervisor, Windows-to-WSL motion worker, Pi network motor service or separate PC monitoring dashboard is implemented. This documentation reconciliation does not complete the Phase 0 runtime audit, benchmarks or safety gates. Preserve current modes as legacy.

1. **First, on Windows:** Audit the legacy baseline, verify Windows gamepad mapping and a separate WSL2 PlaCo/LeRobot IK environment, self-collision/path checker and timing. Reconcile model/units against reference measurements, with approval for new physical checks. Benchmark before committing to integration.
2. **Then:** Implement separate Windows gamepad gateway + PC startup orchestration + Pi sole motor service with validated joint targets, feedback, watchdog, short disconnect recovery and modular offline tests.
3. **Then:** Hardware-validate low-speed 50 Hz control and build the PC-hosted monitoring web dashboard.
4. **Later:** tabletop avoidance, presets/plan migration, native Android (including ArUco), secure remote phone access, Pi camera streaming and optional event-driven motor updates.

Preserve exact calibration/tuning, validate physical model and gripper unit conversion, and collect latency/rejection measurements across component boundaries. Physical safety and legacy compatibility are release gates.

## Previous planned direction (historical snapshot; superseded where conflicting)

Earlier plans centered on Pi-local LeRobot IK, combined self/table/environment collision work, presets/recording and phone commands through a Pi-hosted web dashboard. Standalone Pi IK and read-only browser telemetry were subsequently implemented; phone motion control and collision checking were not. The approved PC-led target now takes priority: self-collision/path validation first, PC-hosted monitoring, and a deferred Android input source through the PC gateway. Presets/recording migration and table collision are deferred. The Pi remains the motor host rather than the development or IK host for the new path.

## Preserved validation and hardware history

- **2026-10-30 — OFFLINE:** 159 tests passed under Python 3.12.9 and the robot environment's Python 3.10.19; a historical custom-model baseline.
- **2026-10-31 — OFFLINE:** 168 tests passed in the Python 3.11.14 Windows venv; `pip check` passed. This migration verification did not connect to hardware.
- **2026-11-02 — OFFLINE:** 204 tests passed on the Pi with Python 3.13.5 / LeRobot 0.6.1; `pip check` and calibration arithmetic passed. Serial connections were blocked and no physical gamepad was exposed. These results do not validate the future PC-led runtime.
- **2026-11-02 — owner-approved calibration synchronization:** The completed Pi LeRobot calibration was copied unchanged into the project and the degree export regenerated. No recalibration or motor-register writes were performed by that synchronization. The controller loads calibration directly from the project.
- **Subsequent owner hardware reports (not newly tested here):** Standalone LeRobot Pi IK fine movement worked. Full IK commands restored extended-arm retraction; both the old measured-joint 5° rejection and coordinated 5° partial commands had stalled. Preserve this evidence separately from passing mocks; see [IK hardware lessons](context/IK_TELEOP_LESSONS.md).

Physical model alignment, collision validation, gripper conversion and the proposed stop/hold policy remain open. Migration test results and owner motion reports have different scopes; neither proves the target architecture or collision safety. See [README verification history](../README.md#tests-and-diagnostics) and [current priorities](../TODO.md).
