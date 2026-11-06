# TODO — approved PC-led architectural refactor

The mesh-free Windows-to-Pi path is verified in simulation and awaits owner-approved hardware commissioning. See the [current remote IK setup](readme_files/REMOTE_IK_SETUP.md). The checklist below follows the broader [November 5 design](readme_files/PC_PI_ARCHITECTURAL_REFACTOR.md) and its [phase gates](readme_files/PC_PI_IMPLEMENTATION_PLAN.md).


## Priority 1 — new offline modular runtime

- [ ] Implement Windows gamepad-only input/gateway, controller-neutral `MotionIntent` interface and **PC orchestrator** that will start the supervised Pi service remotely.
- [ ] Implement WSL2 IK worker with actual-joint feedback, validated movement path, self-collision, explicit frame/unit conversions and separate gripper normalization.
- [ ] Implement exclusive Pi motor-service skeleton (fake bus first): safe limits, one-session control, fresh sequenced commands, watchdog, bounded hold, reconnect/re-arm and structured telemetry.
- [ ] Mock and test every boundary including malformed/stale/reordered commands, dashboard crash, IK rejection and Wi-Fi dropouts; no uncontrolled movement or torque disable on brief loss.

## Priority 2 — integrated approved hardware control and monitoring

- [ ] Integrate tuned `SO101Follower` on Pi only; verify calibration, motor serial ownership, gripper normalization, motor limits and stop/torque policy before movements.
- [ ] Test low-speed gamepad IK and gripper, retraction from extended arm, actual feedback reconciliation, self-collision rejection, deadman, dropouts and startup/shutdown with owner approval.
- [ ] Instrument 50 Hz scheduling, IK/collision time, IPC/network jitter, bus read/write and actual mechanical response (median/p95/p99/worst).
- [ ] Implement **PC-hosted read-only web dashboard** with state, actual joints, requested/accepted targets, collision warnings, connectivity and performance; telemetry must not block control.
- [ ] Document actual new-path setup commands and rollback, update progress/status, and test no breakage to legacy modes.

## Deferred — not in initial implementation

- [ ] Table collision: whole-arm geometry and paths above measured table plane, not only tool Z. No wall/furniture checks planned.
- [ ] Port presets and recorded plan MOVE/PAUSE, looping, plan editing and safe cancellation through same validated executor.
- [ ] Native Android controller app with virtual joystick and ArUco tracking. **No phone control in first implementation.** One active input source at a time; future phone enters through PC, not Pi dashboard.
- [ ] Secure phone control from untrusted/public networks via authenticated/encrypted PC gateway, preferably private VPN/tunnel; never publish Pi motor endpoint.
- [ ] Robot-mounted Pi camera feed to PC through a separately scheduled/isolated stream.
- [ ] Evaluate event-driven Pi motor writes under safe maximum bus rate and compare against fixed 50 Hz.
- [ ] Consider native-Windows IK or other library only if WSL2/PlaCo proves inadequate in reliability, collision functionality or measured performance.

## Completed legacy preparation (preserved history)

- [x] Python 3.11 Windows venv baseline, calibration/tuning retained, offline tests documented. See [setup](readme_files/CONNECT_SETUP.md).
- [x] Separate headless Raspberry Pi LeRobot URDF IK mode with working fine increments. It remains **legacy**, not the final distributed pipeline.
- [x] Acquired the 13 URDF-referenced meshes and recorded provenance/checksums/license; see [assets](assets/README.md).
- [x] Replaced all 13 STLs with the reduced `IK-collision` asset set and matching source manifest; preserved the model geometry and verified file integrity offline. Physical clearance validation remains open.
- [x] Implemented a read-only Pi-hosted legacy LeRobot IK dashboard. It is **not** the target PC-hosted monitoring dashboard.

## Superseded proposals (historical context)

The previous backlog proposed phone commands through the Pi web dashboard, combined self/table/environment collision work, and presets/recording for standalone Pi IK. The approved target replaces phone-through-Pi control with a deferred Android-to-PC gateway, prioritizes self-collision and transition validation first, and defers table checks and plan/preset migration. These older proposals were not completed features; their historical intent does not determine the next implementation order.

**Important known gaps:** No validated self-collision in current active control modes; current plan playback is not collision-aware; legacy dashboard Act and presets can bypass manual LB hold; `gripper` normalized-unit handling differs from degree-limit export; physical model and path accuracy require measurement. See [legacy architecture](readme_files/ARCHITECTURE.md), [calibration](readme_files/JOINT_LIMITS_IMPLEMENTATION.md) and [IK hardware lessons](readme_files/context/IK_TELEOP_LESSONS.md).
