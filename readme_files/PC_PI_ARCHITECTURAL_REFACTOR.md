# PC-led SO-101 control: architectural refactor (target design)

**Status: APPROVED DIRECTION / NOT YET IMPLEMENTED**  
**Decision date: 2026-11-05**  
[README](../README.md) · [Current architecture](ARCHITECTURE.md) · [Implementation plan](PC_PI_IMPLEMENTATION_PLAN.md) · [Current TODO](../TODO.md)

> **Priority and scope:** This document specifies the *new architecture to build*. It takes precedence over older future-work proposals in README/TODO/current-architecture notes where they disagree about the **target**. It does **not** assert that the target architecture already exists, invalidate observed legacy behavior, or authorize moving hardware without explicit approval. The existing modes remain available as legacy during this refactor. Review actual source code before implementation; these design decisions were based on the supplied Markdown documentation and project discussions, not a complete live source audit.

**Development location:** Resume source review, implementation and offline testing in the **Windows PC checkout, not on the Raspberry Pi**. WSL2 is the provisional PC-local solver runtime to verify in Phase 0. The Pi remains the motor-service deployment host and legacy hardware reference. This canonical copy was promoted from the supplied proposal bundle on 2026-11-05; the accompanying documentation review did not implement or benchmark the design.

## 1. Goals, constraints, and boundaries

### Immediate goals (first usable release)

- Use a **Windows-connected physical gamepad only**. Preserve manual 6-channel control, including the gripper, with the new URDF/LeRobot IK path and verified self-collision rejection.
- The **Windows PC is the mothership**: the user starts one PC-side launcher; it checks and starts the Pi runtime remotely, starts/coordinates the PC and WSL2 components, supervises status, and handles orderly shutdown. The Pi remains responsible for safe execution even when the PC/Wi-Fi connection drops briefly.
- Offload expensive **URDF IK and self-collision calculations to the PC**, provisionally in **WSL2 using the existing LeRobot/PlaCo approach**, contingent on measured performance and collision feasibility. Avoid rewriting the solver merely to remove WSL2.
- Pi is the **only serial/motor-bus owner**. It receives validated targets, applies independent motor-side safety rules, reads actual joints, and sends feedback; no direct Windows/WSL USB serial access in the new path.
- Target a **50 Hz control / motor-command cycle (20 ms period)** with low, predictable *end-to-end* delay. Measure, do not assume, timing and jitter at each boundary; adjust rate if measured limits require it.
- Provide a **PC-hosted, monitoring-first web dashboard**; its rendering/telemetry path cannot block motor control. A dashboard is *not* a second motion controller in the first release.
- Build modular boundaries and offline tests so each component can be verified independently before connecting hardware.

### Explicitly deferred

- Android native app (virtual joystick / ArUco pose tracking), any phone control, and control from outside the trusted LAN.
- Robot-mounted camera feed, video transport, vision-driven actions.
- Table collision and surrounding workspace/environment constraints. Only **self-collision** is an immediate collision requirement; future table checks must consider the **whole robot**, not just end-effector Z.
- Presets, recorded plans, looping, plan editing and general autonomous actions **in the new architecture**. Legacy modes retain their own already implemented capabilities.
- Event-driven motor writes, alternate native-Windows IK solvers, ROS2, and advanced smoothing as optional optimizations, **not** prerequisites.

### Non-goals / prohibited shortcuts

- Do not replace the current working modes in place or change their behavior while building the new path. Do not delete legacy code/commands until explicitly requested.
- Do not treat the current LeRobot FK/IK wrapper as collision-aware: **it does not enable self-collision by itself**. Collision validation must be separately implemented and demonstrated before moving the new path on hardware.
- Do not substitute raw joint clamping for collision validation, or test only a target pose while ignoring intermediate configurations.
- Do not turn a loss of network/PC dashboard into immediate exit, serial release, or torque-off. Conversely, do not allow indefinite movement on stale commands.
- Do not silently recalculate calibration, change Feetech PID/acceleration/torque registers, edit URDF geometry, or re-home hardware. Changes to physical behavior require owner approval.

## 2. Component and process topology

```text
                   Windows PC (mothership)
   Gamepad -> Windows input adapter / mode + deadman -> control gateway
                    |                             |  \
                    |                             |   -> PC web dashboard
                    |                          local IPC       (read-only telemetry)
                    v                             |
              WSL2 motion worker <----------------+
              - LeRobot / PlaCo / URDF FK + IK
              - self-collision & path validator
              - feedback-aware target generation
                    |
                    | sequenced fresh validated joint targets
                    | PC Ethernet -> router -> Pi Wi-Fi (LAN)
                    v
              Raspberry Pi motor service (one owner)
              - command/auth/session checks and watchdog
              - joint/rate limits; safe hold / fault states
              - serialized 50 Hz motor-bus writes/reads
                    |
                    v
              Feetech adapter -> SO-101 motors
                    |
                    +-- measured joint positions/health ----> WSL worker & dashboard

Future (NOT implemented): Android app -> authenticated PC input gateway;
  camera on Pi -> separate media path -> PC; remote access via secured tunnel.
```

**Ownership contracts**

| Component | Owns | Must not own |
| --- | --- | --- |
| Windows supervisor / input adapter | Launch/stop orchestration; gamepad sampling; input enable/deadman; exclusive controller mode; PC gateway; telemetry publication | Motor serial device; unvalidated movement; collision-free assumptions |
| WSL2 motion worker | Authoritative *requested motion* / IK state; URDF transforms; safe candidate/trajectory validation; reconciliation to actual feedback | Physical motor serial; independent arbitrary pose changes by UI |
| Pi motor service | Sole motor serial connection; measured hardware state; limits/rate enforcement; safe holding and fault handling; sending actual-state feedback | Expensive IK/collision solving; phone/dashboard control arbitration |
| PC monitoring dashboard | Display requested/validated/actual state, telemetry, faults, timing, connection state | Direct hardware writes or a command path that bypasses validation |

**State is not a single shared variable:** PC/WSL tracks desired/accepted targets; Pi reports measured joint state and the last *actually issued* command. Each command/feedback report must be associated with a session and sequence ID so UI and diagnostics can tell requested, accepted, transmitted, and measured apart.

## 3. Control and feedback semantics

1. Windows samples the selected gamepad. The first implementation will support only gamepad mode; the adapter must export a controller-independent intent interface so a future Android app can replace it **without changing the motion/safety pipeline**. Only one input source may be active per session. Verify the Windows device's actual mapping and neutral/deadman behavior; the current fallback mapping is not proof of device validation.
2. Gated input (including an explicit **hold-to-move/deadman** for manual motion) yields bounded incremental Cartesian/orientation/gripper intent. Do not infer that legacy LB gating cancels presets or is a universal stop; no preset or plan execution is in initial scope.
3. WSL starts from **recent actual joint feedback** (and explicitly tracked commanded state), transforms input in the selected frame, solves IK, verifies finite values/URDF + calibration limits, checks self-collision and **the intervening motion**, and emits a complete validated joint target. Reject safely on stale feedback, impossible solutions, invalid models, or uncertain clearance.
4. Pi accepts only fresh authorized commands for the active session, preferably with monotonic sequence IDs and expiration. Discard late/out-of-order targets rather than buffering an ever-growing trajectory. Validate joint bounds, change rates, connection health and motor status *again* without repeating expensive geometric checks. Any Pi-side slew limiter or interpolator must remain within a WSL-validated trajectory/corridor; blindly clipping joint targets can invalidate earlier collision checks.
5. Pi performs serialized motor reads/writes at a target 50 Hz; state and health return asynchronously to WSL and monitoring. Dashboard sampling can be lower (e.g. 10–20 Hz) and must never slow the control path.
6. Loss of joystick actuation/deadman or unsafe/rejected request means **do not advance**. The Pi stops advancing motion and applies a **physically tested stop/hold policy**. Simply retaining a distant last target may let the servo keep travelling; repeatedly latching a new drifting measured hold can cause stutter/sag. Define and test a one-time bounded stop target or braking strategy with the arm loaded, and distinguish normal rejected-input handling from network-loss emergency pausing.

**Trajectory caveat:** A safe endpoint does not imply a safe transition, particularly when the arm follows joint-space motion under load. Initial path checking can use conservative, tested joint-space subdivision with documented sampling resolution, margins and timeout budget; continuous/swept collision checking is a future improvement if needed. If checking cannot establish safety, reject the movement. Independent gripper/roll actions and any motion sent outside the IK candidate path must pass the relevant validation too.

**IK rejection lessons to preserve:** The old `>5°` measured-joint-step rejection caused stalled retraction near full extension; sending full validated IK commands solved the observed hardware problem, whereas coordinated 5° partial commands also stalled. Do not reinstate that limiter without new reproducible physical evidence. Distinguish model/solver failures, collision-guard rejection, transport delivery, actual motor movement, and loss of torque. See [IK hardware lessons](context/IK_TELEOP_LESSONS.md).

## 4. Frames, units, calibration and representation

- **Preferred canonical robot frame:** URDF base frame with X forward, Y left, Z up (right-handed); use **meters** for Cartesian positions and **radians** for general mathematical transforms, following ROS-style conventions. Verify the *actual* URDF frame orientation and physical origin rather than assuming it from labels.
- Existing LeRobot wrapper uses meters for FK and **degrees for IK joint inputs/outputs**. Convert only at explicit adapter boundaries; do not impose radians on an API documented to consume degrees. The custom legacy model instead uses centimeters/degrees and historical motor sign/offset corrections; leave its conventions unchanged.
- Name all frames and transforms (`base`, `tool`, future `table`, future `camera` and `phone_marker`) rather than using unlabeled XYZ. Future ArUco marker coordinates require camera extrinsics and a deliberate mapping/calibration to the robot frame.
- Use one explicit **ordered joint schema** including five kinematic joints plus a **separate normalized gripper channel** (0–100 in the installed follower). Revalidate the URDF's joint ordering, handedness, zero, mesh alignment, and the existing elbow-offset question against measured hardware before trusting collision margins.
- The Pi alone owns the actual bus calibration and unit conversions. Preserve project files `calibration/lerobot_arm.json` and `calibration/lerobot_arm_with_degrees.json`; the current custom controller has an **unresolved normalized-gripper vs exported-degree-bound mismatch**. Resolve this in the new adapter before hardware trials; do not copy the custom clamping mismatch into the new path.
- Joint boundaries must be established from both the model and calibrated hardware range, expressed consistently after converting units; software checks are supplemental to physical clearance.

## 5. Safety behavior and failure isolation

### Safety principles

- Default to **not moving until ready**: valid calibration, connected Pi serial bus, actual joint state read, model and collision checker ready, one input source selected, deadman active, connection authenticated for commands.
- **One arm owner**: legacy modes and new Pi service must not run simultaneously. Startup checks should detect a busy port or another active session and fail clearly rather than trying to take ownership.
- Pi-side joint limits, max velocity/acceleration/slew and command-age watchdog remain effective when the PC, WSL2 or Wi-Fi fails. Those rate constraints must not repeat the historical retraction-stall issue; distinguish legitimate velocity limiting from arbitrary IK candidate rejection.
- Self-collision policy must include exclusions for adjacent/contacting links, body/base geometry, adjustable clearance margin, gripper configuration, and transition checks. Verify collision library capabilities; a plain PlaCo IK result is **not** accepted as safe merely for reaching a target.
- A software emergency-stop command should bypass normal planning; make status obvious and require explicit re-arm. **Software stop is not a physical E-stop**: don't claim safety certification, and ensure a usable power-disconnect method exists for hardware tests.
- Torque-off is **not** the default safe reaction to brief network loss: an arm may fall under gravity. A controlled bounded stop/hold is preferred pending hardware validation; do not continue to a distant stale target, and avoid indefinite motion/continually recapturing a sagging pose.

### Proposed state machine (to implement and test)

`OFFLINE` -> `STARTING` -> `READY` -> `ARMED` -> `MOVING`. `PAUSED` for deadman release/stale commands/brief network interruptions. `FAULT` for model/motor/serial/critical safety failure; `ESTOP` for explicit emergency stop. Recover from `PAUSED` after fresh actual-state sync, fresh valid commands and deadman re-engagement; `FAULT`/`ESTOP` require manual acknowledgement/re-arm. On an intentional PC shutdown, ask the Pi to stop safely before terminating its process and document resulting torque policy.

- **Brief PC/Wi-Fi disconnection:** Pi process stays alive and owns the serial port, ceases advancing trajectory promptly, enters a tested bounded hold, continues reading motors, exposes reconnection status. PC reconnects, discards old session/commands and resynchronizes actual joints. No automatic continuation of the prior commanded movement.
- **Lost/missing camera/dashboard:** no effect on motor command loop; camera is future only.
- **Missed/slow IK tick:** do not block Pi waiting; use the latest safe target only within an explicit validity window, then pause. Never queue old commands and play them late.
- **Unexpected motor motion, stale feedback, unsafe geometry, servo error:** stop advancement and expose a specific fault/reject reason. Some faults may require torque policy to be chosen after physical tests.

**Watchdog timeout is not a fixed decision yet.** Start testing around 100–200 ms and tune against Wi-Fi jitter, safety and hold behavior; use Pi-local monotonic arrival times. A command-carried wall-clock timestamp is useful for telemetry but is not a trustworthy sole expiry mechanism when clocks differ.

## 6. Communication, performance, and observability

**Network:** PC Ethernet -> home router -> Pi Wi-Fi. WSL2 <-> Windows is local process/network communication; Pi connection is over trusted LAN. The new motor service must have a narrowly scoped command API; do not reuse `.runtime/*.json` file IPC as a network command protocol.

**Transport decision pending benchmark:** Prefer a compact versioned schema, lightweight local Windows/WSL IPC, and a low-latency, sequenced PC/WSL-to-Pi command transport (UDP is a candidate, TCP also viable); authentication, timeout/age handling, telemetry and reconnection are mandatory regardless of choice. Never bind an unauthenticated control endpoint to the public internet. For initial trusted LAN use, document credentials, bind/interface restrictions and minimal firewall access. Future Android/internet control should arrive through an authenticated/encrypted **PC gateway** (e.g. VPN/tunnel), not directly at the Pi servo endpoint.

**Proposed message fields (logical contract, not chosen serialization):**

- `protocol_version`, `session_id`, `sequence`, `command_mode`, complete joint target with explicit units and joint order, gripper target; optional diagnostic origin/type and sender timestamp.
- Feedback: Pi receive/apply sequence, Pi-local receive/apply timestamps, actual joints, last sent target, bus errors, watchdog/torque/fault/connection state.
- Validate schema, finite numbers, ranges, session authorization, duplicate/out-of-order IDs and latest-command policy. Treat malformed, stale, or unauthenticated messages as non-motion; avoid command queues that build up during slow periods.

**50 Hz is a target, not a guarantee or a claim about reaction time.** Measure separately:

1. Gamepad poll and input sampling/wait to next tick.
2. Windows ↔ WSL2 IPC time (both directions if relevant).
3. WSL IK computation, self-collision/path check, and total motion-worker time.
4. PC ↔ Pi one-way latency estimates, round-trip time, jitter, packet loss/reordering.
5. Pi receive/validate, wait to next motor write, bus write and read, servo feedback freshness.
6. Command accepted-to-first-observed-motor-response and full joystick-to-observed-motion latency (separate mechanical response from software timing).

Record **median, p95, p99, worst-case, dropped ticks, missed deadlines, stale/rejected commands** and synchronized event correlation via session/sequence IDs. If clocks are not synchronized, use local durations and explicit RTT-based estimates; do not subtract raw cross-host wall-clock timestamps. The supplied planning documents cite a historical Pi **six-motor read benchmark ~2.78 ms average**. Preserve it as reported context, not a newly reproduced measurement or evidence for complete read+write, Wi-Fi or WSL pipeline performance; record setup and reproduce it during approved measurement work.

An initial aspirational target is typical joystick-to-first-motion under roughly **50 ms** with stable 50 Hz command cadence, subject to measured servo physics and realistic acceptance criteria after benchmarking. No claimed precise latency figures without tests. If IK/collision takes longer than 20 ms, decouple worker computations from Pi's bounded motor loop; reject/hold instead of violating safety to meet timing.

**Deferred optimization:** An *event-driven Pi motor writer* could apply a freshly validated command immediately, under a tested maximum motor-write frequency and exclusive serialized bus access, rather than always waiting for the next 50 Hz tick. Benchmark the simpler fixed-rate loop first. Smoothing and interpolation must not turn safe validated samples into an unchecked path.

## 7. Monitoring dashboard

- Runs as a **PC service**, accessed via browser on PC/LAN, not embedded in the Android app. Start with **read-only monitoring** and make it independent of Pi motor service lifetime.
- Distinguish requested pose, accepted IK solution, transmitted joint target, **measured joints/estimated FK**, collision rejection reason, session/controller state, watchdog status and error counters.
- Expose timing percentiles and last-message age. Read-only dashboard updates at approximately 10–20 Hz are sufficient initially; UI can render faster. Backpressure and dashboard disconnections must never stall IK or Pi motor commands.
- Legacy Pygame and Pi-hosted read-only dashboard remain available *in their legacy execution paths*. They are not the target hosting model.

## 8. Extensibility requirements (interfaces now; features later)

| Future capability | Design provision NOW | Implementation LATER |
| --- | --- | --- |
| Native Android joystick/ArUco | Controller-neutral, exclusive `MotionIntent` input gateway; explicit frame/time info | App, ArUco tracking, calibration, filtering, handover UX |
| Public/untrusted network phone control | Separate transport/auth from safety, PC gateway, session/re-arm and command expiry | VPN/encrypted channel, user/device enrollment/revocation, remote UX and stricter jitter rules |
| Robot-mounted camera | Pi runs isolated motor priority; process and network bandwidth isolation; telemetry not coupled to video | Camera server/stream to PC, codecs/bandwidth and safety testing |
| Table collision | URDF link geometry, named table frame, collision-world interface | Plane/table geometry, clearance for all links, measured table frame, path checks |
| Presets and recordings | Reuse one validated motion execution interface; no direct servo bypass | Migrated stateful plan engine, playback cancellation and safe path validation |
| Alternative IK / host | Solver interface and benchmark harness separate from gamepad/transport | Native Windows solver evaluation, optional RPi solver or different host |
| Event-driven motor writes | Isolated motor writer and latest-command buffer with one bus owner | Bounded immediate writes after latency measurements |

## 9. Agreed decisions vs still open

**Decided:** Preserve existing modes as legacy; resume development on Windows PC; new separate pipeline; gamepad only now; PC orchestrator; Pi owns motors; WSL2 + LeRobot/PlaCo first if measured feasible; self-collision required now, table later; 50 Hz initial target; PC monitoring dashboard; connection recovery without Pi-process exit; one active controller; future native Android and camera; protect testing modularity and existing calibration.

**Require inspection/measurement before committing details:** exact source-code reuse; WSL2 environment/version and solver setup; collision implementation/geometry and acceptable path-sampling margin; joint command/gripper conversion; wire protocol; motor read/write scheduling; safe hold/torque behavior; watchdog timeout; practical hardware performance criteria; PC remote Pi service start/stop mechanism (SSH plus service supervision suggested); any credentials/firewall deployment details.

**Legacy behavior is reference, not new-safety proof.** See [ARCHITECTURE.md](ARCHITECTURE.md), [CONNECT_SETUP.md](CONNECT_SETUP.md), [JOINT_LIMITS_IMPLEMENTATION.md](JOINT_LIMITS_IMPLEMENTATION.md), and [IK_TELEOP_LESSONS.md](context/IK_TELEOP_LESSONS.md).
