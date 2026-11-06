# PC-led SO-101 control: phased implementation plan

**Status: PHASE 0 IN PROGRESS / PHASES 1–5 NOT STARTED**
**Decision date: 2026-11-05**  
[Target architecture](PC_PI_ARCHITECTURAL_REFACTOR.md) · [Current implementation](ARCHITECTURE.md) · [TODO](../TODO.md)

This is an **incremental architectural refactor alongside the existing system**, not a ground-up rewrite or replacement. The existing direct-axis, custom IK, LeRobot Pi IK and plan modes continue as **legacy** and must not be broken by early work. The project owner must approve all physical tests, calibration changes, motor register writes and nontrivial torque behavior. The coding AI must follow [AGENTS.md](../AGENTS.md) and make **no Git commits**.

**Work location and status:** Development is in the **Windows PC checkout, not on the Raspberry Pi**. Phase 0 is **IN PROGRESS**, with offline source audit, an isolated WSL2 solver stack and feasibility benchmarks recorded in the [Phase 0 report](PHASE0_FEASIBILITY.md). Phases 1–5 remain **NOT STARTED**. The Pi remains the future motor-service host and legacy hardware reference; no new distributed control path or hardware verification is implied. These canonical plans supersede conflicting older plans; use the copies in `readme_files/` for future updates.

## Cross-phase rules

1. **Survey first.** Inspect actual code and environments before implementing: all launchers and alternate motor paths, LeRobot/Placo versions, URDF assets, calibration files, `SO101Follower` tuning, gamepad mapping, dashboard startup and motor ownership. This plan is based on the supplied docs; verify any divergent behavior against source. Record discrepancies rather than treating design text as implemented facts.
2. **Keep legacy working.** Put the new PC→WSL2→Pi path behind distinct entry point(s), configs, files and service names. Do not silently change old input mappings, limits, presets, plan playback or dashboards. One motor owner at a time.
3. **Design for test seams.** Independently replace input, IK, collision checker, transport, motor hardware, and dashboard with fakes. Keep wall-clock/monotonic timers injectable in tests. Explicit unit conversions and frame transforms are testable boundaries.
4. **Safety before speed.** A 50 Hz loop is a goal; do not skip safety checks to attain it. No arm motion until local safety and geometry checks are verified against actual hardware.
5. **Clear status labels.** Use `NOT STARTED`, `IN PROGRESS`, `OFFLINE VERIFIED`, `HARDWARE VERIFIED`, or `DEFERRED` in implementation reports. Never mark a task complete from a passing mock alone.
6. **No uncontrolled motor operation.** Begin with offline tests; require a user-approved, low-speed, low-risk hardware test for each move/control milestone. Be prepared to support the arm physically, especially during torque transitions.
7. **Performance evidence.** Save benchmark setup, hardware/software versions, timing distributions and known outliers. Optimize only after locating the bottleneck.

## Phase 0 — baseline and feasibility gates (START HERE)

**Objective:** Decide whether the provisional WSL2/PlaCo architecture meets both self-collision feasibility and control-loop timing before building a large distributed system.

**Tasks**

- [ ] Confirm current working Pi motor path, motor port, owner-approved calibration, motor tuning, current LeRobot IK script and actual rejection/hold behavior. Identify every path capable of hardware motion.
- [x] Record the existing legacy modes and their tests/launch commands. Capture a reproducible baseline with the robot disconnected, where possible. **OFFLINE VERIFIED:** Windows baseline retains one known dashboard test failure; see report.
- [x] Verify or provision a separate WSL2 Linux environment, compatible Python/LeRobot/PlaCo stack, URDF/meshes and LeRobot IK import. **OFFLINE VERIFIED:** isolated x86-64 Python 3.12.12 / LeRobot 0.6.1 / PlaCo 0.9.15 stack; Windows and Pi environments preserved.
- [ ] Check that URDF joint order, frames, limits and tool pose match recorded real hardware measurements before assuming physically meaningful collision clearance. Include the historical elbow/second-pitch offset and end-effector height question; this is an unconfirmed discrepancy to revalidate, not permission to change calibration. Verify links/meshes and exclusions needed for self-collision (adjacent links, robot base, gripper). Any new physical measurements require owner approval; establish model alignment before live integration.
- [x] Build a **CPU-only offline benchmark** over representative poses: neutral, extended, retraction from full reach, near singularity/limit, wrist-roll, unreachable and collision-near poses; measure IK and collision check separately and together (median/p95/p99/worst, accepted/rejected ratio, variance and path-check cost). **OFFLINE VERIFIED:** see raw samples and experimental-setting limits in the report.
- [x] Prove the candidate collision checker really detects self-collisions and can check *transitions*; Placo/LeRobot IK alone does not do this. **OFFLINE VERIFIED:** analytic/BVH fixtures and endpoint-clear/interior-colliding robot path pass with a separate Pinocchio/Coal margin predicate. Exclusions, margins and physical alignment remain unapproved; this is not live-motion clearance.
- [x] Benchmark Windows gamepad polling and Windows↔WSL2 message transit without motors. **HARDWARE VERIFIED (input only):** owner checked Windows mapping in the standalone dashboard. **OFFLINE VERIFIED:** software hold-to-move behavior with a fake arm. Physical stopping is a separate hardware check.
- [ ] Measure Pi serial read and command write separately and together, **only with owner approval for hardware operations**. The supplied proposal's historical ~2.78 ms all-motor **read** figure needs its setup recorded and reproduction; it is not a complete motor update measurement or a result of this documentation review.
- [x] Write a short performance/feasibility report: feasible, feasible with changes, or blocked. **OFFLINE: feasible with changes.** Retain WSL2/PlaCo provisionally for IK; exact all-pair distances fail the timing budget, native margin predicates warrant further work. See [report](PHASE0_FEASIBILITY.md).

**Gate to Phase 1:** PlaCo/URDF FK/IK operates correctly in WSL2; collision safety approach demonstrably feasible; explicit plan to measure/respect 50 Hz. If not, pause and revisit solver choice rather than hiding a blocker.

**Current gate disposition:** Not approved. Physical model alignment, collision-policy review and Pi timing remain open. The Phase 0 report records the gamepad input check and the remaining manual measurements. Phase 1 has not started.

## Phase 1 — shared contracts and offline motor service

**Objective:** Build the separation without requiring a connected robot.

**Tasks**

- [ ] Define typed/documented **gamepad intent**, joint target, actual feedback, fault/status and telemetry schemas. Include frame, units, ordered joints, gripper normalization, session ID, sequence and protocol version. Avoid assumptions that all motor channels are degrees.
- [ ] Implement a Windows supervisor/launcher skeleton that will eventually start Pi service via SSH/service manager, Windows input adapter, WSL2 worker and PC dashboard. Fail clearly when the Pi is unreachable; do not assume a remote shell process survives SSH disconnection without supervision.
- [ ] Implement an isolated Pi motor service with a **fake bus** and explicit ownership guard. It accepts commands through a mockable transport, enforces validity/rate/watchdog rules, returns measured-state-shaped telemetry and keeps running through simulated PC disconnects.
- [ ] Implement Windows↔WSL2 IPC and PC↔Pi link adapters with a mocked receiver. Choose and document the actual transport after measurement (UDP/TCP/other). Use a *latest-command* policy, not a backlog. Validate malformed, duplicate, out-of-order, expired and wrong-session inputs.
- [ ] Implement an initial state machine (`OFFLINE`, `STARTING`, `READY`, `ARMED`, `MOVING`, `PAUSED`, `FAULT`, `ESTOP`) and explicit deadman/re-arm transitions. Software E-stop must bypass motion computation; physical emergency power-off remains separate.
- [ ] Implement non-blocking timestamped instrumentation and structured reject/fault reasons. Keep timestamps with host identity; do not make cross-machine one-way latency claims from unsynchronized clocks.

**Offline acceptance tests**

- [ ] Start/stop/restart sequence works with fake Pi, Windows and WSL2 worker; existing modes remain untouched.
- [ ] Pi fake receives at most one active session and will not accept old-session/reordered/expired commands.
- [ ] Simulated 100–200 ms-class Wi-Fi disruptions cause pause/hold, **not Pi process exit or torque-off**, and reconnection only resumes after actual-state resynchronization plus fresh authorized input. Final watchdog timeout comes from testing, not a hard-coded assumption.
- [ ] Dashboard or IPC consumer crash/disconnect does not block the motor thread, and malformed requests never reach the hardware adapter.

**Gate to Phase 2:** An entirely fake distributed loop executes correctly and can be replayed deterministically.

## Phase 2 — WSL2 motion worker and independent collision validation

**Objective:** Produce validated, feedback-aware motion targets safely **without connecting motors**.

**Tasks**

- [ ] Reuse the LeRobot URDF/PlaCo FK/IK adapter in `robot_program.lerobot_ik_controller` and relevant input behavior from `scripts.main_teleop_lerobot_ik` behind a solver interface; the current adapter combines solver logic with follower ownership and needs separation. Do not fork large source files unnecessarily. Match the behavior of existing fine input increments initially, then introduce time-scaled velocity logic if needed so speed does not drift with loop frequency.
- [ ] Implement actual-state reconciliation, position/orientation frames, joint order/units, finite checks, hardware+URDF limits and separate gripper handling. Avoid copying the custom controller's normalized-gripper/degree-bound mismatch.
- [ ] Implement self-collision checking including explicit margin, link exclusion pairs, base and geometry mapping; validate the full proposed motion transition via conservative path sampling or stronger method. Reject uncertain or unsafe solutions.
- [ ] Ensure all accepted motion, including wrist roll and gripper actions, passes the relevant safety checks. A clipping/slew stage on the Pi must **not** transform a checked safe target into an unchecked path; coordinate safe trajectory constraints across worker and Pi or reject a command that can't be executed safely.
- [ ] Test rejection handling: failed IK/collision does not incrementally recapture a sagging measured pose as a new hold on every tick; do not reintroduce the historical 5° measured-joint step filter that stalled retraction.
- [ ] Preserve telemetry distinction: requested pose; candidate IK pose; accepted safe command; actually sent target; measured joint feedback and timestamp/age.

**Offline acceptance tests**

- [ ] Known colliding poses and transitions are rejected; known non-colliding samples accepted within documented margins. Include robot base/contact geometry and near-clearance cases.
- [ ] Under unreachable targets, stale feedback, no solution, NaN/inf, wrong joint order or changed calibration, no motor target is issued.
- [ ] End-effector FK against independent pose fixtures and recorded real measured reference poses is inside a **documented physical error tolerance** chosen before live operation. Replaying measurements is offline verification; gathering new measurements requires a separately approved hardware check and its own evidence label.
- [ ] The extended-arm retraction regression is reproduced in an offline replay without the old stall-causing policy; controller response to repeated rejection is stable.

**Gate to Phase 3:** There is credible evidence for model alignment and collision checking, and no unvalidated path to the motor writer.

## Phase 3 — controlled integrated hardware gamepad teleop

**Objective:** Make the first safe PC gamepad → WSL2 → Pi → SO-101 control session.

**Prerequisites:** Owner approval, clear work area, arm supported appropriately, serial ownership verified, calibrated joints read, motors tested at low speed and a physical means to cut power.

**Tasks**

- [ ] Implement PC-led remote Pi service startup and orderly shutdown (e.g. SSH initiates a supervised service). Pi runtime continues locally through brief Wi-Fi/PC disconnects; initial user session controls arming. Define restart policy and ensure startup does not cause automatic movement.
- [ ] Bind the Pi motor service to the existing `SO101Follower` and current project-local calibration/tuning, with **one** serialized read/write owner. Do not require forwarding USB into WSL2.
- [ ] Gamepad on Windows uses the mapping verified for the actual device in Phase 0 and a hold-to-move gate; implement enable/stop/re-arm semantics and no automatic motion after loss/reconnect. The legacy Windows fallback alone is not verified device mapping.
- [ ] Run slow, bounded, one-joint-at-a-time tests first, then low-speed Cartesian motions. Verify actual joint units and gripper conversion; no collision-rejected motion executes.
- [ ] Test all target limits, retraction, deadman release, stalled IK, short Wi-Fi interruptions and repeated reconnects; validate safe stop/hold under realistic arm loading (neither continuing toward a distant stale target nor repeated sag/recapture). Monitor bus health and unexpected motion.
- [ ] Profile the true end-to-end pipeline using sequence-based traces, Pi bus timings and motion feedback. Use median/p95/p99/worst; log tick misses, command age and reason for rejection. Treat mechanical response separately from command-delivery latency.

**Initial performance criteria (proposed, adjust after Phase 0)**

- [ ] Pi motor cycle sustains target **50 Hz** under representative load; if not, document measured rate and nonblocking fallback before proceeding.
- [ ] Movement starts predictably after joystick action; review measured joystick-to-bus and joystick-to-observed-motion distributions. **<50 ms** typical joystick-to-first-motion is aspirational, not a claim or fixed release requirement until servo timing is tested.
- [ ] In normal operation, stale commands do not accumulate; late/out-of-order messages cannot cause sudden delayed motion.
- [ ] A brief disconnect safely pauses without exiting Pi service; recovery never resumes motion without fresh input and re-arm.

**Gate to Phase 4:** Live gamepad teleop with self-collision guard is reproducible, safe under tested faults, and timing evidence is recorded.

## Phase 4 — PC-hosted monitoring dashboard

**Objective:** Expose useful telemetry without compromising the critical motor path.

**Tasks**

- [ ] Host a read-only web dashboard on the PC; do **not** reuse the Pi legacy web dashboard as the new system's control gateway.
- [ ] Show requested pose, IK candidate/accepted target, motor target and actual feedback separately, joint units, collision/rejection reasons, state machine/deadman/session, connection health and latency distributions.
- [ ] Send telemetry asynchronously, initially around 10–20 Hz. Slow browser, dashboard outage and remote viewer disconnections cannot affect motor control.
- [ ] Allow view on trusted LAN with sensible access controls; avoid exposing motor commands via UI and avoid public binding without authorization.

**Acceptance:** A dashboard can be killed/restarted while the motor service and safe gamepad control continue; state remains correct after reconnection.

## Phase 5 — stabilization and documented rollout

- [ ] Repeat all offline tests and approved hardware tests on the final architecture; compare command cadence, latency, faults and CPU usage against baseline.
- [ ] Verify mutual exclusion between new and legacy motor-owning modes. Provide a clean rollback path to the previously working launchers.
- [ ] Update [`CONNECT_SETUP.md`](CONNECT_SETUP.md), [`ARCHITECTURE.md`](ARCHITECTURE.md), [`README.md`](../README.md), and [`PROJECT_PROGRESS.md`](PROJECT_PROGRESS.md) with **actual commands/configuration, not speculative examples**.
- [ ] Keep measured results and known gaps. Do not claim collision safety certified by mock tests or simply because the URDF has meshes.

## Deferred backlog (DO NOT implement as part of initial refactor)

| Later feature | Preconditions / interface to preserve now |
| --- | --- |
| Tabletop collision avoidance | Define world/table frame, margin, all-link and full-path checking; no walls/furniture needed |
| Presets, plan recording/playback | All movement routes through one validated executor; cancellation and user ownership rules |
| Native Android phone app, virtual joystick, ArUco | Controller-neutral exclusive input gateway, timestamped pose intents, camera-to-base calibration, noisy-input filtering, fresh-session handover |
| Phone on untrusted/public network | Authenticated/encrypted PC gateway (prefer VPN/tunnel), session pairing/revocation and internet jitter policy; never expose Pi control port directly |
| Robot-mounted camera | Separate Pi camera process and network stream to PC, independently rate-limited; test impacts on 50 Hz control |
| Event-driven motor writes | Compare to baseline fixed-rate 50 Hz; serialize motor bus and limit write frequency; never bypass validation |
| Native Windows IK replacement | Compare actual CPU and integration complexity to WSL2 PlaCo, including self-collision/path validation |
| Smoothing / predictive tracking | Filter input, but collision-check every resulting executed path; stop stale tracking safely |

## Coding AI delivery protocol (per phase)

Each implementation pass should report: changed modules and interfaces; which existing modules were reused; requirements completed; tests with results and whether offline or on physical hardware; latency distributions (if applicable); unresolved safety/compatibility concerns; launch/rollback instructions if changed; a concise proposed commit message (max 12 words), and **explicit confirmation that no Git commit was created**. If uncertain about wiring, the user's environment, motion policy, or calibration, stop and ask instead of guessing.
