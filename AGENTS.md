# Project preferences

- Never create Git commits. After each implementation, give a commit message that summarizes all the changes that are not yet committed to the repository.
- Commit messages should aim to be concise and kept to a maximum of 12 words.
- End every implementation summary by stating that no Git commit was created and that committing is left to the user.
- Prefer automatic detection and sensible defaults so routine commands need as few flags as possible. Explicit flags are appropriate for safety, security, or deterministic automated and AI-assisted testing.
- When reviewing the latest remote IK run, check `.runtime/remote-ik-latest.json` in the Windows checkout first. Check its modification time and `provenance.mode` (`real-arm`, `dry-run`, or `pi-sim`). A run with zero recorded ticks or a failed write preserves the previous capture. For a run launched with `--trace PATH`, read the named file.
- In this Windows setup, the owner's cable-connected controller is reported as XInput and the Bluetooth connection is reported as PS4 Controller through SDL. Both names refer to the same physical controller.
- For changes to LeRobot IK or arm motion, read the [IK teleop hardware lessons](readme_files/context/IK_TELEOP_LESSONS.md).

## Manual testing and pre-commit cleanup

- Run targeted offline checks for logic and regressions. Use manual tests when the result depends on the real controller, network, motor bus, or arm behavior under load. Get owner approval before physical motor tests and report OFFLINE and HARDWARE results separately.
- When manual testing is needed, give the owner numbered steps with prerequisites, where to run each command, the exact inputs to try, the expected behavior, how to stop, and what observations or errors to report back.
- If recording helps, keep temporary results and benchmark data in a local scratch file, preferably outside the checkout, and share its path. Identify any temporary instrumentation added to the code and keep it available for the manual test and follow-up.
- When the owner asks for pre-commit refinement, remove temporary test data, logs, scratch files, and one-off recording code from the proposed changes. Check both the diff and untracked files before summarizing what is ready to commit.

## Approved PC–Pi architectural refactor (2026-11-05)

- Resume development in the Windows PC checkout. WSL2 is the provisional PC-side IK/collision runtime; the Raspberry Pi is the future motor-service host, not the primary development machine. Preserve its existing environment as the hardware/legacy reference.
- Before changing distributed control, read [target architecture](readme_files/PC_PI_ARCHITECTURAL_REFACTOR.md), [phased implementation plan](readme_files/PC_PI_IMPLEMENTATION_PLAN.md), [legacy implementation reference](readme_files/ARCHITECTURE.md), and [IK hardware lessons](readme_files/context/IK_TELEOP_LESSONS.md).
- **Architecture target is not yet implemented.** Preserve existing direct-axis, custom IK, plan, and standalone Pi LeRobot IK modes as legacy. Make new entry points/modules, and do not break existing commands or calibration/tuning.
- Work phase-by-phase: source audit, offline fake tests and benchmarks first; owner approval before any physical motor tests or changes to calibration, Feetech registers, torque or motion policy. Mark test results OFFLINE vs HARDWARE verified.
- Windows PC is the orchestrator; Windows gamepad only initially; WSL2 LeRobot/PlaCo IK **plus separately verified self-collision/path checks**; Pi is the single motor/serial owner with measured feedback, independent limits/watchdog, safe pause/reconnect. Target 50 Hz but measure before optimizing.
- Avoid unsafe shortcuts: no self-collision claims from the URDF/PlaCo wrapper alone; no collisions checked only at the endpoint; no device or dashboard bypass of motion validation; no arbitrary joint-step cap known to stall retraction; no automatic resuming stale commands after reconnect; do not assume gripper degrees.
- New PC dashboard initially monitors only; Android, table collision, camera streaming, presets/plan migration, public-network control and event-driven motor writes are deferred. Preserve modular interfaces for later input/transport additions.
- Treat approved refactor documents as authoritative **future target** wherever older TODO/README suggestions differ, but preserve verified descriptions of current behavior. Record unsettled protocol, collision implementation, watchdog/hold policy and performance criteria as decisions to benchmark, not invented facts.
