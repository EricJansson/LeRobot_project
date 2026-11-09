# Adaptive remote IK and limit recovery

> Current controls: see [manual torque states](REMOTE_IK_TORQUE_STATES.md). Start/Options advances; Back/Share/Create stops/releases by default, retaining the Start-enable/Back-stop pairing below. `--advance-button Back` swaps these actions. All components now require protocol 4, policy `boundary-capture-v4`. Owner-reported boundary capture/recovery succeeded; broader fault checks remain pending.

Current behavior and verification as of 2026-11-09. Applies to the Windows → WSL →
Pi remote teleop entry point; legacy launchers remain unchanged.

**HARDWARE — owner reported:** the tight-wrist escape now works, with slight jitter
while adjusting. This confirms the owner's observed improvement; it does not
complete fault, watchdog, clearance, load or timing verification. This pre-commit
refactor has not been deployed or tested on hardware by the agent. Jitter tuning
is deferred; this cleanup preserves the working motion decisions and thresholds.

## Motion behavior

- Precise Cartesian motion retains priority. If its achieved forward progress is
  less than one eighth of the requested step, try position-only IK, then nearby
  paths within 45°. With owner approval, a 75° escape is tried only when the
  earlier stages still make negligible progress. Useful narrow movement wins.
- Every adapted candidate must be finite, satisfy normal command limits and Pi
  rate allowances, and have positive forward gain. Its exact FK translation is
  no longer than the requested increment and stays inside its chosen angle bound.
  Wider escape ranks forward gain ahead of sideways accuracy, so upward input
  can produce mostly outward movement. Search work is bounded to 43 candidates.
- Accepted adaptation adopts achieved XYZ after exact Pi acknowledgement.
  Unexecuted input is discarded; there is no delayed return to an abandoned line.
  Orientation-only and legacy solver behavior are unchanged.
- Normal arm command limits are inset 0.5° from the intersection of URDF and
  calibrated motor ranges. The separate gripper stays normalized 0–50.

**Self-collision and environment collision checks remain absent.** Joint-range,
rate and FK checks do not establish physical clearance.

## Recovery and stop behavior

Operating-limit excursions inside calibrated ranges recover while armed, even
with active input or LB released. Recovery never arms a disarmed session.
An explicit near-boundary capture can temporarily admit up to 1° of arm feedback
outside a recorded calibration endpoint while all motor goals remain inside it.
This applies only to the captured joint/side and ends on successful recovery;
larger or unrelated powered violations still fault. See [capture rules and tests](REMOTE_IK_TORQUE_STATES.md).

The Pi latches a **fixed destination 5° inside each violated limit**, capped at the
range midpoint. It advances the rate-limited command all the way to that finite
legal destination; a long retreat is not declared stalled merely because one
second has elapsed. The destination does not move in response to tracking error,
and the old sequence of progressively deeper small targets has been removed.
Unaffected joints retain their targets unless fresh input coordinates them. This
does not change motor tuning or repeatedly recapture sag. The five-degree inset
is a destination margin, not a joint-step cap.

Fresh XYZ input can coordinate the other arm joints while offending joints follow
exactly the Pi's mandatory inward step. Coordination is bounded differential IK;
its FK must improve agreement with current input relative to automatic retreat.
The Pi independently validates the proposed joint segment. Recovery advancement
is limited to 10°/s or the configured joint rate, whichever is lower. Gripper-only
limit recovery uses normalized units, capped at 5 units/s and its configured rate;
operator coordination cannot move the jaw. Direction changes during solving drop the coordination
candidate while automatic retreat continues. Centering stops user coordination.

Recovery finishes only after the fixed target has been issued and measured
offending joints remain at least **1° inside the operating limits for 300 ms**.
The equivalent jaw margin is one normalized unit. A sample nearer the boundary
resets the clearance timer. Exact servo tracking is not required: a static load
can leave a measured-versus-commanded offset while still giving adequate clearance.
On success the Pi **retains the issued inward holding target**; it does not replace
it with near-limit measured feedback. Idle FK/reference generation and a later
Holding → Control enabled transition preserve that target. Fresh manual movement
starts from the retained reference with the usual startup ramp.

After the fixed goal has been issued, one
second without 0.1° measured improvement holds **Armed—constrained** (normalized
units for the jaw). A single roughly 0.088° encoder tick does not restart a blocked
episode. An invalid inward transition also holds armed. Retry requires measured
improvement or a meaningful input change. Repeated solver proposals from the same
held input and analog noise do not restart automatic attempts.

**OFFLINE VERIFIED:** regression tests reproduce a static load offset and verify fixed-margin retreat,
sustained measured clearance, retained targets through idle/re-arm, fresh-input
reference continuity, long arm/jaw retreats and blocked encoder/solver noise.
**HARDWARE — owner reported, 2026-11-09:** the final near-boundary capture and recovery
worked well in their trial. Broader stop/reconnect and fault-injection checks remain
pending; the agent performed no physical tests. See the [supervised run sheet](REMOTE_IK_TORQUE_STATES.md).

Explicit Back/stop, control loss/watchdog, invalid feedback, calibrated-range
violations, bus failures and incompatible components retain serious stop/fault
behavior. Back and Esc stop recovery. Exit torque behavior is unchanged. Optional
stationary correction remains disabled by default and is suspended in recovery.

## Components and diagnostics

Windows, WSL and Pi require protocol **4**, policy **boundary-capture-v4**. Startup
compares model identity plus calibrated, operating and command ranges. Reinstall
the stopped Pi service when updating code; WSL runs the Windows checkout directly.
The installer includes the new recovery module and refuses to replace a live service.

`robot_program/remote_ik_recovery.py` owns Pi recovery state, transitions and pure
validation. `MotorSession` owns serialized motor operations and applies those
validation results. The WSL worker uses named candidates and explicit numerical
thresholds. The earlier extraction preserved wire fields; manual release adds explicit torque state and capture/release commands.

Dashboard states distinguish Following path, Adapting pitch, Adapting path,
Recovering and Armed—constrained. Wider moves explain `75° path escape`. Traces
include requested/candidate/issued/measured state, path bound/angle, limit excess,
recovery goal/margin/remaining error and coordination/blocked state. Optional trace,
FK analysis and benchmarks are maintained diagnostics, not temporary motor probes.

## Offline verification

From Windows PowerShell in `C:\Users\rapid\Documents\GitHub\LeRobot_project`:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_remote_ik_benchmark.py tests/test_remote_ik_adaptive.py tests/test_remote_ik.py tests/test_remote_ik_path.py tests/test_lerobot_ik.py tests/test_lerobot_web_dashboard.py -q
wsl.exe -d Ubuntu --exec sh -c 'cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project && ~/.local/share/lerobot-phase0/venv/bin/python -m pytest tests/test_remote_ik_benchmark.py tests/test_remote_ik_adaptive.py tests/test_remote_ik.py tests/test_remote_ik_path.py tests/test_lerobot_ik.py tests/test_lerobot_web_dashboard.py -q'
```

Coverage includes upper/lower/simultaneous excursions, lag, quantization, dead-zone
and load-release fixtures, permanent stalls, fresh-input cancellation, independent
Pi rejection, serious stops, restart exclusivity, isolated Pi bundle imports,
strict legacy behavior and real-PlaCo recorded-pose fixtures.
Final pre-commit results: **Windows 147 passed, 35 skipped; WSL 181 passed,
one platform skip**. The platform/dependency-specific cases skipped on Windows
are exercised by the applicable WSL checks.

The pre-commit comparison matched feedback and writes exactly over 2,990 simulated
motor ticks in 13 scenarios. All 2,463 saved recorded worker requests and the
coordinated-recovery fixture matched numerically exactly. Recordings and comparison
snapshots stay outside the proposed changes and are not test prerequisites.

Benchmark without serial or SSH:

```powershell
wsl.exe -d Ubuntu --exec sh -c 'cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project && ~/.local/share/lerobot-phase0/venv/bin/python -m scripts.benchmark_remote_ik --baseline-revision a95f395 --check'
wsl.exe -d Ubuntu --exec sh -c 'cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project && ~/.local/share/lerobot-phase0/venv/bin/python -m scripts.benchmark_remote_ik --baseline-revision a95f395 --arm-rate 20 --check'
```

The loader now loads the baseline worker's complete namespace, including helpers,
constants and dataclasses, without running its main entry point. This fixes using
the new worker revision as a future baseline. The comparison still uses the current
kinematics adapter; it is a worker-logic comparison, not an isolated historical
Python/environment benchmark. Timing excludes controller, transport and motors.
The 200 and 20°/s progress/reference benchmark gates passed after cleanup.

Earlier recorded-state replay of the approved 75° policy raised median modeled
forward gain in the plateau segment from 0.00177 to 0.50436 mm per request. That
is supporting command evidence, not a prediction of physical tracking. Numerical
corner fixtures are not recommended physical poses. No physical accuracy or strict
20 ms runtime guarantee follows from offline tests.

## Supervised post-cleanup smoke test — owner approval required

1. **Prerequisites/update:** supported arm, clear workspace with outward/sideways
   clearance, physical power disconnect, Windows XInput controller and working
   WSL/Pi. Obtain owner approval for deployment and motion. Stop other serial
   owners and the live service with the arm supported. In Windows PowerShell run
   `cd C:\Users\rapid\Documents\GitHub\LeRobot_project`, then
   `.\.venv\Scripts\python.exe -m scripts.install_remote_ik_pi`.
2. **Launch/ordinary input:** run
   `.\.venv\Scripts\python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 0.1 --trace .runtime/precommit-smoke-01.json`.
   Choose a fresh trace filename to preserve existing recordings. Open the printed
   dashboard URL. Press Start with neutral controls and LB released. From a clear
   bent-arm pose, hold LB and try small XYZ moves and short taps, centering between
   them. Expect the previously working movement, including its existing jitter.
3. **Matched wrist check:** approach the previously successful posture slowly with
   the triggers and the same load/clearance. Hold LB, push RY upward (negative RY,
   positive Z) for at most 0.3 seconds, center for one second, then reverse RY or
   nudge LX/LY for at most 0.3 seconds without cycling LB. Expect the same bounded
   adaptation or armed hold as before cleanup; a 75° escape may move mostly
   sideways. If a limit excursion occurs naturally, observe slow inward recovery
   and fresh-input coordination. Do not physically force an excursion.
4. **Stop/report:** stop on unexpected movement, increased jitter, hunting, failed
   stopping or clearance loss. Back disarms and stops recovery; Esc exits and
   saves the trace. Use the physical disconnect if software stopping fails,
   supporting the arm. Report trace, starting joints/pose/load, exact inputs,
   path bound/angle, recovery fields, jitter and stopping; side-view video helps.

Fault/reconnect and loaded recovery checks still require their own supervised
verification. Roll back using a separate saved checkout/export with a matching
Pi bundle and Windows/WSL code; never reset the working tree or alter calibration
or torque policy as part of rollback.
