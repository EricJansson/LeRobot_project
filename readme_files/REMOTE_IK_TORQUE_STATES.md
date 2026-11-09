# Manual repositioning and torque states

**OFFLINE VERIFIED / HARDWARE — OWNER-REPORTED SUCCESS.** Applies only to
`scripts.main_teleop_remote_ik`; standalone Pi and legacy modes are unchanged.
No calibration files, PID, acceleration or command limits are changed.
On 2026-11-09 the owner reported that the final boundary capture and recovery worked
well. This confirms their normal-use trial; the full stop, reconnect, partial-release
and fault-injection matrix remains pending. The agent performed no physical tests.

## Controls

| State | Meaning | Advance press | Stop/release button |
| --- | --- | --- | --- |
| Released (blue) | All six motors, including the gripper, have confirmed torque off | Capture measured pose and enable Holding only | No change |
| Holding | Powered holding; operator movement, correction and recovery disabled | Enable control with neutral input | Fresh continuous two-second hold releases arm torque |
| Control enabled | LB gates manual driving; existing limit recovery can run independently of LB | No change | Immediately return to Holding |

Default advance: **XInput Start**, usually **Options** on PlayStation pads.
Default stop/release: **XInput Back**, usually **Share/Create**. These are distinct
physical buttons; Options and XInput Start are ordinarily the same button.
`--advance-button Back` swaps them. The default preserves the previous
Start-to-enable and Back-to-stop pairing. Verify your actual XInput translation in local simulation;
the dashboard graphic stays in raw physical mapping and the hints show the selected
actions. Older recovery/path run sheets use the same default pairing.

Every advance needs a new press with sticks, triggers and D-pad neutral and LB
released. A held advance cannot jump from Released straight into Control enabled.
A stop press used to leave Control enabled cannot also release torque: release
that button, then hold it again from Holding. Letting go, operating any movement
control/LB or the advance button, or losing fresh input cancels the countdown and
requires a new stop press. A sampling gap of 50 ms also cancels a release gesture.
The read-only dashboard animates a circle using producer-reported progress; the
browser cannot issue commands or complete the gesture itself.

At the threshold the dashboard can show **Releasing…** while waiting for the Pi.
Only successful disable and read-back of all six motor torque registers confirms
Released. Partial release or bus errors latch Fault; do not assume every joint is
free. Support the arm throughout and report errors. The gesture releases the gripper
as well as all five arm joints. Use an empty gripper when testing because held objects
can fall. The separate `--relax-on-exit` option still
releases all channels, including the gripper, on requested shutdown.

Restoring Holding writes a measured six-channel goal **before** enabling torque,
with the bounded boundary adjustment described below when needed, clears prior
motion references and leaves controller motion disabled. Keep
the arm still and supported until Holding is confirmed. Capturing a pose outside
operating limits but inside calibrated ranges does not initiate recovery; explicitly
enabling control can initiate existing inward recovery. Capture/release does not
establish collision clearance.

Disconnect, watchdog and ordinary exit never automatically restore torque on a
Released arm. Service restart reads existing torque on all six channels without toggling it.
Mixed/unknown torque displays unavailable and needs explicit capture with valid
feedback, or investigation if Fault is shown. Communication loss while powered
retains existing stop/hold behavior, with controller movement disabled on reconnect.

While Released, moving outside a calibrated range gives a blue-state warning
naming the joint and its recorded range, rather than latching a motor fault.
Explicit Options capture may accept an arm joint **up to 1° outside that range**.
It writes the nearest recorded boundary as the holding goal before enabling torque;
the arm can make a small inward adjustment of up to 1°. All issued goals stay
inside recorded calibration, and telemetry keeps the actual unmodified feedback.
Holding does not automatically run the larger recovery. A separate Options press
enables controlled inward recovery toward the existing five-degree legal margin.

The temporary measurement allowance applies only to the originally captured
joint and exceeded side, until recovery completes. It cannot authorize a new
joint/side violation, an excursion beyond 1°, or normal out-of-range driving.
Stop/watchdog/reconnect disable controller motion and retain a legal holding goal.
The gripper has no angular tolerance. Larger arm excursions must be repositioned
by hand closer to the recorded range before capture is allowed. Ordinary powered
range violations, nonfinite feedback, serial failures and partial torque release
remain latched faults.

This allowance handles small near-boundary discrepancies without changing
calibration files, command bounds or unmeasured full-retraction endpoints.
Correcting larger missing calibration ranges requires a separately approved
hardware measurement/recalibration procedure.

The gripper's calibrated feedback range is obtained from its actual normalization
(0–100 for the installed follower); its commanded-motion range remains 0–50.
A manually opened gripper can be captured at its measured calibrated position.
Holding does not close it; enabling control can initiate inward recovery back
toward the command range, including gripper movement without LB. Recovery uses a
fixed target 5° inside the violated arm limit (5 normalized units for the jaw),
capped at the range midpoint, at up to 10°/s for arm joints and 5 normalized units/s
for the gripper, always bounded by configured rates. Completion requires the
target issued and measured clearance of at least 1° for 300 ms (one normalized
unit for the jaw). The Pi retains that inward holding command after completion;
idle and re-arming do not replace it with near-limit feedback. A failed attempt
holds without repeated retries from unchanged stick input or a single encoder
tick. See [recovery behavior](REMOTE_IK_ADAPTIVE_MOTION.md).

Windows, WSL and Pi require protocol **4**, policy **boundary-capture-v4**. Install the
updated Pi service only when stopped and with the arm supported.

## Offline checks

From PowerShell in `C:\Users\rapid\Documents\GitHub\LeRobot_project`:

```powershell
.venv\Scripts\python.exe -m pytest tests/test_remote_ik_release.py tests/test_remote_ik_recovery.py tests/test_remote_ik.py tests/test_remote_ik_adaptive.py tests/test_remote_ik_path.py tests/test_lerobot_web_dashboard.py -q
```

These checks use fake motors/registers and loopback TCP. Platform/dependency skips
do not establish physical behavior. No temporary motor instrumentation is added.

## Manual local simulation (no physical arm)

1. **Prerequisites/location:** use the Windows checkout above, its `.venv`, the WSL2
   worker from [setup](REMOTE_IK_SETUP.md), and a Windows XInput gamepad. In PowerShell:

   ```powershell
   .venv\Scripts\python.exe -m scripts.main_teleop_remote_ik --dry-run
   ```

   Open the printed dashboard URL. Confirm Local simulation. Check which raw
   button lights for Share/Create and Options. If necessary, stop with Esc and
   relaunch with `--advance-button Back` to swap actions.
2. **Release/cancel:** from Holding with neutral controls and LB released, hold
   Share/Create for one second, then release. Expect a partial circle then cancellation,
   with Holding unchanged. Make a fresh hold for over two seconds: expect blue
   Released. Holding longer must not generate repeated transitions.
3. **Capture/enable:** tap Options once. Expect Holding. Keep it pressed for
   three seconds: it must stay Holding. Release and tap again: expect Control
   enabled. Hold LB and nudge LX slightly for 0.2 seconds, then center/release LB.
   Expect a small simulated move. Tap Share/Create: expect Holding. Keep that stopping
   press held for three seconds: torque must stay enabled. Release, then hold
   Share/Create for over two seconds to enter Released.
4. **Gates/reconnect:** while Holding, try a two-second Share/Create hold separately with
   LB held, LX deflected, a trigger depressed and D-pad up. None may release torque.
   Centering while Share/Create stays held must not restart the countdown. Disconnect
   and reconnect the gamepad during a countdown: expect cancellation and a fresh
   press requirement. While Released, controller reconnect must stay Released.
   Stop with Esc; report mapping, observed states, circle behavior and any errors.

## Supervised hardware trial — separate owner approval required

1. **Prerequisites:** complete simulation, obtain owner approval for physical torque
   testing, use an empty gripper and clear bent-arm pose, stop other serial owners,
   support the arm and keep a physical power disconnect accessible. Preserve approved
   calibration and motion settings. Stop the Pi service before replacement. From
   Windows PowerShell in the checkout above, after approval:

   ```powershell
   .venv\Scripts\python.exe -m scripts.install_remote_ik_pi
   .venv\Scripts\python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 0.1
   ```

   Do not use `--relax-on-exit` for this trial. The existing local hardware config
   must contain the approved `--allow-motion`, cycle, watchdog and rate settings;
   do not invent or change them for this test. Confirm Real arm enabled.
2. **Supported release:** if already Released at startup, hold it still and press
   neutral Options once to restore Holding. If unavailable or Fault is shown,
   stop and report it. From Holding, support the arm before holding Share/Create. Cancel
   at one second: expect no release. Make a fresh two-second hold: expect all six
   motors released, blue Released and a freely movable empty gripper.
3. **Reposition/capture:** gently move the supported arm a small distance within a
   known clear calibrated range. Keep it still and tap Options once. Expect
   powered Holding at the new pose with no return toward the old target. Support it
   until Holding is confirmed. Gently reposition the empty gripper too: expect torque
   off while Released and holding at its new position after capture. If a range warning
   appears during normal repositioning, return the named joint inside the shown range,
   then press Options again: expect Holding without a restart. Within the one-degree
   arm allowance, expect a legal boundary holding target and a small inward capture
   adjustment; a second separate Options press starts recovery. Do not deliberately
   force joints past a limit. Do not assume collision protection.
4. **Enable/stop:** release and tap Options again. Expect Control enabled. Hold
   LB and make one small LX nudge for 0.2 seconds, then center/release LB. Tap Share/Create
   to enter Holding. Keep that press held for three seconds: it must stay Holding.
   Release and retry a fresh two-second hold while supporting the arm. Disconnect
   and reconnect the controller while Released: torque must remain off, and one
   advance press must restore Holding without enabling controller movement.
5. **Stop/report:** end immediately on a jump, unexpected torque transition, failed
   stop, partial release, loss of clearance or bus error. Keep supporting the arm;
   use the physical disconnect if software stopping fails. Esc stops the launcher
   and its Pi service, preserving torque state by default. Report actual mapping,
   pose/load, release/capture behavior, jerk/sag, gripper behavior, reconnect result
   and full dashboard error. For bus failures, collect on the Pi:

   ```bash
   journalctl --user -u lerobot-remote-ik.service -n 40 --no-pager
   ```

   Hardware results remain separate from offline results until reported by the
   owner. The agent performed no hardware trial.

## Supervised recovery regression after hand repositioning

1. **Prerequisites/location:** obtain owner approval for physical recovery testing;
   finish the simulation and supported release checks above. Use the same empty,
   clear, supported pose/load as the failing recording, only if the owner confirms
   clearance. Keep calibrated ranges and motor tuning unchanged and have a physical
   disconnect available. With the Pi service stopped, from PowerShell in
   `C:\Users\rapid\Documents\GitHub\LeRobot_project`:

   ```powershell
   .venv\Scripts\python.exe -m scripts.install_remote_ik_pi
   .venv\Scripts\python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 0.1 --trace "$env:TEMP\remote-ik-legal-recovery.json"
   ```

   `--speed` scales manual input; automatic recovery retains its own 10°/s arm ceiling
   and configured-rate bound. The gripper uses normalized units, not degrees.
2. **Inputs:** with neutral sticks/triggers/D-pad and LB released, enter Released
   using a fresh two-second Share/Create hold. Gently place the supported arm in
   the previously failing pose, inside calibrated ranges. Do not force any joint
   past a limit. Press Options once and release: expect Holding at that pose and
   no automatic recovery. Press Options again and release, leaving LB released.
   If the same natural retracted pose reports a boundary warning within 1°, the
   first Options press should capture a boundary goal instead of remaining Released.
   Support the arm for that small adjustment. Do not deliberately push farther past
   any physical limit. A warning over 1° still requires manual repositioning.
3. **Expected:** Control enabled can show Recovering while offending joints
   advance inward at the capped rate toward a fixed target 5° inside the boundary.
   Measured clearance must remain at least 1° inside for 300 ms before completion.
   After completion leave controls neutral and LB released for two seconds: expect
   the inward holding target to remain unchanged, no repeated recovery and no
   reset to near-limit feedback. Press Share/Create once, then neutral Options:
   expect the retained holding target to survive re-arming. Unaffected joints
   should hold because there is no stick coordination. Repeat with one small
   deliberate stick direction only after neutral recovery succeeds; expect the
   first movement to continue from the retained reference. A stalled attempt may
   hold constrained, but unchanged held input must not produce repeated jiggles.
4. **Stop/report:** Share/Create immediately returns to Holding; Esc exits and saves
   `%TEMP%\remote-ik-legal-recovery.json`, preserving torque state by default.
   Stop immediately on unexpected movement, failed stop, hunting or lost clearance;
   support the arm and use the physical disconnect if necessary. Report that trace,
   starting pose/load, which joint failed, measured versus issued movement, whether
   normal control resumed, and any jitter or constrained status. A fixed side-view
   recording helps separate physical movement from encoder feedback. No motor
   instrumentation is added and the agent has performed no physical trial.
