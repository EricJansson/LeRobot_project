# Remote SO-101 IK from Windows

Windows samples a gamepad through native XInput or Pygame's SDL controller API and shows a read-only dashboard on loopback. Ubuntu WSL2 runs Hugging Face LeRobot 0.6.1 `RobotKinematics` with PlaCo. The worker strips visual and collision elements from the URDF and preserves the kinematics. This implementation has no self-collision or environment collision checks.

## Set up and run a simulation

Use the Windows project `.venv`. The launcher defaults to `Ubuntu` and `~/.local/share/lerobot-phase0/venv/bin/python` inside WSL2. Override those with `--wsl-distro` and `--wsl-python` if needed. To reproduce the verified WSL2 environment in Ubuntu with Python 3.12 available, run the following.

```bash
python3.12 -m venv ~/.local/share/lerobot-phase0/venv
source ~/.local/share/lerobot-phase0/venv/bin/activate
python -m pip install torch==2.7.1+cpu torchvision==0.22.1+cpu --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'lerobot[feetech,placo-dep]==0.6.1' placo==0.9.15 numpy==2.2.6 cmeel-urdfdom==4.0.1 cmeel-tinyxml2==10.0.0
```

From the Windows checkout, activate `.venv` and run:

```powershell
python.exe -m scripts.main_teleop_remote_ik --dry-run
```

The launcher prints the dashboard link and never opens a browser automatically.
Open http://127.0.0.1:18743/ manually and keep the tab open: it shows the stop
reason after shutdown and reconnects automatically when the launcher restarts.
Use `--dashboard-port 18744` to choose another fixed port, or `--dashboard-port 0`
for a random available port. A busy port causes a startup error instead of an
automatic port change. Open the browser manually. The launcher no longer accepts
`--no-browser` or `--commission-ik`. Pass `--dry-run` or `--pi-sim` explicitly
for simulation because the Windows launcher defaults to the physical arm.

The launcher looks for a gamepad at startup and again after a disconnect. While waiting it checks about once a second, trying XInput slots in order, then the first mapped SDL device. This covers cable and Bluetooth connections, including DualShock 4. The terminal shows the selected controller. Controller loss disarms control. Once the controller returns, release the controls and press the configured advance button (Options/Start by default) to advance from the current torque state. For a specific device or a repeatable test, use `--gamepad xinput` or `--gamepad sdl`, with `--index N` if needed. Either explicit backend defaults to index 0 and reconnects within that selection.

In this Windows setup, the same DualShock 4 appears as **Windows XInput** over cable and **PS4 Controller** over Bluetooth.

1. In Windows **Settings → Bluetooth & devices**, confirm **Wireless Controller** is connected, or plug in the cable. Use the Windows project `.venv`, which includes Pygame, and the WSL2 environment above.
2. In Windows PowerShell from the repository root, run `.\.venv\Scripts\python.exe -m scripts.main_teleop_remote_ik --dry-run`. Expect one gamepad status line when the controller is detected, such as `Gamepad: PS4 Controller` over Bluetooth or `Gamepad: Windows XInput (slot 0)` over cable.
3. Open the printed dashboard URL. While Released, test both sticks, each trigger, D-pad up/down, L1, and then Share. Expect the matching axes and buttons to light up and return to neutral on release. Tap Options last and expect the dashboard to show Start and advance the simulated arm to Holding.
4. With the controls neutral, press Options again to reach **Control enabled**. Turn off the Bluetooth controller or unplug its cable. Expect the dashboard to show the controller disconnected and return to **Holding**. Reconnect over Bluetooth or plug in the cable while the launcher is still running. The gamepad should reappear while the simulated arm stays in **Holding**. Hold L1 and move a stick slightly to confirm there is no movement. Release both, press Options, and repeat the L1 and stick input to confirm control resumes. Press **Esc** to exit. Report the connection type, the terminal messages, and the dashboard state if any step behaves differently.

There are three states: **Released** (blue, all six motors off, including the gripper), **Holding** (position held, controller movement disabled), and **Control enabled** (LB still required for ordinary manual movement). By default **Start / Options** advances Released → Holding → Control enabled using separate neutral presses. **Back / Share / Create** immediately stops control into Holding; a **fresh two-second hold from Holding** releases arm torque. The dashboard displays an animated progress circle. Neutral sticks, triggers and D-pad with LB released are required for advancing or releasing. To swap the two physical buttons, pass `--advance-button Back`. The dashboard and console show the selected mapping. Verify the actual device mapping in simulation first. See [torque states and supervised test steps](REMOTE_IK_TORQUE_STATES.md).

Hold **LB** to drive. The left stick controls XY, right-stick vertical controls Z, right-stick horizontal controls roll, the triggers control pitch, and D-pad up/down operates the gripper. **Esc** stops teleop and exits the Windows launcher, including while the dashboard has focus on the active Windows desktop. The motion scale defaults to 1. Use `--speed` to reduce it. Reconnect disables controller movement. A Released arm stays Released; restoring Holding captures its new measured pose without enabling motion.

**Default actions retain Start/Options to advance and Back/Share/Create to stop, as in the run sheets below.** Neither Holding nor Released runs automatic recovery or stationary correction. Torque-state changes passed offline checks; the owner reported successful boundary capture and recovery. All components now require protocol **4**, policy **boundary-capture-v4**; the launcher updates the stopped Pi service automatically before hardware testing.

Arm motion starts at 10% of the selected scale and eases to full scale over 300 ms. Every observed arm-input gap restarts that ramp. Centering controls or releasing LB stops ordinary operator advancement while retaining intended XYZ. Adaptation adopts achieved XYZ, limit recovery discards the previous path, and disarm/fault/new-session arming resets the reference. LB still enables ordinary movement; automatic limit recovery works independently of LB. The jaw uses its usual scale.

The WSL worker searches bounded fractions of each Cartesian increment within the Pi's reported joint-rate allowance. If precise movement is unavailable or extremely short, it tries orientation relaxation and then a nearby forward path within 45° of input. When those stages make less than one eighth of requested forward progress, an owner-approved escape can deviate up to 75°, prioritizing useful forward gain at the cost of more sideways travel. Every adapted translation remains no longer than the requested increment. Adapted motion continues from achieved, acknowledged XYZ without returning to the abandoned line. Unexecuted input is discarded. Ordinary measured-stop reference recovery remains bounded to 1 mm per command. Legacy solver defaults and explicit orientation-only behavior are unchanged.

Operating-limit excursions inside calibrated motor ranges now recover inward while armed, even during active input and with LB released. Normal arm commands reserve 0.5° of joint margin. Recovery targets a fixed destination 5° inside the violated limit (capped at the range midpoint) and can coordinate other arm joints with fresh input, with independent Pi validation and a 10°/s arm recovery ceiling (5 normalized units/s for the gripper). Blocked recovery holds **Armed—constrained**; serious faults, watchdog/disconnect and explicit stops still disarm. Windows, WSL and Pi require protocol version 4; the launcher updates changed Pi service code automatically. The owner reports that the tight-wrist escape works, with slight adjustment jitter. The pre-commit refactor is offline verified; its post-cleanup hardware smoke test and broader fault checks remain pending. See [current behavior, verification and supervised run sheet](REMOTE_IK_ADAPTIVE_MOTION.md).

Stationary correction is **disabled by default**. An explicitly commissioned JSON profile can be supplied with `--correction-profile PATH`; see [path verification and supervised tests](REMOTE_IK_PATH_VERIFICATION.md). Correction never runs with LB released. This software has not been hardware verified for the new behavior.

The launcher saves a [trace for diagnostics](#traces-and-diagnostics) when you stop the simulation.

To test the real SSH link with the Pi’s simulated bus:

```powershell
python.exe -m scripts.main_teleop_remote_ik --pi-sim
```

The installer defaults to `eric@rpi`, uses the Pi’s existing `~/lerobot-env`, and deploys separate user services with private tokens. SSH needs a trusted host key and noninteractive authentication. The Pi user service manager must stay available after SSH disconnect. This Pi has `Linger=yes` for `eric`.

### Automatic Pi service updates

The real-arm mode and `--pi-sim` check the installed Pi release before starting the control loop. You no longer need to run the installer between implementations. The launcher prints **Checking Pi service**, followed by **Pi service is up to date** or **Installing Pi service** and **Installed Pi service** with a short fingerprint. A fresh Pi or an older installation without a fingerprint is installed automatically using the existing `~/lerobot-env`; Python dependencies are not installed or upgraded.

The fingerprint covers the existing Pi deployment files (including the calibration snapshot), generated kinematic URDF, service units, and deployment recipe. Uncommitted edits count. Timestamps, archive metadata, and PC-only files do not. Hardware settings continue to synchronize separately. Deployment does not perform physical calibration or change motion policy. The payload is captured once at launch; editing code during a session takes effect on the next launch. `--dry-run` never contacts the Pi, and reconnects never deploy code. Startup/reconnect checks the expected release under the deployment lock and refuses a changed or incomplete installation.

Updates refuse to proceed while either Pi service is active, activating, or stopping. The error names the service: close its launcher first, supporting the arm before stopping a live service. Updates never automatically stop another session. An unchanged installation retains the existing service ownership rules. SSH, upload, validation, and installation errors abort startup rather than falling back to stale code. Installation and lifecycle operations share bounded locks; if another operation is busy, wait for it to finish and relaunch. Failed activation attempts restore the previous release and units. Interrupted activation or failed rollback leaves an incomplete-transaction marker that blocks startup even if the old fingerprint matches; the next launch must successfully reinstall with both services stopped. Any rollback error explicitly requires inspection/repair before teleop.

For an unconditional reinstall or repair, stop both Pi services and run this from the Windows checkout with `.venv` active. It preserves existing tokens/settings and starts neither service:

```powershell
python.exe -m scripts.install_remote_ik_pi
```

Both commands accept `--pi user@host` to override `eric@rpi`.

#### Manual simulation update check

Prerequisites: trusted noninteractive SSH, the existing Pi Python environment/user service manager, working Windows gamepad/WSL setup, and both Pi services stopped. Use `--pi-sim` for this check. Run all commands in Windows PowerShell from `C:\Users\rapid\Documents\GitHub\LeRobot_project` with `.venv` active.

1. Run `python.exe -m scripts.main_teleop_remote_ik --pi-sim`. Expect installation if the release is missing/outdated, then the normal simulation dashboard. Open the printed URL; use separate neutral **Start** presses to advance and **LB** plus a small stick input to move the simulated arm. Press **Esc** or **Ctrl+C** to stop.
2. Repeat the same command. Expect **Pi service is up to date**, no upload, and no new release. For confirmation, run `ssh eric@rpi 'readlink ~/.local/share/lerobot-remote-ik/current'` before and after; the path should match. Press **Esc**.
3. Add a temporary `# automatic deployment check` comment to `scripts/remote_ik_motor_service.py`, save, then run the same simulation command. Expect one installation and a new fingerprint/release path. Press **Esc**, remove the comment, save, and launch again to restore the original payload. Expect one installation; stop with **Esc**.
4. Leave simulation running. In a second PowerShell window in the same checkout/venv run `python.exe -m scripts.install_remote_ik_pi`. Expect refusal naming `lerobot-remote-ik-sim.service`; the existing simulation should continue responding. Press **Esc** in its launcher to stop. Report deployment messages, fingerprints/release paths, errors, and whether simulation remained uninterrupted. Remove the temporary comment even if a check fails.

**OFFLINE VERIFIED (2026-11-10):** Windows mocked deployment/startup tests and Linux/WSL fake-systemd transactions cover content fingerprints, fresh/legacy installations, active-service refusal, upload/validation failure, activation rollback, concurrent lifecycle operations, and bounded locks. **PI SIMULATION / HARDWARE:** this update workflow has not yet been verified on the Pi or physical arm; the owner simulation check above remains pending.

## Real arm setup and testing

On Windows, the launcher defaults to the Pi service for the physical arm. You can select it explicitly with `--real-arm`. With `--allow-motion` in the Pi configuration, the dashboard shows **Real arm enabled**. Without that setting it shows **Observe only**. A normal exit with **Esc** or **Ctrl+C** stops the Pi service started by that launcher. If Windows or the network drops out, the Pi watchdog handles the pause. Closing the dashboard tab leaves the launcher running.

Edit `remote_ik.hardware.conf` in the Windows checkout to change settings such as `--arm-rate`. This project config can be tracked in Git. If the file is missing, the first real-arm launch imports the existing settings from the Pi. Subsequent launches use the local file as the source of truth. Press **Esc**, edit the file, then launch again. The launcher copies changed settings to `~/.config/lerobot-remote-ik/hardware.env` on the Pi before starting the service. Service code is checked and updated automatically at launch.

```powershell
notepad .\remote_ik.hardware.conf
python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 0.5
```

In real-arm mode, add `--relax-on-exit` to release all motor torque, including the gripper, when the Pi service stops normally. This requires `--allow-motion` in the Pi settings. Support the arm before exiting because it can drop when the motors release. By default, exit preserves the existing torque state: a holding arm keeps holding and a Released arm stays Released. The in-session release gesture also releases all six channels, including the gripper.

```powershell
python.exe -m scripts.main_teleop_remote_ik --real-arm --relax-on-exit --speed 0.25
```

### Observe the physical arm first

1. From the Windows checkout with `.venv` active, deploy the current service and run the simulator with the Windows gamepad.

   ```powershell
   python.exe -m scripts.main_teleop_remote_ik --pi-sim
   ```

   Verify that neutral **Start** arms, **LB** plus a stick changes the simulated positions, and **Back** disarms. Press **Esc** to exit.

2. After the owner approves read-only serial access, support the arm in a clear area and stop any standalone teleop or other serial owner. From Windows, list the Pi’s adapter paths and open the local settings file.

   ```powershell
   ssh eric@rpi 'ls -l /dev/serial/by-id/'
   notepad .\remote_ik.hardware.conf
   ```

   Put this single line in the local file, substituting the actual adapter ID if it differs.

   ```ini
   REMOTE_IK_HARDWARE_ARGS="--serial-port /dev/serial/by-id/usb-1a86_USB_Single_Serial_5AB0182759-if00"
   ```

3. Connect the gamepad to Windows and run the commissioning launcher from the Windows checkout.

   ```powershell
   python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 0.1
   ```

   Use `--pi user@host` on both the installer and launcher if your SSH destination differs from `eric@rpi`. Open the printed dashboard URL and check for **Observe only**, six measured positions, and plausible XYZ for the starting pose. Press **Start** with neutral controls and LB released, then try **LB** plus a stick. The Pi should stay disarmed and report that motion is disabled. The last motor target and Pi write timing should remain blank. Observation checks the existing calibration and reads positions without changing torque or writing goals.

4. After about 30 seconds, report the six positions, the displayed XYZ, feedback rate, longest feedback gap, longest Pi read, and any error. A screenshot is enough. Press **Esc** on Windows. For startup or bus errors, collect `journalctl --user -u lerobot-remote-ik.service -n 40 --no-pager` on the Pi.

### First supervised motion

Use the observation results to agree on the initial cycle and watchdog before enabling motion. In `remote_ik.hardware.conf`, add `--allow-motion`, `--hz`, `--watchdog-ms`, `--arm-rate`, and `--gripper-rate` to the serial path. Motion mode requires explicit values for all four settings. Arm rate uses degrees per second. Gripper rate uses normalized units per second, with a command range of 0–50. For the first trial, use `--real-arm --speed 0.1`.

1. After separate owner approval, start from a supported, bent-arm pose with clearance and a physical power disconnect available. Confirm **Real arm enabled** and **Holding** in the dashboard; if Released, use one neutral advance press to capture Holding first.
2. Press **Start** with the controls neutral and LB released. Check that the arm holds its starting pose. Hold **LB**, nudge the left stick slightly left for less than a second, then release **LB**. Expect a small sideways movement in the model’s positive Y direction and no further advancement after release. Press **Back**, then repeat the nudge with LB held and check that it produces no movement. End the trial on unexpected movement or a failed stop. Use the physical disconnect if software stop fails.
3. Report the stick direction, visible movement, measured joint change, longest Pi write time, stop behavior, and any fault. Verify the other axes, gripper, retraction, controller loss, and SSH interruption in follow-up trials. A reconnect must stay disarmed until a fresh neutral **Start**.
4. Press **Esc** to exit and stop the Pi service. Support the arm before stopping the service or cutting power. If shutdown fails or an older launcher left the service running, run `systemctl --user stop lerobot-remote-ik.service` on the Pi with the arm supported. Remove `--allow-motion` from the local `remote_ik.hardware.conf` to return the next launch to observation mode.

## Service startup reports address already in use

If the service journal shows `OSError: [Errno 98] Address already in use` at
`listener.bind`, startup failed before opening the motor bus. Check the current
owner from Windows with `ssh eric@rpi 'ss -ltnp sport = :8766'`; do not terminate
an unidentified owner. The Pi listener now permits immediate Linux restart after
a closed connection enters TIME_WAIT, while still rejecting a second live
listener. Windows uses exclusive binding. Close the launcher and relaunch; changed
service code deploys automatically. Use `python.exe -m scripts.install_remote_ik_pi`
with both services stopped if an explicit repair is needed.
The restart/exclusivity fix passed targeted offline checks (99 WSL tests and
56 Windows tests, three Windows platform/dependency skips); no physical service
restart or motor test was performed during this fix.

## Verification status

- **OFFLINE VERIFIED, torque states** · Fake motor/register tests cover confirmed six-motor release, recoverable Released-range warnings, new-pose capture before torque enable, release errors, reconnects and neutral/two-second gesture rules. Frontend checks cover countdown, confirmation and blue Released styling. The owner reported successful boundary capture and recovery on 2026-11-09; broader fault checks remain pending. The agent performed no hardware tests; see the [run sheet](REMOTE_IK_TORQUE_STATES.md).
- **OFFLINE VERIFIED** · Official WSL2 IK accepted a reachable pose and rejected an unreachable pose in a checkout with no assets directory. Regression tests cover command age, reconnect, held targets, and Linux serial exclusivity on a pseudo-terminal.
- **OFFLINE VERIFIED** · Interrupting the SSH tunnel during simulated motion caused a disconnect. The launcher reconnected disarmed while LB was still held.
- **OFFLINE VERIFIED** · Local simulation sustained 50 updates/s. The Pi simulation over SSH delivered 29.2 updates/s with the Pi cycle set to 50 Hz. Measure physical bus timing before choosing live settings.
- **HARDWARE VERIFIED, initial supervised trial** · The owner observed a small sideways move from the Windows gamepad and confirmed that releasing LB stopped advancement. The trial used a configured Pi cycle of 50 Hz, a 200 ms watchdog, an arm rate of 20 degrees/s, a gripper rate of 10 normalized units/s, and Windows input speed `0.1`. The Windows dashboard reported 24.9 Hz feedback during the trial.
- **HARDWARE VERIFICATION PENDING** · Other axes, gripper, extended-arm retraction, controller loss, and SSH interruption remain follow-up tests.

## Control-code ownership

The launcher owns arguments, Windows gamepad and Esc sampling, configurable advance/stop buttons, WSL/Pi connections and dashboard lifecycle. `robot_program/remote_ik_control.py` owns input-edge tracking, the release timer, per-session `MotionState`, startup ramping, post-solve input checks, issued-reference acceptance and reporting. Stationary correction remains optional in `robot_program/remote_ik_correction.py`. The worker owns IK and candidate selection; `robot_program/remote_ik_recovery.py` owns Pi-local recovery state and pure validation, while the motor service alone owns serialized torque and goal operations. Manual release requires protocol 4, policy `boundary-capture-v4`; legacy standalone launchers are unchanged.

## Traces and diagnostics

From the Windows checkout root, either command starts the real-arm mode and uses the default trace path.

```powershell
python -m scripts.main_teleop_remote_ik
python -m scripts.main_teleop_remote_ik --real-arm --trace .runtime/remote-ik-latest.json
```

All launch modes save up to the last 3,000 completed control ticks on shutdown and print the saved path. A successful write with recorded ticks replaces the previous capture. A run with zero ticks or a failed write preserves the previous capture. Check the file modification time and `provenance.mode` when reviewing the latest run. To preserve captures across runs, pass a fresh path such as `--trace .runtime/remote-ik-test.json`.

Summarize the latest capture in Windows PowerShell from the checkout root.

```powershell
python -m scripts.remote_ik_diagnostics .runtime/remote-ik-latest.json
```

For modeled issued and measured XYZ, enrich a copy with WSL and pass that copy to the same diagnostics command.

```powershell
wsl.exe -d Ubuntu --exec sh -lc '~/.local/share/lerobot-phase0/venv/bin/python -m scripts.remote_ik_worker --analyze-trace .runtime/remote-ik-latest.json' |
    Set-Content -Encoding utf8 .runtime/remote-ik-latest-xyz.json
python -m scripts.remote_ik_diagnostics .runtime/remote-ik-latest-xyz.json
```

Compare `desired_xyz_m`, `issued_xyz_m`, and `measured_xyz_m` around a drop. The `xyz_reset` and `op` fields identify reference resets and holds. The difference between `issued_before` and `issued` shows the joint target step. See the supervised smoke test in [path verification](REMOTE_IK_PATH_VERIFICATION.md).
