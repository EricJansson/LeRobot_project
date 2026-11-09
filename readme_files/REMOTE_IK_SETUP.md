# Remote SO-101 IK from Windows

Windows samples a gamepad through native XInput and shows a read-only dashboard on loopback. Ubuntu WSL2 runs Hugging Face LeRobot 0.6.1 `RobotKinematics` with PlaCo. The worker strips visual and collision elements from the URDF and preserves the kinematics. This implementation has no self-collision or environment collision checks.

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
automatic port change. Browser opening is always manual; the obsolete `--no-browser` and `--commission-ik` flags have been removed. Use `--real-arm` for physical hardware.
These defaults apply to the Windows remote IK launcher; standalone Pi defaults
are unchanged.

Connect an XInput gamepad to Windows. The launcher uses slot 0 by default. Use `--index` to select another slot. The dashboard shows sticks, triggers, D-pad and buttons before arming.

Press **Start** with the sticks, triggers, and D-pad neutral and LB released. Hold **LB** to drive. The left stick controls XY, right-stick vertical controls Z, right-stick horizontal controls roll, the triggers control pitch, and D-pad up/down operates the gripper. **Back** disarms. **Esc** stops teleop and exits the Windows launcher, including while the dashboard has focus on the active Windows desktop. The motion scale defaults to 1. Use `--speed` to reduce it. Reconnect requires another neutral Start press.

Arm motion starts at 10% of the selected scale and eases to full scale over 300 ms. Every observed arm-input gap restarts that ramp. Centering controls or releasing LB stops ordinary operator advancement while retaining intended XYZ. Adaptation adopts achieved XYZ, limit recovery discards the previous path, and disarm/fault/new-session arming resets the reference. LB still enables ordinary movement; automatic limit recovery works independently of LB. The jaw uses its usual scale.

The WSL worker searches bounded fractions of each Cartesian increment within the Pi's reported joint-rate allowance. If precise movement is unavailable or extremely short, it tries orientation relaxation and then a nearby forward path within 45° of input. When those stages make less than one eighth of requested forward progress, an owner-approved escape can deviate up to 75°, prioritizing useful forward gain at the cost of more sideways travel. Every adapted translation remains no longer than the requested increment. Adapted motion continues from achieved, acknowledged XYZ without returning to the abandoned line. Unexecuted input is discarded. Ordinary measured-stop reference recovery remains bounded to 1 mm per command. Legacy solver defaults and explicit orientation-only behavior are unchanged.

Operating-limit excursions inside calibrated motor ranges now recover inward while armed, even during active input and with LB released. Normal arm commands reserve 0.5° of joint margin. If that initial retreat stalls, recovery can try a finite deeper destination up to 2° inside the limit and coordinate other arm joints with fresh input, with independent Pi validation and a 5°/s recovery ceiling. Blocked recovery holds **Armed—constrained**; serious faults, watchdog/disconnect and explicit stops still disarm. Windows, WSL and Pi require protocol version 3; reinstall the stopped Pi service when updating code. The owner reports that the tight-wrist escape works, with slight adjustment jitter. The pre-commit refactor is offline verified; its post-cleanup hardware smoke test and broader fault checks remain pending. See [current behavior, verification and supervised run sheet](REMOTE_IK_ADAPTIVE_MOTION.md).

Stationary correction is **disabled by default**. An explicitly commissioned JSON profile can be supplied with `--correction-profile PATH`; see [path verification and supervised tests](REMOTE_IK_PATH_VERIFICATION.md). Correction never runs with LB released. This software has not been hardware verified for the new behavior.

For a short diagnostic run, add `--trace .runtime/remote-ik-test.json` to the launcher command. After you press **Esc**, the launcher saves up to the last 3,000 completed control ticks, overwriting any existing file at that path. If no ticks were recorded, the existing file is left unchanged. From the Windows checkout, add modeled issued and measured XYZ to a copy using the default WSL environment.

```powershell
wsl.exe -d Ubuntu --exec sh -lc '~/.local/share/lerobot-phase0/venv/bin/python -m scripts.remote_ik_worker --analyze-trace .runtime/remote-ik-test.json' |
    Set-Content -Encoding utf8 .runtime/remote-ik-test-xyz.json
```

Compare `desired_xyz_m`, `issued_xyz_m`, and `measured_xyz_m` around a drop. The `xyz_reset` and `op` fields identify reference resets and holds. The difference between `issued_before` and `issued` shows the joint target step.

To test the real SSH link with the Pi’s simulated bus:

```powershell
python.exe -m scripts.install_remote_ik_pi
python.exe -m scripts.main_teleop_remote_ik --pi-sim
```

The installer defaults to `eric@rpi`, uses the Pi’s existing `~/lerobot-env`, and deploys separate user services with private tokens. SSH needs a trusted host key and noninteractive authentication. The Pi user service manager must stay available after SSH disconnect. This Pi has `Linger=yes` for `eric`.

## Real arm setup and testing

On Windows, `--real-arm` starts and connects to the Pi service for the physical arm. With `--allow-motion` in the Pi configuration, the dashboard shows **Real arm enabled**. Without that setting it shows **Observe only**. A normal exit with **Esc** or **Ctrl+C** stops the Pi service started by that launcher. If Windows or the network drops out, the Pi watchdog handles the pause. Closing the dashboard tab leaves the launcher running.

Edit `remote_ik.hardware.conf` in the Windows checkout to change settings such as `--arm-rate`. This project config can be tracked in Git. If the file is missing, the first real-arm launch imports the existing settings from the Pi. Subsequent launches use the local file as the source of truth. Press **Esc**, edit the file, then launch again. The launcher copies changed settings to `~/.config/lerobot-remote-ik/hardware.env` on the Pi before starting the service. Run the installer to deploy service code when needed.

```powershell
notepad .\remote_ik.hardware.conf
python.exe -m scripts.main_teleop_remote_ik --real-arm --speed 0.5
```

Add `--relax-on-exit` to release motor torque when the Pi service stops normally. This requires `--allow-motion` in the Pi settings and the updated Pi service installed with `python.exe -m scripts.install_remote_ik_pi`. Support the arm before exiting because it can drop when the motors release. By default, the motors keep holding their last position after exit.

```powershell
python.exe -m scripts.main_teleop_remote_ik --real-arm --relax-on-exit --speed 0.25
```

### Observe the physical arm first

1. From the Windows checkout with `.venv` active, deploy the current service and run the simulator with the Windows gamepad.

   ```powershell
   python.exe -m scripts.install_remote_ik_pi
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

1. After separate owner approval, start from a supported, bent-arm pose with clearance and a physical power disconnect available. Confirm **Real arm enabled** and **Disarmed** in the dashboard.
2. Press **Start** with the controls neutral and LB released. Check that the arm holds its starting pose. Hold **LB**, nudge the left stick slightly left for less than a second, then release **LB**. Expect a small sideways movement in the model’s positive Y direction and no further advancement after release. Press **Back**, then repeat the nudge with LB held and check that it produces no movement. End the trial on unexpected movement or a failed stop. Use the physical disconnect if software stop fails.
3. Report the stick direction, visible movement, measured joint change, longest Pi write time, stop behavior, and any fault. Verify the other axes, gripper, retraction, controller loss, and SSH interruption in follow-up trials. A reconnect must stay disarmed until a fresh neutral **Start**.
4. Press **Esc** to exit and stop the Pi service. Support the arm before stopping the service or cutting power. If shutdown fails or an older launcher left the service running, run `systemctl --user stop lerobot-remote-ik.service` on the Pi with the arm supported. Remove `--allow-motion` from the local `remote_ik.hardware.conf` to return the next launch to observation mode.

## Service startup reports address already in use

If the service journal shows `OSError: [Errno 98] Address already in use` at
`listener.bind`, startup failed before opening the motor bus. Check the current
owner from Windows with `ssh eric@rpi 'ss -ltnp sport = :8766'`; do not terminate
an unidentified owner. The Pi listener now permits immediate Linux restart after
a closed connection enters TIME_WAIT, while still rejecting a second live
listener. Windows uses exclusive binding. Reinstall the stopped service with
`python.exe -m scripts.install_remote_ik_pi`, then run the normal launcher.
The restart/exclusivity fix passed targeted offline checks (99 WSL tests and
56 Windows tests, three Windows platform/dependency skips); no physical service
restart or motor test was performed during this fix.

## Verification status

- **OFFLINE VERIFIED** · Official WSL2 IK accepted a reachable pose and rejected an unreachable pose in a checkout with no assets directory. Regression tests cover command age, reconnect, held targets, and Linux serial exclusivity on a pseudo-terminal.
- **OFFLINE VERIFIED** · Interrupting the SSH tunnel during simulated motion caused a disconnect. The launcher reconnected disarmed while LB was still held.
- **OFFLINE VERIFIED** · Local simulation sustained 50 updates/s. The Pi simulation over SSH delivered 29.2 updates/s with the Pi cycle set to 50 Hz. Measure physical bus timing before choosing live settings.
- **HARDWARE VERIFIED, initial supervised trial** · The owner observed a small sideways move from the Windows gamepad and confirmed that releasing LB stopped advancement. The trial used a configured Pi cycle of 50 Hz, a 200 ms watchdog, an arm rate of 20 degrees/s, a gripper rate of 10 normalized units/s, and Windows input speed `0.1`. The Windows dashboard reported 24.9 Hz feedback during the trial.
- **HARDWARE VERIFICATION PENDING** · Other axes, gripper, extended-arm retraction, controller loss, and SSH interruption remain follow-up tests.

## Control-code ownership

The launcher owns arguments, Windows XInput/Esc sampling, WSL/Pi connections and dashboard lifecycle. `robot_program/remote_ik_control.py` owns input-edge tracking, per-session `MotionState`, startup ramping, post-solve input checks, issued-reference acceptance and reporting. Stationary correction remains optional in `robot_program/remote_ik_correction.py`. The worker owns IK and candidate selection; `robot_program/remote_ik_recovery.py` owns Pi-local recovery state and pure validation, while the motor service owns serialized hardware operations. This pre-commit extraction preserves protocol 3, policy `adaptive-limits-v3` and numerical motion behavior.

The cleanup preserves this command and all of its defaults:

```powershell
python.exe -m scripts.main_teleop_remote_ik --real-arm --trace .runtime/feedback-test.json
```

Trace files are still overwritten when a run records ticks; select a fresh path when preserving an earlier recording. See the cleanup verification and supervised smoke test in [path verification](REMOTE_IK_PATH_VERIFICATION.md).
