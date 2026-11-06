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

Connect an XInput gamepad to Windows. The launcher uses slot 0 by default. Use `--index` to select another slot. The dashboard shows sticks, triggers, D-pad and buttons before arming.

Press **Start** with the sticks, triggers, and D-pad neutral and LB released. Hold **LB** to drive. The left stick controls XY, right-stick vertical controls Z, right-stick horizontal controls roll, the triggers control pitch, and D-pad up/down operates the gripper. **Back** disarms. **Esc** stops teleop and exits the Windows launcher, including while the dashboard has focus on the active Windows desktop. Use `--speed` to reduce the motion scale below the default 0.25. Reconnect requires another neutral Start press.

To test the real SSH link with the Pi’s simulated bus:

```powershell
python.exe -m scripts.install_remote_ik_pi
python.exe -m scripts.main_teleop_remote_ik --pi-sim
```

The installer defaults to `eric@rpi`, uses the Pi’s existing `~/lerobot-env`, and deploys separate user services with private tokens. SSH needs a trusted host key and noninteractive authentication. The Pi user service manager must stay available after SSH disconnect. This Pi has `Linger=yes` for `eric`.

## Hardware commissioning

Hardware mode is disabled until the owner approves the serial, torque, and stop/hold trials and supplies a Pi configuration at `~/.config/lerobot-remote-ik/hardware.env`. Keep it private with mode `0600`. Set `REMOTE_IK_HARDWARE_ARGS` to the arguments `--serial-port`, `--hz`, `--watchdog-ms`, `--arm-rate`, and `--gripper-rate` with verified values. Use the Pi’s `/dev/serial/by-id/` path, not a guessed `/dev/ttyACM*` index. The installed adapter was `usb-1a86_USB_Single_Serial_5AB0182759-if00`. `--arm-rate` is in degrees per second and `--gripper-rate` is in normalized units per second. The gripper command range is 0–50 normalized units.

Commission with the arm supported, a clear work area, and power disconnect available. Verify that existing motor tuning is present and that calibration and measured joint directions match the model. The service checks calibration on attach and uses the existing tuning. On first arm it writes a measured hold before enabling torque. On a pause it writes one measured hold and leaves the Pi service running. Validate this policy under load, including extended-arm retraction, LB release, controller loss, and a network outage. The service does not turn torque off when Windows exits. Stop it with the arm supported before switching to a legacy motor owner. A software stop depends on a working motor bus.

Once commissioned, use the one-command launcher:

```powershell
python.exe -m scripts.main_teleop_remote_ik
```

To see service status, use `ssh eric@rpi 'systemctl --user status lerobot-remote-ik.service'`. For a configuration or motor fault, inspect `journalctl --user -u lerobot-remote-ik.service` on the Pi before restarting. The simulated unit is `lerobot-remote-ik-sim.service`.

## Verification status

- **OFFLINE VERIFIED** · Official WSL2 IK accepted a reachable pose and rejected an unreachable pose in a checkout with no assets directory. Regression tests cover command age, reconnect, held targets, and Linux serial exclusivity on a pseudo-terminal.
- **OFFLINE VERIFIED** · Interrupting the SSH tunnel during simulated motion caused a disconnect. The launcher reconnected disarmed while LB was still held.
- **OFFLINE VERIFIED** · Local simulation sustained 50 updates/s. The Pi simulation over SSH delivered 29.2 updates/s with the Pi cycle set to 50 Hz. Measure physical bus timing before choosing live settings.
- **HARDWARE VERIFICATION PENDING** · The owner confirmed the GAM-139 works with native XInput in the local simulator. Actual motor cadence, stop/hold, and motion under load still require owner-approved tests.
