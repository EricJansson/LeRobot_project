# Connection and run guide

> **Development resumes on the Windows PC, not on the Raspberry Pi.** Current commands here run existing legacy modes and diagnostics. The approved PC-started Windows→WSL2 IK/self-collision→Pi motor service is **not yet implemented**. Do not interpret `usbipd` WSL USB-forwarding or Pi-connected gamepad steps below as instructions for the new design. See [target architecture](PC_PI_ARCHITECTURAL_REFACTOR.md) and [implementation phases](PC_PI_IMPLEMENTATION_PLAN.md). New-path commands will be documented when implemented and verified; Phase 5 consolidates rollout instructions.

[Home](../README.md) | [Architecture and controls](ARCHITECTURE.md) | [Sources](SOURCES.md)

## Resuming development on Windows

Use the Windows checkout for source review, editing and offline verification. Start with [Phase 0](PC_PI_IMPLEMENTATION_PLAN.md#phase-0--baseline-and-feasibility-gates-start-here), not a Pi-local feature implementation. The Windows venv below preserves the legacy baseline. Provision and verify a **separate WSL2 Linux solver environment** during Phase 0; neither Windows LeRobot 0.4.0 requirements nor the Pi's ARM64 CPU package pins establish a compatible WSL2 stack.

For the new path, connect the gamepad to Windows and leave the motor adapter on the Pi. WSL2 will compute motion and self-collision/path checks without owning USB serial. The future Windows supervisor will start a supervised Pi motor service over the LAN; that launcher/service does not exist yet. Keep the existing Pi environment/calibration as the hardware reference, and defer hardware operations until owner-approved tests.

## Windows setup

**Existing Windows baseline; this does not install the proposed distributed runtime.**

Use Python 3.11 and a normal project venv. From the repository root:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

If the Python launcher is unavailable, use the path to your installed Python 3.11 executable in place of `py -3.11`. Create the environment once; activate it when opening a new terminal. Dependencies are listed in `requirements.txt`. Optional test and development tools are in `requirements-dev.txt`. Use the [offline diagnostics](#motor-reading-and-offline-diagnostics) for the legacy baseline; a passing baseline does not verify the proposed distributed runtime.

## Raspberry Pi (Debian 13 ARM64)

**Legacy Pi environment and local controller instructions.**

Use the existing `~/lerobot-env` with Python 3.13.5, LeRobot 0.6.1,
Torch 2.11.0+cpu, and torchvision 0.26.0+cpu. Windows retains its separate
Python 3.11 / LeRobot 0.4.0 requirements. From the repository root:

```bash
~/lerobot-env/bin/python -m pip install -r requirements-pi.txt
~/lerobot-env/bin/python -m pip check
~/lerobot-env/bin/python -m robot_program.debug_env
```

The supplement adds Pygame 2.6.1 only. Do not install `requirements.txt` into
the Pi environment or replace its tested CPU PyTorch stack. Optional offline
test dependencies and checks are:

```bash
~/lerobot-env/bin/python -m pip install pytest==8.4.2
~/lerobot-env/bin/python validate_calibration.py
SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  ~/lerobot-env/bin/python -m pytest -q -o addopts= --tb=short
```

For the separate LeRobot IK mode, install `requirements-pi-ik.txt` instead.
See [LeRobot IK setup and controls](../README.md#lerobot-ik-raspberry-pi).

### Check ports and gamepad without connecting to the arm

Connect the gamepad directly to the Pi by USB or Bluetooth. Inspect device
paths and access as the normal user:

```bash
id
ls -l /dev/serial/by-id/ /dev/ttyACM* /dev/ttyUSB* /dev/input/js* /dev/input/event*
```

Missing globs may print an error. Prefer the verified adapter's
`/dev/serial/by-id/...` path over an index that can change after reboot. Confirm
read/write access to the resolved serial device (commonly group `dialout`).
Gamepad access depends on the session ACLs and the actual input-device group.
If access is denied, fix that specific user's group/ACL setup and start a new
login session; avoid running the robot controller as root or granting broad
device permissions. Keep only one robot process running.

This standalone monitor imports only Pygame and never opens a robot port.
Press each button, move each stick/trigger, and check the raw indices and D-pad:

```bash
SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  ~/lerobot-env/bin/python - <<'PY'
import time
import pygame

pygame.init()
try:
    count = pygame.joystick.get_count()
    print("Gamepads:", count)
    if not count:
        raise SystemExit("No accessible gamepad; check connection and permissions.")
    js = pygame.joystick.Joystick(0)  # Change to the intended --index.
    js.init()
    print(js.get_name(), "axes/buttons/hats:", js.get_numaxes(), js.get_numbuttons(), js.get_numhats())
    while True:
        pygame.event.pump()
        print("axes:", [round(js.get_axis(i), 2) for i in range(js.get_numaxes())],
              "buttons:", [js.get_button(i) for i in range(js.get_numbuttons())],
              "hats:", [js.get_hat(i) for i in range(js.get_numhats())], flush=True)
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
PY
```

Sony/DualShock controllers on Linux now use the device's SDL mapping in all
gamepad modes and the dashboard. The observed Pi controller has raw axes
0–5 = LX, LY, LT, RX, RY, RT, with neutral readings near
[0, 0, -1, 0, 0, -1]. Buttons are also mapped through SDL so digital trigger
buttons do not masquerade as Back/Start. An incomplete or unsupported Sony
mapping raises an error rather than guessing.

Windows and other devices retain the existing configured mapping:
axes 0–5 = LX, LY, RX, RY, LT, RT; buttons 0–9 = Cross, Circle, Square,
Triangle, LB, RB, Back, Start, L3, R3. Hat 0 remains the D-pad. Triggers are
converted from [-1, +1] to [0, 1]. Verify especially LB, Back, and idle trigger
values before any approved arm operation.

### Hardware calibration prerequisite

Hardware calibration now loads directly from the project, on Windows and Linux:

```text
calibration/lerobot_arm.json
```

The path is resolved from the Python source location, independent of the
working directory. No calibration cache copy, shell variable, or per-Pi
configuration is required. Explicit `SO101FollowerConfig.calibration_dir`
remains available for callers that deliberately select a different directory.
The repository follower does not use `HF_LEROBOT_CALIBRATION` by default.

Keep the existing raw hardware calibration and the separate controller-limit
export `calibration/lerobot_arm_with_degrees.json` together in the project.
Neither file is regenerated automatically. Preserve robot ID `lerobot_arm` and
motor IDs 1–6.

On 2026-11-02, with the owner's approval, the Pi's completed LeRobot
calibration was copied unchanged from its upstream `so_follower` cache into
`calibration/lerobot_arm.json`. The degree-limit export was regenerated from
that file. The controller now loads these project files directly; the upstream
cache is neither required nor modified by this project. No recalibration or
motor-register writes were performed. If calibration is repeated later,
explicitly synchronize the raw file and regenerate the derived export together.

Missing/incomplete calibration fails before opening the port. A mismatch found
after opening the bus closes it without changing torque or entering interactive
calibration. Recalibration or calibration-register changes require approval.

### Headless operating commands (require hardware approval)

After calibration and input checks, choose one mode and replace the example
adapter path. These commands connect to hardware and enable torque; they were
not executed during migration verification:

```bash
SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  ~/lerobot-env/bin/python -m scripts.teleop_axis_control \
  --port /dev/serial/by-id/REPLACE_WITH_VERIFIED_ADAPTER --index 0

SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  ~/lerobot-env/bin/python -m scripts.main_teleop_ik \
  --port /dev/serial/by-id/REPLACE_WITH_VERIFIED_ADAPTER --index 0 --no-dashboard

SDL_VIDEODRIVER=dummy SDL_AUDIODRIVER=dummy \
  ~/lerobot-env/bin/python -m scripts.main_teleop_plan \
  --port /dev/serial/by-id/REPLACE_WITH_VERIFIED_ADAPTER --index 0 --no-dashboard
```

`--no-dashboard` skips the UI, dashboard IPC, telemetry, and dashboard-liveness
dependency. Gamepad presets, recording, and playback retain their existing
behavior. Recorded plans remain in memory; dashboard editing and save/load are
unavailable. `--plan-file` does not automatically load or save a plan.
Ctrl+C and controller disconnect exit the loop with normal torque/disconnect
cleanup. Releasing LB does not cancel an existing preset or plan playback.

Omitting `--no-dashboard` retains the desktop dashboard and requires a working
graphical session; use that session's normal SDL settings instead of the dummy
drivers. The separate LeRobot IK mode starts a
[read-only LAN dashboard](../README.md#lerobot-ik-raspberry-pi) by default.
Use `--no-web-dashboard` to disable it. Live `keyboard` control is
optional and deferred on Linux; gamepad modes do not require that package.
The already tested USB camera needs no changes because these modes configure
no cameras.

## Run the arm

**Legacy instructions only. Physical operation requires owner approval.**

For hardware use, connect arm power, the Feetech/CH343 USB adapter, and the controller. Keep the existing adapter driver; install its vendor driver if Device Manager does not expose a COM port. Identify the adapter under Device Manager → Ports (COM & LPT). Choose exactly one mode and substitute the actual port and gamepad index:

```powershell
# Direct joint control, without the IK dashboard
python -m scripts.teleop_axis_control --port COM3 --index 0

# Custom Cartesian IK; launches its dashboard
python -m scripts.main_teleop_ik --port COM3 --index 0

# Custom IK manual driving plus recorded-plan playback/dashboard
python -m scripts.main_teleop_plan --port COM3 --index 0

# Optional plan save/load path; parent directory must already exist
python -m scripts.main_teleop_plan --port COM3 --index 0 --plan-file .runtime/my_plan.json
```

--port and --index are optional, but explicit ports are recommended: Windows auto_port currently returns COM3 rather than probing devices. Use module commands from the root so package imports resolve.

Hold LB for manual movement; R3 selects fine increments. See the [complete control mapping](ARCHITECTURE.md#controller-mappings) before using presets or playback. Releasing LB does not cancel an already-started preset or plan. Plan playback stops on Back, manual takeover, or loop exit; dashboard Act submits a direct joint target. Ctrl+C exits the loop and normal cleanup disables torque/disconnects. Closing the IK/plan dashboard also shuts down its teleop process. Support the arm when torque is disabled.

The IK/plan controller requires the repository's six-motor degree-limit export.
LeRobot separately loads hardware calibration for robot ID lerobot_arm. The
repository follower rejects missing or mismatched hardware calibration instead
of entering upstream's interactive workflow; restoring the exported
degree-limit file alone is insufficient. See [calibration details](JOINT_LIMITS_IMPLEMENTATION.md).

## Motor reading and offline diagnostics

With the venv active, these commands do not connect to the arm:

```powershell
python -m pip install pytest==8.4.2
python -m pip check
python -m robot_program.debug_env
python -X utf8 validate_calibration.py
python -m pytest -q -o addopts= --tb=short
```

The calibration validator prints Unicode symbols; `-X utf8` avoids encoding errors in Windows terminals using a legacy code page.

The offline suite covers the custom solver, joint limits, motor configuration, command-line entry points, mocked teleop integration, gamepad mapping and read-only web telemetry. LeRobot IK tests require LeRobot 0.6.1 and are skipped in the legacy Windows 0.4.0 environment; real solver cases additionally require PlaCo. Review skipped tests before comparing counts across environments. These checks do not validate the proposed distributed runtime; hardware behavior still needs physical validation.

The motor reader **does connect to hardware**. Default torque off disables torque so the arm can be moved by hand:

```powershell
python -m robot_program.read_motors --port COM3 --torque off
```

Its displayed Degrees column uses the exported normalized-to-degree conversion. The --deg flag changes bus output units, but the script still applies that conversion; do not treat that column with --deg as authoritative measured angles. This is a known diagnostic limitation, not corrected by this documentation change.

## WSL 2 alternate setup

**Legacy USB-forwarding option, not the new architecture.** The new WSL2 worker will not own the motor serial bus.

Windows and WSL cannot simultaneously own the USB serial adapter. Close other control sessions before handing it over.

In elevated PowerShell, inspect the current BUSID (the old 1-10 value was an example, not a permanent identifier):

```powershell
usbipd list
# If the device is not already shared, bind its actual BUSID first:
usbipd bind --busid <BUSID>
usbipd attach --busid <BUSID> --wsl
```

Replace <BUSID> with the value from usbipd list. The existing scripts/attach-robot.bat helper accepts a BUSID or tries to detect a common USB serial adapter; it requires elevation and does not replace initial binding.

In WSL:

```bash
ls /dev/ttyACM* /dev/ttyUSB*
cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project
python3 -m venv ~/.venvs/lerobot
source ~/.venvs/lerobot/bin/activate
python -m pip install -r requirements.txt
python -m robot_program.read_motors --port /dev/ttyACM0 --torque off
```

Use the actual device path and a separately provisioned WSL environment. Do not reuse the Windows `.venv` in WSL. The control launchers also accept Linux ports, but gamepad access, GUI support, permissions, runtime dependencies, and hardware behavior require separate WSL validation.

## LeRobot tuning

`robot_program.so101_follower.SO101Follower` preserves the arm's custom motor tuning. All hardware entry points use this subclass; installed LeRobot files are left unchanged.

| Motor | P | I | D |
| --- | ---: | ---: | ---: |
| shoulder_pan | 10 | 0 | 48 |
| shoulder_lift | 12 | 0 | 44 |
| elbow_flex | 14 | 0 | 40 |
| wrist_flex | 14 | 0 | 40 |
| wrist_roll | 16 | 0 | 36 |
| gripper | 18 | 0 | 32 |

Configuration runs with torque disabled, uses position mode, and retains acceleration values of 254. Gripper protection remains Max_Torque_Limit=500, Protection_Current=250, and Overload_Torque=25.

Hardware calibration defaults to the project's `calibration/lerobot_arm.json`; no user-cache copy is required. An explicit `SO101FollowerConfig.calibration_dir` still overrides the directory. The separate `calibration/lerobot_arm_with_degrees.json` supplies the custom controller's degree limits. The raw file and derived export use the owner-approved Pi calibration; control mappings are unchanged.

## Troubleshooting and runtime files

- Check the active python executable/version and environment report if imports fail. Activate `.venv` before running the program.
- Confirm --port, USB ownership, and that no other control or motor-reader process holds the connection.
- Check controller --index and assumed button mapping if input is wrong; the dashboard shows the configured mapping, not an automatic device profile.
- For dashboard crashes inspect scripts/dashboard_log.txt or scripts/dashboard_plan_log.txt.
- .runtime contains shared command/telemetry files and default saved plans. Only one IK/plan session should run, even with different serial ports. Do not delete a saved plan as routine IPC cleanup.
