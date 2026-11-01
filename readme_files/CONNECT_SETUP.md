# Connection and run guide

[Home](../README.md) | [Architecture and controls](ARCHITECTURE.md) | [Sources](SOURCES.md)

## Windows setup

Use Python 3.11 and a normal project venv. From the repository root:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

If the Python launcher is unavailable, use the path to your installed Python 3.11 executable in place of `py -3.11`. Create the environment once; activate it when opening a new terminal. Dependencies are listed in `requirements.txt`. Optional test and development tools are in `requirements-dev.txt`.

## Run the arm

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

The IK/plan controller requires the repository's six-motor degree-limit export. LeRobot separately loads hardware calibration for robot ID lerobot_arm. A missing or mismatched hardware calibration can trigger LeRobot's interactive calibration workflow on connect; restoring the exported degree-limit file alone is insufficient. See [calibration details](JOINT_LIMITS_IMPLEMENTATION.md).

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

The offline suite covers the custom solver, joint limits, motor configuration, and command-line entry points. Hardware behavior still needs physical validation.

The motor reader **does connect to hardware**. Default torque off disables torque so the arm can be moved by hand:

```powershell
python -m robot_program.read_motors --port COM3 --torque off
```

Its displayed Degrees column uses the exported normalized-to-degree conversion. The --deg flag changes bus output units, but the script still applies that conversion; do not treat that column with --deg as authoritative measured angles. This is a known diagnostic limitation, not corrected by this documentation change.

## WSL 2 alternate setup

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

Hardware calibration remains in the existing Hugging Face user cache (`HF_LEROBOT_CALIBRATION / robots / so101_follower / lerobot_arm.json`); switching Python environments on this computer does not move it. An explicit `SO101FollowerConfig.calibration_dir` still overrides the directory. The separate `calibration/lerobot_arm_with_degrees.json` supplies the custom controller's degree limits. These values and the existing controls are unchanged.

## Troubleshooting and runtime files

- Check the active python executable/version and environment report if imports fail. Activate `.venv` before running the program.
- Confirm --port, USB ownership, and that no other control or motor-reader process holds the connection.
- Check controller --index and assumed button mapping if input is wrong; the dashboard shows the configured mapping, not an automatic device profile.
- For dashboard crashes inspect scripts/dashboard_log.txt or scripts/dashboard_plan_log.txt.
- .runtime contains shared command/telemetry files and default saved plans. Only one IK/plan session should run, even with different serial ports. Do not delete a saved plan as routine IPC cleanup.
