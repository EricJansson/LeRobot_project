# Connection and run guide

[Home](../README.md) | [Architecture and controls](ARCHITECTURE.md) | [Sources](SOURCES.md)

Last reviewed: 2026-10-30. Native Windows with the existing Conda environment is the current hardware-used setup. WSL instructions remain an alternate path, without a recent hardware-verification claim.

## Current environment

Observed in C:\Users\rapid\miniconda3\envs\lerobot:

| Component | Observed version / role |
| --- | --- |
| Python | 3.10.19 |
| LeRobot | 0.4.0; locally modified installed package |
| pygame | 2.6.1; controller polling and dashboard |
| Feetech SDK / pyserial | Present for motor/serial communication |
| keyboard | Present for older keyboard experiment |
| pytest | Present for offline tests |
| placo | Absent; required for upstream RobotKinematics, not current custom IK |

This is an observed working environment, not a complete dependency lock. requirements-dev.txt supplies testing/quality tools, not the runtime. A fresh Windows venv is planned, not implemented or validated. Preserve tuning and hardware calibration before migration; installing the same LeRobot version alone will not reproduce this setup.

## Native Windows

1. Connect the arm's power and Feetech/CH343 USB adapter, and connect the controller.
2. Identify the adapter COM port in Device Manager → Ports (COM & LPT).
3. Open a terminal, activate the existing environment, and change to the repository root:

```powershell
conda activate lerobot
cd C:\Users\rapid\Documents\GitHub\LeRobot_project
```

4. Choose exactly one control mode. Substitute the actual COM port and gamepad index:

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

These offline commands do not open the robot connection:

```powershell
python -m robot_program.debug_env
python -X utf8 validate_calibration.py
python -m pytest -q -o addopts= --tb=short
```

The calibration validator prints Unicode symbols; -X utf8 avoids encoding errors in Windows terminals using a legacy code page. Its check passed for all six exported motors during this review.

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
conda activate lerobot
cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project
python -m robot_program.read_motors --port /dev/ttyACM0 --torque off
```

Use the actual device path. A working Windows Conda environment does not establish an equivalent WSL environment. The control launchers also accept Linux ports, but gamepad access, GUI support, permissions, runtime dependencies, and hardware behavior require separate WSL validation.

## External LeRobot customization

The inspected installed file is:

`C:\Users\rapid\miniconda3\envs\lerobot\Lib\site-packages\lerobot\robots\so101_follower\so101_follower.py`

It contains two configure definitions; Python uses the later one, marked CUSTOM MADE. That active implementation applies:

| Motor | P | I | D |
| --- | ---: | ---: | ---: |
| shoulder_pan | 10 | 0 | 48 |
| shoulder_lift | 12 | 0 | 44 |
| elbow_flex | 14 | 0 | 40 |
| wrist_flex | 14 | 0 | 40 |
| wrist_roll | 16 | 0 | 36 |
| gripper | 18 | 0 | 32 |

It sets position mode, invokes bus.configure_motors(), and sets gripper Max_Torque_Limit=500, Protection_Current=250, Overload_Torque=25. These are observed settings, not newly recommended tuning. The installed bus helper defaults maximum_acceleration and acceleration to 254; a complete package-diff audit remains a migration task.

This customization is outside version control here. Preserve/diff the installed package and calibration before replacing the environment, and move required behavior into an explicit maintained integration during the future migration. This documentation does not change the installed package.

## Troubleshooting and runtime files

- Check the active python executable/version and environment report if imports fail. The default Python environment has pytest but no LeRobot.
- Confirm --port, USB ownership, and that no other control or motor-reader process holds the connection.
- Check controller --index and assumed button mapping if input is wrong; the dashboard shows the configured mapping, not an automatic device profile.
- For dashboard crashes inspect scripts/dashboard_log.txt or scripts/dashboard_plan_log.txt.
- .runtime contains shared command/telemetry files and default saved plans. Only one IK/plan session should run, even with different serial ports. Do not delete a saved plan as routine IPC cleanup.
