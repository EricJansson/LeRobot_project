# SO-101 robot arm workspace

This personal project controls a physical **SO-101 follower arm** using Python, a PlayStation-style gamepad, and LeRobot's Feetech hardware interface. It includes direct joint control, custom Cartesian inverse kinematics (IK), recorded motion playback, a desktop dashboard, calibration data, and offline tests.

A separate Pi mode uses **LeRobot's URDF-based FK/IK with fine movement** and starts a LAN dashboard by default. Existing IK and plan modes retain their custom solver. See [LeRobot IK setup and controls](#lerobot-ik-raspberry-pi). Collision checking and phone control remain planned.

## Windows gamepad through the Raspberry Pi

The Windows launcher reads an XInput gamepad, opens a local dashboard, and uses official LeRobot IK in WSL2 with a mesh-free model. The Pi service owns the serial connection. Self-collision checking is outside this implementation’s scope.

```powershell
python.exe -m scripts.main_teleop_remote_ik --dry-run
python.exe -m scripts.install_remote_ik_pi
python.exe -m scripts.main_teleop_remote_ik --pi-sim
```

The PC and Pi simulations are **OFFLINE VERIFIED**. Live control requires commissioning and owner-approved hardware tests. See the [remote IK setup and verification guide](readme_files/REMOTE_IK_SETUP.md).

## Start here

- [PC–Pi architectural refactor](readme_files/PC_PI_ARCHITECTURAL_REFACTOR.md) and [implementation plan](readme_files/PC_PI_IMPLEMENTATION_PLAN.md) describe the broader November 5 design and phase gates.
- [Connection and run guide](readme_files/CONNECT_SETUP.md): **existing/legacy** Raspberry Pi headless setup, Windows venv setup, alternate WSL instructions, and commands.
- [Architecture and control reference](readme_files/ARCHITECTURE.md): code ownership, controls, units, state, dashboard communication, and extension boundaries.
- [Joint limits and calibration](readme_files/JOINT_LIMITS_IMPLEMENTATION.md): current limits and enforcement.
- [Project status](readme_files/PROJECT_PROGRESS.md), [TODO](TODO.md), and [sources](readme_files/SOURCES.md).
- [Phi adaptation tests](readme_files/PHI_ADAPTATION_TESTS.md) and [historical physical-pose observations](readme_files/helper_files/robot_orientations.md).

## Implementation status

| Capability | Current status |
| --- | --- |
| Direct gamepad joint control | Used successfully on hardware; talks directly to SO101Follower |
| Cartesian gamepad IK | Used successfully on hardware; custom base-yaw + three-joint planar solver |
| Plan recording/playback | Used successfully on hardware; interpolated joint MOVE actions and timed PAUSE actions |
| Desktop dashboard | Gamepad display, manual joint targets, telemetry; plan editing in plan mode |
| Offline tests | 159-test historical baseline; venv validation below |
| web_sim | Outdated standalone browser experiment; retained for historical reference |
| LeRobot URDF FK/IK integration | Separate headless Pi mode; owner confirms working fine movement |
| PC-led distributed control | Simulation verified offline; initial Windows gamepad motion and LB release verified on hardware |
| Collision avoidance / phone control | Outside the remote IK implementation scope |

Hardware use of the prior setup is confirmed by the owner. The venv migration did not connect to or move the arm; physical validation of the new environment remains pending. Passing offline tests does not validate physical accuracy or collision safety.

## Run an existing mode

**Legacy commands; not the new PC–Pi refactor.**

From the repository root in the existing environment, choose **one**:

```powershell
.\.venv\Scripts\python.exe -m scripts.teleop_axis_control --port COM3 --index 0
```

```powershell
.\.venv\Scripts\python.exe -m scripts.main_teleop_ik --port COM3 --index 0
```

```powershell
.\.venv\Scripts\python.exe -m scripts.main_teleop_plan --port COM3 --index 0
```

Replace COM3 with the actual adapter port. Windows automatic port selection currently returns COM3; it does not discover the arm. IK and plan launch their dashboard automatically. Only one process should own the robot connection, and IK/plan sessions also share runtime files. Read the [run guide](readme_files/CONNECT_SETUP.md) before operating the arm.

## Architecture

**This section describes the currently implemented legacy paths.**

```text
Gamepad -> gamepad_mapper -> teleop loop
  axis mode --------------------------------> SO101Follower -> Feetech motors
  IK mode -> custom RobotArm -> RobotArmController -> SO101Follower
  LeRobot IK mode -> RobotKinematics (URDF/Placo) -> SO101Follower
  plan mode -> recorded joints / interpolation -> RobotArmController

Dashboard <-> .runtime JSON files <-> IK or plan teleop process
```

RobotArm is a hardware-independent model: solving IK changes internal state. RobotArmController connects it to LeRobot, reads hardware positions, and sends joint commands. The dashboard never opens the serial port. Plan mode uses custom IK for manual driving but joint interpolation for playback.

| Location | Responsibility |
| --- | --- |
| [scripts/](scripts/) | Mode entry points, reusable teleop loops, dashboard/panels, WSL attachment helper |
| [robot_program/](robot_program/) | Custom model/controller; older keyboard/input experiments; motor and environment diagnostics |
| [robot_program/utils/](robot_program/utils/) | Custom FK/IK, port helpers, standalone guardrail/workspace helpers |
| [controller/input/](controller/input/) | Pygame polling, bindings, input-state profile, example |
| [controller/scripts/run_teleop.py](controller/scripts/run_teleop.py) | Input-profile example; does not send commands to the arm |
| [calibration/](calibration/) | Project-local hardware calibration, degree-limit export, and conversion utility |
| [so101_new_calib.urdf](so101_new_calib.urdf) | Model for the LeRobot IK mode; all 13 referenced meshes are in [assets/](assets/) |
| [tests/](tests/) | Offline model/limits, hardware-adapter, gamepad, dashboard and conditional LeRobot IK regressions |
| [web_sim/](web_sim/) | Early JavaScript FK/IK visualization, disconnected from Python/hardware |
| [readme_files/](readme_files/) | Setup, implementation references, status, sources, historical design material |
| .runtime/ | Ignored dashboard IPC and default saved-plan location |

## Setup

The steps below set up the legacy modes. For distributed teleop, see the [remote IK setup guide](readme_files/REMOTE_IK_SETUP.md).

On the Raspberry Pi (Debian 13 ARM64), reuse `~/lerobot-env`. Install only the
supplement with `~/lerobot-env/bin/python -m pip install -r requirements-pi.txt`.
The Pi uses Python 3.13 and LeRobot 0.6.1 with its existing CPU PyTorch stack;
do not install the Windows requirements into that environment. See the
[Pi guide](readme_files/CONNECT_SETUP.md#raspberry-pi-debian-13-arm64) for
headless gamepad checks and the `--no-dashboard` option for IK/plan modes.

On Windows, use Python 3.11 and a normal `.venv` for dependencies:

```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

After activation, run any mode above using `python`. See the [setup guide](readme_files/CONNECT_SETUP.md) for connection details.

The [project follower](robot_program/so101_follower.py) preserves custom PID tuning
and motor protection settings. Hardware calibration loads directly from
`calibration/lerobot_arm.json` on both LeRobot versions; no Pi cache copy is
needed. The raw calibration and derived degree-limit export were synchronized
from the owner-approved Pi calibration; control mappings are unchanged. Missing or
mismatched hardware calibration stops connection instead of starting interactive recalibration.

## LeRobot IK (Raspberry Pi)

**Existing standalone Pi IK path (legacy); not the new WSL2/PC-led path.**

In the existing Python 3.13 / LeRobot 0.6.1 Pi environment:

```bash
python -m pip install -r requirements-pi-ik.txt
python -m scripts.main_teleop_lerobot_ik
```

Use `--port /dev/serial/by-id/YOUR_ADAPTER` and `--index 0` when needed.
The optional supplement adds Placo 0.9.15 and pins the existing CPU Torch/NumPy
stack. This mode retains the tuned follower and project-local calibration.

Hold the **left upper shoulder button** to move. Left stick controls
forward/backward and right/left; right stick controls height and wrist roll.
Triggers control pitch, and D-pad up/down increases/decreases the gripper.
Translation stays fixed relative to the base/table: model +X is forward, -Y
is right, and +Z is up. Pitch/roll use tool-local axes with soft orientation
tracking. Face-button presets and recording are not implemented in this mode;
migration of these features to the PC-led path is [deferred](TODO.md#deferred--not-in-initial-implementation).

The LeRobot IK command starts the read-only browser dashboard by default and
prints its URLs using the Pi's IPv4 addresses. Open a printed LAN URL on a phone
or PC connected to the same trusted network. If address detection fails, the
launcher prints `http://127.0.0.1:8000/` for use on the Pi with the default port.
The page updates at up to 10 Hz and shows measured joint angles, normalized
gripper position, LB status, and end-effector XYZ in millimeters relative to the
robot base (+X forward, +Y left, +Z up). Use `--web-host` to bind to a specific IPv4 address or `--web-port`
to select another port. Use `--no-web-dashboard` to run without the server.
The server stops when teleop exits.

At 50 Hz, full input requests 2.0 mm of translation, 0.4 degrees of pitch,
or 0.75 degrees of wrist roll per tick. The gripper command adds 1.0 normalized
unit per tick to its measured position in the requested direction.
Ctrl+C or controller loss disables torque and disconnects.
The existing control modes and calibration/URDF files are unchanged.

The owner confirms working movement. The IK solution must be finite, within the
URDF and calibration joint limits, and within 0.5 mm of the requested XYZ. This
limits reach according to the arm model. Teleop sends the validated IK solution
directly to the follower. Measured model alignment and collision checking remain
pending.

## Tests and diagnostics

The current suite includes custom-model tests, mocked hardware/teleop integration, gamepad mapping, read-only dashboard HTTP checks and [LeRobot IK regressions](tests/test_lerobot_ik.py). The LeRobot IK module is skipped unless LeRobot 0.6.1 is installed; real solver cases also require PlaCo. The legacy Windows LeRobot 0.4.0 baseline therefore does not run those IK cases. Check skips as well as passes; do not replace the Windows environment just to match historical Pi results. None of these tests verifies the future distributed runtime or physical collision safety.

Run offline tests from the root:

```powershell
.\.venv\Scripts\python.exe -m pytest -q -o addopts= --tb=short
```

Baseline on 2026-10-30: **159 passed** under default Python 3.12.9 and the robot environment's Python 3.10.19. These tests cover custom mathematics/model behavior, not hardware, full dashboard/plan integration, upstream IK, or collision checking.

Venv verification on 2026-10-31: **168 passed** with Python 3.11.14; `pip check` passed. The nine additional checks cover preserved motor configuration and CLI help without connecting to hardware. Physical validation remains pending.

Pi verification on 2026-11-02: **204 passed** with Python 3.13.5 and LeRobot
0.6.1; `pip check` and calibration arithmetic passed. Tests blocked serial
connections and covered both import layouts, mocked Windows dashboard launch,
headless gamepad loops, recording/playback, and calibration rejection. Headless
SDL initialized, but no physical gamepad was exposed to the session.
The owner-approved Pi calibration and derived degree export are synchronized
in the project. Tests verify project-local loading and consistent calibration
data; approved physical validation remains pending.

The observed Linux Sony controller uses axes LX, LY, LT, RX, RY, RT, rather
than the Windows fallback order. Linux Sony controllers now use SDL's device
mapping for axes/buttons in teleoperation and the dashboard. Regression tests
replay the reported neutral readings with LB held and verify zero logical input.

Using the venv interpreter, `python -m robot_program.debug_env` reports environment information. `python -X utf8 validate_calibration.py` checks exported raw-to-degree arithmetic, not physical calibration. The motor reader is a hardware operation described in the run guide.

## Boundaries for future work

The approved target and priority order are in [PC–Pi architectural refactor](readme_files/PC_PI_ARCHITECTURAL_REFACTOR.md) and [implementation plan](readme_files/PC_PI_IMPLEMENTATION_PLAN.md). In particular, the old proposal of phone input through the Pi dashboard is **superseded**; a **future native Android app** must connect to the PC gateway, with one controller active at a time. A PC-hosted monitoring dashboard replaces the Pi dashboard **only in the new path**.

- Keep existing mode entry points, custom kinematics, calibration values, and dashboard behavior intact while adding the new path.
- Unused guardrail helpers and the old web simulator provide no runtime protection. Current controls have joint/input checks but no self-, table-, or obstacle-collision checking.
- Manual LB gating is not a universal motion stop: presets and plan playback can continue after LB release. Dashboard **Act** submits a direct joint target. See [motion protection boundaries](readme_files/ARCHITECTURE.md#motion-protection-boundaries).
- Verify conversions: custom XYZ uses centimeters, URDF geometry uses meters, joint conventions need alignment, and LeRobot's gripper channel is normalized rather than degrees.
- Follow [TODO](TODO.md). The previously reported second-pitch-joint/height discrepancy needs revalidation; it is not a confirmed current defect.

Documentation reconciled with local source and the proposed architecture: 2026-11-05. Dated test and hardware reports above remain historical evidence; no tests or hardware operations were performed in this documentation update.
