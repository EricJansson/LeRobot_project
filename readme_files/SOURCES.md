# Project Sources

> **Implementation sources/history, not proof of the planned refactor.** Approved design decisions and phase gates: [PC–Pi architectural refactor](PC_PI_ARCHITECTURAL_REFACTOR.md) and [implementation plan](PC_PI_IMPLEMENTATION_PLAN.md). Existing references below do not establish self-collision, Windows gamepad→WSL2 or the distributed PC gateway/Pi motor service as implemented.

## Architecture documentation reconciliation (2026-11-05)

The two PC–Pi planning documents supplied in the temporary proposal bundle are the primary references for the target architecture and future implementation order. Duplicate planning copies were identical at review; their content was promoted and reconciled into the canonical [target architecture](PC_PI_ARCHITECTURAL_REFACTOR.md) and [phased implementation plan](PC_PI_IMPLEMENTATION_PLAN.md) in `readme_files/`. The temporary bundle was removed after the documentation review at the owner's request. Use the canonical documents for future updates.

The owner's instruction sets **Windows PC development**, with a provisional WSL2 solver runtime and Pi motor execution. Conflicting older phone-through-Pi-dashboard and Pi-only-compute plans are superseded. Current commands, dated test results, calibration synchronization and owner-reported hardware lessons remain legacy evidence.

Local source cross-checks included [standalone Pi IK](../scripts/main_teleop_lerobot_ik.py), [LeRobot IK controller](../robot_program/lerobot_ik_controller.py), [read-only web server](../scripts/lerobot_web_dashboard.py), [gamepad device mapping](../controller/input/device_mapping.py), [custom controller](../robot_program/robot_arm_controller.py), [tuned follower](../robot_program/so101_follower.py), teleop loops and dependency files. This was a documentation review, not a new environment, performance or hardware validation. The full Phase 0 audit remains future work.

## Implementation references (reviewed 2026-10-30)

Repository behavior is described in the [architecture reference](ARCHITECTURE.md). Upstream links establish library behavior, not integration status in this workspace.

- [LeRobot 0.4.0 URDF kinematics source](https://raw.githubusercontent.com/huggingface/lerobot/v0.4.0/src/lerobot/model/kinematics.py): RobotKinematics, its transform-based FK/IK interface, and optional placo dependency. This wrapper does not configure collision avoidance.
- [LeRobot 0.4.0 SO101Follower source](https://github.com/huggingface/lerobot/blob/v0.4.0/src/lerobot/robots/so101_follower/so101_follower.py): hardware interface and normalized gripper configuration. The repository-owned follower preserves the former installed-package PID tuning; see the [setup guide](CONNECT_SETUP.md#lerobot-tuning).
- [Microsoft WSL USB connection guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb): historical USB-forwarding reference for the legacy alternate setup, requiring local validation. The planned WSL2 motion worker does not own the motor USB adapter; the Pi owns serial in the PC-led target.
- [SO-ARM100/SO101 simulation models](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101): original source of the URDF and 13 mesh geometries. The original meshes were acquired on 2026-10-30. On 2026-11-05 they were replaced with the reduced [so101-sim2real meshes](https://github.com/isaka1022/so101-sim2real/tree/addc4e13f22f6f8ac1e29d15b78380b02387a1e5/web/assets/meshes) and manifest from `IK-collision` commit `e836a3ede4df3d8e7d4c022fc47cb25ac46e9032`. The local URDF now matches the pinned upstream bytes after line-ending normalization; its parsed geometry is unchanged. See [asset provenance and checksums](../assets/README.md). Physical model alignment and reduced-mesh clearance validation remain pending.

The resources below are historical research/setup references. They are not evidence that this repository implements learning, phone control, or collision avoidance.

<!-- Top navigation -->
<p align="center">
  <a href="../README.md">Home</a> |
  <a href="./CONNECT_SETUP.md">Start Manual</a> |
  <b><u><a href="./SOURCES.md">Sources</a></u></b>
</p>

---

## Youtube video (LeRobot & SO-ARM100)
This video shows off the robot in action and how to use it. 
It contains some interesting segments:
* How it works 
* Leader / Follower teleoperation and interaction with them
* PS4 Controller teleoperation
* Custom robot configuration with camera and alternative gripper
* Machine learning
  * Dataset collection
  * Dataset visualization
  * Training policies 
  * DOT-Policy
  * Training
  * Running

Link: https://www.youtube.com/watch?v=DeBLc2D6bvg  
His custom Github repo: https://github.com/IliaLarchenko/lerobot
<p>&nbsp;</p>



## Main GitHub Repository (SO-ARM100)
Contains the bill of materials, 3D print files, and mechanical documentation for the robot 

Link: https://github.com/TheRobotStudio/SO-ARM100
<p>&nbsp;</p>


## Installation Guide
Detailed instructions for installing LeRobot and setting up the working environment.

Link: https://huggingface.co/docs/lerobot/installation
<p>&nbsp;</p>


## Assembly, Initial Setup, and Calibration (SO-101)
Step-by-step guide for assembling, wiring, and calibrating the SO-101 robotic arm.

Link: https://huggingface.co/docs/lerobot/so101?example=Linux&assembly=Follower
<p>&nbsp;</p>


## Imitation Learning on Real-World Robots
Overview of how LeRobot performs imitation learning on physical robot platforms.

Link: https://huggingface.co/docs/lerobot/il_robots
<p>&nbsp;</p>

