# Project Sources

## Implementation references (reviewed 2026-10-30)

Repository behavior is described in the [architecture reference](ARCHITECTURE.md). Upstream links establish library behavior, not integration status in this workspace.

- [LeRobot 0.4.0 URDF kinematics source](https://raw.githubusercontent.com/huggingface/lerobot/v0.4.0/src/lerobot/model/kinematics.py): RobotKinematics, its transform-based FK/IK interface, and optional placo dependency. This wrapper does not configure collision avoidance.
- [LeRobot 0.4.0 SO101Follower source](https://github.com/huggingface/lerobot/blob/v0.4.0/src/lerobot/robots/so101_follower/so101_follower.py): hardware interface and normalized gripper configuration. The repository-owned follower preserves the former installed-package PID tuning; see the [setup guide](CONNECT_SETUP.md#lerobot-tuning).
- [Microsoft WSL USB connection guide](https://learn.microsoft.com/en-us/windows/wsl/connect-usb): usbipd installation, device sharing/binding, and attachment. WSL remains an alternate setup requiring local validation.
- The pinned TheRobotStudio model supplies the URDF and original meshes. The installed STLs are the reduced copies from so101-sim2real. Physical model validation remains pending. [Original SO-101 model](https://github.com/TheRobotStudio/SO-ARM100/tree/5f6d2b876a53a4872e405b991dd925556c9e38a4/Simulation/SO101) · [Asset provenance and checksums](../assets/README.md)

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

