## Start the SO-101 Project

<!-- Top navigation -->
<p align="center">
  <a href="../README.md">Home</a> |
  <b><u><a href="./CONNECT_SETUP.md">Start Manual</a></u></b> |
  <a href="./SOURCES.md">Sources</a>
</p>

---

## Windows

### 1. Connect the Robot
Plug in the Feetech / CH343 USB adapter and note the assigned COM port 
(e.g., `COM3`, `COM4`, etc.). 
You can find it in **Device Manager → Ports (COM & LPT)**.

### 2. Activate the Environment
```powershell
conda activate lerobot
```

### 3. Navigate to the Project Folder
```powershell
cd C:\Users\rapid\Documents\GitHub\LeRobot_project
```

### 4. Run the Robot arm script with controller 
Here you can choose between two control modes:

1. With axis control (teleop):
```powershell
python -m scripts.teleop_axis_control
```

2. With inverse kinematics control:
```powershell
python -m scripts.main_teleop_ik
```

#### The robot is now ready for use on Windows.

---

## Linux (WSL 2)

### 1. Connect the Robot
Plug in the Feetech / CH343 USB adapter.

### 2. Share the USB Device with WSL
Open **PowerShell (Run as Administrator)** and run:
```powershell
usbipd list
usbipd attach --busid 1-10 --wsl
```

### 3. Verify the Device in WSL
In your **WSL (Ubuntu)** terminal:
```bash
ls /dev/tty*
```
You should see a device such as `/dev/ttyACM0` (or possibly `/dev/ttyUSB0`).

### 4. Activate the Environment
```bash
conda activate lerobot
```

### 5. Navigate to the Project Folder
```bash
cd /mnt/c/Users/rapid/Documents/GitHub/LeRobot_project
```

### 6. Run the Control Script
```bash
python robot_program/read_motors.py --port /dev/ttyACM0
```

#### The robot is now ready for use under WSL 2.

---
