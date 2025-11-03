## 🚀 Start the SO-101 Project in WSL 2 

<!-- Top navigation -->
<p align="center">
  <a href="../README.md">Home</a> |
  <b><u><a href="./CONNECT_SETUP.md">Start Manual</a></u></b> |
  <a href="./SOURCES.md">Sources</a>
</p>

---


### 1. Connect the Robot 
Plug in the Feetech / CH343 USB adapter. 
### 2. Share the USB Device with WSL 
Open **PowerShell (Run as Administrator)** and run: 
```powershell 
usbipd list 
usbipd attach --busid 1-10 --wsl 
``` 
### 3. Verify in WSL 
In your **WSL (Ubuntu)** terminal: 
```bash 
ls /dev/tty* 
``` 
You should see something like `/dev/ttyACM0`. (Possibly `/dev/ttyUSB0`). 
### 4. Activate Environment 
```bash 
conda activate lerobot 
``` 

<p>&nbsp;</p>

### The robot is now ready for use