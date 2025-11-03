@echo off
setlocal

REM ---- Require Administrator ----
net session >nul 2>&1
if %errorlevel% neq 0 (
  echo [!] Please run this script as Administrator.
  pause
  exit /b 1
)

REM ---- If BUSID provided as an argument, use it directly ----
if not "%~1"=="" (
  set "BUSID=%~1"
  goto :ATTACH
)

REM ---- Auto-detect BUSID via usbipd list (match common USB-serial chips) ----
for /f "usebackq tokens=*" %%L in (`usbipd list`) do (
  echo %%L | findstr /i /r "1a86:55d3 10C4:EA60 0403:6001 CH34 CP210 FTDI" >nul
  if not errorlevel 1 (
    for /f "tokens=1" %%B in ("%%L") do (
      set "BUSID=%%B"
      goto :ATTACH
    )
  )
)

echo [!] Could not auto-detect BUSID. Make sure the adapter is plugged in.
echo     Run:  usbipd list
echo     Then run:  attach-robot.bat ^<BUSID^>   (example: attach-robot.bat 1-10)
pause
exit /b 2

:ATTACH
echo [+] Attaching BUSID %BUSID% to WSL...
usbipd attach --busid %BUSID% --wsl
if %errorlevel% neq 0 (
  echo [!] usbipd failed to attach device. Is it already in use by Windows or another WSL?
  pause
  exit /b 3
)

echo [+] Attached. Open WSL and check for /dev/ttyUSB* or /dev/ttyACM*.
echo     Example:  ls /dev/tty*    or   dmesg ^| tail
pause
endlocal
