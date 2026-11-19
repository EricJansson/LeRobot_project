# Mobile teleop development

1. In PowerShell at the robot repository root, activate the project `.venv`. Install `websockets==16.1.1` and set up the [WSL2 IK environment](REMOTE_IK_SETUP.md#set-up-and-run-a-simulation). Put the PC and phone on the same trusted LAN.
2. Generate a shared token with `python -c "import secrets; print(secrets.token_urlsafe(32))"`. Run `notepad mobile-teleop.token` from the repository root and save the generated value as the file's only line. The launcher reuses this Git-ignored file on each start. Keep a copy of the same value for Android app setup.
3. Run `python -m scripts.main_mobile_teleop --dry-run` and open the printed dashboard URL. In the updated debug Android app, save the printed **Touchscreen arm control** WebSocket URL and bearer token under **Connection settings**. Existing URLs for the same launcher also work. Open **Touchscreen control** and expect **Receiving** under **Phone arm input**. If Windows Firewall blocks access, scope inbound TCP port `18744` to the private LAN.
4. From **Released**, tap **Holding** and wait for confirmation. Try the sticks and movement buttons. The phone should show their movement and presses while the PC shows neutral input and the arm stays still. Center the sticks and release the buttons, then tap **Control enabled**. Check X/Y with the left stick, Z and roll with the right stick, and wrist tilt and gripper with their buttons. Lift your finger to stop motion or tap **Holding** to disable control. From **Holding**, hold **Released** for two seconds to test simulated torque release. Background the app during **Control enabled** and verify it reconnects in **Holding** on resume. Tap **Control enabled** again to drive. Esc or Ctrl+C exits. Report the last displayed input, arm state, and any WebSocket or IK error if behavior differs.

The mobile launcher uses PC-side WSL2 IK. Running it without flags selects `--real-arm --speed 1`. The Pi must have `--allow-motion` configured before the arm can move. Use `--dry-run` for local simulation, `--pi-sim` for Pi simulation, or `--input-only` for ArUco telemetry without WSL2. The input server selects the private IPv4 address of the PC's default network route. If it prints `127.0.0.1`, input is available only on the PC. You can override the address with `--listen <PC-LAN-IP>` if needed. The dashboard is always local on `127.0.0.1:18745`; its robot-workspace editor saves configuration, while arm enable/hold actions stay on the phone.

The dashboard follows the phone app's current screen. **ArUco control** opens **ArUco pose**. **Touchscreen control** opens **Phone arm input** with XYZ, roll, pitch and gripper intent. Returning to the app menu shows **Waiting for phone**. In simulation, **Simulated arm** stays visible across screen changes. Open its **Diagnostics** to see the IK candidate, last sent motor target, and feedback timings.

Real-arm startup automatically clears a leftover idle Pi motor service before deployment: the service must be at least 10 seconds old, have no controller connection on either of two checks one second apart, and retain the same invocation/process. Active controllers, fresh/changing services, inspection failures and failed shutdown block startup before camera checks. Support the arm before launching because recovery can stop the previous session and its existing shutdown/torque policy still applies. This is not run on ordinary reconnects, input-only launches or simulation. The new session still requires explicit control enable. For ArUco without Pi video, use `python -m scripts.main_mobile_teleop --real-arm --speed 0.1 --no-camera`; the phone camera remains active. See the [startup recovery checks](REMOTE_IK_SETUP.md#real-arm-setup-and-testing) for offline verification and supervised acceptance steps.

## ArUco XYZ position control

Use updated **Phone-vision** and **LeRobot_project** checkouts and install the debug APK. ArUco XYZ is an absolute position target, separate from touchscreen velocity intent. Ignore marker rotation; keep the gripper target unchanged. Existing IK may adapt tool orientation during translation.

### Configure and verify in local simulation

1. Prerequisites: Windows `.venv`, the WSL2 IK environment, a fixed upright phone, a rigid marker, shared connection credentials, and a trusted LAN. In PowerShell at this repository root, run `python -m scripts.main_mobile_teleop --dry-run --speed 0.1 --no-camera`. Open the printed dashboard and save the printed ArUco URL/token on the phone.
2. Expand **Robot workspace** on the PC. Enter all seven values in centimeters: X min/max, Y min/max, min/max height above the table, and table Z in the IK model's base frame. +X is forward, +Y left, +Z up. Use table Z = 0 only when the table actually lies at robot Z = 0. Review the suggested minimum X = 10 cm and minimum height = 5 cm, complete the remaining bounds, check the review box and save. The Git-ignored `mobile-teleop.workspace.json` is loaded on startup; missing/corrupt settings disable motion. Saving changes cancels following.
3. Open **ArUco control**, calibrate the phone workspace along all three axes, and enable **Settings → LeRobot pose stream → Normalize XYZ**. Each axis maps `−1 → robot minimum`, `0 → midpoint`, `+1 → robot maximum`. Robot Z is table Z plus configured height, so Z = −1 maps to the minimum table clearance. These bounds apply to the model's tool reference point.
4. Tap **Holding** first if Released. Follow the phone's X/Y/Z corrections or use the dashboard's **Arm**, **Target**, **Allowed**, and **Marker adjustment** columns. Each correction is measured tool minus mapped marker target, in robot centimeters. Positive X means move the marker forward, positive Y left, positive Z up; negative means backward, right, down. Marker travel depends on the calibrated scaling. Both positions must lie inside the robot box and the total Euclidean error must be ≤ 2 cm before **Control enabled** becomes available (not 2 cm independently on each axis). If the starting tool is outside the box, a warning shows the offending axis, measured coordinate and bound; moving the marker cannot align there. Reposition the arm using touchscreen control or review the bounds before returning to ArUco. ArUco does not automatically return an out-of-bounds tool to the box. The editor shows calculated allowed robot Z = Table Z + height, to make the effective limits explicit.
5. Move the marker slowly in +X, +Y and +Z, then hold it still. Expect corresponding bounded motion toward the mapped target. The resultant speed is capped at 10 cm/s times `--speed`, with a startup ramp and no target overshoot. Rotate the marker around a fixed center and verify it produces no rotation/gripper input.
6. Cover the marker or leave its calibrated phone workspace. Expect holding within the pose deadline; restore tracking on the same connection and expect gradual automatic resumption. Tap **Holding** and confirm it cancels following. Background the phone, disconnect/reconnect the socket, change calibration, or save new robot bounds and verify that alignment plus a fresh enable action is required. Stop with **Esc** or **Ctrl+C**. Report mapped/measured XYZ, alignment, arm mode, pause/rejection reason, and any unexpected motion, omitting the token.

The loop rejects commanded tool FK and sampled modeled joint transitions outside the box (samples at most 1° apart, including adapted paths and measured/sent references). IK/joint/rate checks and Pi watchdogs remain in place. These restrictions do **not** prove whole-arm self-collision or table clearance; the elbow and other links can extend outside the tool box. Physical tracking, Wi-Fi behavior and loaded-arm clearance require HARDWARE verification.

### Protocol and compatibility

Negotiate `X-LeRobot-ArUco-Control: xyz-v1` on the existing `/api/v1/marker-pose` WebSocket. The server echoes the capability; old clients and servers retain monitoring. Motion requires normalized units and robot axes. The 29-byte binary pose format is unchanged. A single negotiated socket also accepts `{"type":"aruco-control","action":"heartbeat"}` every 20 ms and one-shot `enable`, `hold`, or `invalidate` actions. Heartbeats do not refresh pose age or enable control. The 256-byte message limit accommodates these commands; binary length checks still apply.

The PC cancels control after a 100 ms heartbeat gap. Loss or a 100 ms accepted-pose gap pauses and retains the enable request. Duplicate/decreasing pose sequences do not refresh age. The phone discards observations more than 100 ms old from analysis start; these local checks are not synchronized capture-to-motion latency measurements. Every 100 ms the server sends `arm-status:<state>` and `aruco-status:<JSON>` carrying enable/readiness, alignment, mapped target and reason. Missing ArUco status for 500 ms makes the phone reset and reconnect without replaying enable.

Status also carries optional `marker_adjustment_cm` (tool minus mapped marker target, XYZ in robot cm), `measured_xyz_m`, `workspace_m` (`min`/`max` robot XYZ in meters), `tool_in_workspace` (boolean or null), and `workspace_warning`. The current target is used consistently for both axis differences and total distance. Stale/lost marker input clears directional guidance; measured workspace violations stay visible. Existing enable, hold, watchdog and motion limits are unchanged. Restart the launcher and install the updated phone APK for the new guidance.

The mobile-only `GET/POST /api/robot-workspace` endpoint reads/saves a version-1 JSON object containing the seven form field names (`x_min_cm`, `x_max_cm`, `y_min_cm`, `y_max_cm`, `height_min_cm`, `height_max_cm`, `table_z_cm`). POST requires the exact loopback Host/Origin, current `X-Dashboard-Session`, JSON content, and at most 4096 bytes. Other dashboards keep rejecting POST. Workspace restrictions apply to ArUco only.

Offline regression commands from Windows PowerShell at the repository root:

```powershell
.\.venv\Scripts\python.exe -m pytest tests/test_aruco_control.py tests/test_mobile_teleop.py tests/test_remote_ik_path.py tests/test_remote_ik_adaptive.py tests/test_lerobot_web_dashboard.py
$env:LEROBOT_WSL_TESTS = '1'
.\.venv\Scripts\python.exe -m pytest tests/test_aruco_control.py -m integration
Remove-Item Env:LEROBOT_WSL_TESTS
```

The opt-in integration uses WSL IK, temporary workspace settings, loopback WebSockets and a local simulated motor bus; it never SSHs to the Pi or opens serial. Before physical trials, obtain owner approval, complete the existing [supervised commissioning procedure](REMOTE_IK_SETUP.md#first-supervised-motion), review the saved box against the actual setup, and repeat the checks at `python -m scripts.main_mobile_teleop --real-arm --speed 0.1 --no-camera`. Support a released arm before Holding; keep the physical power disconnect accessible. Label results OFFLINE versus HARDWARE.

## Innomaker arm camera

With `ustreamer` installed on the Pi and the phone on the same trusted LAN, the launcher automatically starts or reuses the camera in every mode. Windows checks that the camera is producing frames and prints the direct Pi URL for **Connection settings → Arm camera URL**. Camera startup failures are reported in the terminal and teleop continues. The launcher uses the Pi's SSH-side private IPv4 address and port `8080`. It looks for Innomaker's stable USB capture path, with `/dev/video0` as a fallback. Use `--camera-device <path>` to override detection on a fresh camera service start. Use `--no-camera` to skip startup and verification for both camera feeds. A camera service already running on the Pi continues to serve video until it is stopped.

The default `control` camera profile requests 640×480 at 15 fps per camera to reduce traffic while steering. `--camera-profile balanced` requests 1280×720 at 30 fps for Innomaker and 1296×972 at 45 fps for the camera module. The launcher replaces its own running camera services when their profile differs. The phone uses the same URLs and caps preview decoding at about 15 fps in either profile.

1. Stop any earlier FFmpeg capture of the same USB camera. From Windows PowerShell in this checkout with `.venv` active, test video without starting the IK worker or motor service. The command uses the default SSH destination `eric@rpi`. Pass `--pi user@host` if you normally connect using a different trusted SSH destination.

   ```powershell
   python -m scripts.main_mobile_teleop --input-only
   ```

2. Expect **Arm camera is live** and a URL ending in `/stream?extra_headers=1`. Save the printed URL on the debug phone app and open **Touchscreen control**. Exit and rerun the launcher to verify it reuses the live feed.
3. Press Esc or Ctrl+C to exit the launcher. The camera remains reachable on the Pi while its user service is running. To stop the service the launcher started, run `systemctl --user stop lerobot-arm-camera.service` in the Pi shell. If startup fails, run `journalctl --user -u lerobot-arm-camera.service -n 20` there and report the launcher warning and relevant log lines.

## Raspberry Pi camera module

The same launcher starts the CSI camera as a separate Picamera2 service on TCP port `8081` and serves MJPEG directly to the phone on the trusted LAN. Devices that can reach the port can view the feed.

The launcher reuses a managed camera only when fresh frames are available. A running service with no live frames is restarted, even if its profile matches. CSI services also carry a source revision so a capture-code update is installed on the next check. If CSI capture produces no fresh JPEG for 10 seconds, the capture process exits and the camera-only systemd service restarts it after 2 seconds. A healthy manual stream is still reused; the launcher does not terminate unrelated capture processes.

If `journalctl --user` has no entries on this Pi, use `journalctl _SYSTEMD_USER_UNIT=lerobot-rpi-camera.service -n 40 --no-pager`, or `journalctl -u user@$(id -u).service -n 40 --no-pager` in the Pi shell. The camera warning is nonfatal for arm control. These recovery checks affect video services, not the motor service.

**Camera verified:** the stalled CSI service was restored and three fresh 640×480 JPEGs were decoded from Windows. Arm and Innomaker service PIDs were unchanged. Offline tests cover stale-service replacement, source revision checks and capture watchdog expiry; no motor tests were performed.

1. In the Pi shell, install Picamera2 if needed with `sudo apt install --no-install-recommends python3-picamera2`. Stop any manual `rpicam-vid` capture so the launcher can open the camera module. [Raspberry Pi setup guide](https://www.raspberrypi.com/documentation/accessories/camera-software.html#use-libcamera-from-python-with-picamera2)
2. In Windows PowerShell with `.venv` active, run `python -m scripts.main_mobile_teleop --input-only`. Expect **Raspberry Pi camera is live** and a URL ending in `:8081/stream`. In the debug phone app, save it under **Connection settings → Raspberry Pi camera URL**. With both URLs saved, **Touchscreen control** shows Innomaker above the camera module.
3. In the Pi shell, run `systemctl --user stop lerobot-rpi-camera.service` to stop the module feed. The lower preview should clear while Innomaker continues. Exit the Windows launcher with Esc or Ctrl+C and rerun it to restore the feed. For a startup failure, report the launcher warning and `journalctl --user -u lerobot-rpi-camera.service -n 20` output from the Pi.

If touchscreen control drops out, compare **Pause video** and **Resume video** in a `--dry-run` session. When paused, the phone closes both video connections and continues sending control input. Report the phone's reconnect message and the PC dashboard's arm and phone input statuses if the input link drops again.

## Pi simulation and supervised touchscreen motion

Both Pi modes reuse the WSL2 worker, Pi motor service and SSH startup from the Windows gamepad workflow. With **Control enabled**, deflect a stick or press and hold a movement button. The left stick controls X/Y. The right stick controls Z vertically and wrist roll horizontally. The gripper buttons open or close the jaws incrementally, and the tilt buttons use the gamepad's wrist pitch axis. Lift your finger to end the corresponding input. ArUco position control uses the separately configured robot workspace described above.

1. Follow the [remote IK setup](REMOTE_IK_SETUP.md#set-up-and-run-a-simulation). Close other teleop launchers. On Windows, activate the project's virtual environment and run `python -m scripts.main_mobile_teleop --pi-sim --speed 0.1`. Add `--pi user@host` if your SSH destination differs from `eric@rpi`. Use the debug phone app on the same trusted LAN and the URL and token from the steps above. The launcher checks the Pi service release before opening the motor connection.
2. Confirm **Pi simulation** on the PC. Open **Touchscreen control** on the phone. If the arm is **Released**, tap **Holding** first, then tap **Control enabled**. Try left stick up for positive X, left for positive Y, right stick up for positive Z, and right stick right for positive roll in the gripper frame. The measured wrist-joint angle decreases for that positive roll. Compare **Phone arm input**, issued targets, and measured positions. Center the stick and then try lifting your finger while deflected. Both should stop advancement. Hold each button briefly and lift your finger. Check that **Open gripper** increases the measured gripper value and **Close gripper** decreases it. **Tilt wrist up** and **Tilt wrist down** should show opposite pitch inputs and move the wrist in opposite directions.
3. In simulation, tap **Holding** and confirm that the sticks and movement buttons no longer move the arm. Hold **Released** for less than two seconds and lift your finger. The countdown should cancel. Hold it again for two full seconds and confirm **Released** on both screens. Tap **Holding**, then **Control enabled**, and try backgrounding the app or interrupting Wi-Fi during movement. The arm should return to **Holding** and require another tap of **Control enabled**. Press **Esc** or **Ctrl+C** on Windows to exit. If the dashboard says **Use the gamepad to recover**, exit the mobile launcher and use the gamepad workflow before reopening mobile control.
4. Follow the [real-arm observation and commissioning procedure](REMOTE_IK_SETUP.md#first-supervised-motion) with the owner. With the arm supported and the Pi configured for observation, run `python -m scripts.main_mobile_teleop --real-arm --speed 0.1` from Windows. Confirm **Observe only**, plausible measured positions, and no motion in response to phone input. After owner approval and a successful simulation and observation test, enable the existing `--allow-motion` setting and the approved Pi rates and watchdog. Restart the same command and confirm **Real arm enabled**. Begin from a supported bent-arm pose with clearance and a physical power disconnect available. Tap **Control enabled**, nudge the left stick left briefly, then lift your finger. Verify a small sideways movement and that advancement stops. Check the remaining stick directions, then press each tilt and gripper button individually and verify that motion stops on lift. Support the arm before testing release. **Released** turns off torque on all six motors, including the gripper. The existing IK path checks joint limits and rates. Geometric collision validation remains separate.
5. Stop at unexpected motion, an incorrect direction, or a failed stop. Tap **Holding** to disarm and press **Esc** to exit. Support the arm before shutting down the service or disconnecting power. Report the launcher mode, input used, visible movement, measured position change, stop behavior, and any fault or longest Pi feedback gap shown in **Diagnostics**.

## Legacy touchscreen monitoring

Older clients can connect to `/api/v1/touchscreen` with `Authorization: Bearer <token>`. Send the 17-byte frame described under Phone input v1 with a zero flags byte. Begin each connection at sequence zero and send monotonically increasing sequences. The receiver rejects control flags, and duplicate and decreasing sequences leave the last accepted sample's age unchanged.

After 500 ms without a fresh sample, the dashboard shows **Input stale** and clears the axes. New input before the two-second idle deadline refreshes the readings on the same socket. At the idle deadline the receiver closes the connection. Monitoring input feeds the dashboard only.

## Phone input v2

Connect to `/api/v2/input` with `Authorization: Bearer <token>`. Each input is a 26-byte binary WebSocket frame in big-endian order.

| Offset | Field | Encoding |
| --- | --- | --- |
| 0 | Sequence | Unsigned 32-bit counter starting at zero for each socket |
| 4 | Flags | Bit 0 Motion requested, bit 1 Advance, bit 2 Hold, bit 3 Release held |
| 5 | X, Y, Z, roll, pitch | Five finite float32 values in `[-1, 1]` |
| 25 | Gripper | Signed byte, `+1` open, `-1` close, `0` neutral |

Start with sequence zero and all other fields clear. Send Advance, Hold, or Release held alone with neutral axes. Advance captures Holding from Released or enables control from Holding. Hold disarms and keeps torque applied. Release held requires its own fresh press from Holding and two continuous seconds of fresh frames. The PC advertises `X-LeRobot-Arm-Release: hold` and reports the countdown as `arm-release-progress:<fraction>`. The motor service confirms Released. A stick or movement button sets Motion requested only after control is enabled. Clear it when input ends. After Hold or reconnect, enable control again before sending motion. Positive roll and pitch use the existing right-stick-right and RT directions. Positive gripper input opens the jaws.

The Android app streams arm input at about 100 Hz. The gateway expires motion input after 75 ms without a fresh frame. Every 100 ms it sends a text message `arm-status:<state>` with **Monitoring only**, **Holding**, **Control enabled**, **Released**, **Fault**, or **Unavailable**. The phone clears input and reconnects if status goes missing for 500 ms. **Monitoring only** indicates input-only mode. Pi modes require v2 for motion and expire the phone motion session if the Pi motor link drops. Older PC launchers that only support sticks in Pi modes advertise `X-LeRobot-Arm-Controls: joysticks`.

## Phone input v1

Connect to `/api/v1/input` for legacy velocity control. Send one **17-byte binary WebSocket message**, big-endian, on each update.

| Offset | Field | Encoding |
| --- | --- | --- |
| 0 | Sequence | Unsigned 32-bit integer. Start at zero on each new socket and increment per update. |
| 4 | Flags | Bit 0 holds the deadman. Bit 1 advances simulated control on a neutral rising edge. Clear the other bits. |
| 5 | X, Y, Z | Three IEEE 754 float32 values in the IK model's base frame. Each is a signed normalized velocity intent in `[-1, 1]`. Zero means no requested movement. |

Kotlin can encode a test sample with `ByteBuffer.allocate(17).order(ByteOrder.BIG_ENDIAN).putInt(seq).put(0.toByte()).putFloat(0.25f).putFloat(0f).putFloat(0f).array()`. Send it as a binary WebSocket message. Touch axes can map directly to velocity intent. ArUco uses the absolute normalized-position mapping on its marker-pose endpoint; do not route marker positions through this legacy velocity protocol.

Keep only the newest unsent sample and avoid filling the WebSocket send queue. If writes back up, close and reconnect with a fresh neutral stream. The receiver keeps one latest sample, drops duplicate or decreasing sequences, and closes a stream after 75 ms without fresh input once streaming begins. Reconnection requires a new advance for simulated motion. The TCP connection uses one upgrade and carries input without compression or application acknowledgements per update. Measure Wi-Fi jitter on the actual phone before treating the 50 Hz target as verified. [OkHttp WebSocket support](https://okhttp.org/okhttp-websocket-support/)
