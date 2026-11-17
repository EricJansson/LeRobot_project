# Mobile teleop development

1. In PowerShell at the robot repository root, activate the project `.venv`. Install `websockets==16.1.1` and set up the [WSL2 IK environment](REMOTE_IK_SETUP.md#set-up-and-run-a-simulation). Put the PC and phone on the same trusted LAN.
2. Generate a shared token with `python -c "import secrets; print(secrets.token_urlsafe(32))"`. Run `notepad mobile-teleop.token` from the repository root and save the generated value as the file's only line. The launcher reuses this Git-ignored file on each start. Keep a copy of the same value for Android app setup.
3. Run `python -m scripts.main_mobile_teleop --dry-run` and open the printed dashboard URL. In the updated debug Android app, save the printed **Touchscreen monitoring** WebSocket URL and bearer token under **Connection settings**. Existing URLs for the same launcher also work. Open **Touchscreen control** and expect **Receiving** under **Phone XYZ input**. If Windows Firewall blocks access, scope inbound TCP port `18744` to the private LAN.
4. Select **Arm controls** on the phone. With the sticks centered and all movement buttons released, tap **Start control**. Wait for **Control enabled** on the phone and dashboard, then hold **Hold to move**. Check X/Y with the left stick, Z and roll with the right stick, and both wrist tilt and gripper directions with their buttons. Release **Hold to move** to pause movement. Tap **Stop control** and expect **Holding** on both screens. Background the app to verify that control stops and the screen returns to **Monitor** when resumed. Reopen **Arm controls** and verify that a fresh Start and Hold to move are required. Esc or Ctrl+C exits. Report the last displayed input, arm state, and any WebSocket or IK error if behavior differs.

The mobile launcher uses PC-side WSL2 IK. Running it without flags selects `--real-arm --speed 1`. The Pi must have `--allow-motion` configured before the arm can move. Use `--dry-run` for local simulation, `--pi-sim` for Pi simulation, or `--input-only` to check the phone connection without WSL2. The input server selects the private IPv4 address of the PC's default network route. If it prints `127.0.0.1`, input is available only on the PC. You can override the address with `--listen <PC-LAN-IP>` if needed. The read-only dashboard is always local on `127.0.0.1:18745`.

The dashboard follows the phone app's selected mode. **ArUco control** opens **ArUco pose**. Within **Touchscreen control**, **Monitor** opens **Phone XYZ input** and **Arm controls** opens **Phone arm input** with XYZ, roll, pitch and gripper intent. Returning to the app menu shows **Waiting for phone**. In simulation, **Simulated arm** stays visible across mode changes. Open its **Diagnostics** to see the IK candidate, last sent motor target, and feedback timings.

## Pi simulation and supervised joystick motion

Both Pi modes reuse the WSL2 worker, Pi motor service and SSH startup from the Windows gamepad workflow. The phone's left stick controls X/Y. The right stick controls Z vertically and wrist roll horizontally. Use **Hold to move** with a separate finger while moving either stick. Pitch and gripper commands are disabled in both Pi modes. ArUco and Monitor report telemetry only.

1. Follow the [remote IK setup](REMOTE_IK_SETUP.md#set-up-and-run-a-simulation). Close other teleop launchers. On Windows, activate the project's virtual environment and run `python -m scripts.main_mobile_teleop --pi-sim --speed 0.1`. Add `--pi user@host` if your SSH destination differs from `eric@rpi`. Use the debug phone app on the same trusted LAN and the URL and token from the steps above. The launcher checks the Pi service release before opening the motor connection.
2. Confirm **Pi simulation** on the PC. Open **Arm controls** on the phone, center both sticks, release **Hold to move**, and tap **Start control**. If the arm starts **Released**, one neutral press captures **Holding** and a separate press enables control. Try left stick up for positive X, left for positive Y, right stick up for positive Z, and right stick right for positive roll in the gripper frame. The measured wrist-joint angle decreases for that positive roll. Try each with **Hold to move** released, then held. Compare **Phone joystick input**, issued targets, and measured positions. Center the stick while still holding **Hold to move**, then try again and lift only the **Hold to move** finger. Each release should stop advancement.
3. In simulation, verify **Stop control**, app backgrounding, and a brief Wi-Fi interruption. The motor session should return to **Holding** and require a fresh neutral Start. Press **Esc** or **Ctrl+C** on Windows to exit. If the dashboard says **Use the gamepad to recover**, exit the mobile launcher and use the existing gamepad workflow to recover from the joint limit before reopening mobile control.
4. Follow the [real-arm observation and commissioning procedure](REMOTE_IK_SETUP.md#first-supervised-motion) with the owner. With the arm supported and the Pi configured for observation, run `python -m scripts.main_mobile_teleop --real-arm --speed 0.1` from Windows. Confirm **Observe only**, plausible measured positions, and no motion in response to phone input. After owner approval and a successful simulation and observation test, enable the existing `--allow-motion` setting and the approved Pi rates and watchdog. Restart the same command and confirm **Real arm enabled**. Begin from a supported bent-arm pose with clearance and a physical power disconnect available. Hold **Hold to move**, nudge the left stick left briefly, then release. Verify a small sideways movement and that advancement stops. Try the other directions and wrist roll individually after that check passes. The existing IK path checks joint limits and rates. Geometric collision validation remains separate.
5. Stop at unexpected motion, an incorrect direction, or a failed stop. Use **Stop control** to disarm and **Esc** to exit. Support the arm before shutting down the service or disconnecting power. Report the launcher mode, stick direction, visible movement, measured position change, stop behavior, and any fault or longest Pi feedback gap shown in **Diagnostics**.

## Touchscreen monitoring

Connect to `/api/v1/touchscreen` with `Authorization: Bearer <token>` in either launcher mode. Send the 17-byte frame described under Phone input v1 with a zero flags byte. Begin each connection at sequence zero and send monotonically increasing sequences. The receiver rejects control flags, and duplicate and decreasing sequences leave the last accepted sample's age unchanged.

After 500 ms without a fresh sample, the dashboard shows **Input stale** and clears the axes. New input before the two-second idle deadline refreshes the readings on the same socket. At the idle deadline the receiver closes the connection. Monitoring input feeds the dashboard only.

## Phone input v2

Connect to `/api/v2/input` with `Authorization: Bearer <token>`. Each input is a 26-byte binary WebSocket frame in big-endian order.

| Offset | Field | Encoding |
| --- | --- | --- |
| 0 | Sequence | Unsigned 32-bit counter starting at zero for each socket |
| 4 | Flags | Bit 0 Hold to move, bit 1 Start, bit 2 Stop |
| 5 | X, Y, Z, roll, pitch | Five finite float32 values in `[-1, 1]` |
| 25 | Gripper | Signed byte, `+1` open, `-1` close, `0` neutral |

Start with sequence zero and all other fields clear. Send Start and Stop with just their own flag and neutral axes. Stop maps to Holding with motor torque applied, including when a client holds the Stop flag. Motion requires a fresh Start and Hold to move after any Stop or reconnection. Positive roll and pitch use the existing right-stick-right and RT directions. Positive gripper input opens the jaws.

The Android app streams arm input at about 100 Hz. The gateway expires motion input after 75 ms without a fresh frame. Every 100 ms it sends a text message `arm-status:<state>` with **Monitoring only**, **Holding**, **Control enabled**, **Released**, **Fault**, or **Unavailable**. The phone clears input and reconnects if status goes missing for 500 ms. **Monitoring only** indicates input-only mode. Pi modes advertise `X-LeRobot-Arm-Controls: joysticks` during the v2 WebSocket upgrade. The gateway ignores pitch and gripper values and rejects legacy v1 motion connections in those modes. It expires the phone motion session if the Pi motor link drops.

## Phone input v1

Connect to `/api/v1/input` for legacy velocity control. Send one **17-byte binary WebSocket message**, big-endian, on each update.

| Offset | Field | Encoding |
| --- | --- | --- |
| 0 | Sequence | Unsigned 32-bit integer. Start at zero on each new socket and increment per update. |
| 4 | Flags | Bit 0 holds the deadman. Bit 1 advances simulated control on a neutral rising edge. Clear the other bits. |
| 5 | X, Y, Z | Three IEEE 754 float32 values in the IK model's base frame. Each is a signed normalized velocity intent in `[-1, 1]`. Zero means no requested movement. |

Kotlin can encode a test sample with `ByteBuffer.allocate(17).order(ByteOrder.BIG_ENDIAN).putInt(seq).put(0.toByte()).putFloat(0.25f).putFloat(0f).putFloat(0f).array()`. Send it as a binary WebSocket message. Touch axes can map directly to velocity intent. For ArUco, establish a neutral pose, align the tracking axes to the robot base frame, and map relative displacement to bounded velocity. Release the deadman and send zeros when tracking is lost.

Keep only the newest unsent sample and avoid filling the WebSocket send queue. If writes back up, close and reconnect with a fresh neutral stream. The receiver keeps one latest sample, drops duplicate or decreasing sequences, and closes a stream after 75 ms without fresh input once streaming begins. Reconnection requires a new advance for simulated motion. The TCP connection uses one upgrade and carries input without compression or application acknowledgements per update. Measure Wi-Fi jitter on the actual phone before treating the 50 Hz target as verified. [OkHttp WebSocket support](https://okhttp.org/okhttp-websocket-support/)
