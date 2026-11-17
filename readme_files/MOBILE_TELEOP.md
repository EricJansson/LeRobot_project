# Mobile teleop development

1. In PowerShell at the robot repository root, activate the project `.venv`. Install `websockets==16.1.1` and set up the [WSL2 IK environment](REMOTE_IK_SETUP.md#set-up-and-run-a-simulation). Put the PC and phone on the same trusted LAN.
2. Generate a shared token with `python -c "import secrets; print(secrets.token_urlsafe(32))"`. Run `notepad mobile-teleop.token` from the repository root and save the generated value as the file's only line. The launcher reuses this Git-ignored file on each start. Keep a copy of the same value for Android app setup.
3. Run `python -m scripts.main_mobile_teleop` and open the printed dashboard URL. In the updated debug Android app, save the printed **Touchscreen monitoring** WebSocket URL and bearer token under **Connection settings**. Existing URLs for the same launcher also work. Open **Touchscreen control** and expect **Receiving** under **Phone XYZ input**. If Windows Firewall blocks access, scope inbound TCP port `18744` to the private LAN.
4. Select **Arm controls** on the phone. With the sticks centered and all movement buttons released, tap **Start control**. Wait for **Control enabled** on the phone and dashboard, then hold **Hold to move**. Check X/Y with the left stick, Z and roll with the right stick, and both wrist tilt and gripper directions with their buttons. Release **Hold to move** to pause movement. Tap **Stop control** and expect **Holding** on both screens. Background the app to verify that control stops and the screen returns to **Monitor** when resumed. Reopen **Arm controls** and verify that a fresh Start and Hold to move are required. Esc or Ctrl+C exits. Report the last displayed input, arm state, and any WebSocket or IK error if behavior differs.

The mobile launcher runs only PC-side WSL2 IK and the local simulated motor bus. Use `--input-only` to check the phone connection before setting up WSL2. The input server selects the private IPv4 address of the PC's default network route. If it prints `127.0.0.1`, input is available only on the PC. You can override the address with `--listen <PC-LAN-IP>` if needed. The read-only dashboard is always local on `127.0.0.1:18745`.

The dashboard follows the phone app's selected mode. **ArUco control** opens **ArUco pose**. Within **Touchscreen control**, **Monitor** opens **Phone XYZ input** and **Arm controls** opens **Phone arm input** with XYZ, roll, pitch and gripper intent. Returning to the app menu shows **Waiting for phone**. In simulation, **Simulated arm** stays visible across mode changes. Open its **Diagnostics** to see the IK candidate, last sent motor target, and feedback timings.

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

Stream current input at about 50 Hz. The gateway applies the v1 freshness rules. Every 100 ms it sends a text message `arm-status:<state>` with **Monitoring only**, **Holding**, **Control enabled**, **Released**, **Fault**, or **Unavailable**. The phone clears input and reconnects if status goes missing for 500 ms. **Monitoring only** indicates input-only mode.

## Phone input v1

Connect to `/api/v1/input` for legacy velocity control. Send one **17-byte binary WebSocket message**, big-endian, on each update.

| Offset | Field | Encoding |
| --- | --- | --- |
| 0 | Sequence | Unsigned 32-bit integer. Start at zero on each new socket and increment per update. |
| 4 | Flags | Bit 0 holds the deadman. Bit 1 advances simulated control on a neutral rising edge. Clear the other bits. |
| 5 | X, Y, Z | Three IEEE 754 float32 values in the IK model's base frame. Each is a signed normalized velocity intent in `[-1, 1]`. Zero means no requested movement. |

Kotlin can encode a test sample with `ByteBuffer.allocate(17).order(ByteOrder.BIG_ENDIAN).putInt(seq).put(0.toByte()).putFloat(0.25f).putFloat(0f).putFloat(0f).array()`. Send it as a binary WebSocket message. Touch axes can map directly to velocity intent. For ArUco, establish a neutral pose, align the tracking axes to the robot base frame, and map relative displacement to bounded velocity. Release the deadman and send zeros when tracking is lost.

Keep only the newest unsent sample and avoid filling the WebSocket send queue. If writes back up, close and reconnect with a fresh neutral stream. The receiver keeps one latest sample, drops duplicate or decreasing sequences, and closes a stream after 75 ms without fresh input once streaming begins. Reconnection requires a new advance for simulated motion. The TCP connection uses one upgrade and carries input without compression or application acknowledgements per update. Measure Wi-Fi jitter on the actual phone before treating the 50 Hz target as verified. [OkHttp WebSocket support](https://okhttp.org/okhttp-websocket-support/)
