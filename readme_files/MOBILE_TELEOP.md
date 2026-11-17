# Mobile teleop development

1. In PowerShell at the robot repository root, activate the project `.venv`. Install `websockets==16.1.1` and set up the [WSL2 IK environment](REMOTE_IK_SETUP.md#set-up-and-run-a-simulation). Put the PC and phone on the same trusted LAN.
2. Generate a shared token with `python -c "import secrets; print(secrets.token_urlsafe(32))"`. Run `notepad mobile-teleop.token` from the repository root and save the generated value as the file's only line. The launcher reuses this Git-ignored file on each start. Keep a copy of the same value for Android app setup.
3. Run `python -m scripts.main_mobile_teleop` and open the printed dashboard URL. In Android, connect to the printed `ws://…:18744/api/v1/input` URL and send `Authorization: Bearer <your token>` on the WebSocket upgrade. For LAN debugging, configure cleartext support only in the app's debug build. If Windows Firewall blocks access, scope inbound TCP port `18744` to the private LAN. [Android network security configuration](https://developer.android.com/privacy-and-security/security-config)
4. Send a 50 Hz stream with `X=0.25`, `Y=0`, `Z=0` and the deadman released. Expect **Receiving**, an increasing sequence, and a positive X bar. To move the simulation, stream neutral XYZ with both flags clear, send one neutral frame with the advance flag set, then continue neutral until **Control enabled**. Hold deadman and send `X=-0.25`, `Y=0`, `Z=0` for half a second. The requested and measured FK positions should change. Release deadman and zero XYZ to stop simulated motion. Close the socket to check for **Holding** and **Waiting for phone**. Exit with Esc or Ctrl+C. Report the last displayed XYZ, arm state, and any WebSocket or IK error if the behavior differs.

The mobile launcher runs only PC-side WSL2 IK and the local simulated motor bus. Use `--input-only` to check the phone connection before setting up WSL2. The input server selects the private IPv4 address of the PC's default network route. If it prints `127.0.0.1`, input is available only on the PC. You can override the address with `--listen <PC-LAN-IP>` if needed. The read-only dashboard is always local on `127.0.0.1:18745`.

The dashboard follows the phone app's selected mode. **ArUco control** opens the **ArUco pose** panel and **Touchscreen control** opens **Phone XYZ input**. Returning to the app menu shows **Waiting for phone**. In simulation, **Simulated arm** stays visible across mode changes. Open its **Diagnostics** to see the IK candidate, last sent motor target, and feedback timings.

## Phone input v1

Send one **17-byte binary WebSocket message**, big-endian, on each update. The version is part of the endpoint path.

| Offset | Field | Encoding |
| --- | --- | --- |
| 0 | Sequence | Unsigned 32-bit integer. Start at zero on each new socket and increment per update. |
| 4 | Flags | Bit 0 holds the deadman. Bit 1 advances simulated control on a neutral rising edge. Clear the other bits. |
| 5 | X, Y, Z | Three IEEE 754 float32 values in the IK model's base frame. Each is a signed normalized velocity intent in `[-1, 1]`. Zero means no requested movement. |

Kotlin can encode a test sample with `ByteBuffer.allocate(17).order(ByteOrder.BIG_ENDIAN).putInt(seq).put(0.toByte()).putFloat(0.25f).putFloat(0f).putFloat(0f).array()`. Send it as a binary WebSocket message. Touch axes can map directly to velocity intent. For ArUco, establish a neutral pose, align the tracking axes to the robot base frame, and map relative displacement to bounded velocity. Release the deadman and send zeros when tracking is lost.

Keep only the newest unsent sample and avoid filling the WebSocket send queue. If writes back up, close and reconnect with a fresh neutral stream. The receiver keeps one latest sample, drops duplicate or decreasing sequences, and closes a stream after 75 ms without fresh input once streaming begins. Reconnection requires a new advance for simulated motion. The TCP connection uses one upgrade and carries input without compression or application acknowledgements per update. Measure Wi-Fi jitter on the actual phone before treating the 50 Hz target as verified. [OkHttp WebSocket support](https://okhttp.org/okhttp-websocket-support/)
