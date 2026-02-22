#!/usr/bin/env python3
import time
import argparse
import keyboard  # pip install keyboard
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from .utils.ports import normalize_port, auto_port

# ---- Speed/feel knobs ----
BASE_STEP_DEG     = 0.6   # main speed per tick (↑ to move faster)
FINE_STEP_DEG     = 0.2   # while holding Shift
RATE_HZ           = 25     # control loop rate
ALPHA             = 0.35   # low-pass blend: 0..1 (higher = snappier, lower = smoother)
CHANGE_THRESHOLD  = 0.03   # send only if any joint changed by >= this many degrees
MIN_SEND_INTERVAL = 0.02   # min seconds between sends (rate limit)
# --------------------------

KEYS = {
    "shoulder_pan":  ("a", "d"),
    "shoulder_lift": ("w", "s"),
    "elbow_flex":    ("e", "q"),
    "wrist_flex":    ("r", "f"),
    "wrist_roll":    ("t", "g"),
    "gripper":       ("z", "x"),  # 0..100 scale
}

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=auto_port(), help="Serial port (COM# or /dev/ttyACM#)")
    ap.add_argument("--deadman", default="space", help="Hold this key to allow motion (default: space)")
    args = ap.parse_args()

    port = normalize_port(args.port)
    cfg = SO101FollowerConfig(port=port, id="lerobot_arm", use_degrees=True)
    f = SO101Follower(cfg); f.connect(); f.bus.enable_torque()

    # Start from current pose
    pos = f.bus.sync_read("Present_Position")  # filtered command we send
    target = dict(pos)                         # instantaneous target from keys
    last_sent = dict(pos)
    last_send_t = 0.0

    print("\nLive keyboard control (hold SPACE; ESC to exit)\n")
    for joint, (neg, posk) in KEYS.items():
        print(f"{joint:<15}: {neg}/{posk}")
    print(f"\nStep={BASE_STEP_DEG}°, fine={FINE_STEP_DEG}°, rate={RATE_HZ} Hz, alpha={ALPHA}\n")

    dt = 1.0 / RATE_HZ
    try:
        while True:
            if keyboard.is_pressed("esc"):
                break

            allowed = keyboard.is_pressed(args.deadman)
            step = FINE_STEP_DEG if keyboard.is_pressed("shift") else BASE_STEP_DEG

            if allowed:
                # --- build instantaneous target from keys ---
                changed_keys = False
                for joint, (neg, posk) in KEYS.items():
                    before = target[joint]
                    if keyboard.is_pressed(neg):   target[joint] -= step
                    if keyboard.is_pressed(posk):  target[joint] += step
                    if target[joint] != before:    changed_keys = True

                # clamp gripper 0..100
                target["gripper"] = clamp(target["gripper"], 0.0, 100.0)

                # --- low-pass filter toward target (smooths jitter) ---
                for j in pos:
                    pos[j] = pos[j] + ALPHA * (target[j] - pos[j])

                # --- micro-smoothing: only send if meaningful change & not too fast ---
                need_send = any(abs(pos[j] - last_sent[j]) >= CHANGE_THRESHOLD for j in pos)
                now = time.time()
                if need_send and (now - last_send_t) >= MIN_SEND_INTERVAL:
                    f.send_action({f"{j}.pos": v for j, v in pos.items()})
                    last_sent = dict(pos)
                    last_send_t = now

                    # print when we actually sent
                    print("=" * 40)
                    for j, v in pos.items():
                        whole, frac = int(v), abs(v - int(v))
                        print(f"{j:<15}: {whole:>6d}{' ' * 10}{frac:.3f}")
                    print()

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        f.bus.disable_torque(); f.disconnect()
        print("Disconnected and torque disabled.")

if __name__ == "__main__":
    main()
