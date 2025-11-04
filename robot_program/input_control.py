#!/usr/bin/env python3
import time
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
import argparse
from robot_program.utils.ports import normalize_port, auto_port

ap = argparse.ArgumentParser()
ap.add_argument("--port", default=auto_port(), help="Serial port (COM# or /dev/ttyACM#)")
args = ap.parse_args()
PORT = normalize_port(args.port)
ROBOT_ID = "lerobot_arm"

STEP = 10.0
FINE_STEP = 0.5

CONTROL_MAP = {
    "a": ("shoulder_pan", -1),
    "d": ("shoulder_pan", 1),
    "w": ("shoulder_lift", 1),
    "s": ("shoulder_lift", -1),
    "e": ("elbow_flex", 1),
    "q": ("elbow_flex", -1),
    "r": ("wrist_flex", 1),
    "f": ("wrist_flex", -1),
    "t": ("wrist_roll", 1),
    "g": ("wrist_roll", -1),
    "z": ("gripper", -1),
    "x": ("gripper", 1),
}

def print_help():
    print("\nControls:")
    print(" a/d: shoulder pan      w/s: shoulder lift")
    print(" e/q: elbow flex        r/f: wrist flex")
    print(" t/g: wrist roll        z/x: gripper")
    print(" .  : toggle fine/normal step")
    print(" p  : print joint positions")
    print(" Enter empty line or 'exit' to quit\n")

def main():
    cfg = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=True,
    )
    f = SO101Follower(cfg)
    f.connect()
    f.bus.enable_torque()

    # Example: reduce acceleration for all joints
    #for joint in f.bus.motors:
    #    f.bus.write("Acceleration", joint, 50)  # 0–254, lower = slower accel
    #    f.bus.write("Maximum_Acceleration", joint, 100)  # if supported (Protocol 0)


    pos = f.bus.sync_read("Present_Position")
    step = STEP
    print_help()

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "" or cmd == "exit":
                break
            if cmd == ".":
                step = FINE_STEP if step == STEP else STEP
                print(f"Step size: {step}°")
                continue
            if cmd == "p":
                for j, v in pos.items():
                    print(f"{j:<15}: {v:>7.1f}")
                print()
                continue
            if cmd not in CONTROL_MAP:
                print("Unknown key.")
                continue

            joint, direction = CONTROL_MAP[cmd]
            pos[joint] += direction * step

            action = {f"{j}.pos": v for j, v in pos.items()}
            f.send_action(action)
            print(f"{joint:<15}: {pos[joint]:>7.1f}°")

    except KeyboardInterrupt:
        pass
    finally:
        f.bus.disable_torque()
        f.disconnect()
        print("Disconnected and torque disabled.")

if __name__ == "__main__":
    main()
