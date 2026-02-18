#!/usr/bin/env python3
import json
import time
import argparse
from pathlib import Path
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from utils.ports import normalize_port, auto_port

CALIB_PATH = Path(__file__).parent.parent / "calibration" / "lerobot_arm_with_degrees.json"

def load_joint_ranges():
    with open(CALIB_PATH) as f:
        data = json.load(f)
    ranges = {}
    for m in data["motors"]:
        # Joints whose physical minimum is near zero are normalized 0..100
        norm_min = 0 if m["degree_min"] >= -10 else -100
        ranges[m["name"]] = (m["degree_min"], m["degree_max"], norm_min)
    return ranges

def norm_to_degrees(value, degree_min, degree_max, norm_min=-100):
    """Map normalized value to physical degrees."""
    norm_range = 100 - norm_min
    t = (value - norm_min) / norm_range
    return degree_min + t * (degree_max - degree_min)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=auto_port(), help="Serial port (COM# or /dev/ttyACM#)")
    ap.add_argument("--torque", choices=["on", "off"], default="off", help="Enable or disable motor torque")
    ap.add_argument("--deg", action="store_true", help="Report in degrees (body joints).")
    args = ap.parse_args()

    PORT = normalize_port(args.port)
    ROBOT_ID = "lerobot_arm"

    cfg = SO101FollowerConfig(
        port=PORT,
        id=ROBOT_ID,
        use_degrees=bool(args.deg),
    )
    f = SO101Follower(cfg)
    f.connect()
    joint_ranges = load_joint_ranges()

    try:
        if args.torque == "on":
            f.bus.enable_torque()
            print("Torque: ENABLED")
        else:
            f.bus.disable_torque()
            print("Torque: DISABLED (you can move joints by hand)")

        print(f"Reading Present_Position on {PORT} ... (Ctrl+C to stop)\n")
        print(f"{'Joint':<15}  {'Raw':>6}   {'Degrees':>9}   {'Frac':>5}")
        print("-" * 46)
        while True:
            positions = f.bus.sync_read("Present_Position")
            print("=" * 46)
            for joint, value in positions.items():
                whole = int(value)
                frac = abs(value - whole)
                if joint in joint_ranges:
                    dmin, dmax, norm_min = joint_ranges[joint]
                    degrees = norm_to_degrees(value, dmin, dmax, norm_min)
                else:
                    degrees = float("nan")
                print(f"{joint:<15}: {whole:>6d}   {degrees:>+8.2f}°   {frac:.3f}")
            print()
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        f.disconnect()

if __name__ == "__main__":
    main()
