#!/usr/bin/env python3
import time
import argparse
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from utils.ports import normalize_port, auto_port

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

    try:
        if args.torque == "on":
            f.bus.enable_torque()
            print("Torque: ENABLED")
        else:
            f.bus.disable_torque()
            print("Torque: DISABLED (you can move joints by hand)")

        print(f"Reading Present_Position on {PORT} ... (Ctrl+C to stop)\n")
        while True:
            positions = f.bus.sync_read("Present_Position")
            print("=" * 40)
            for joint, value in positions.items():
                whole = int(value)
                frac = abs(value - whole)
                print(f"{joint:<15}: {whole:>6d}{' ' * 10}{frac:.3f}")
            print()
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    finally:
        f.disconnect()

if __name__ == "__main__":
    main()
