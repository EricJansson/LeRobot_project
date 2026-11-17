"""Receive phone control and marker poses on Windows with local IK simulation."""

import argparse
from contextlib import ExitStack
from ipaddress import IPv4Address, IPv4Network
from pathlib import Path
import re
import socket
from threading import Event, Thread

from robot_program.remote_ik_control import ControlLoop
from scripts.lerobot_web_dashboard import WebDashboard
from scripts.main_teleop_remote_ik import (
    DEFAULT_WSL_PYTHON, TELEOP_ERRORS, VK_ESCAPE, Worker, load_key_state, local_simulated_link,
    publish_remote_dashboard,
)
from scripts.mobile_teleop_gateway import (
    ARM_INPUT_PATH, INPUT_PATH, POSE_PATH, TOUCHSCREEN_PATH, MobileGateway, MobileInput, mobile_arm_status,
)


TOKEN_PATH = Path(__file__).resolve().parents[1] / "mobile-teleop.token"
LOOPBACK = "127.0.0.1"
LAN_NETWORKS = tuple(IPv4Network(cidr) for cidr in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))


def load_phone_token(path):
    try:
        token = path.read_text(encoding="utf-8-sig").strip()
    except FileNotFoundError as exc:
        raise ValueError(f"Add your phone's shared bearer token to {path} (one line).") from exc
    except UnicodeError as exc:
        raise ValueError(f"Save {path} as UTF-8 text.") from exc
    if not re.fullmatch(r"[A-Za-z0-9._~+/-]+=*", token):
        raise ValueError(f"Enter a single bearer token in {path}, with no 'Bearer ' prefix.")
    return token


def default_input_address():
    try:
        # Ask Windows for the route's source address. No datagram is sent.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
            route.connect(("192.0.2.1", 9))
            address = IPv4Address(route.getsockname()[0])
    except OSError:
        return LOOPBACK
    return str(address) if any(address in network for network in LAN_NETWORKS) else LOOPBACK


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--dry-run", action="store_true", help="Use WSL2 IK and a local simulated arm (default)")
    modes.add_argument("--input-only", action="store_true", help="Show phone input without starting WSL2")
    parser.add_argument("--listen", help="Override the automatically selected LAN IPv4 address")
    parser.add_argument("--input-port", type=int, default=18744, help="Phone WebSocket port (default %(default)s)")
    parser.add_argument("--dashboard-port", type=int, default=18745, help="Local dashboard port (default %(default)s)")
    parser.add_argument("--wsl-distro", default="Ubuntu")
    parser.add_argument("--wsl-python", default=DEFAULT_WSL_PYTHON, help="Linux path, absolute or relative to WSL home")
    args = parser.parse_args()
    if any(not 0 <= port <= 65535 for port in (args.input_port, args.dashboard_port)):
        parser.error("Ports must be between 0 and 65535")

    phone = MobileInput()
    stop = Event()
    page = Path(__file__).with_name("mobile_teleop_dashboard.html")
    control = None

    def arm_status():
        if args.input_only:
            return mobile_arm_status(None)
        return mobile_arm_status(control.status if control is not None else {})

    try:
        token = load_phone_token(TOKEN_PATH)
        host = args.listen or default_input_address()
        get_key_state = load_key_state()
        with ExitStack() as stack:
            gateway = stack.enter_context(MobileGateway(host, args.input_port, phone, token, arm_status=arm_status))
            dashboard = stack.enter_context(WebDashboard(LOOPBACK, args.dashboard_port, page=page))
            mode = "Input only" if args.input_only else "Local simulation"
            if not args.input_only:
                worker = stack.enter_context(Worker(args.wsl_distro, args.wsl_python))
                connect = local_simulated_link(stack, worker, stop)
                control = ControlLoop(worker, connect, phone, stop, 1., mode, input_name="phone")
                thread = Thread(target=control.run, daemon=True)
                thread.start()
                stack.callback(thread.join, 2)
                stack.callback(stop.set)

            pose_heading = "Phone Vision (ArUco) - paste this URL into the phone app"
            if gateway.address == LOOPBACK:
                pose_heading = "ArUco pose (PC-local testing)"
                print("Phone access requires the PC's LAN IPv4 address. Restart with --listen <PC-LAN-IPv4>.",
                      flush=True)
            ws_address = f"ws://{gateway.address}:{gateway.port}"
            print(f"{pose_heading}\n"
                  f"  {ws_address}{POSE_PATH}\n"
                  f"  In the app's bearer token field, enter the value from {TOKEN_PATH.name}.\n"
                  f"{mode} dashboard (PC browser)\n"
                  f"  http://{LOOPBACK}:{dashboard.port}/\n"
                  "Touchscreen monitoring\n"
                  f"  {ws_address}{TOUCHSCREEN_PATH}\n"
                  "Touchscreen arm control\n"
                  f"  {ws_address}{ARM_INPUT_PATH}\n"
                  "Legacy joystick / XYZ velocity input\n"
                  f"  {ws_address}{INPUT_PATH}\n"
                  "Esc or Ctrl+C exits.", flush=True)
            while not stop.is_set():
                if get_key_state(VK_ESCAPE) < 0:
                    dashboard.exit_reason = "Esc pressed"
                    break
                status = dict(control.status) if control else {"mode": mode, "state": "Monitoring"}
                status["arm_status"] = mobile_arm_status(None if args.input_only else status)
                status.update(gateway.telemetry())
                publish_remote_dashboard(dashboard, status)
                stop.wait(.05)
            if control and control.error:
                dashboard.exit_reason = str(control.error)
                raise control.error
    except KeyboardInterrupt:
        pass
    except TELEOP_ERRORS as exc:
        parser.exit(1, f"Mobile teleop: {exc}\n")


if __name__ == "__main__":
    main()
