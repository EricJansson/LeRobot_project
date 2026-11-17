"""Receive phone input on Windows for monitoring, IK simulation, or Pi arm control."""

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
    DEFAULT_WSL_PYTHON, TELEOP_ERRORS, VK_ESCAPE, PiTunnel, Worker, load_key_state, local_simulated_link,
    publish_remote_dashboard,
)
from scripts.mobile_teleop_gateway import (
    ARM_INPUT_PATH, INPUT_PATH, POSE_PATH, TOUCHSCREEN_PATH, MobileGateway, MobileInput, mobile_arm_status,
)
from scripts.remote_ik_deploy import ensure_installed


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
    modes.add_argument("--dry-run", action="store_true", help="Use WSL2 IK and a local simulated arm")
    modes.add_argument("--input-only", action="store_true", help="Show phone input without starting WSL2")
    modes.add_argument("--pi-sim", action="store_true", help="Use the Pi's simulated motor bus over SSH")
    modes.add_argument("--real-arm", action="store_true", help="Connect to the physical arm on the Pi (default)")
    parser.add_argument("--pi", default="eric@rpi", help="SSH destination")
    parser.add_argument("--speed", type=float, help="Motion scale (default: 0.1 for --pi-sim, 1 otherwise)")
    parser.add_argument("--listen", help="Override the automatically selected LAN IPv4 address")
    parser.add_argument("--input-port", type=int, default=18744, help="Phone WebSocket port (default %(default)s)")
    parser.add_argument("--dashboard-port", type=int, default=18745, help="Local dashboard port (default %(default)s)")
    parser.add_argument("--wsl-distro", default="Ubuntu")
    parser.add_argument("--wsl-python", default=DEFAULT_WSL_PYTHON, help="Linux path, absolute or relative to WSL home")
    args = parser.parse_args()
    if any(not 0 <= port <= 65535 for port in (args.input_port, args.dashboard_port)):
        parser.error("Ports must be between 0 and 65535")
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*", args.pi):
        parser.error("--pi must be user@host")
    real_arm = args.real_arm or not (args.dry_run or args.input_only or args.pi_sim)
    uses_pi = args.pi_sim or real_arm
    speed = args.speed if args.speed is not None else (.1 if args.pi_sim else 1.)
    if not 0 < speed <= 1:
        parser.error("--speed must be greater than 0 and at most 1")
    if real_arm and args.listen:
        try:
            address = IPv4Address(args.listen)
        except ValueError:
            parser.error("Use a private LAN IPv4 address or 127.0.0.1 in real-arm mode")
        if address != IPv4Address(LOOPBACK) and not any(address in network for network in LAN_NETWORKS):
            parser.error("Use a private LAN IPv4 address or 127.0.0.1 in real-arm mode")

    phone = MobileInput()
    stop = Event()
    page = Path(__file__).with_name("mobile_teleop_dashboard.html")
    control = None
    mode = ("Input only" if args.input_only else "Real arm" if real_arm else
            "Pi simulation" if args.pi_sim else "Local simulation")

    def arm_status():
        if args.input_only:
            return mobile_arm_status(None)
        return mobile_arm_status(control.status if control is not None else {})

    def release_progress():
        return control.status.get("release_progress", 0.) if control is not None else 0.

    try:
        token = load_phone_token(TOKEN_PATH)
        host = args.listen or default_input_address()
        get_key_state = load_key_state()
        with ExitStack() as stack:
            gateway = stack.enter_context(MobileGateway(host, args.input_port, phone, token, arm_status=arm_status,
                                                       release_progress=release_progress, allow_legacy_input=not uses_pi))
            dashboard = stack.enter_context(WebDashboard(LOOPBACK, args.dashboard_port, page=page))
            if not args.input_only:
                worker = stack.enter_context(Worker(args.wsl_distro, args.wsl_python))
                if uses_pi:
                    fingerprint = ensure_installed(args.pi)
                    tunnel = stack.enter_context(PiTunnel(args.pi, args.pi_sim, fingerprint=fingerprint))
                    connect = tunnel.connect
                else:
                    connect = local_simulated_link(stack, worker, stop)
                control = ControlLoop(worker, connect, phone, stop, speed, mode, input_name="phone",
                                      allow_limit_recovery=not uses_pi,
                                      on_link_lost=phone.end if uses_pi else None)
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
            legacy_endpoint = "" if uses_pi else f"Legacy joystick / XYZ velocity input\n  {ws_address}{INPUT_PATH}\n"
            print(f"{pose_heading}\n"
                  f"  {ws_address}{POSE_PATH}\n"
                  f"  In the app's bearer token field, enter the value from {TOKEN_PATH.name}.\n"
                  f"{mode} dashboard (PC browser)\n"
                  f"  http://{LOOPBACK}:{dashboard.port}/\n"
                  "Touchscreen monitoring\n"
                  f"  {ws_address}{TOUCHSCREEN_PATH}\n"
                  "Touchscreen arm control\n"
                  f"  {ws_address}{ARM_INPUT_PATH}\n"
                  f"{legacy_endpoint}"
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
