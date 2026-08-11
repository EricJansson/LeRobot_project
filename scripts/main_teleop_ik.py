#!/usr/bin/env python3
"""
Thin entry point for the PS-controller XYZ IK teleop of the SO101 arm.

All reusable logic (controller setup, IK movement primitives, dashboard IPC,
and the run loop) lives in teleop_ik_control.py. This script only wires the
hardware, launches the dashboard, and drives that loop.

Run as:
    python -m scripts.main_teleop_ik
"""

import subprocess
import sys
import argparse
from pathlib import Path

from robot_program.robot_arm_controller import RobotArmController
from robot_program.utils.ports import normalize_port, auto_port

try:
    from teleop_ik_control import (
        TeleopState,
        setup_gamepad,
        setup_teleop_bindings,
        run_ik_teleop_loop,
    )
    from dashboard_panels import default_command_file, default_telemetry_file
except ModuleNotFoundError:  # running as `python -m scripts.main_teleop_ik`
    from scripts.teleop_ik_control import (
        TeleopState,
        setup_gamepad,
        setup_teleop_bindings,
        run_ik_teleop_loop,
    )
    from scripts.dashboard_panels import default_command_file, default_telemetry_file


def main() -> None:
    ap = argparse.ArgumentParser(description="PS controller XYZ teleop for SO101")
    ap.add_argument("--port", default=None, help="Serial port (COM# or /dev/ttyACM#)")
    ap.add_argument("--index", type=int, default=0, help="Gamepad device index")
    args = ap.parse_args()

    port = normalize_port(args.port) if args.port else normalize_port(auto_port())

    # Shared command file: the dashboard writes manual-motor goals / mute state
    # here, and this teleop process (which owns the serial port) honours them.
    cmd_file = default_command_file()
    telemetry_file = default_telemetry_file()
    for f in (cmd_file, telemetry_file):
        try:
            f.unlink(missing_ok=True)
        except OSError:
            pass

    # Dashboard's stdout/stderr is captured to a log file so that if it crashes
    # (its own console closes too fast to read), the traceback survives here.
    dashboard_log = Path(__file__).parent / "dashboard_log.txt"

    # Launch the controller dashboard in a separate process
    dashboard_script = Path(__file__).parent / "controller_dashboard.py"
    with open(dashboard_log, "w") as dlf:
        dashboard_proc = subprocess.Popen(
            [sys.executable, str(dashboard_script), "--index", str(args.index),
             "--cmd-file", str(cmd_file), "--telemetry-file", str(telemetry_file)],
            stdout=dlf,
            stderr=subprocess.STDOUT,
            creationflags=subprocess.CREATE_NEW_CONSOLE if sys.platform == "win32" else 0,
        )
    print(f"Controller dashboard launched (PID {dashboard_proc.pid}).")
    print(f"Dashboard log: {dashboard_log}")

    with RobotArmController.from_port(port=port) as robot:
        # connect() → enable_torque() → sync_from_hardware() already called by __enter__

        # Seed targets from the arm's actual current pose
        state = TeleopState.from_robot(robot)
        print(f"Connected. Starting pose: x={state.target_x:.2f} y={state.target_y:.2f} "
              f"z={state.target_z:.2f} phi={state.target_phi:.2f}")
        print("Hold LB to move. Ctrl+C to exit.\n")

        # Shared, mutable input containers observed by both the bindings
        # callbacks and the run loop. "hat" is a 2-element mutable list that the
        # callback updates in place (setup_teleop_bindings writes hat[0]/hat[1]).
        axes = {name: 0.0 for name in ("LX", "LY", "RX", "RY", "LT", "RT")}
        buttons_held: set = set()
        hat = [0, 0]

        bindings = setup_teleop_bindings(robot, state, buttons_held, axes, hat)

        gp = setup_gamepad(index=args.index)
        gp.set_bindings(bindings, profile="xyz_teleop")
        gp.switch_profile("xyz_teleop")

        run_ik_teleop_loop(
            robot, state, gp, buttons_held, axes, hat,
            cmd_file, telemetry_file,
        )

    print("\nDisconnected and torque disabled.")

    # Shut down the dashboard window
    if dashboard_proc.poll() is None:
        dashboard_proc.terminate()
        dashboard_proc.wait(timeout=3)
    print("Controller dashboard closed.")

    # If the dashboard crashed, point the user at the captured log.
    if dashboard_log.exists() and dashboard_log.stat().st_size:
        print(f"Dashboard output was logged to: {dashboard_log}")

    # Remove the shared files so a later run starts from a clean slate.
    for f in (cmd_file, telemetry_file):
        try:
            f.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    main()
