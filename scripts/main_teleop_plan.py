#!/usr/bin/env python3
"""
Thin entry point for PS-controller plan-based control of the SO101 arm.

All reusable logic (plan model, controller bindings, playback, dashboard IPC,
and the run loop) lives in teleop_plan_control.py. This script only wires the
hardware, launches the dashboard, and drives that loop.

Run as:
    python -m scripts.main_teleop_plan
"""

import subprocess
import sys
import argparse
from pathlib import Path

from robot_program.robot_arm_controller import RobotArmController
from robot_program.utils.ports import normalize_port, auto_port

try:
    from teleop_plan_control import Plan, plan_file_default, run_plan_teleop_loop
    from teleop_ik_control import TeleopState, setup_gamepad
    from dashboard_panels import default_command_file, default_plan_cmd_file, default_telemetry_file
except ModuleNotFoundError:  # running as `python -m scripts.main_teleop_plan`
    from scripts.teleop_plan_control import Plan, plan_file_default, run_plan_teleop_loop
    from scripts.teleop_ik_control import TeleopState, setup_gamepad
    from scripts.dashboard_panels import default_command_file, default_plan_cmd_file, default_telemetry_file


def main() -> None:
    ap = argparse.ArgumentParser(description="PS controller plan teleop for SO101")
    ap.add_argument("--port", default=None, help="Serial port (COM# or /dev/ttyACM#)")
    ap.add_argument("--index", type=int, default=0, help="Gamepad device index")
    ap.add_argument("--plan-file", default=None,
                    help="Path to plan JSON for save/load (default: .runtime/teleop_plan.json)")
    args = ap.parse_args()

    port = normalize_port(args.port) if args.port else normalize_port(auto_port())
    plan_path = Path(args.plan_file) if args.plan_file else plan_file_default()

    # Shared command file: the dashboard writes manual-motor goals, mute state,
    # and plan-edit commands here; this teleop process (which owns the serial
    # port) honours them.
    cmd_file = default_command_file()
    telemetry_file = default_telemetry_file()
    plan_cmd_file = default_plan_cmd_file()
    for f in (cmd_file, telemetry_file, plan_cmd_file):
        try:
            f.unlink(missing_ok=True)
        except OSError:
            pass

    # Dashboard's stdout/stderr is captured to a log file so that if it crashes
    # (its own console closes too fast to read), the traceback survives here.
    dashboard_log = Path(__file__).parent / "dashboard_plan_log.txt"

    # Launch the controller dashboard in a separate process.
    # Windows: use CREATE_NO_WINDOW so no extra (empty) console window pops up.
    CREATE_NO_WINDOW = 0x08000000
    dashboard_script = Path(__file__).parent / "controller_dashboard.py"
    with open(dashboard_log, "w") as dlf:
        dashboard_proc = subprocess.Popen(
            [sys.executable, str(dashboard_script), "--index", str(args.index),
             "--cmd-file", str(cmd_file), "--telemetry-file", str(telemetry_file),
             "--plan-file", str(plan_path), "--plan-cmd-file", str(plan_cmd_file)],
            stdout=dlf,
            stderr=subprocess.STDOUT,
            creationflags=CREATE_NO_WINDOW if sys.platform == "win32" else 0,
        )
    print(f"Controller dashboard launched (PID {dashboard_proc.pid}).")
    print(f"Dashboard log: {dashboard_log}")

    try:
        with RobotArmController.from_port(port=port) as robot:
            # connect() → enable_torque() → sync_from_hardware() already called by __enter__

            # Seed targets from the arm's actual current pose
            state = TeleopState.from_robot(robot)
            print(f"Connected. Starting pose: x={state.target_x:.2f} y={state.target_y:.2f} "
                f"z={state.target_z:.2f} phi={state.target_phi:.2f}")

            # Shared, mutable input containers observed by both the bindings
            # callbacks and the run loop.
            axes = {name: 0.0 for name in ("LX", "LY", "RX", "RY", "LT", "RT")}
            buttons_held: set = set()
            hat = [0, 0]

            plan = Plan()
            # Plan mode disables the Gamepad-level deadman (LB gating) so that
            # Back (cancel) and RB + Cross (record) always fire even when LB is
            # not held. LB is instead enforced for manual movement inside
            # run_plan_teleop_loop, keeping driving safe.
            gp = setup_gamepad(index=args.index, deadman_button=None)

            run_plan_teleop_loop(
                robot, state, gp, buttons_held, axes, hat,
                cmd_file, telemetry_file, plan_cmd_file, plan, plan_path,
                dashboard_alive=lambda: dashboard_proc.poll() is None,
            )
    finally:
        if dashboard_proc.poll() is None:
            dashboard_proc.terminate()
            try:
                dashboard_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                dashboard_proc.kill()
                dashboard_proc.wait()

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
    for f in (cmd_file, telemetry_file, plan_cmd_file):
        try:
            f.unlink(missing_ok=True)
        except OSError:
            pass


if __name__ == "__main__":
    main()
