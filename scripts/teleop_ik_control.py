#!/usr/bin/env python3
"""
Teleop with controller

PS controller teleop for the SO101 arm using Cartesian IK control.
Instead of moving individual joints, the sticks move the end-effector
in XYZ world space and the IK solver determines the joint angles.

Control mapping
---------------
  LB              : deadman (must hold; release stops all movement)
  Left stick LX   : move end-effector in Y  (forward / backward)
  Left stick LY   : move end-effector in X  (left / right)
  Right stick RY  : move end-effector in Z  (up / down)
  Right stick RX  : wrist_roll increment
  LT              : tilt end-effector down  (phi_deg -)
  RT              : tilt end-effector up    (phi_deg +)
  DPad up         : gripper close
  DPad down       : gripper open
  Face buttons    : quick preset poses (see QUICK_POSES below)
  R3 (hold)       : switch to fine-control step sizes
  Ctrl+C          : exit cleanly
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

# Ensure the project root is on sys.path regardless of how this script is invoked

from robot_program.robot_arm_controller import RobotArmController
from robot_program.utils.ports import normalize_port, auto_port
from controller.input.gamepad_mapper import Gamepad, Bindings

try:
    from dashboard_panels import (
        default_command_file,
        default_telemetry_file,
        read_command,
        write_command,
        write_json,
    )
except ModuleNotFoundError:  # running as `python -m scripts.teleop_ik_control`
    from scripts.dashboard_panels import (
        default_command_file,
        default_telemetry_file,
        read_command,
        write_command,
        write_json,
    )

# ---------------------------------------------------------------------------
# Speed / feel knobs
# ---------------------------------------------------------------------------
BASE_XYZ_STEP   = 0.25    # cm per tick at full stick deflection
FINE_XYZ_STEP   = 0.08   # cm per tick while R3 is held

BASE_ROLL_STEP  = 1.5    # degrees per tick
FINE_ROLL_STEP  = 0.4    # degrees per tick while R3 is held

BASE_PHI_STEP   = 1.5    # degrees per tick (LT/RT)
FINE_PHI_STEP   = 0.4    # degrees per tick while R3 is held

BASE_GRIP_STEP  = 2.0    # gripper units per tick
FINE_GRIP_STEP  = 0.7    # gripper units per tick while R3 is held

PHI_MIN_DEG     = -90.0  # end-effector tilt lower bound (degrees)
PHI_MAX_DEG     =  90.0  # end-effector tilt upper bound (degrees)

RATE_HZ         = 25
DEADZONE        = 0.10
AX_DELTA_THRESH = 0.02

# How stale (seconds) a dashboard mute/active status may be before it is ignored.
# The dashboard refreshes this every ~0.25s, so a value <= this means the
# dashboard is alive; older values mean it was closed/crashed -> don't stay muted.
STALE_DASHBOARD_S = 0.8

# How often (seconds) live joint positions are pushed to the dashboard.
TELEMETRY_INTERVAL_S = 0.1

# ---------------------------------------------------------------------------
# Quick preset poses
# ---------------------------------------------------------------------------
# Fast "Act"-style commands bound to the face buttons. Each is a hardcoded set
# of motor degrees (all six joints) that is applied with set_joint_angles when
# the matching button is pressed (like the dashboard's manual Act, but preset).
#
# Units are degrees, matching the calibrated motor limits. A "None" entry keeps
# the joint's current position. Edit these to your desired poses.
QUICK_POSES = {
    "Cross": { # Home / neutral pose
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.0,
    },
    "Circle": { # Stand tall with gripper open
        "shoulder_pan": 0.0,
        "shoulder_lift": -40.0,
        "elbow_flex": -10.0,
        "wrist_flex": 35.0,
        "wrist_roll": 0.0,
        "gripper": 30.0,
    },
    "Square": { # Table grab pose
        "shoulder_pan": 0.0,
        "shoulder_lift": -25.0,
        "elbow_flex": 50.0,
        "wrist_flex": 35.0,
        "wrist_roll": 0.0,
        "gripper": 15.0,
    },
    "Triangle": { # Eye contact pose
        "shoulder_pan": -45.0,
        "shoulder_lift": -60.0,
        "elbow_flex": 45.0,
        "wrist_flex": 55.0,
        "wrist_roll": -45.0,
        "gripper": 5.0,
    },
}


# ---------------------------------------------------------------------------
# Status display
# ---------------------------------------------------------------------------

def _print_status(
    mode: str,
    tx: float, ty: float, tz: float,
    phi: float, roll: float, grip: float,
    ik_ok: bool,
    motors: list[float] | None = None,
) -> None:
    ik_str = "OK  " if ik_ok else "FAIL"
    if motors is None:
        motors_str = "n/a"
    else:
        motors_str = "[" + ", ".join(f"{m:6.1f}" for m in motors) + "]"
    print(
        f"\r[{mode:<4}]  "
        f"xyz=({tx:6.2f}, {ty:6.2f}, {tz:6.2f})  "
        f"phi={phi:+6.1f}°  roll={roll:+6.1f}°  grip={grip:4.1f}  "
        f"motors={motors_str}  "
        f"IK:{ik_str}",
        end="", flush=True,
    )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser(description="PS controller XYZ teleop for SO101")
    ap.add_argument("--port",  default=None,       help="Serial port (COM# or /dev/ttyACM#)")
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

        # Seed target from the arm's actual current pose
        x, y, z, phi = robot.end_effector_pose()
        target_x, target_y, target_z = x, y, z
        target_phi  = phi
        target_roll = robot.wrist_roll
        target_grip = robot.gripper

        print(f"Connected. Starting pose: x={x:.2f} y={y:.2f} z={z:.2f} phi={phi:.2f}")
        print("Hold LB to move. Ctrl+C to exit.\n")

        # ---- controller labels -----------------------------------------
        axis_labels   = {0: "LX", 1: "LY", 2: "RX", 3: "RY", 4: "LT", 5: "RT"}
        button_labels = {
            0: "Cross", 1: "Circle", 2: "Square", 3: "Triangle",
            4: "LB", 5: "RB", 6: "Back", 7: "Start", 8: "L3", 9: "R3",
        }
        hat_labels = {0: "DPad"}

        # ---- live input state -------------------------------------------
        axes        = {name: 0.0 for name in axis_labels.values()}
        buttons_held: set = set()
        hat         = (0, 0)      # D-pad (x, y); y is +1 for up, -1 for down
        deadman     = "LB"

        def on_button_down(name: str, _pressed: bool) -> None:
            buttons_held.add(name)
            # Quick preset poses fire on the press edge (once per press).
            if name in QUICK_POSES:
                _apply_joints_and_resync(QUICK_POSES[name])

        def on_button_up(name: str, _pressed: bool) -> None:
            buttons_held.discard(name)

        def on_axis(name: str, value: float) -> None:
            axes[name] = float(value)

        def on_hat(_name: str, xy) -> None:
            nonlocal hat
            hat = (int(xy[0]), int(xy[1]))

        def _apply_joints_and_resync(joints: dict) -> None:
            """Apply a joint-angle goal and re-sync teleop's local targets.

            Reuses the robot controller (which clamps against calibrated limits)
            exactly like the dashboard's manual Act does, but for preset poses.
            Values left as None keep their current position.
            """
            robot.set_joint_angles(**joints)
            nonlocal target_x, target_y, target_z, target_phi, target_roll, target_grip
            x, y, z, phi = robot.end_effector_pose()
            target_x, target_y, target_z, target_phi = x, y, z, phi
            target_roll = robot.wrist_roll
            target_grip = robot.gripper
            robot.sync_from_hardware()

        bindings = Bindings(
            buttons_down={k: on_button_down for k in button_labels.values()},
            buttons_up  ={k: on_button_up   for k in button_labels.values()},
            axes        ={k: on_axis        for k in axes},
            hats        ={"DPad": on_hat},
        )

        gp = Gamepad(
            index=args.index,
            axis_labels=axis_labels,
            button_labels=button_labels,
            hat_labels=hat_labels,
            deadzone=DEADZONE,
            axis_change_threshold=AX_DELTA_THRESH,
            poll_hz=120,
            triggers_are_unit=True,   # LT/RT reported as 0..1
            deadman_button=deadman,
        )
        gp.set_bindings(bindings, profile="xyz_teleop")
        gp.switch_profile("xyz_teleop")

        def motor_degrees() -> list[float]:
            s = robot.arm.state
            return [
                s.base_yaw_deg,
                s.theta1_deg,
                s.theta2_deg,
                s.theta3_deg,
                robot.wrist_roll,
                robot.gripper,
            ]

        def handle_dashboard_command() -> dict | None:
            """Process any manual-motor goal / mute request from the dashboard.

            Teleop owns the serial port, so it is the one that actually drives
            the arm. Joints are applied via set_joint_angles, which clamps each
            value against the calibrated limits as the final safety net.
            Returns the raw command dict (or None) for the caller to inspect.
            """
            nonlocal last_pending_ts
            data = read_command(cmd_file)
            if not data:
                return None

            status = data.get("status")
            now = time.time()

            # Only trust mute/active while the dashboard is alive (fresh writes).
            if status in ("active", "muted"):
                if (now - data.get("ts", 0)) > STALE_DASHBOARD_S:
                    return None  # dashboard closed / stale -> keep controller live
                return data  # mute handled by the caller below

            if status == "pending":
                ts = data.get("ts", 0)
                if ts == last_pending_ts:
                    # Same command as the one we already applied on a previous
                    # tick — skip so we don't re-command the arm every frame.
                    return read_command(cmd_file)

                joints = data.get("joints") or {}
                # IMPORTANT: This is an INSTANT jump to the requested joint angles.
                # If you later want smooth/interpolated motion instead, replace
                # set_joint_angles with a small stepping loop that animates toward
                # `joints` over time before returning.
                robot.set_joint_angles(**joints)
                last_pending_ts = ts

                # Re-sync the teleop's own targets so they don't fight the arm.
                nonlocal target_x, target_y, target_z, target_phi, target_roll, target_grip
                x, y, z, phi = robot.end_effector_pose()
                target_x, target_y, target_z, target_phi = x, y, z, phi
                target_roll = robot.wrist_roll
                target_grip = robot.gripper

                # Acknowledge back with the achieved angles so the dashboard can
                # re-populate its fields from hardware truth. This goes to the
                # telemetry file (teleop -> dashboard direction).
                robot.sync_from_hardware()
                achieved = {
                    "shoulder_pan":  robot.state.base_yaw_deg,
                    "shoulder_lift": robot.state.theta1_deg,
                    "elbow_flex":    robot.state.theta2_deg,
                    "wrist_flex":    robot.state.theta3_deg,
                    "wrist_roll":    robot.wrist_roll,
                    "gripper":       robot.gripper,
                }
                write_json(telemetry_file, {"status": "done", "joints": achieved})
                # Clear the pending state on the command file so the dashboard's
                # own mute-writer resumes (it is suppressed while "pending").
                write_json(cmd_file, {"status": "active"})
                return read_command(cmd_file)  # now the "done" ack

            return data

        def push_telemetry() -> None:
            """Stream live joint positions to the dashboard (throttled)."""
            nonlocal _last_telemetry
            if time.time() - _last_telemetry < TELEMETRY_INTERVAL_S:
                return
            joints = {
                "shoulder_pan":  robot.state.base_yaw_deg,
                "shoulder_lift": robot.state.theta1_deg,
                "elbow_flex":    robot.state.theta2_deg,
                "wrist_flex":    robot.state.theta3_deg,
                "wrist_roll":    robot.wrist_roll,
                "gripper":       robot.gripper,
            }
            write_json(telemetry_file, {"status": "telemetry", "joints": joints})
            _last_telemetry = time.time()

        dt = 1.0 / RATE_HZ
        last_ik_ok = True  # tracked across ticks for status display
        _last_telemetry = 0.0
        last_pending_ts = 0  # timestamp of the last dashboard goal we applied

        try:
            while True:
                if not gp.step():
                    break  # controller disconnected

                dashboard_cmd = handle_dashboard_command()
                push_telemetry()

                controller_muted = (
                    dashboard_cmd is not None
                    and dashboard_cmd.get("status") == "muted"
                    and (time.time() - dashboard_cmd.get("ts", 0)) <= STALE_DASHBOARD_S
                )

                if deadman not in buttons_held or controller_muted:
                    _print_status("IDLE", target_x, target_y, target_z,
                                  target_phi, target_roll, target_grip, last_ik_ok,
                                  motors=motor_degrees())
                    time.sleep(dt)
                    continue

                # Fine-control mode when R3 is held
                fine = "R3" in buttons_held
                xyz_step  = FINE_XYZ_STEP  if fine else BASE_XYZ_STEP
                roll_step = FINE_ROLL_STEP if fine else BASE_ROLL_STEP
                phi_step  = FINE_PHI_STEP  if fine else BASE_PHI_STEP
                grip_step = FINE_GRIP_STEP if fine else BASE_GRIP_STEP

                # --- XYZ increments from sticks --------------------------
                target_x   += xyz_step  * -axes["LY"]   # invert for natural feel
                target_y   += xyz_step  *  axes["LX"]
                target_z   += xyz_step  * -axes["RY"]   # invert Y: stick up → arm up
                target_roll += roll_step *  axes["RX"]

                # --- End-effector tilt from triggers ----------------------
                target_phi += phi_step * (axes["RT"] - axes["LT"])

                # --- Gripper from D-pad ----------------------------------
                # DPad up (y=+1) closes, DPad down (y=-1) opens.
                hat_y = hat[1]
                if hat_y < 0:
                    target_grip -= grip_step   # up -> close
                elif hat_y > 0:
                    target_grip += grip_step   # down -> open

                # Clamp phi before passing to IK — no lower-level guard exists for it
                target_phi = max(PHI_MIN_DEG, min(PHI_MAX_DEG, target_phi))

                # --- Send to hardware via IK ------------------------------
                ok = robot.move_to_xyz(
                    target_x, target_y, target_z,
                    phi_deg    = target_phi,
                    wrist_roll = target_roll,
                    gripper    = target_grip,
                )

                if ok:
                    # Sync roll/grip locals back from the robot's clamped values so the
                    # local targets don't wind up past the hardware limits (send_state()
                    # clamps before every write, but un-synced locals cause input lag on reversal).
                    target_roll = robot.wrist_roll
                    target_grip = robot.gripper
                else:
                    # IK failed — snap all targets back to last known good pose
                    # so we don't drift further into unreachable space
                    target_x, target_y, target_z, target_phi = robot.end_effector_pose()
                    target_roll = robot.wrist_roll
                    target_grip = robot.gripper

                last_ik_ok = ok
                mode = "FINE" if fine else "MOVE"
                _print_status(mode, target_x, target_y, target_z,
                              target_phi, target_roll, target_grip, last_ik_ok,
                              motors=motor_degrees())

                time.sleep(dt)

        except KeyboardInterrupt:
            pass

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
