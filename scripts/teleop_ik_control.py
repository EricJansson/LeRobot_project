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
  Cross           : gripper close
  Circle          : gripper open
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


# ---------------------------------------------------------------------------
# Status display
# ---------------------------------------------------------------------------

def _print_status(
    mode: str,
    tx: float, ty: float, tz: float,
    phi: float, roll: float, grip: float,
    ik_ok: bool,
) -> None:
    ik_str = "OK  " if ik_ok else "FAIL"
    print(
        f"\r[{mode:<4}]  "
        f"xyz=({tx:6.2f}, {ty:6.2f}, {tz:6.2f})  "
        f"phi={phi:+6.1f}°  roll={roll:+6.1f}°  grip={grip:4.1f}  "
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

    # Launch the controller dashboard in a separate process
    dashboard_script = Path(__file__).parent / "controller_dashboard.py"
    dashboard_proc = subprocess.Popen(
        [sys.executable, str(dashboard_script), "--index", str(args.index)],
        creationflags=subprocess.CREATE_NEW_CONSOLE if sys.platform == "win32" else 0,
    )
    print(f"Controller dashboard launched (PID {dashboard_proc.pid}).")

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
        deadman     = "LB"

        def on_button_down(name: str, _pressed: bool) -> None:
            buttons_held.add(name)

        def on_button_up(name: str, _pressed: bool) -> None:
            buttons_held.discard(name)

        def on_axis(name: str, value: float) -> None:
            axes[name] = float(value)

        def on_hat(_name: str, _xy) -> None:
            pass

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

        dt = 1.0 / RATE_HZ
        last_ik_ok = True  # tracked across ticks for status display

        try:
            while True:
                if not gp.step():
                    break  # controller disconnected

                if deadman not in buttons_held:
                    _print_status("IDLE", target_x, target_y, target_z,
                                  target_phi, target_roll, target_grip, last_ik_ok)
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

                # --- Gripper from face buttons ----------------------------
                if "Cross"  in buttons_held:
                    target_grip -= grip_step
                if "Circle" in buttons_held:
                    target_grip += grip_step

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
                              target_phi, target_roll, target_grip, last_ik_ok)

                time.sleep(dt)

        except KeyboardInterrupt:
            pass

    print("\nDisconnected and torque disabled.")

    # Shut down the dashboard window
    if dashboard_proc.poll() is None:
        dashboard_proc.terminate()
        dashboard_proc.wait(timeout=3)
    print("Controller dashboard closed.")


if __name__ == "__main__":
    main()
