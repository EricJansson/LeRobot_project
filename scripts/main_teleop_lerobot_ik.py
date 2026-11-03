"""Gamepad control using LeRobot IK with an optional LAN dashboard."""

from contextlib import nullcontext
from dataclasses import dataclass, field
import argparse
import os
import sys
from time import perf_counter, sleep

import numpy as np

from robot_program.lerobot_ik_controller import LeRobotIKController
from scripts.lerobot_web_dashboard import WebDashboard

RATE_HZ = 50
TELEMETRY_INTERVAL_S = 0.1
XYZ_STEP_M = 0.002
PITCH_STEP_DEG = 0.4
ROLL_STEP_DEG = 0.75  # Match the legacy 37.5 degrees per second at 50 Hz.
GRIPPER_STEP = 1.0
AXIS_LABELS = {0: "LX", 1: "LY", 2: "RX", 3: "RY", 4: "LT", 5: "RT"}
BUTTON_LABELS = {
    0: "Cross", 1: "Circle", 2: "Square", 3: "Triangle", 4: "LB", 5: "RB",
    6: "Back", 7: "Start", 8: "L3", 9: "R3",
}


@dataclass
class InputState:
    axes: dict[str, float] = field(default_factory=lambda: dict.fromkeys(AXIS_LABELS.values(), 0.0))
    hat: tuple[int, int] = (0, 0)
    active: bool = False
    was_active: bool = False
    buttons: dict[str, bool] = field(default_factory=lambda: dict.fromkeys(BUTTON_LABELS.values(), False))


def setup_gamepad(index: int, state: InputState):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    from controller.input.gamepad_mapper import Bindings, Gamepad

    gp = Gamepad(
        index=index, axis_labels=AXIS_LABELS, button_labels=BUTTON_LABELS,
        hat_labels={0: "DPad"}, deadzone=0.10, axis_change_threshold=0.02,
        poll_hz=120, triggers_are_unit=True, deadman_button=None,
    )

    def button(name, pressed):
        state.buttons[name] = pressed
        if name == "LB":
            state.active = pressed

    def axis(name, value):
        state.axes[name] = value

    def hat(_name, value):
        state.hat = value

    gp.set_bindings(Bindings(
        axes={name: axis for name in state.axes},
        buttons_down=dict.fromkeys(state.buttons, button),
        buttons_up=dict.fromkeys(state.buttons, button),
        hats={"DPad": hat},
    ))
    return gp


def tick(robot: LeRobotIKController, state: InputState):
    if not state.active:
        state.was_active = False
        return None
    if not state.was_active:
        robot.sync_from_hardware()
        state.was_active = True

    axes = state.axes
    # Fixed base/table axes: +X forward, -Y right, +Z up in the URDF.
    delta = XYZ_STEP_M * np.array([-axes["LY"], -axes["LX"], -axes["RY"]])
    pitch = np.deg2rad(PITCH_STEP_DEG * (axes["RT"] - axes["LT"]))
    roll = np.deg2rad(ROLL_STEP_DEG * axes["RX"])
    grip = GRIPPER_STEP * state.hat[1]
    if not np.isfinite([*delta, pitch, roll, grip]).all():
        raise ValueError("Gamepad reported nonfinite input")
    if not np.any(delta) and pitch == 0 and roll == 0:
        if grip:
            robot.move_gripper(grip)
        return None

    target = robot.target_pose.copy()
    target[:3, 3] += delta
    cp, sp = np.cos(pitch), np.sin(pitch)
    cr, sr = np.cos(roll), np.sin(roll)
    target[:3, :3] = target[:3, :3] @ np.array([
        [cp, 0, sp], [0, 1, 0], [-sp, 0, cp],
    ]) @ np.array([[cr, -sr, 0], [sr, cr, 0], [0, 0, 1]])
    return robot.move_to_pose(target, gripper_delta=grip)


def run_loop(robot, gp, state, dashboard=None):
    last_warning = float("-inf")
    next_sample = float("-inf")
    while True:
        start = perf_counter()
        if not gp.step():
            break
        result = tick(robot, state)
        if result is not None and not result.accepted and start - last_warning >= 1:
            print(f"IK rejected: {result.reason}")
            last_warning = start
        if dashboard is not None and start >= next_sample:
            joints, gripper, xyz_m = robot.read_telemetry()
            dashboard.publish(joints, gripper, xyz_m, state.active, gamepad={
                "name": gp.name,
                "axis_count": gp.num_axes,
                "button_count": gp.num_buttons,
                "hat_count": gp.num_hats,
                "deadzone": gp.deadzone,
                "axes": state.axes,
                "buttons": state.buttons,
                "dpad": state.hat,
            })
            # Absorb up to one tick of loop jitter without replaying missed samples.
            next_sample = max(next_sample, start - 1 / RATE_HZ) + TELEMETRY_INTERVAL_S
        sleep(max(0, 1 / RATE_HZ - (perf_counter() - start)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial adapter path (auto-selected if omitted)")
    parser.add_argument("--index", type=int, default=0, help="Gamepad device index")
    # Accept the old diagnostic option for existing launch commands.
    parser.add_argument("--no-joint-step-limit", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-web-dashboard", action="store_true", help="Disable read-only LAN telemetry")
    parser.add_argument("--web-host", default="0.0.0.0", help="Dashboard bind address (default: all IPv4 interfaces)")
    parser.add_argument("--web-port", type=int, default=8000, help="Dashboard HTTP port (default: 8000)")
    args = parser.parse_args()
    if not 0 <= args.web_port <= 65535:
        parser.error("--web-port must be between 0 and 65535")
    try:
        # Validate the model/dependencies and gamepad before opening serial.
        robot = LeRobotIKController.from_port(args.port)
        state = InputState()
        gp = setup_gamepad(args.index, state)
        web = nullcontext() if args.no_web_dashboard else WebDashboard(args.web_host, args.web_port)
        with web as dashboard, robot:
            if dashboard is not None:
                host = "<pi-lan-ip>" if args.web_host == "0.0.0.0" else args.web_host
                print(f"Dashboard available at http://{host}:{dashboard.port}/")
            print("LeRobot IK ready. Hold the left shoulder button to move. Ctrl+C exits.")
            run_loop(robot, gp, state, dashboard)
    except KeyboardInterrupt:
        pass
    except (ImportError, RuntimeError, ValueError, KeyError, OSError) as exc:
        parser.exit(1, f"LeRobot IK: {exc}\n")
    finally:
        pygame = sys.modules.get("pygame")
        if pygame is not None:
            pygame.quit()


if __name__ == "__main__":
    main()
