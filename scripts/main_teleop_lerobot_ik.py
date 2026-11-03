"""Headless gamepad control using LeRobot IK and the validated fine increments."""

from dataclasses import dataclass, field
import argparse
import os
import sys
from time import perf_counter, sleep

import numpy as np

from robot_program.lerobot_ik_controller import LeRobotIKController

RATE_HZ = 50
XYZ_STEP_M = 0.002
ANGLE_STEP_DEG = 0.4
GRIPPER_STEP = 0.7
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


def setup_gamepad(index: int, state: InputState):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    from controller.input.gamepad_mapper import Bindings, Gamepad

    gp = Gamepad(
        index=index, axis_labels=AXIS_LABELS, button_labels=BUTTON_LABELS,
        hat_labels={0: "DPad"}, deadzone=0.10, axis_change_threshold=0.02,
        poll_hz=120, triggers_are_unit=True, deadman_button=None,
    )

    def button(_name, pressed):
        state.active = pressed

    def axis(name, value):
        state.axes[name] = value

    def hat(_name, value):
        state.hat = value

    gp.set_bindings(Bindings(
        axes={name: axis for name in state.axes},
        buttons_down={"LB": button}, buttons_up={"LB": button},
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
    pitch = np.deg2rad(ANGLE_STEP_DEG * (axes["RT"] - axes["LT"]))
    roll = np.deg2rad(ANGLE_STEP_DEG * axes["RX"])
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


def run_loop(robot, gp, state):
    last_warning = float("-inf")
    while True:
        start = perf_counter()
        if not gp.step():
            break
        result = tick(robot, state)
        if result is not None and not result.accepted and start - last_warning >= 1:
            print(f"IK rejected: {result.reason}")
            last_warning = start
        sleep(max(0, 1 / RATE_HZ - (perf_counter() - start)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="Serial adapter path (auto-selected if omitted)")
    parser.add_argument("--index", type=int, default=0, help="Gamepad device index")
    args = parser.parse_args()
    try:
        # Validate the model/dependencies and gamepad before opening serial.
        robot = LeRobotIKController.from_port(args.port)
        state = InputState()
        gp = setup_gamepad(args.index, state)
        with robot:
            print("LeRobot IK ready with fine increments. Hold the left shoulder button to move; Ctrl+C exits.")
            run_loop(robot, gp, state)
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
