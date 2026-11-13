"""Read a mapped SDL controller for Windows remote teleop."""

from contextlib import contextmanager

from controller.input.pygame_support import quiet_pygame_import
from controller.input.xinput import STICK_DEADZONE, TRIGGER_DEADZONE


with quiet_pygame_import():
    import pygame
    from pygame._sdl2 import controller


STICKS = {
    "LX": pygame.CONTROLLER_AXIS_LEFTX, "LY": pygame.CONTROLLER_AXIS_LEFTY,
    "RX": pygame.CONTROLLER_AXIS_RIGHTX, "RY": pygame.CONTROLLER_AXIS_RIGHTY,
}
TRIGGERS = {"LT": pygame.CONTROLLER_AXIS_TRIGGERLEFT, "RT": pygame.CONTROLLER_AXIS_TRIGGERRIGHT}
BUTTONS = {
    "Cross": pygame.CONTROLLER_BUTTON_A, "Circle": pygame.CONTROLLER_BUTTON_B,
    "Square": pygame.CONTROLLER_BUTTON_X, "Triangle": pygame.CONTROLLER_BUTTON_Y,
    "LB": pygame.CONTROLLER_BUTTON_LEFTSHOULDER, "RB": pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,
    "Back": pygame.CONTROLLER_BUTTON_BACK, "Start": pygame.CONTROLLER_BUTTON_START,
    "L3": pygame.CONTROLLER_BUTTON_LEFTSTICK, "R3": pygame.CONTROLLER_BUTTON_RIGHTSTICK,
}


class NoSDLGamepad(RuntimeError):
    """No mapped SDL device matches the requested selection."""


@contextmanager
def open_sdl_gamepad(index=None):
    # Initialize the event pump without opening a window. The browser can keep focus.
    try:
        pygame.display.init()
        controller.init()
        try:
            pygame.event.clear()
            mapped = [i for i in range(controller.get_count()) if controller.is_controller(i)]
            if index is None and mapped:
                index = mapped[0]
            if index not in mapped:
                available = ", ".join(
                    f"{i}: {controller.name_forindex(i)}" for i in mapped
                ) or "none"
                selection = "" if index is None else f" at index {index}"
                raise NoSDLGamepad(
                    f"No mapped SDL gamepad{selection}. Available gamepads: {available}. "
                    "Connect the gamepad in Windows, then relaunch with its --index."
                )
            try:
                pad = controller.Controller(index)
            except pygame.error as exc:
                # The device may disappear between enumeration and opening.
                raise NoSDLGamepad(f"Could not open SDL gamepad at index {index}: {exc}") from exc
            try:
                yield SDLGamepad(pad)
            finally:
                pad.quit()
        finally:
            controller.quit()
    except pygame.error as exc:
        raise RuntimeError(f"SDL gamepad error: {exc}") from exc
    finally:
        pygame.display.quit()


class SDLGamepad:
    def __init__(self, pad):
        self.pad = pad
        self.name = pad.name

    def read(self):
        pygame.event.clear()  # Pump input and drain events from the headless polling loop.
        pad = self.pad
        # Keep the original SDL instance. Device indices can be reassigned on disconnect.
        if not pad.attached():
            return None

        axes = {}
        for label, axis in STICKS.items():
            value = pad.get_axis(axis)
            normalized = value / (32768 if value < 0 else 32767)
            axes[label] = normalized if abs(normalized) >= STICK_DEADZONE else 0.
        for label, axis in TRIGGERS.items():
            normalized = pad.get_axis(axis) / 32767
            axes[label] = normalized if normalized >= TRIGGER_DEADZONE else 0.

        buttons = {label: bool(pad.get_button(button)) for label, button in BUTTONS.items()}
        dpad = (
            bool(pad.get_button(pygame.CONTROLLER_BUTTON_DPAD_RIGHT))
            - bool(pad.get_button(pygame.CONTROLLER_BUTTON_DPAD_LEFT)),
            bool(pad.get_button(pygame.CONTROLLER_BUTTON_DPAD_UP))
            - bool(pad.get_button(pygame.CONTROLLER_BUTTON_DPAD_DOWN)),
        )
        return {"name": self.name, "axes": axes, "buttons": buttons, "dpad": dpad,
                "axis_count": len(axes), "button_count": len(buttons), "hat_count": 1, "deadzone": STICK_DEADZONE}
