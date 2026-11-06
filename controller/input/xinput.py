"""Read gamepad state from the Windows XInput API."""

import ctypes
import sys


STICK_DEADZONE = 0.10
TRIGGER_DEADZONE = 0.04


class XInputPad(ctypes.Structure):
    _fields_ = [("buttons", ctypes.c_uint16), ("lt", ctypes.c_uint8), ("rt", ctypes.c_uint8),
                ("lx", ctypes.c_int16), ("ly", ctypes.c_int16), ("rx", ctypes.c_int16), ("ry", ctypes.c_int16)]


class XInputState(ctypes.Structure):
    _fields_ = [("packet", ctypes.c_uint32), ("pad", XInputPad)]


def load_xinput():
    if sys.platform != "win32":
        raise RuntimeError("Remote teleop requires Windows XInput")
    get_state = ctypes.WinDLL("xinput1_4.dll", winmode=0x800).XInputGetState  # Search System32.
    get_state.argtypes = (ctypes.c_uint32, ctypes.POINTER(XInputState))
    get_state.restype = ctypes.c_uint32
    return get_state


class XInputGamepad:
    """Read a single Windows XInput slot as a teleop and dashboard sample."""

    BUTTONS = {"Cross": 0x1000, "Circle": 0x2000, "Square": 0x4000, "Triangle": 0x8000,
               "LB": 0x0100, "RB": 0x0200, "Back": 0x0020, "Start": 0x0010, "L3": 0x0040, "R3": 0x0080}

    def __init__(self, slot):
        self.get_state = load_xinput()
        self.slot = slot

    def read(self):
        state = XInputState()
        if self.get_state(self.slot, ctypes.byref(state)) != 0:
            return None
        pad = state.pad

        # XInput Y is positive up. Teleop and the dashboard use positive down.
        axes = {}
        for label, value in (("LX", pad.lx), ("LY", pad.ly), ("RX", pad.rx), ("RY", pad.ry)):
            axis = value / (32768 if value < 0 else 32767)
            if label in ("LY", "RY"):
                axis = -axis
            axes[label] = axis if abs(axis) >= STICK_DEADZONE else 0.
        for label, value in (("LT", pad.lt), ("RT", pad.rt)):
            axis = value / 255
            axes[label] = axis if axis >= TRIGGER_DEADZONE else 0.
        buttons = {label: bool(pad.buttons & mask) for label, mask in self.BUTTONS.items()}
        dpad = (bool(pad.buttons & 0x0008) - bool(pad.buttons & 0x0004),
                bool(pad.buttons & 0x0001) - bool(pad.buttons & 0x0002))
        return {"name": f"Windows XInput (slot {self.slot})", "axes": axes, "buttons": buttons,
                "dpad": dpad, "axis_count": len(axes), "button_count": len(buttons),
                "hat_count": 1, "deadzone": STICK_DEADZONE}
