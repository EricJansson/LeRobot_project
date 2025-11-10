# teleop_profile.py
from dataclasses import dataclass
from typing import Dict, Optional, Tuple
from .gamepad_mapper import Gamepad, Bindings

def stick_to_speed(x: float, *, deadzone=0.10,
                   vmax_slow=80, vmax_fast=200,
                   fast_on=0.85, fast_off=0.80,
                   snap_fast=True, nonlinear=2.0) -> float:
    s = abs(x)
    if s < deadzone:
        return 0.0
    u = (s - deadzone) / (1.0 - deadzone)
    u = max(0.0, min(1.0, u))
    u_shaped = u ** nonlinear
    if snap_fast and u >= fast_on:
        mag = vmax_fast
    elif snap_fast and u <= fast_off:
        mag = vmax_slow * u_shaped
    else:
        mag = vmax_slow * u_shaped
    return mag if x >= 0 else -mag

@dataclass
class TeleopConfig:
    axis_labels: Dict[int, str]
    button_labels: Dict[int, str]
    hat_labels: Optional[Dict[int, str]] = None
    deadzone: float = 0.10
    axis_change_threshold: float = 0.02
    poll_hz: int = 120
    triggers_are_unit: bool = True
    deadman_button: Optional[str] = "LB"
    invert_y: bool = True  # common for forward=+Y

class RobotTeleop:
    """
    Creates a Gamepad and exposes a simple data state you can read
    (vx, vy, vz, yaw, gripper). Bindings map sticks/triggers/buttons.
    """
    def __init__(self, config: TeleopConfig):
        self.cfg = config
        self.state = {
            "mode": "cartesian",
            "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0,
            "gripper": 0.0,
        }
        self.gp = Gamepad(
            index=0,
            axis_labels=config.axis_labels,
            button_labels=config.button_labels,
            hat_labels=config.hat_labels or {0: "DPad"},
            deadzone=config.deadzone,
            axis_change_threshold=config.axis_change_threshold,
            poll_hz=config.poll_hz,
            triggers_are_unit=config.triggers_are_unit,
            deadman_button=config.deadman_button,
        )
        self._install_bindings()

    # --- Callbacks ---
    def _on_button_down(self, name: str, pressed: bool):
        if name in ("Cross", "A"):
            self.state["gripper"] = 0.0  # open
        elif name in ("Circle", "B"):
            self.state["gripper"] = 1.0  # close
        elif name in ("L3",):
            self.state["mode"] = "cartesian"
        elif name in ("R3",):
            self.state["mode"] = "joints"
        elif name == "Start":
            print("[Teleop] Start pressed")
        elif name == "Back":
            print("[Teleop] Back pressed")

    def _on_axis(self, name: str, value: float):
        if name == "LX":
            self.state["vx"] = stick_to_speed(value, deadzone=self.cfg.deadzone)
        elif name == "LY":
            v = -value if self.cfg.invert_y else value
            self.state["vy"] = stick_to_speed(v, deadzone=self.cfg.deadzone)
        elif name == "RX":
            self.state["yaw"] = stick_to_speed(value, deadzone=self.cfg.deadzone,
                                               vmax_slow=0.8, vmax_fast=2.5,
                                               fast_on=0.85, fast_off=0.80)
        elif name == "LT":  # 0..1
            self.state["vz"] = -value * 200.0
        elif name == "RT":  # 0..1
            self.state["vz"] = value * 200.0

        # Example: publish/log here
        # print(self.state)

    def _on_hat(self, name: str, xy: Tuple[int, int]):
        # Optional: nudge or profile switch; currently just prints
        pass

    def _install_bindings(self):
        b = Bindings(
            buttons_down = {
                "Cross": self._on_button_down,
                "Circle": self._on_button_down,
                "L3": self._on_button_down,
                "R3": self._on_button_down,
                "Start": self._on_button_down,
                "Back": self._on_button_down,
            },
            axes = {
                "LX": self._on_axis,
                "LY": self._on_axis,
                "RX": self._on_axis,
                "LT": self._on_axis,
                "RT": self._on_axis,
            },
            hats = {
                "DPad": self._on_hat,
            }
        )
        self.gp.set_bindings(b, profile="teleop")

    # --- Run loops ---
    def run_forever(self):
        self.gp.switch_profile("teleop")
        self.gp.run()

    def step(self) -> bool:
        # Call this from your own control loop; then read self.state
        return self.gp.step()
