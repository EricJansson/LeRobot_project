#!/usr/bin/env python3
"""
gamepad_mapper.py
A tiny helper to bind controller inputs to your own functions (pygame backend).

Features:
- Button press/release callbacks
- Axis change callbacks with deadzone & change threshold
- Hat (D-pad) change callbacks
- Optional "deadman" gating (require a button held to allow actions)
- Profiles/modes (different bindings sets you can switch between)
- Index-agnostic: you provide the mapping from SDL indices → logical names

Usage:
    from gamepad_mapper import Gamepad, Bindings

    g = Gamepad(index=0, axis_labels={0:"LX",1:"LY",3:"RX",4:"RY",2:"LT",5:"RT"})
    g.set_bindings(my_bindings)  # see example below
    g.run()  # loop until Ctrl+C or disconnect
"""

from __future__ import annotations
import time
import pygame
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, Tuple, Set, Any

# ---------- Types ----------
AxisValue = float
HatValue = Tuple[int, int]
ButtonIndex = int

ButtonCallback = Callable[[str, bool], None]  # (name, pressed)
AxisCallback = Callable[[str, AxisValue], None]  # (name, value)
HatCallback = Callable[[str, HatValue], None]  # (name, (x,y))

# ---------- Binding containers ----------

@dataclass
class Bindings:
    """
    Logical-name → callback maps. Fill only what you use.
    - buttons_down: called on press (edge)
    - buttons_up: called on release (edge)
    - axes: called when axis value changes past threshold (level)
    - hats: called on change (edge)
    """
    buttons_down: Dict[str, ButtonCallback] = field(default_factory=dict)
    buttons_up: Dict[str, ButtonCallback] = field(default_factory=dict)
    axes: Dict[str, AxisCallback] = field(default_factory=dict)
    hats: Dict[str, HatCallback] = field(default_factory=dict)

# ---------- Core class ----------

class Gamepad:
    def __init__(
        self,
        index: int = 0,
        *,
        axis_labels: Dict[int, str],
        button_labels: Optional[Dict[int, str]] = None,
        hat_labels: Optional[Dict[int, str]] = None,
        deadzone: float = 0.08,
        axis_change_threshold: float = 0.02,
        poll_hz: int = 120,
        triggers_are_unit: bool = True,
        deadman_button: Optional[str] = None,  # e.g. "LB" must be held for actions
    ):
        """
        axis_labels: map SDL axis indices -> logical names you use in callbacks (e.g. {0:"LX",1:"LY",3:"RX",4:"RY",2:"LT",5:"RT"})
        button_labels: map SDL button indices -> logical names (e.g. {0:"Cross",1:"Circle",... 9:"R3"})
        hat_labels: map SDL hat indices -> names (default "HAT0", "HAT1"...)
        deadzone: applied to stick axes (not triggers)
        axis_change_threshold: minimum delta to trigger axis callback after deadzone
        triggers_are_unit: if True, remap trigger raw [-1..+1] to [0..1]
        deadman_button: if set, only fire callbacks while that named button is currently held
        """
        self.index = index
        self.axis_labels = dict(axis_labels)
        self.button_labels = dict(button_labels or {})
        self.hat_labels = dict(hat_labels or {})
        self.deadzone = float(deadzone)
        self.axis_change_threshold = float(axis_change_threshold)
        self.poll_hz = int(poll_hz)
        self.triggers_are_unit = bool(triggers_are_unit)
        self.deadman_name = deadman_button

        # runtime state
        pygame.init(); pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No controllers detected.")

        self.js = pygame.joystick.Joystick(self.index)
        self.js.init()

        self.name = self.js.get_name()
        self.num_axes = self.js.get_numaxes()
        self.num_buttons = self.js.get_numbuttons()
        self.num_hats = self.js.get_numhats()

        self._axis_prev = [0.0]*self.num_axes
        self._button_prev = [0]*self.num_buttons
        self._hat_prev = [(0,0)]*self.num_hats

        self._deadman_pressed = False  # track current deadman state

        # binding profiles
        self._profiles: Dict[str, Bindings] = {}
        self._active_profile: Optional[str] = None

        self._held_buttons: Set[str] = set()

        self._clock = pygame.time.Clock()

    # ----- Profiles -----

    def set_bindings(self, bindings: Bindings, profile: str = "default") -> None:
        self._profiles[profile] = bindings
        if self._active_profile is None:
            self._active_profile = profile

    def switch_profile(self, profile: str) -> None:
        if profile not in self._profiles:
            raise KeyError(f"No such profile: {profile}")
        self._active_profile = profile

    # ----- Helpers -----

    def _axis_is_trigger(self, label: str) -> bool:
        return label.upper() in {"LT", "RT", "L2", "R2"}

    def _maybe_deadman(self) -> bool:
        if not self.deadman_name:
            return True
        return self.deadman_name in self._held_buttons
    
    def _emit_safety_stop(self, profile: "Bindings") -> None:
        """
        Immediately zero all continuous controls regardless of gating.
        Called when the deadman is released.
        """
        # Zero all axes that have callbacks bound
        for ai in range(self.num_axes):
            label = self.axis_labels.get(ai, f"AX{ai}")
            cb = profile.axes.get(label)
            if cb:
                # Triggers & sticks both go to 0.0
                cb(label, 0.0)
            self._axis_prev[ai] = 0.0  # reset history so next change is seen

        # Center all hats that have callbacks bound
        for hi in range(self.num_hats):
            name = self.hat_labels.get(hi, f"HAT{hi}")
            cb = profile.hats.get(name)
            if cb:
                cb(name, (0, 0))
            self._hat_prev[hi] = (0, 0)

    # ----- Public loop APIs -----

    def step(self) -> bool:
        """
        Process one cycle of events. Returns False if the device disconnected or window closed.
        Use this in your own loop if you don't want run().
        """
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                return False
            if e.type == pygame.JOYDEVICEREMOVED and e.instance_id == self.js.get_instance_id():
                return False

        # poll & dispatch
        profile = self._profiles.get(self._active_profile or "", None)
        if profile is None:
            self._clock.tick(self.poll_hz)
            return True

        # Buttons
        for bi in range(self.num_buttons):
            state = self.js.get_button(bi)
            if state != self._button_prev[bi]:
                name = self.button_labels.get(bi, f"B{bi}")
                pressed = bool(state)

                # Track held set
                if pressed:
                    self._held_buttons.add(name)
                else:
                    self._held_buttons.discard(name)

                # DEADMAN transition handling (always process)
                if self.deadman_name and name == self.deadman_name:
                    if pressed and not self._deadman_pressed:
                        self._deadman_pressed = True
                    elif not pressed and self._deadman_pressed:
                        # Transition: pressed -> released  ==> EMERGENCY STOP
                        self._deadman_pressed = False
                        if profile:
                            self._emit_safety_stop(profile)

                # Normal button callbacks (still respect deadman gating).
                # Exception: the deadman button's own release always fires so
                # callers can reliably track its held state.
                if pressed:
                    if self._maybe_deadman():
                        cb = profile.buttons_down.get(name)
                        if cb: cb(name, True)
                else:
                    if self._maybe_deadman() or name == self.deadman_name:
                        cb = profile.buttons_up.get(name)
                        if cb: cb(name, False)

                self._button_prev[bi] = state


        # Axes
        for ai in range(self.num_axes):
            raw = self.js.get_axis(ai)
            label = self.axis_labels.get(ai, f"AX{ai}")
            val = raw
            if self._axis_is_trigger(label) and self.triggers_are_unit:
                # map [-1..+1] -> [0..1]
                val = (raw + 1.0) * 0.5
            else:
                # deadzone for non-triggers
                if abs(val) < self.deadzone:
                    val = 0.0

            if abs(val - self._axis_prev[ai]) >= self.axis_change_threshold:
                if self._maybe_deadman():
                    if abs(val - self._axis_prev[ai]) >= self.axis_change_threshold:
                        cb = profile.axes.get(label)
                        if cb: cb(label, float(val))
                        self._axis_prev[ai] = float(val)
                else:
                    # When deadman not held, converge history to zero
                    self._axis_prev[ai] = 0.0


        # Hats
        for hi in range(self.num_hats):
            v = self.js.get_hat(hi)
            if v != self._hat_prev[hi]:
                name = self.hat_labels.get(hi, f"HAT{hi}")
                if self._maybe_deadman():
                    cb = profile.hats.get(name)
                    if cb: cb(name, v)
                self._hat_prev[hi] = v if self._maybe_deadman() else (0, 0)


        self._clock.tick(self.poll_hz)
        return True

    def run(self) -> None:
        """Simple loop that runs until disconnect or Ctrl+C."""
        print(f"[Gamepad] Opened '{self.name}' axes={self.num_axes} buttons={self.num_buttons} hats={self.num_hats}")
        try:
            while self.step():
                pass
        except KeyboardInterrupt:
            print("\n[Gamepad] Stopped.")
        finally:
            pygame.joystick.quit()
            pygame.quit()
