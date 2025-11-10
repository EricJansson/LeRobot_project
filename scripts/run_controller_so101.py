#!/usr/bin/env python3
"""
Controller teleop for SO101 using gamepad_mapper
- LB is the deadman (release triggers immediate STOP)
- Left stick:  shoulder_pan (LX), shoulder_lift (LY)
- Triggers:    elbow_flex   (LT down / RT up)
- Right stick: wrist_roll (RX), wrist_flex (RY)
- Cross/Circle: gripper open/close (0..100)
"""

import time
import argparse
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from robot_program.utils.ports import normalize_port, auto_port

from controller.input.gamepad_mapper import Gamepad, Bindings

# ---- Speed/feel knobs (same spirit as your keyboard script) ----
# Wrists and shoulders
BASE_STEP_DEG     = 1.2     # degrees per tick at full stick
FINE_STEP_DEG     = 0.4     # while R3 is held (acts like "Shift")
# Gripper
BASE_GRIPPER_STEP = 2.0    # % per tick
FINE_GRIPPER_STEP = 0.7    # % per tick while in fine mode (R3)

RATE_HZ           = 25
ALPHA             = 0.35
CHANGE_THRESHOLD  = 0.03
MIN_SEND_INTERVAL = 0.02
DEADZONE          = 0.10
AX_DELTA_THRESH   = 0.02     # mapper axis-change threshold
# ---------------------------------------------------------------

def clamp(v, low, high): return low if v < low else high if v > high else v
def sign(x): return -1.0 if x < 0 else (1.0 if x > 0 else 0.0)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=auto_port(), help="Serial port (COM# or /dev/ttyACM#)")
    ap.add_argument("--index", type=int, default=0, help="Gamepad index")
    args = ap.parse_args()

    port = normalize_port(args.port)
    cfg = SO101FollowerConfig(port=port, id="lerobot_arm", use_degrees=True)
    f = SO101Follower(cfg); f.connect(); f.bus.enable_torque()

    # --- start pose/state ---
    pos = f.bus.sync_read("Present_Position")   # filtered pose we send
    target = dict(pos)                          # instantaneous target from controller
    last_sent = dict(pos)
    last_send_t = 0.0

    # --- map your controller indices to logical names (use your working map) ---
    axis_labels = {0:"LX", 1:"LY", 2:"RX", 3:"RY", 4:"LT", 5:"RT"}
    button_labels = {
        0:"Cross", 1:"Circle", 2:"Square", 3:"Triangle",
        4:"LB", 5:"RB", 6:"Back", 7:"Start", 8:"L3", 9:"R3"
        # Guide omitted
    }
    hat_labels = {0: "DPad"}

    # --- live controller state we maintain via callbacks ---
    axes = {name: 0.0 for name in ("LX","LY","RX","RY","LT","RT")}
    buttons_held = set()
    deadman_name = "LB"

    # --- callbacks for the mapper (no robot calls here; just update state) ---
    def on_button_down(name: str, pressed: bool):
        buttons_held.add(name)

    def on_button_up(name: str, pressed: bool):
        buttons_held.discard(name)

    def on_axis(name: str, value: float):
        axes[name] = float(value)

    def on_hat(name: str, xy):  # unused for joints, but here if you want nudges
        pass

    bindings = Bindings(
        buttons_down = { k: on_button_down for k in button_labels.values() },
        buttons_up   = { k: on_button_up   for k in button_labels.values() },
        axes         = { k: on_axis        for k in axes.keys() },
        hats         = { "DPad": on_hat },
    )

    # Important: we let the mapper enforce deadman and do the emergency stop on release.
    gp = Gamepad(
        index=args.index,
        axis_labels=axis_labels,
        button_labels=button_labels,
        hat_labels=hat_labels,
        deadzone=DEADZONE,
        axis_change_threshold=AX_DELTA_THRESH,
        poll_hz=120,
        triggers_are_unit=True,     # LT/RT in 0..1
        deadman_button=deadman_name # safety gating + instant zero on release (you patched this)
    )
    gp.set_bindings(bindings, profile="so101")
    gp.switch_profile("so101")

    print("\nController teleop (hold LB; Ctrl+C to exit)\n")
    dt = 1.0 / RATE_HZ

    try:
        while True:
            # process controller events
            if not gp.step():
                break

            # Determine step size (R3 acts like “Shift” for fine control)
            step_deg = FINE_STEP_DEG if "R3" in buttons_held else BASE_STEP_DEG
            grip_step = FINE_GRIPPER_STEP if "R3" in buttons_held else BASE_GRIPPER_STEP

            # If deadman not held, do nothing (mapper already zeroed outputs on release)
            if deadman_name not in buttons_held:
                time.sleep(dt)
                continue

            # --- Build instantaneous joint targets from axes ---
            # Sticks are proportional increments; triggers act like up/down on elbow.
            # You can invert any axis by flipping the sign() usage below.
            if "shoulder_pan" in target:
                target["shoulder_pan"]  += step_deg * axes["LX"]
            if "shoulder_lift" in target:
                target["shoulder_lift"] += step_deg * (-axes["LY"])  # invert Y for natural feel
            if "wrist_roll" in target:
                target["wrist_roll"]    += step_deg * axes["RX"]
            if "wrist_flex" in target:
                target["wrist_flex"]    += step_deg * (-axes["RY"])

            if "elbow_flex" in target:
                # LT (0..1) down, RT (0..1) up
                target["elbow_flex"]    += step_deg * (axes["RT"] - axes["LT"])
            
            # Gripper: Cross = close (- or + as you prefer), Circle = open
            if "Cross" in buttons_held:
                target["gripper"]       -= grip_step
            if "Circle" in buttons_held:
                target["gripper"]       += grip_step

            # clamp gripper
            target["gripper"] = clamp(target.get("gripper", 0.0), 0.0, 50.0)

            # --- Low-pass toward target (same smoothing as keyboard script) ---
            for j in pos:
                pos[j] = pos[j] + ALPHA * (target[j] - pos[j])

            # --- rate-limited send on meaningful change ---
            need_send = any(abs(pos[j] - last_sent[j]) >= CHANGE_THRESHOLD for j in pos)
            now = time.time()
            if need_send and (now - last_send_t) >= MIN_SEND_INTERVAL:
                f.send_action({f"{j}.pos": v for j, v in pos.items()})
                last_sent = dict(pos)
                last_send_t = now

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
    finally:
        f.bus.disable_torque(); f.disconnect()
        print("Disconnected and torque disabled.")

if __name__ == "__main__":
    main()
