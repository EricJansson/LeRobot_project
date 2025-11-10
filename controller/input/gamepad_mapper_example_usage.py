#!/usr/bin/env python3
from gamepad_mapper import Gamepad, Bindings

# ---- Your app logic ----

state = {
    "mode": "cartesian",  # or "joints"
    "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0,
    "gripper": 0.0,  # 0..1
}

def on_button_down(name: str, pressed: bool):
    if name == "Cross":      # open gripper
        state["gripper"] = 0.0
        print("[BTN] Open gripper")
    elif name == "Circle":   # close gripper
        state["gripper"] = 1.0
        print("[BTN] Close gripper")
    elif name == "L3":
        state["mode"] = "cartesian"
        print("[MODE] Cartesian")
    elif name == "R3":
        state["mode"] = "joints"
        print("[MODE] Joints")
    elif name == "Back":
        print("[SYS] Back pressed")
    elif name == "Start":
        print("[SYS] Start pressed")
    else:   
        print(f"[BTN] {name} pressed")

def on_button_up(name: str, pressed: bool):
    # Optional: handle button releases if you need edge-triggered behavior
    pass

def on_axis(name: str, value: float):
    # Map sticks/triggers to robot intents
    if name == "LX":
        state["vx"] = value  # left/right
    elif name == "LY":
        state["vy"] = -value  # invert if forward should be +Y
    elif name == "RX":
        state["yaw"] = value
    elif name == "RY":
        # maybe unused, or use as speed scale
        pass
    elif name == "LT":  # 0..1
        state["vz"] = -value
    elif name == "RT":  # 0..1
        state["vz"] = value

    # Here you'd publish to your robot loop instead of print:
    print(f"[AX] vx={state['vx']:+.2f} vy={state['vy']:+.2f} vz={state['vz']:+.2f} yaw={state['yaw']:+.2f} grp={state['gripper']:.0f}")

def on_hat(name: str, xy):
    x, y = xy
    print(f"[HAT] {name} {xy}")
    # could nudge position, swap profiles, etc.

# ---- Configure mappings for YOUR controller ----
# Reuse the axis/button indices you already validated in your dashboard.
axis_labels = {
    0: "LX",
    1: "LY",
    2: "RX",  # adjust to your pad
    3: "RY",
    4: "LT",
    5: "RT",
}
button_labels = {
    0: "Cross",
    1: "Circle",
    2: "Square",     # MR mirrored to Square on your pad
    3: "Triangle",   # ML mirrored to Triangle on your pad
    4: "LB",
    5: "RB",
    6: "Back",
    7: "Start",
    8: "L3",
    9: "R3",
    # 10 was Guide; omitted intentionally
}

# Optional: name hats (D-pad)
hat_labels = {0: "DPad"}

# Build bindings (you can have multiple profiles)
default_bindings = Bindings(
    buttons_down = {
        "Cross": on_button_down,
        "Circle": on_button_down,
        "L3": on_button_down,
        "R3": on_button_down,
        "Back": on_button_down,
        "Start": on_button_down,
    },
    buttons_up = {
        # Add if you need release edges
    },
    axes = {
        "LX": on_axis,
        "LY": on_axis,
        "RX": on_axis,
        "LT": on_axis,
        "RT": on_axis,
        # "RY": on_axis,  # if you use it
    },
    hats = {
        "DPad": on_hat,
    }
)

# ---- Create and run the gamepad ----
if __name__ == "__main__":
    gp = Gamepad(
        index=0,
        axis_labels=axis_labels,
        button_labels=button_labels,
        hat_labels=hat_labels,
        deadzone=0.10,
        axis_change_threshold=0.02,
        poll_hz=120,
        triggers_are_unit=True,
        deadman_button="LB",  # require LB held to emit actions; set to None to disable
    )
    gp.set_bindings(default_bindings)
    gp.run()
