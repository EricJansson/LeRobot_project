# run_teleop.py
from controller.input.teleop_profile import TeleopConfig, RobotTeleop

# Use the indices you already validated in your dashboard:
axis_labels = {0:"LX", 1:"LY", 2:"RX", 3:"RY", 4:"LT", 5:"RT"}
button_labels = {
    0:"Cross", 1:"Circle", 2:"Square", 3:"Triangle",
    4:"LB", 5:"RB", 6:"Back", 7:"Start", 8:"L3", 9:"R3",
    # Guide omitted
}

if __name__ == "__main__":
    cfg = TeleopConfig(axis_labels=axis_labels, button_labels=button_labels,
                       deadman_button="LB", deadzone=0.10)
    teleop = RobotTeleop(cfg)
    # Simple loop that prints the state at ~50 Hz:
    import time
    try:
        while True:
            if not teleop.step():
                break
            # Here you'd publish to your robot; we just print occasionally
            # print(teleop.state)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
