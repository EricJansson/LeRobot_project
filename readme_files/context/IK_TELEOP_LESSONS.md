# LeRobot IK teleop hardware lessons

## Extended-arm retraction

The owner reported that the arm stalled near full extension and sometimes resumed after a physical assist. The legacy Cartesian controller could retract from the same kind of pose. In LeRobot mode, the console reported `IK rejected: joint step exceeds 5 degrees` during some stalls. That check came from the project controller. It rejected an entire solver candidate if any joint was more than 5° from its measured position.

On hardware, retraction worked with `--no-joint-step-limit`. A later version that sent coordinated partial commands capped at 5° also stalled, even though it passed offline motion tests. The normal launcher now sends the full validated IK solution, matching the command path the owner tested with the flag. The launcher still accepts the old flag for compatibility. [Controller](../../robot_program/lerobot_ik_controller.py) · [Launcher](../../scripts/main_teleop_lerobot_ik.py)

## What the offline replay established

- With fixed simulated joint feedback at an extended pose, the original check sent two of eight retraction commands. The bypass sent all eight and reached a 13.1° command lead. This reproduced command suppression in software. Physical retraction under load required the owner's trial.
- In a separate simulation where joints tracked at different speeds, the uncapped path accumulated about 20° of tool rotation during translation. Treat that as a simulation result when investigating orientation drift.

## Diagnosing similar stalls

- Separate IK validation, command transmission, and measured movement. The controller checks absolute URDF and calibration joint limits and requires the modeled gripper position to be within 0.5 mm of the requested XYZ. This constrains commands to the modeled reach. Collision clearance is a separate concern.
- Compare one change at a time from the same starting pose with the same stick direction, deflection, and duration. Look for measured retraction without assistance. When available, compare the requested pose, IK result, joint command, and measured joints. Console rejection messages are rate-limited.
- Treat Cartesian increments and loop rate as coupled. At 2 mm per tick and 50 Hz, full single-axis input requests 100 mm/s if the loop keeps up. Raising the rate at a fixed increment raises the requested speed.
- Use offline replay to verify which commands the controller sends. Verify a new limiter or motion policy at the known failing pose on hardware before making it the normal path.
