#!/usr/bin/env python3
"""
Step-by-step plan-based roboarm control (SO101).

TODO: Implement this next phase. The idea:

1. Recording — while the user drives the arm with the controller (reusing the
   IK/movement primitives from teleop_ik_control.py), a button press snapshots
   the current pose onto the plan. Each step stores the end-effector pose
   (x, y, z, phi) plus wrist_roll and gripper, read from the robot.

2. Playback — walk through the recorded steps, calling robot.move_to_xyz() for
   each (optionally interpolating/sleeping between them) to follow the plan.

Reusable building blocks already available in teleop_ik_control:
    TeleopState.from_robot(robot)         -> snapshot a pose into targets
    apply_controller_input(...)           -> drive the arm during recording
    apply_joint_preset(robot, state, j)   -> apply a preset pose dict
    setup_gamepad(...) / setup_teleop_bindings(...)
    run_ik_teleop_loop(...)               -> live (non-plan) teleop mode
"""

# from dataclasses import dataclass, field
# from teleop_ik_control import (
#     TeleopState,
#     setup_gamepad,
#     setup_teleop_bindings,
#     apply_controller_input,
# )
