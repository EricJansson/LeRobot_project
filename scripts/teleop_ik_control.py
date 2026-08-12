#!/usr/bin/env python3
"""
Teleop with controller

PS controller teleop for the SO101 arm using Cartesian IK control.
Instead of moving individual joints, the sticks move the end-effector
in XYZ world space and the IK solver determines the joint angles.

Control mapping
---------------
  LB              : deadman (must hold; release stops all movement)
  Left stick LX   : move end-effector in Y  (forward / backward)
  Left stick LY   : move end-effector in X  (left / right)
  Right stick RY  : move end-effector in Z  (up / down)
  Right stick RX  : wrist_roll increment
  LT              : tilt end-effector down  (phi_deg -)
  RT              : tilt end-effector up    (phi_deg +)
  DPad up         : gripper close
  DPad down       : gripper open
  Face buttons    : quick preset poses (see QUICK_POSES below)
  R3 (hold)       : switch to fine-control step sizes
  Ctrl+C          : exit cleanly
"""

from __future__ import annotations

import math
import time
from typing import Callable

# Ensure the project root is on sys.path regardless of how this script is invoked

from robot_program.robot_arm_controller import RobotArmController
from controller.input.gamepad_mapper import Gamepad, Bindings

try:
    from dashboard_panels import read_command, write_json
except ModuleNotFoundError:  # running as `python -m scripts.teleop_ik_control`
    from scripts.dashboard_panels import read_command, write_json

# ---------------------------------------------------------------------------
# Speed / feel knobs
# ---------------------------------------------------------------------------
BASE_XYZ_STEP   = 0.25    # cm per tick at full stick deflection
FINE_XYZ_STEP   = 0.08   # cm per tick while R3 is held

BASE_ROLL_STEP  = 1.5    # degrees per tick
FINE_ROLL_STEP  = 0.4    # degrees per tick while R3 is held

BASE_PHI_STEP   = 1.5    # degrees per tick (LT/RT)
FINE_PHI_STEP   = 0.4    # degrees per tick while R3 is held

BASE_GRIP_STEP  = 2.0    # gripper units per tick
FINE_GRIP_STEP  = 0.7    # gripper units per tick while R3 is held

PHI_MIN_DEG     = -90.0  # end-effector tilt lower bound (degrees)
PHI_MAX_DEG     =  90.0  # end-effector tilt upper bound (degrees)

RATE_HZ         = 25
DEADZONE        = 0.10
AX_DELTA_THRESH = 0.02

# Quick-pose animation tuning. Quick poses are interpolated (synchronized across
# joints) rather than sent as an instant jump. Speed (0..1) scales the absolute
# per-tick motion: total_steps = travel_degrees / (speed * PRESET_MAX_STEP_DEG).
#
#   DEFAULT_PRESET_SPEED : default speed used when a pose has no "speed" key.
#   PRESET_MAX_STEP_DEG  : joint degrees moved per tick at speed=1.0. THIS is the
#                          "full speed" anchor — tune it so 1.0 feels as fast as
#                          the arm should ever go. Lower speeds scale linearly
#                          (0.5 = half, 0.08 = 8% = a genuine crawl), so the
#                          travel distance is what makes low speeds feel slow.
DEFAULT_PRESET_SPEED = 0.2   # default speed when a pose omits "speed"
PRESET_MAX_STEP_DEG = 20.0   # full-speed deg/tick at speed=1.0 (tune this)

# How stale (seconds) a dashboard mute/active status may be before it is ignored.
# The dashboard refreshes this every ~0.25s, so a value <= this means the
# dashboard is alive; older values mean it was closed/crashed -> don't stay muted.
STALE_DASHBOARD_S = 0.8

# How often (seconds) live joint positions are pushed to the dashboard.
TELEMETRY_INTERVAL_S = 0.1

# ---------------------------------------------------------------------------
# Quick preset poses
# ---------------------------------------------------------------------------
# Fast "Act"-style commands bound to the face buttons. Each is a hardcoded set
# of motor degrees (all six joints) that is applied with set_joint_angles when
# the matching button is pressed (like the dashboard's manual Act, but preset).
#
# Units are degrees, matching the calibrated motor limits. A "None" entry keeps
# the joint's current position. Edit these to your desired poses.
#
# Each pose may include a "speed" key (0.0..1.0, default DEFAULT_PRESET_SPEED)
# controlling how fast the arm moves to it (1.0 = max). It is interpolated in
# a synchronized fashion: every joint reaches the goal at the same time.
QUICK_POSES = {
    "Cross": { # Home / neutral pose
        "shoulder_pan": 0.0,
        "shoulder_lift": 0.0,
        "elbow_flex": 0.0,
        "wrist_flex": 0.0,
        "wrist_roll": 0.0,
        "gripper": 0.0,
        # "speed": 0.5,
    },
    "Circle": { # Stand tall with gripper open
        "shoulder_pan": 0.0,
        "shoulder_lift": -40.0,
        "elbow_flex": -10.0,
        "wrist_flex": 35.0,
        "wrist_roll": 0.0,
        "gripper": 30.0,
        # "speed": DEFAULT_PRESET_SPEED,
    },
    "Square": { # Table grab pose
        "shoulder_pan": 0.0,
        "shoulder_lift": -25.0,
        "elbow_flex": 50.0,
        "wrist_flex": 35.0,
        "wrist_roll": 0.0,
        "gripper": 15.0,
        # "speed": 0.1,
    },
    "Triangle": { # Eye contact pose
        "shoulder_pan": -45.0,
        "shoulder_lift": -60.0,
        "elbow_flex": 45.0,
        "wrist_flex": 55.0,
        "wrist_roll": -45.0,
        "gripper": 5.0,
        # "speed": 0.01,
    },
}


# ---------------------------------------------------------------------------
# Status display
# ---------------------------------------------------------------------------

def _print_status(
    mode: str,
    tx: float, ty: float, tz: float,
    phi: float, roll: float, grip: float,
    ik_ok: bool,
    motors: list[float] | None = None,
) -> None:
    ik_str = "OK  " if ik_ok else "FAIL"
    if motors is None:
        motors_str = "n/a"
    else:
        motors_str = "[" + ", ".join(f"{m:6.1f}" for m in motors) + "]"
    print(
        f"\r[{mode:<4}]  "
        f"xyz=({tx:6.2f}, {ty:6.2f}, {tz:6.2f})  "
        f"phi={phi:+6.1f}°  roll={roll:+6.1f}°  grip={grip:4.1f}  "
        f"motors={motors_str}  "
        f"IK:{ik_str}",
        end="", flush=True,
    )


# ---------------------------------------------------------------------------
# Reusable teleop library
# ---------------------------------------------------------------------------
# This module exposes the *primitives* behind the IK teleop loop so that both
# the thin main entry point (main_teleop_ik.py) and future controller modes
# (e.g. a step-by-step plan recorder in teleop_plan_control.py) can reuse the
# same IK / movement logic without duplicating it or depending on the run loop.
#
# Public API
# ----------
#   TeleopState                      Mutable teleop target/spool state
#   PresetAnimation                  Synchronized interpolation toward a pose
#   setup_gamepad(index)             Build a Gamepad (returned along with labels)
#   setup_teleop_bindings(...)       Wire input callbacks -> Bindings
#   apply_joint_preset(robot, state, joints, speed=None)
#                                    Start a synchronized animated move to a pose
#   animate_preset_tick(robot, state)
#                                    Advance one step of an active preset animation
#   apply_controller_input(robot, state, axes, buttons_held, hat, fine) -> bool
#                                    Turn raw controller input into a move_to_xyz
#   handle_dashboard_command(robot, state, cmd_file, telemetry_file) -> dict|None
#   push_telemetry(robot, state, telemetry_file)
#   motor_degrees(robot) -> list[float]
#   run_ik_teleop_loop(...)          The full controller game loop (main entry)
# ---------------------------------------------------------------------------

from dataclasses import dataclass, field


@dataclass
class PresetAnimation:
    """A synchronized interpolation toward a quick-pose target.

    Interpolation is deterministic: the target each tick is computed from a
    captured start position plus a fixed per-joint step delta, NOT from the
    live (lagging) hardware position. Deriving targets from lagging motors each
    tick caused a slow-then-accelerate motion; this avoids that feedback.

    ``start_joints``/``goal_joints`` are parallel maps (only joints the pose
    targets). ``per_step`` holds ``(goal - start) / total_steps`` per joint.
    ``steps_done`` increments each tick; the arm reaches its goal when
    ``steps_done == total_steps``.
    """
    start_joints: dict[str, float] = field(default_factory=dict)
    goal_joints: dict[str, float] = field(default_factory=dict)
    per_step: dict[str, float] = field(default_factory=dict)
    total_steps: int = 0
    steps_done: int = 0


@dataclass
class TeleopState:
    """Mutable targets that drive the arm, shared between teleop and plan modes.

    Holds the desired end-effector pose plus bookkeeping timestamps used by the
    dashboard IPC. Mutated in place by the reusable functions in this module.
    """
    target_x: float = 0.0
    target_y: float = 0.0
    target_z: float = 0.0
    target_phi: float = 0.0
    target_roll: float = 0.0
    target_grip: float = 20.0
    last_pending_ts: int = 0
    last_telemetry_ts: float = 0.0
    last_ik_ok: bool = True
    preset_animation: PresetAnimation | None = None

    @classmethod
    def from_robot(cls, robot: RobotArmController) -> "TeleopState":
        """Seed a new state from the arm's actual current pose."""
        x, y, z, phi = robot.end_effector_pose()
        return cls(
            target_x=x,
            target_y=y,
            target_z=z,
            target_phi=phi,
            target_roll=robot.wrist_roll,
            target_grip=robot.gripper,
        )

    def snap_to_robot(self, robot: RobotArmController) -> None:
        """Re-sync all targets from the arm's current hardware truth."""
        x, y, z, phi = robot.end_effector_pose()
        self.target_x, self.target_y, self.target_z, self.target_phi = x, y, z, phi
        self.target_roll = robot.wrist_roll
        self.target_grip = robot.gripper


# ---------------------------------------------------------------------------
# Controller setup
# ---------------------------------------------------------------------------

# Canonical controller labels, shared by setup_gamepad and callers that need
# to refer to a logical name (e.g. "R3").
AXIS_LABELS   = {0: "LX", 1: "LY", 2: "RX", 3: "RY", 4: "LT", 5: "RT"}
BUTTON_LABELS = {
    0: "Cross", 1: "Circle", 2: "Square", 3: "Triangle",
    4: "LB", 5: "RB", 6: "Back", 7: "Start", 8: "L3", 9: "R3",
}
HAT_LABELS    = {0: "DPad"}


def setup_gamepad(index: int = 0) -> Gamepad:
    """Create the Gamepad instance wired with the canonical label set."""
    return Gamepad(
        index=index,
        axis_labels=AXIS_LABELS,
        button_labels=BUTTON_LABELS,
        hat_labels=HAT_LABELS,
        deadzone=DEADZONE,
        axis_change_threshold=AX_DELTA_THRESH,
        poll_hz=120,
        triggers_are_unit=True,   # LT/RT reported as 0..1
        deadman_button="LB",
    )


def setup_teleop_bindings(
    robot: RobotArmController,
    state: TeleopState,
    buttons_held: set,
    axes: dict[str, float],
    hat: list[int],
) -> Bindings:
    """Wire input callbacks into a Bindings object.

    The callbacks mutate the caller-owned ``buttons_held`` / ``axes`` / ``hat``
    containers in place (passed by reference): ``axes`` must map every logical
    axis name to an initial float, and ``hat``/``buttons_held`` are plain
    mutable inputs. Both this setup and the run loop (or a plan recorder)
    observe the same shared state.

    Quick preset poses fire on the press edge and re-route through
    :func:`apply_joint_preset` so targets stay consistent with the arm.
    """
    def on_button_down(name: str, _pressed: bool) -> None:
        buttons_held.add(name)
        if name in QUICK_POSES:
            apply_joint_preset(robot, state, QUICK_POSES[name])

    def on_button_up(name: str, _pressed: bool) -> None:
        buttons_held.discard(name)

    def on_axis(name: str, value: float) -> None:
        axes[name] = float(value)

    def on_hat(_name: str, xy) -> None:
        hat[0] = int(xy[0])
        hat[1] = int(xy[1])

    return Bindings(
        buttons_down={k: on_button_down for k in BUTTON_LABELS.values()},
        buttons_up  ={k: on_button_up   for k in BUTTON_LABELS.values()},
        axes        ={k: on_axis        for k in axes},
        hats        ={"DPad": on_hat},
    )


# ---------------------------------------------------------------------------
# Reusable movement primitives
# ---------------------------------------------------------------------------

def _read_current_joints(robot: RobotArmController) -> dict[str, float]:
    """Read the six current joint angles (degrees) from the robot's state."""
    s = robot.arm.state
    return {
        "shoulder_pan":  s.base_yaw_deg,
        "shoulder_lift": s.theta1_deg,
        "elbow_flex":    s.theta2_deg,
        "wrist_flex":    s.theta3_deg,
        "wrist_roll":    robot.wrist_roll,
        "gripper":       robot.gripper,
    }


def apply_joint_preset(
    robot: RobotArmController,
    state: TeleopState,
    joints: dict,
    speed: float | None = None,
) -> None:
    """Start an animated, synchronized move to a pose.

    Unlike an instant jump, the arm is interpolated toward ``joints`` over
    several loop ticks so a 0..1 stretch can be applied. All targeted joints
    advance proportionally and arrive at their goals simultaneously (Option B
    synchronized motion), giving natural-looking movement.

    Args:
        robot: The arm controller.
        state: Teleop state; ``preset_animation`` is (re)started here. If one is
            already active it is overwritten with this new goal.
        joints: Dict of joint name -> target degrees. Joints absent or set to
            None keep their current value. May also carry a "speed" key (see
            ``speed``), which is consumed and not treated as a joint.
        speed: Optional 0.0..1.0 override. Defaults to ``joints.get("speed",
            DEFAULT_PRESET_SPEED)``; 1.0 means maximum speed.

    No motor command is sent here — the caller drives :func:`animate_preset_tick`
    from the run loop once per tick.
    """
    if not joints:
        return

    if speed is None:
        speed = float(joints.get("speed", DEFAULT_PRESET_SPEED))
    else:
        speed = float(speed)
    speed = max(0.0, min(1.0, speed))

    # "speed" is a control key, not a joint — drop it from the goal set.
    goal_joints = {k: v for k, v in joints.items() if k != "speed"}

    # Read the actual current positions so the animation starts from truth.
    robot.sync_from_hardware()
    current = _read_current_joints(robot)

    # Only joints that are explicitly targeted (non-None) participate.
    target_joints: dict[str, float] = {}
    for name, goal in goal_joints.items():
        if goal is not None:
            target_joints[name] = goal

    if not target_joints:
        return  # nothing to move toward

    # Absolute-per-tick speed: the joint with the largest travel dictates the
    # total steps so all joints arrive simultaneously, and speed scales the
    # per-tick step linearly (so 0.08 genuinely crawls regardless of pose size).
    max_delta = max(abs(target_joints[name] - current[name]) for name in target_joints)
    max_step = speed * PRESET_MAX_STEP_DEG
    total_steps = max(1, int(math.ceil(max_delta / max_step))) if max_step > 0 else 1

    # Store the start position and a fixed per-joint per-step delta so the
    # animation is deterministic (constant velocity, no feedback from lagging
    # hardware which caused a slow-then-accelerate motion).
    start_joints = {name: current[name] for name in target_joints}
    per_step = {
        name: (target_joints[name] - start_joints[name]) / total_steps
        for name in target_joints
    }

    state.preset_animation = PresetAnimation(
        start_joints=start_joints,
        goal_joints=target_joints,
        per_step=per_step,
        total_steps=total_steps,
    )


def animate_preset_tick(robot: RobotArmController, state: TeleopState) -> None:
    """Advance an active :class:`PresetAnimation` by one interpolation step.

    Nothing happens if no animation is active. Each tick sends the next
    deterministic target (start + per_step * steps_done) so the arm moves at
    constant velocity and all joints reach their goals simultaneously. The
    animation is cleared once the final goal is commanded.
    """
    pa = state.preset_animation
    if pa is None:
        return

    step_joints: dict[str, float] = {}
    for name in pa.goal_joints:
        # Deterministic interpolation from the captured start — independent of
        # the (lagging) live hardware position, so motion stays uniform.
        step_joints[name] = pa.start_joints[name] + pa.per_step[name] * (pa.steps_done + 1)

    robot.set_joint_angles(**step_joints)
    # Read the *planned* pose before sync_from_hardware so targets stay on the
    # destination rather than the (not-yet-moved) hardware position.
    state.snap_to_robot(robot)
    robot.sync_from_hardware()

    pa.steps_done += 1
    if pa.steps_done >= pa.total_steps:
        state.preset_animation = None


def motor_degrees(robot: RobotArmController) -> list[float]:
    """Return the six current joint values, in display order."""
    s = robot.arm.state
    return [
        s.base_yaw_deg,
        s.theta1_deg,
        s.theta2_deg,
        s.theta3_deg,
        robot.wrist_roll,
        robot.gripper,
    ]


def apply_controller_input(
    robot: RobotArmController,
    state: TeleopState,
    axes: dict[str, float],
    buttons_held: set,
    hat: list[int],
    fine: bool = False,
) -> bool:
    """Turn raw controller input into a move_to_xyz command.

    Sticks move the end-effector in XYZ, RX spins wrist roll, LT/RT tilt phi,
    and the D-pad drives the gripper. Updates ``state`` targets in place and,
    when IK fails, snaps them back to the last known good pose. Returns True if
    IK succeeded and the command was sent. This is the core movement primitive
    reused by both the live teleop loop and any plan recorder/player.
    """
    xyz_step  = FINE_XYZ_STEP  if fine else BASE_XYZ_STEP
    roll_step = FINE_ROLL_STEP if fine else BASE_ROLL_STEP
    phi_step  = FINE_PHI_STEP  if fine else BASE_PHI_STEP
    grip_step = FINE_GRIP_STEP if fine else BASE_GRIP_STEP

    state.target_x    += xyz_step  * -axes["LY"]   # invert for natural feel
    state.target_y    += xyz_step  *  axes["LX"]
    state.target_z    += xyz_step  * -axes["RY"]   # invert Y: stick up → arm up
    state.target_roll += roll_step *  axes["RX"]

    # End-effector tilt from triggers
    state.target_phi += phi_step * (axes["RT"] - axes["LT"])

    # Gripper from D-pad: up (y=+1) closes, down (y=-1) opens.
    hat_y = hat[1]
    if hat_y < 0:
        state.target_grip -= grip_step   # up -> close
    elif hat_y > 0:
        state.target_grip += grip_step   # down -> open

    # Clamp phi before passing to IK — no lower-level guard exists for it
    state.target_phi = max(PHI_MIN_DEG, min(PHI_MAX_DEG, state.target_phi))

    ok = robot.move_to_xyz(
        state.target_x, state.target_y, state.target_z,
        phi_deg    = state.target_phi,
        wrist_roll = state.target_roll,
        gripper    = state.target_grip,
    )

    if ok:
        # Sync roll/grip locals back from the robot's clamped values so the
        # local targets don't wind up past the hardware limits (send_state()
        # clamps before every write, but un-synced locals cause input lag on reversal).
        state.target_roll = robot.wrist_roll
        state.target_grip = robot.gripper
    else:
        # IK failed — snap all targets back to last known good pose so we don't
        # drift further into unreachable space.
        state.snap_to_robot(robot)

    return ok


# ---------------------------------------------------------------------------
# Dashboard IPC
# ---------------------------------------------------------------------------

def handle_dashboard_command(
    robot: RobotArmController,
    state: TeleopState,
    cmd_file,
    telemetry_file,
) -> dict | None:
    """Process any manual-motor goal / mute request from the dashboard.

    Teleop owns the serial port, so it is the one that actually drives the arm.
    Joints are applied via set_joint_angles, which clamps each value against the
    calibrated limits as the final safety net.
    Returns the raw command dict (or None) for the caller to inspect.
    """
    # During a quick-pose animation the arm is owned by the interpolator, so the
    # dashboard's manual-Act panel is locked out (only live telemetry is served,
    # which is pushed separately by push_telemetry).
    if state.preset_animation is not None:
        return None

    data = read_command(cmd_file)
    if not data:
        return None

    status = data.get("status")
    now = time.time()

    # Only trust mute/active while the dashboard is alive (fresh writes).
    if status in ("active", "muted"):
        if (now - data.get("ts", 0)) > STALE_DASHBOARD_S:
            return None  # dashboard closed / stale -> keep controller live
        return data  # mute handled by the caller below

    if status == "pending":
        ts = data.get("ts", 0)
        if ts == state.last_pending_ts:
            # Same command as the one we already applied on a previous tick —
            # skip so we don't re-command the arm every frame.
            return read_command(cmd_file)

        joints = data.get("joints") or {}
        # IMPORTANT: This is an INSTANT jump to the requested joint angles.
        # If you later want smooth/interpolated motion instead, replace
        # set_joint_angles with a small stepping loop that animates toward
        # `joints` over time before returning.
        robot.set_joint_angles(**joints)
        state.last_pending_ts = ts

        # Re-sync the teleop's own targets so they don't fight the arm.
        # Read the planned pose BEFORE sync_from_hardware (see apply_joint_preset
        # for why the ordering matters).
        state.snap_to_robot(robot)
        robot.sync_from_hardware()

        # Acknowledge back with the achieved angles so the dashboard can
        # re-populate its fields from hardware truth. This goes to the
        # telemetry file (teleop -> dashboard direction).
        achieved = {
            "shoulder_pan":  robot.state.base_yaw_deg,
            "shoulder_lift": robot.state.theta1_deg,
            "elbow_flex":    robot.state.theta2_deg,
            "wrist_flex":    robot.state.theta3_deg,
            "wrist_roll":    robot.wrist_roll,
            "gripper":       robot.gripper,
        }
        write_json(telemetry_file, {"status": "done", "joints": achieved})
        # Clear the pending state on the command file so the dashboard's own
        # mute-writer resumes (it is suppressed while "pending").
        write_json(cmd_file, {"status": "active"})
        return read_command(cmd_file)  # now the "done" ack

    return data


def push_telemetry(
    robot: RobotArmController,
    state: TeleopState,
    telemetry_file,
) -> None:
    """Stream live joint positions to the dashboard (throttled)."""
    if time.time() - state.last_telemetry_ts < TELEMETRY_INTERVAL_S:
        return
    joints = {
        "shoulder_pan":  robot.state.base_yaw_deg,
        "shoulder_lift": robot.state.theta1_deg,
        "elbow_flex":    robot.state.theta2_deg,
        "wrist_flex":    robot.state.theta3_deg,
        "wrist_roll":    robot.wrist_roll,
        "gripper":       robot.gripper,
    }
    write_json(telemetry_file, {"status": "telemetry", "joints": joints})
    state.last_telemetry_ts = time.time()


# ---------------------------------------------------------------------------
# Main run loop
# ---------------------------------------------------------------------------

def run_ik_teleop_loop(
    robot: RobotArmController,
    state: TeleopState,
    gp: Gamepad,
    buttons_held: set,
    axes: dict[str, float],
    hat: list[int],
    cmd_file,
    telemetry_file,
    dashboard_alive: Callable[[], bool] | None = None,
) -> None:
    """Run the live controller game loop until disconnect or KeyboardInterrupt.

    Caller owns setup: create the Gamepad, wire bindings (setup_teleop_bindings),
    activate the profile, and pass the shared input containers. This loop only
    observes that state — it never re-configures the gamepad.
    """
    dt = 1.0 / RATE_HZ
    try:
        # Grace period before we start treating a missing/stale dashboard as dead.
        # The dashboard is launched in parallel and may take a moment to write its
        # first status, so don't shut down during normal startup.
        while True:
            if not gp.step():
                break  # controller disconnected

            if dashboard_alive is not None and not dashboard_alive():
                print("\nDashboard closed; shutting down arm teleop.")
                break

            # The dashboard always receives live telemetry; its manual-Act
            # commands are locked out during an animation (see handle_dashboard_command).
            push_telemetry(robot, state, telemetry_file)

            # --- Quick-pose animation takes precedence over all other input ---
            if state.preset_animation is not None:
                pa = state.preset_animation
                done_before = pa.steps_done
                animate_preset_tick(robot, state)
                done = state.preset_animation is None
                mode = f"ANIM{done_before + 1}/{pa.total_steps}" if not done else "DONE"
                _print_status(mode, state.target_x, state.target_y, state.target_z,
                              state.target_phi, state.target_roll, state.target_grip,
                              state.last_ik_ok, motors=motor_degrees(robot))
                time.sleep(dt)
                continue

            dashboard_cmd = handle_dashboard_command(robot, state, cmd_file, telemetry_file)

            controller_muted = (
                dashboard_cmd is not None
                and dashboard_cmd.get("status") == "muted"
                and (time.time() - dashboard_cmd.get("ts", 0)) <= STALE_DASHBOARD_S
            )

            if "LB" not in buttons_held or controller_muted:
                _print_status("IDLE", state.target_x, state.target_y, state.target_z,
                              state.target_phi, state.target_roll, state.target_grip,
                              state.last_ik_ok, motors=motor_degrees(robot))
                time.sleep(dt)
                continue

            # Fine-control mode when R3 is held
            fine = "R3" in buttons_held

            ok = apply_controller_input(robot, state, axes, buttons_held, hat, fine=fine)

            state.last_ik_ok = ok
            mode = "FINE" if fine else "MOVE"
            _print_status(mode, state.target_x, state.target_y, state.target_z,
                          state.target_phi, state.target_roll, state.target_grip,
                          state.last_ik_ok, motors=motor_degrees(robot))

            time.sleep(dt)

    except KeyboardInterrupt:
        pass
