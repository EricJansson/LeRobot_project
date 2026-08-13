#!/usr/bin/env python3
"""
teleop_plan_control.py

Reusable logic for step-by-step plan-based control of the SO101 arm.

This module contains ONLY reusable logic (data model, controller bindings,
playback engine, dashboard IPC). It has no CLI / main() — the sole entry point
is main_teleop_plan.py.

Plan model
----------
A Plan is an ordered list of PlanActions. Each action is either:

  * MOVE  - an arm configuration (the six motor joint angles in degrees) to be
            reached at a given 0..1 ``speed``, interpolated smoothly.
  * PAUSE - a wait of ``duration_s`` seconds before continuing to the next
            action.

This lets the sequence express things like MOVE -> PAUSE -> MOVE, with pauses
inserted / removed independently from the movement steps via the dashboard.

Controller controls
-------------------
  LB                 : deadman for manual driving (enforced in the run loop; the
                       Gamepad-level deadman is disabled in plan mode so Back and
                       RB + Cross always fire even when LB is not held)
  RB + Cross         : snapshot the ACTUAL hardware motor angles and append a
                       MOVE action (recording)
  LB + Start         : begin playback (-> PLAY), single pass
  RB + Start         : begin looping playback (-> PLAY, restarts when finished)
  Back               : cancel playback immediately, keep the arm where it is,
                       resync teleop state, return to IDLE
  LB + stick/trigger : during PLAY, manual takeover - cancels playback and
                       hands control back to the user

Reused from teleop_ik_control:
    TeleopState, apply_controller_input, apply_joint_preset, animate_preset_tick,
    handle_dashboard_command, setup_gamepad, QUICK_POSES, AXIS_LABELS, BUTTON_LABELS
"""

from __future__ import annotations

import json
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable

from robot_program.robot_arm_controller import RobotArmController
from controller.input.gamepad_mapper import Gamepad, Bindings

try:
    from teleop_ik_control import (
        AXIS_LABELS,
        BUTTON_LABELS,
        QUICK_POSES,
        TELEMETRY_INTERVAL_S,
        TeleopState,
        apply_controller_input,
        apply_joint_preset,
        animate_preset_tick,
        handle_dashboard_command,
        setup_gamepad,
    )
    from dashboard_panels import read_command, write_json
except ModuleNotFoundError:  # running as `python -m scripts.teleop_plan_control`
    from scripts.teleop_ik_control import (
        AXIS_LABELS,
        BUTTON_LABELS,
        QUICK_POSES,
        TELEMETRY_INTERVAL_S,
        TeleopState,
        apply_controller_input,
        apply_joint_preset,
        animate_preset_tick,
        handle_dashboard_command,
        setup_gamepad,
    )
    from scripts.dashboard_panels import read_command, write_json


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

# Speed is strictly (0, 1]. 0.0 is invalid (it would divide-by-zero / stall in
# the interpolator), so any value below this floor is clamped up to it.
SPEED_MIN = 0.01
SPEED_MAX = 1.0
DEFAULT_SPEED = 0.3
DEFAULT_PAUSE_S = 1.0

# Thresholds for "manual input active" (manual takeover). These mirror the
# inputs apply_controller_input acts on: sticks + triggers + D-pad.
MANUAL_AXIS_NAMES = ("LX", "LY", "RX", "RY", "LT", "RT")
MANUAL_AXIS_THRESH = 0.08

# Stale threshold for mute/active status (dashboard alive check), mirroring
# teleop_ik_control.STALE_DASHBOARD_S.
STALE_DASHBOARD_S = 0.8


# ---------------------------------------------------------------------------
# Plan data model
# ---------------------------------------------------------------------------

class ActionType(Enum):
    MOVE = "move"
    PAUSE = "pause"


@dataclass
class PlanAction:
    """One action in a Plan: either a MOVE or a PAUSE.

    Field order: identity/type first, then MOVE-specific fields, then
    PAUSE-specific fields.
    """
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    type: ActionType = ActionType.MOVE

    # --- MOVE fields ---
    joints: dict[str, float] = field(default_factory=dict)
    speed: float = DEFAULT_SPEED

    # --- PAUSE fields ---
    duration_s: float = DEFAULT_PAUSE_S

    # ---------------------------------------------------------- predicates
    @property
    def is_move(self) -> bool:
        return self.type is ActionType.MOVE

    @property
    def is_pause(self) -> bool:
        return self.type is ActionType.PAUSE

    # ---------------------------------------------------------- serialization
    def to_dict(self) -> dict:
        d: dict = {"id": self.id, "type": self.type.value}
        if self.is_move:
            d["joints"] = {k: round(v, 1) for k, v in self.joints.items()}
            d["speed"] = round(self.speed, 3)
        else:
            d["duration_s"] = round(self.duration_s, 2)
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "PlanAction":
        typ = ActionType(d.get("type", "move"))
        act = cls(id=str(d.get("id", uuid.uuid4().hex[:8])), type=typ)
        if typ is ActionType.MOVE:
            act.joints = {
                str(k): float(v) for k, v in (d.get("joints") or {}).items()
            }
            act.speed = clamp_speed(float(d.get("speed", DEFAULT_SPEED)))
        else:
            act.duration_s = max(0.0, float(d.get("duration_s", DEFAULT_PAUSE_S)))
        return act


@dataclass
class Plan:
    """An ordered list of PlanAction, plus playback bookkeeping."""
    actions: list[PlanAction] = field(default_factory=list)
    play_index: int = -1          # -1 = not playing; else index of active action

    def to_dict(self) -> dict:
        return {"actions": [a.to_dict() for a in self.actions]}

    @classmethod
    def from_dict(cls, d: dict) -> "Plan":
        actions = [PlanAction.from_dict(a) for a in d.get("actions", [])]
        return cls(actions=actions)

    @property
    def is_playing(self) -> bool:
        return self.play_index >= 0


class Mode(Enum):
    IDLE = "IDLE"
    PLAY = "PLAY"


# ---------------------------------------------------------------------------
# Speed helpers
# ---------------------------------------------------------------------------

def clamp_speed(value: float) -> float:
    """Clamp a speed into the valid (0, 1] range, with a safe positive floor."""
    return max(SPEED_MIN, min(SPEED_MAX, float(value)))


# ---------------------------------------------------------------------------
# Serialization helpers
# ---------------------------------------------------------------------------

def plan_file_default() -> Path:
    """Default JSON file for saving/loading a plan (project-local)."""
    return Path(__file__).resolve().parent.parent / ".runtime" / "teleop_plan.json"


def save_plan(plan: Plan, path) -> bool:
    try:
        with open(Path(path), "w") as f:
            json.dump(plan.to_dict(), f, indent=2)
        return True
    except OSError:
        return False


def load_plan(path) -> Plan | None:
    try:
        with open(Path(path), "r") as f:
            return Plan.from_dict(json.load(f))
    except (OSError, ValueError, KeyError):
        return None


# ---------------------------------------------------------------------------
# Recording
# ---------------------------------------------------------------------------

def _read_current_joints(robot: RobotArmController) -> dict:
    """Read the six current joint angles (degrees) from the robot's state.

    NOTE: call robot.sync_from_hardware() first so these reflect the ACTUAL
    servo positions rather than the last commanded IK target.
    """
    s = robot.arm.state
    return {
        "shoulder_pan":  s.base_yaw_deg,
        "shoulder_lift": s.theta1_deg,
        "elbow_flex":    s.theta2_deg,
        "wrist_flex":    s.theta3_deg,
        "wrist_roll":    robot.wrist_roll,
        "gripper":       robot.gripper,
    }


def snapshot_plan_action(robot: RobotArmController, speed: float = DEFAULT_SPEED) -> PlanAction:
    """Record the arm's ACTUAL hardware motor angles as a MOVE action.

    Calls robot.sync_from_hardware() so the snapshot reflects physical servo
    position, not merely cached/commanded values.
    """
    robot.sync_from_hardware()
    return PlanAction(
        type=ActionType.MOVE,
        joints=_read_current_joints(robot),
        speed=clamp_speed(speed),
    )


# ---------------------------------------------------------------------------
# Controller bindings
# ---------------------------------------------------------------------------

def setup_plan_bindings(
    robot: RobotArmController,
    state: TeleopState,
    buttons_held: set,
    axes: dict[str, float],
    hat: list[int],
    mode_ref: dict,
    on_record: Callable[[PlanAction], None] | None = None,
    on_play_start: Callable[[], None] | None = None,
    on_loop_start: Callable[[], None] | None = None,
    on_stop: Callable[[], None] | None = None,
) -> Bindings:
    """Wire controller callbacks for plan mode (mode-aware).

    Callbacks mutate the caller-owned ``buttons_held`` / ``axes`` / ``hat``
    containers in place. ``mode_ref`` is a mutable 1-entry dict {"mode": Mode}
    that the caller (run loop) keeps up to date; quick poses and recording are
    suppressed while it is Mode.PLAY so controller actions never interfere with
    playback.

    Recorded steps are passed to ``on_record(PlanAction)``; single-pass playback
    starts via ``on_play_start``, looping playback via ``on_loop_start``, and
    either is stopped via ``on_stop``.
    """
    def on_button_down(name: str, _pressed: bool) -> None:
        buttons_held.add(name)

        if mode_ref["mode"] is Mode.PLAY:
            # During PLAY only Back does anything; quick poses / recording are
            # suppressed so controller actions never interfere with playback.
            if name == "Back" and on_stop is not None:
                on_stop()
            return

        # --- IDLE actions ---

        # RB + Cross: record the current pose. Checked BEFORE the quick-pose
        # branch because Cross is also a quick-pose button.
        if name == "Cross" and "RB" in buttons_held:
            if on_record is not None:
                on_record(snapshot_plan_action(robot))
            return

        # LB + Start: begin playback (single pass).
        if name == "Start" and "LB" in buttons_held:
            if on_play_start is not None:
                on_play_start()
            return

        # RB + Start: begin looping playback (repeats until stopped).
        if name == "Start" and "RB" in buttons_held:
            if on_loop_start is not None:
                on_loop_start()
            return

        # Quick preset poses (face buttons) fire on their own press edge.
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
        buttons_up={k: on_button_up for k in BUTTON_LABELS.values()},
        axes={k: on_axis for k in axes},
        hats={"DPad": on_hat},
    )


# ---------------------------------------------------------------------------
# Playback / cancellation
# ---------------------------------------------------------------------------

def _cancel_playback(robot: RobotArmController, state: TeleopState,
                     plan: Plan, pause_started: dict) -> None:
    """Immediately cancel playback.

    Kills any mid-flight animation, reads the arm's actual hardware position,
    resyncs the teleop targets to that truth (so the user can take over from
    where the arm actually is), and resets playback bookkeeping.
    """
    state.preset_animation = None       # stop the interpolator mid-flight
    robot.sync_from_hardware()          # read real servo positions
    state.snap_to_robot(robot)          # resync teleop targets to hardware truth
    plan.play_index = -1
    pause_started.clear()


def _cancel_to_idle(robot: RobotArmController, state: TeleopState,
                    plan: Plan, pause_started: dict, mode_ref: dict) -> Mode:
    """Cancel playback and return to IDLE, keeping ``mode``/``mode_ref`` in sync.

    Returns the new mode (Mode.IDLE) so the caller's local ``mode`` can be
    assigned from the return value — guaranteeing the two can never disagree.
    """
    _cancel_playback(robot, state, plan, pause_started)
    mode_ref["mode"] = Mode.IDLE
    return Mode.IDLE


def _pending_manual_cmd(cmd_file) -> bool:
    """True if the dashboard has a manual Act command waiting to be applied.

    Used during PLAY to detect that the user issued a manual Act: that is
    treated as a manual takeover (cancel playback, resync) and the still-pending
    command is then applied by the IDLE branch on the same tick.
    """
    data = read_command(cmd_file)
    return bool(data and data.get("status") == "pending")


def _is_manual_input_active(buttons_held: set, axes: dict[str, float], hat: list[int]) -> bool:
    """Detect intentional manual movement input (used for manual takeover).

    Mirrors apply_controller_input: LB held AND any of the sticks / triggers /
    D-pad pushed beyond the deadzone. Returns True only when the user is
    actively trying to move the arm.
    """
    if "LB" not in buttons_held:
        return False
    if any(abs(axes.get(a, 0.0)) > MANUAL_AXIS_THRESH for a in MANUAL_AXIS_NAMES):
        return True
    if hat[0] != 0 or hat[1] != 0:
        return True
    return False


def _tick_playback(robot: RobotArmController, state: TeleopState,
                   plan: Plan, pause_started: dict) -> bool:
    """Advance playback by one loop tick. Returns True while still running.

    ``pause_started`` is a persistent dict owned by the caller; while a PAUSE
    action is active it holds {"t": float, "duration": float}.
    """
    # Continuing / finishing a PAUSE action.
    if pause_started:
        if time.time() - pause_started["t"] >= pause_started["duration"]:
            plan.play_index += 1
            pause_started.clear()
        return True

    # Move to the next action, or finish.
    if plan.play_index < 0:
        if not plan.actions:
            return False  # nothing to play
        plan.play_index = 0
    elif plan.play_index >= len(plan.actions):
        plan.play_index = -1
        return False

    action = plan.actions[plan.play_index]

    # PAUSE: begin a timed wait (do not touch the arm).
    if action.is_pause:
        pause_started["t"] = time.time()
        pause_started["duration"] = action.duration_s
        return True

    # MOVE: drive the arm toward this configuration.
    if state.preset_animation is None:
        apply_joint_preset(robot, state, action.joints, speed=action.speed)
        return True

    animate_preset_tick(robot, state)
    if state.preset_animation is None:
        # Animation completed -> advance to the next action.
        plan.play_index += 1
    return True


def stop_playback(plan: Plan) -> None:
    """Return to idle (the arm is not moved here; see _cancel_playback)."""
    plan.play_index = -1


# ---------------------------------------------------------------------------
# Dashboard plan IPC (dedup + ack)
# ---------------------------------------------------------------------------

def handle_plan_dashboard_cmd(
    plan_cmd_file,
    plan: Plan,
    plan_path,
    last_cmd_id: int = 0,
) -> tuple[int, bool]:
    """Process at most one dashboard plan command. Returns (last_cmd_id, changed).

    The dashboard stamps every plan command with a monotonically increasing
    ``cmd_id``; this function ignores anything <= the ``last_cmd_id`` it has
    already processed, guaranteeing each command is applied exactly once. It
    writes an ACK (``status: "plan_cmd_ack"``, same ``cmd_id``) back to the
    plan-command file.

    ``changed`` is True when the command structurally alters the plan (which
    must stop playback), and False for live-only edits (speed) that may be
    applied during playback.

    Returns (new_last_cmd_id, changed).
    """
    data = read_command(plan_cmd_file)
    if not data or data.get("status") != "plan_cmd":
        return last_cmd_id, False

    cmd_id = int(data.get("cmd_id", 0) or 0)
    if cmd_id and cmd_id <= last_cmd_id:
        # Already processed; nothing new.
        return last_cmd_id, False

    action = data.get("action")
    changed = True  # most actions structurally alter the plan

    if action == "save":
        path = data.get("file") or str(plan_path)
        save_plan(plan, path)
        changed = False  # saving does not alter the in-memory plan
    elif action == "load":
        path = data.get("file") or str(plan_path)
        loaded = load_plan(path)
        if loaded is not None:
            plan.actions = loaded.actions
        plan.play_index = -1
    elif action == "clear":
        plan.actions.clear()
        plan.play_index = -1
    elif action == "delete_action":
        i = data.get("index", -1)
        if 0 <= i < len(plan.actions):
            plan.actions.pop(i)
            plan.play_index = -1
    elif action == "add_pause":
        i = data.get("index", -1)          # insert AFTER this index
        pause_s = max(0.0, float(data.get("duration_s", DEFAULT_PAUSE_S)))
        new_pause = PlanAction(type=ActionType.PAUSE, duration_s=pause_s)
        pos = min(len(plan.actions), i + 1) if i >= 0 else len(plan.actions)
        plan.actions.insert(pos, new_pause)
        plan.play_index = -1
    elif action == "set_action":
        i = data.get("index", -1)
        if 0 <= i < len(plan.actions):
            a = plan.actions[i]
            fields = data.get("fields") or {}
            if "speed" in fields and a.is_move:
                a.speed = clamp_speed(fields["speed"])
                changed = False  # live speed edit allowed during playback
            elif "duration_s" in fields and a.is_pause:
                a.duration_s = max(0.0, float(fields["duration_s"]))
                # Pause-duration edit is structural -> stops playback.
                plan.play_index = -1
            else:
                changed = False  # nothing applicable
        else:
            changed = False
    else:
        changed = False  # unknown action, do not treat as structural

    # Acknowledge exactly-once processing back to the dashboard.
    write_json(plan_cmd_file, {"status": "plan_cmd_ack", "cmd_id": cmd_id})

    return cmd_id, changed


# ---------------------------------------------------------------------------
# Telemetry
# ---------------------------------------------------------------------------

def push_plan_telemetry(
    robot: RobotArmController,
    state: TeleopState,
    plan: Plan,
    mode: Mode,
    telemetry_file,
) -> None:
    """Stream joint positions plus the live plan/playback state to the dashboard.

    Throttled like teleop_ik_control.push_telemetry.
    """
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
    write_json(telemetry_file, {
        "status": "telemetry",
        "joints": joints,
        "mode": mode.value,
        "play_index": plan.play_index,
        "play_total": len(plan.actions),
        "plan_actions": [a.to_dict() for a in plan.actions],
    })
    state.last_telemetry_ts = time.time()


# ---------------------------------------------------------------------------
# Main run loop
# ---------------------------------------------------------------------------

def run_plan_teleop_loop(
    robot: RobotArmController,
    state: TeleopState,
    gp: Gamepad,
    buttons_held: set,
    axes: dict[str, float],
    hat: list[int],
    cmd_file,
    telemetry_file,
    plan_cmd_file,
    plan: Plan,
    plan_path,
    dashboard_alive: Callable[[], bool] | None = None,
    rate_hz: float = 25.0,
) -> None:
    """Run the plan controller game loop until disconnect or KeyboardInterrupt.

    The caller owns hardware/controller setup (see main_teleop_plan.py). This
    loop handles playback, recording, manual takeover, and dashboard IPC.
    """
    dt = 1.0 / rate_hz
    mode = Mode.IDLE
    mode_ref: dict = {"mode": mode}   # shared with the bindings (mode-aware)
    pause_started: dict = {}          # {"t": float, "duration": float} while paused
    loop_playback = False             # True -> restart playback when the plan finishes
    last_plan_cmd_id = 0

    def on_record(action: PlanAction) -> None:
        plan.actions.append(action)

    def on_play_start() -> None:
        nonlocal mode, loop_playback
        loop_playback = False
        plan.play_index = -1
        pause_started.clear()
        state.preset_animation = None  # playback must start from a clean animation state
        mode = Mode.PLAY
        mode_ref["mode"] = mode

    def on_loop_start() -> None:
        nonlocal mode, loop_playback
        loop_playback = True
        plan.play_index = -1
        pause_started.clear()
        state.preset_animation = None
        mode = Mode.PLAY
        mode_ref["mode"] = mode

    def on_stop() -> None:
        nonlocal mode, loop_playback
        loop_playback = False
        _cancel_playback(robot, state, plan, pause_started)
        mode = Mode.IDLE
        mode_ref["mode"] = mode

    bindings = setup_plan_bindings(
        robot, state, buttons_held, axes, hat, mode_ref,
        on_record=on_record,
        on_play_start=on_play_start,
        on_loop_start=on_loop_start,
        on_stop=on_stop,
    )
    gp.set_bindings(bindings, profile="plan")
    gp.switch_profile("plan")

    print("Plan teleop ready.")
    print("  Drive:  hold LB, use sticks/triggers (R3 = fine).")
    print("  Record: RB + Cross adds the current pose to the plan.")
    print("  Play:   LB + Start plays the plan once; RB + Start loops it.")
    print("          Back or LB+stick stops it.")
    print("  Edit pauses/speed and save/load via the dashboard.\n")

    try:
        while True:
            if not gp.step():
                break  # controller disconnected

            if dashboard_alive is not None and not dashboard_alive():
                print("\nDashboard closed; shutting down arm plan teleop.")
                break

            # Plan edits with dedup/ack (processed every loop regardless of mode).
            # The manual Act / mute / pending commands are handled in the IDLE
            # branch below so they are never applied while playback owns the arm.
            last_plan_cmd_id, plan_changed = handle_plan_dashboard_cmd(
                plan_cmd_file, plan, plan_path, last_plan_cmd_id,
            )

            # A structural plan edit must cancel an in-flight playback before
            # it can take effect.
            if plan_changed and mode is Mode.PLAY:
                mode = _cancel_to_idle(robot, state, plan, pause_started, mode_ref)

            # A manual Act command issued during PLAY counts as a manual
            # takeover: cancel playback, resync, enter IDLE, and let the IDLE
            # branch below apply the still-pending command this same tick.
            if mode is Mode.PLAY and _pending_manual_cmd(cmd_file):
                mode = _cancel_to_idle(robot, state, plan, pause_started, mode_ref)

            push_plan_telemetry(robot, state, plan, mode, telemetry_file)

            # Quick-pose animation (IDLE only; never during PLAY).
            if state.preset_animation is not None and mode is not Mode.PLAY:
                animate_preset_tick(robot, state)
                time.sleep(dt)
                continue

            if mode is Mode.PLAY:
                # Manual takeover: LB + sticks/triggers/D-pad cancels playback.
                if _is_manual_input_active(buttons_held, axes, hat):
                    mode = _cancel_to_idle(robot, state, plan, pause_started, mode_ref)
                    time.sleep(dt)
                    continue

                running = _tick_playback(robot, state, plan, pause_started)
                if not running:
                    if loop_playback and plan.actions:
                        # Restart from the top without resyncing hardware (the
                        # arm is already wherever the last MOVE left it).
                        plan.play_index = -1
                        state.preset_animation = None
                        continue
                    mode = _cancel_to_idle(robot, state, plan, pause_started, mode_ref)
                time.sleep(dt)
                continue

            # --- IDLE: drive the arm (recording-capable) ---
            dashboard_cmd = handle_dashboard_command(robot, state, cmd_file, telemetry_file)
            controller_muted = (
                dashboard_cmd is not None
                and dashboard_cmd.get("status") == "muted"
                and (time.time() - dashboard_cmd.get("ts", 0)) <= STALE_DASHBOARD_S
            )

            if "LB" not in buttons_held or controller_muted:
                time.sleep(dt)
                continue

            fine = "R3" in buttons_held
            apply_controller_input(robot, state, axes, buttons_held, hat, fine=fine)
            time.sleep(dt)

    except KeyboardInterrupt:
        pass
