"""Absolute ArUco input and bounded motion through the existing IK/motor link."""

import json
from math import dist, isfinite
from threading import Lock
from time import perf_counter as monotonic

from robot_program.remote_ik_control import ARM_START_RAMP_S, GamepadSample, MotionState
from robot_program.remote_ik_protocol import finite_vector


CONTROL_HEADER = "X-LeRobot-ArUco-Control"
CONTROL_VERSION = "xyz-v1"
POSE_TIMEOUT_S = .1
HEARTBEAT_TIMEOUT_S = .1
ALIGNMENT_M = .02
MAX_SPEED_M_S = .1


def alignment_readout(workspace, measured_xyz, target_xyz):
    """Display-only guidance: move the mapped marker target toward the measured tool.

    Deltas are robot centimeters, not physical marker travel (the mapping may scale axes).
    Keep bounds diagnostics available even when marker tracking is lost.
    """
    measured = list(measured_xyz) if finite_vector(measured_xyz, 3) else None
    target = list(target_xyz) if finite_vector(target_xyz, 3) else None
    inside = workspace.contains(measured) if workspace is not None and measured is not None else None
    adjustment = [(tool - marker) * 100 for tool, marker in zip(measured, target)] if measured and target else None
    violations = []
    if inside is False:
        for axis, value, low, high in zip("XYZ", measured, workspace.minimum_m, workspace.maximum_m):
            if value < low:
                violations.append(f"{axis} {value * 100:.1f} cm is below minimum {low * 100:.1f} cm")
            elif value > high:
                violations.append(f"{axis} {value * 100:.1f} cm is above maximum {high * 100:.1f} cm")
    warning = ("Arm outside robot workspace: " + "; ".join(violations)
               + ". Marker cannot align here. Reposition the arm using touchscreen control or review the robot bounds."
               if violations else "")
    return {"alignment_cm": dist(target, measured) * 100 if adjustment is not None else None,
            "marker_adjustment_cm": adjustment, "measured_xyz_m": measured,
            "tool_in_workspace": inside, "workspace_m": workspace.limits() if workspace else None,
            "workspace_warning": warning}


class ArucoControl:
    def __init__(self, workspace, *, clock=monotonic):
        self.workspace = workspace
        self.clock = clock
        self.lock = Lock()
        self._active = False
        self.generation = 0
        self.workspace_revision = workspace.snapshot()[1]
        self.negotiated = False
        self.frame = self.units = None
        self.pose_at = self.heartbeat_at = None
        self.xyz = None
        self.enabled = False
        self.action = None
        self.reason = "Connect ArUco control"
        self._status = {}
        self.ramp_started = None
        self.motion = MotionState()

    @property
    def active(self):
        with self.lock:
            return self._active

    def _cancel(self, reason):
        self.generation += 1
        self.enabled = False
        self.action = None
        self.reason = reason

    def cancel(self, reason="Enable control again"):
        with self.lock:
            self._cancel(reason)

    def begin(self, frame, units, negotiated):
        with self.lock:
            self._cancel("Tap Control enabled after aligning the marker")
            self._active = True
            self.frame, self.units, self.negotiated = frame, units, negotiated
            self.pose_at = self.heartbeat_at = self.xyz = None
            self._status = {}
            self.ramp_started = None
            self.motion = MotionState()

    def end(self):
        with self.lock:
            self._cancel("ArUco disconnected. Enable control again")
            self._active = False
            self.pose_at = self.heartbeat_at = self.xyz = None
            self._status = {}

    def publish_pose(self, tracked, xyz):
        with self.lock:
            if not tracked or not all(isfinite(value) and -1 <= value <= 1 for value in xyz):
                # Loss followed by recovery while IK runs must still invalidate that solve.
                self.generation += 1
                self.xyz = self.pose_at = None
            else:
                self.xyz, self.pose_at = tuple(xyz), self.clock()

    def command(self, message):
        try:
            data = json.loads(message)
        except (ValueError, TypeError) as exc:
            raise ValueError("Expected an ArUco control command") from exc
        if (not isinstance(data, dict) or set(data) != {"type", "action"}
                or data["type"] != "aruco-control" or data["action"] not in ("enable", "hold", "heartbeat", "invalidate")):
            raise ValueError("Unknown ArUco control command")
        with self.lock:
            if not self.negotiated:
                raise ValueError("ArUco control was not negotiated")
            now = self.clock()
            if self.heartbeat_at is not None and now - self.heartbeat_at >= HEARTBEAT_TIMEOUT_S:
                self._cancel("Control heartbeat expired. Enable control again")
            self.heartbeat_at = now
            action = data["action"]
            if action != "heartbeat":
                self._cancel("Phone calibration changed. Enable control again" if action == "invalidate" else
                             "Holding" if action == "hold" else "Checking marker alignment")
                self.action = action if action in ("enable", "hold") else None

    def snapshot(self):
        workspace, revision, error = self.workspace.snapshot()
        with self.lock:
            if revision != self.workspace_revision:
                self._cancel("Robot workspace changed. Enable control again")
                self.workspace_revision = revision
            heartbeat = self.heartbeat_at is not None and self.clock() - self.heartbeat_at < HEARTBEAT_TIMEOUT_S
            if not heartbeat and (self.enabled or self.action is not None):
                self._cancel("Control heartbeat expired. Enable control again")
            tracking = self.pose_at is not None and self.clock() - self.pose_at < POSE_TIMEOUT_S
            target = workspace.map_normalized(self.xyz) if workspace and self.xyz is not None else None
            reason = ("Update the phone to support ArUco control" if not self.negotiated else
                      "Enable Normalize XYZ on the phone (robot axes required)" if
                      (self.frame, self.units) != ("robot-axes", "normalized") else
                      error if workspace is None else "Waiting for control heartbeat" if not heartbeat else
                      "Tracking paused; waiting for a fresh in-workspace marker" if not tracking else "")
            return {"workspace": workspace, "revision": revision, "generation": self.generation,
                    "enabled": self.enabled, "action": self.action, "reason": reason,
                    "target_xyz_m": target, "ready": not reason, "pose_at": self.pose_at,
                    "heartbeat": heartbeat, "decision": self.reason}

    def telemetry(self):
        snapshot = self.snapshot()
        with self.lock:
            # Use one current marker target for both the magnitude and axis differences.
            details = alignment_readout(snapshot["workspace"], self._status.get("measured_xyz_m"),
                                        snapshot["target_xyz_m"] if snapshot["ready"] else None)
            return {**self._status, **details, "enabled": self.enabled,
                    "target_xyz_m": snapshot["target_xyz_m"],
                    "reason": details["workspace_warning"] or snapshot["reason"] or self._status.get("reason", self.reason),
                    "can_enable": self._status.get("can_enable", False) and snapshot["ready"] and not self.enabled,
                    "can_hold": self.negotiated and snapshot["heartbeat"] and self._status.get("can_hold", False)}

    def current(self, snapshot, *, require_tracking=True):
        current = self.snapshot()
        return (current["generation"] == snapshot["generation"] and current["revision"] == snapshot["revision"]
                and current["heartbeat"] and self.active
                and (not require_tracking or (current["ready"] and snapshot["pose_at"] is not None
                     and self.clock() - snapshot["pose_at"] < POSE_TIMEOUT_S)))

    def tick(self, loop, link, feedback, *, period, elapsed, mode, started, connected_at, feedback_count,
             last_feedback=None, max_gap_ms=0.):
        """One absolute-input tick; poll/hold keeps the existing Pi watchdog alive."""
        snapshot = self.snapshot()
        measured = feedback.get("measured")
        steps = [rate * period for rate in feedback["rates"]] if finite_vector(feedback.get("rates"), 6) else None
        result = {}
        if finite_vector(measured, 6):
            result = loop.worker.solve(measured, [0.] * 6, sent=feedback.get("sent"), desired_xyz_m=None,
                                       joint_step_limits=steps, correction=False, recovery_step_m=.001)
        measured_xyz = result.get("xyz_m")
        workspace = snapshot["workspace"]
        target = snapshot["target_xyz_m"]
        details = alignment_readout(workspace, measured_xyz, target if snapshot["ready"] else None)
        alignment = dist(target, measured_xyz) if finite_vector(target, 3) and finite_vector(measured_xyz, 3) else None
        available = mode != "Observe only" and not feedback.get("fault") and not feedback.get("recovery_required")
        inside = workspace is not None and workspace.contains(measured_xyz)
        ready = available and inside and snapshot["ready"]
        reason = ("Observe only" if mode == "Observe only" else "Robot fault or joint-limit handoff" if not available else
                  details["workspace_warning"] or snapshot["reason"] or (
                  "Tool position unavailable" if not finite_vector(measured_xyz, 3) else ""))
        if snapshot["enabled"] and (not available or not inside or not feedback["armed"]):
            self.cancel(reason or "Robot disarmed. Enable control again")
            snapshot = self.snapshot()
        op = "hold" if feedback["moving"] else "poll"
        if not snapshot["enabled"] and feedback["armed"]:
            op = "disarm"
        enable = snapshot["action"] == "enable"
        if snapshot["action"] == "hold":
            op = "capture" if available and feedback.get("arm_torque") == "released" else op
        elif enable:
            if ready and alignment is not None and alignment <= ALIGNMENT_M and self.current(snapshot):
                op = "arm" if feedback.get("arm_torque", "holding") == "holding" else "poll"
                if op != "arm":
                    reason = "Tap Holding before enabling control"
            else:
                reason = reason or "Align mapped marker within 2 cm of the measured tool position"
        arm_input = False
        if snapshot["enabled"] and ready and feedback["armed"]:
            issued = result.get("issued_xyz_m")
            if finite_vector(issued, 3):
                offset = [goal - actual for goal, actual in zip(target, issued)]
                distance = dist(target, issued)
                if distance > .00005:
                    if self.ramp_started is None:
                        self.ramp_started = started
                    progress = min(1., max(0., (started - self.ramp_started) / ARM_START_RAMP_S))
                    ramp = .1 + .9 * progress * progress * (3 - 2 * progress)
                    fraction = min(1., MAX_SPEED_M_S * loop.speed * elapsed * ramp / distance)
                    delta = [value * fraction for value in offset] + [0., 0., 0.]
                    result = loop.worker.solve(measured, delta, sent=feedback["sent"], desired_xyz_m=issued,
                                               joint_step_limits=steps, correction=False, recovery_step_m=.001,
                                               workspace_m=workspace.limits())
                    arm_input = True
                    if result.get("target") is not None:
                        if not finite_vector(result.get("accepted_xyz_m"), 3):
                            raise ValueError("Worker arm result requires finite accepted_xyz_m")
                        op = "move"
                    reason = result.get("reason", "")
        else:
            self.ramp_started = None
        # This also expires heartbeats and invalidates workspace changes while IK was running.
        if op in ("move", "arm") and (loop.stop.is_set() or not self.current(snapshot)):
            op = "hold" if feedback["armed"] else "poll"
            self.ramp_started = None
        if op == "capture" and (loop.stop.is_set() or not self.current(snapshot, require_tracking=False)):
            op = "poll"
        if op != "move":
            self.ramp_started = None
        feedback = link.request(op, **({"target": result["target"]} if op == "move" else {}))
        with self.lock:
            if self.generation == snapshot["generation"] and self.action == snapshot["action"]:
                self.action = None
                if enable:
                    self.enabled = op == "arm" and feedback["armed"] and not feedback["fault"]
                    self.reason = "Following marker" if self.enabled else reason or "Robot did not enable control"
        _, diverged = loop._acknowledge(self.motion, feedback, result, op=op, correcting=False,
                                       arm_input=arm_input, started=started)
        if diverged:
            self.cancel("Pi shortened the workspace-validated target. Enable control again")
        if feedback["fault"]:
            self.cancel("Robot fault. Enable control again")
        with self.lock:
            self._status = {**details,
                            "can_enable": bool(ready and alignment is not None and alignment <= ALIGNMENT_M
                                               and feedback.get("arm_torque", "holding") == "holding"),
                            "can_hold": bool(available), "reason": reason or self.reason}
        received = self.clock()
        gap_ms = (received - (started if last_feedback is None else last_feedback)) * 1000
        loop._publish_status(self.motion, GamepadSample(received, None), feedback, result,
                             issuance_diverged=diverged, fresh=True, mode=mode, measured=feedback.get("measured"),
                             feedback_count=feedback_count, received=received, connected_at=connected_at,
                             max_gap_ms=max(max_gap_ms, gap_ms))
        loop.status["reason"] = self.telemetry()["reason"]
        return feedback
