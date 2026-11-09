"""Pi-local recovery state and joint validation, without motor I/O or IK."""

from dataclasses import dataclass
from enum import Enum, auto
import math

from robot_program.remote_ik_protocol import (
    RECOVERY_TARGET_MARGIN, RECOVERY_MEASURED_MARGIN, RECOVERY_SETTLE_S,
    recovery_step_limits, within_limits,
)

PROGRESS_DEG = .1
INPUT_CHANGE_THRESHOLD = .1
NO_PROGRESS_TIMEOUT_S = 1.
JOINT_EPS_DEG = 1e-8


class RecoveryAction(Enum):
    ADVANCE = auto()
    WAIT = auto()
    COMPLETE = auto()
    STALLED = auto()


class CoordinationResult(Enum):
    ACCEPT = auto()
    AUTOMATIC = auto()
    REJECT = auto()


@dataclass
class RecoveryState:
    goal: list | None = None
    anchor: list | None = None
    blocked: bool = False
    input: list | None = None
    best: float = math.inf
    remaining: float | None = None
    progress_at: float = 0.
    margin: float = RECOVERY_TARGET_MARGIN
    coordinated: bool = False
    goal_issued_at: float | None = None
    inside_since: float | None = None

    @property
    def active_axes(self):
        return [a != g for a, g in zip(self.anchor, self.goal)]

    def clear(self, *, completed=False):
        self.goal = self.anchor = None
        self.blocked = False
        self.remaining = 0. if completed else None
        self.input = None
        self.best = math.inf
        self.goal_issued_at = None
        self.margin = RECOVERY_TARGET_MARGIN
        self.coordinated = False
        self.inside_since = None

    def begin(self, measured, limits, command_limits, *, boundary_sides=None):
        self.clear()
        self.anchor = measured.copy()
        sides = boundary_sides or [0] * len(measured)
        self.goal = []
        for q, (cmd_lo, cmd_hi), (lo, hi), side in zip(measured, command_limits, limits, sides):
            inward_low = max(cmd_lo, min((lo + hi) / 2, lo + self.margin))
            inward_high = min(cmd_hi, max((lo + hi) / 2, hi - self.margin))
            if q < lo or (side == -1 and q < inward_low):
                self.goal.append(inward_low)
            elif q > hi or (side == 1 and q > inward_high):
                self.goal.append(inward_high)
            else:
                self.goal.append(q)

    def restart_trial(self, now):
        self.best = math.inf
        self.progress_at = now
        self.goal_issued_at = None
        self.inside_since = None

    def include_new_violations(self, measured, sent, limits):
        """Return a stop anchor only for newly offending joints."""
        stop_anchor = sent.copy()
        changed = False
        for i, (q, bounds) in enumerate(zip(measured, limits)):
            if self.anchor[i] == self.goal[i] and not bounds[0] <= q <= bounds[1]:
                self.anchor[i] = stop_anchor[i] = q
                midpoint = sum(bounds) / 2
                self.goal[i] = (max(midpoint, bounds[1] - self.margin) if q > bounds[1]
                                else min(midpoint, bounds[0] + self.margin))
                changed = True
        return stop_anchor if changed else None

    def next_target(self, sent, rates, period):
        if self.goal is None or sent is None:
            return None
        return [s + max(-step, min(step, goal - s)) if anchor != goal else s
                for s, goal, anchor, step in zip(sent, self.goal, self.anchor, recovery_step_limits(rates, period))]

    def evaluate(self, measured, sent, intent, limits, now):
        """Advance episode state, leaving all stop/write operations to the owner."""
        active = self.active_axes
        remaining = max((abs(q - g) for q, g, moving in zip(measured, self.goal, active) if moving), default=0.)
        self.remaining = remaining
        progressed = remaining <= self.best - PROGRESS_DEG + JOINT_EPS_DEG
        changed_input = (self.input is None
                         or any(abs(a - b) >= INPUT_CHANGE_THRESHOLD for a, b in zip(intent, self.input)))
        if changed_input:
            self.input = intent.copy()
        if progressed:
            self.best, self.progress_at = remaining, now
        goal_issued = all(abs(s - g) < JOINT_EPS_DEG for s, g, moving in zip(sent, self.goal, active) if moving)
        recovered = all(lo + min(RECOVERY_MEASURED_MARGIN, (hi - lo) / 2) <= q
                        <= hi - min(RECOVERY_MEASURED_MARGIN, (hi - lo) / 2)
                        for q, (lo, hi), moving in zip(measured, limits, active) if moving)
        if goal_issued and recovered and within_limits(measured, limits):
            if self.inside_since is None:
                self.inside_since = now
            if now - self.inside_since >= RECOVERY_SETTLE_S:
                return RecoveryAction.COMPLETE
            return RecoveryAction.ADVANCE
        self.inside_since = None
        if self.blocked:
            if not (progressed or changed_input):
                return RecoveryAction.WAIT
            self.blocked = False
            self.progress_at = now
        if not goal_issued:
            # Allow the rate-limited command to reach its finite legal destination
            # before timing lack of tracking. Long recoveries are not stalls.
            self.goal_issued_at = None
            return RecoveryAction.ADVANCE
        if self.goal_issued_at is None:
            self.goal_issued_at = now
        waiting = now - max(self.progress_at, self.goal_issued_at)
        if waiting >= NO_PROGRESS_TIMEOUT_S:
            self.blocked = True
            return RecoveryAction.STALLED
        return RecoveryAction.ADVANCE


def check_coordination(proposed, automatic, sent, active, rates, period, limits, calibrated_limits):
    """Pi validation independent of the WSL candidate generator."""
    valid = within_limits(proposed, calibrated_limits)
    steps = recovery_step_limits(rates, period)
    for i, (q, expected, last, moving, step, (lo, hi)) in enumerate(zip(
            proposed, automatic, sent, active, steps, limits)):
        valid &= (abs(q - expected) <= JOINT_EPS_DEG if moving or i == 5 else
                  lo <= q <= hi and abs(q - last) <= step + JOINT_EPS_DEG)
    if valid:
        return CoordinationResult.ACCEPT
    if (any(abs(q - last) > step + JOINT_EPS_DEG for q, last, step in zip(proposed, sent, steps))
            or not within_limits(proposed, calibrated_limits)):
        return CoordinationResult.REJECT
    return CoordinationResult.AUTOMATIC


def inward_segment_valid(sent, measured, target, limits, calibrated_limits):
    """Analytical joint-range check; does not establish collision clearance."""
    legal = within_limits(target, calibrated_limits)
    for start, current, end, (lo, hi) in zip(sent, measured, target, limits):
        if current > hi:
            legal &= end <= current and end <= start and end >= lo
        elif current < lo:
            legal &= end >= current and end >= start and end <= hi
        else:
            legal &= lo <= end <= hi
    return legal
