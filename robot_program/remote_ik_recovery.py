"""Pi-local recovery state and joint validation, without motor I/O or IK."""

from dataclasses import dataclass
from enum import Enum, auto
import math

from robot_program.remote_ik_protocol import (
    COMMAND_MARGIN_DEG, RECOVERY_DEEPEN_S, RECOVERY_MAX_MARGIN_DEG,
    RECOVERY_RATE_DEG_S, within_limits,
)

PROGRESS_DEG = .05
COMPLETION_TOLERANCE_DEG = .1
INPUT_CHANGE_THRESHOLD = .1
NO_PROGRESS_TIMEOUT_S = 1.
MARGIN_STEP_DEG = .5
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
    margin: float = COMMAND_MARGIN_DEG
    coordinated: bool = False

    @property
    def active_axes(self):
        return [a != g for a, g in zip(self.anchor, self.goal)]

    def clear(self, *, completed=False):
        self.goal = self.anchor = None
        self.blocked = False
        self.remaining = 0. if completed else None

    def begin(self, measured, limits, command_limits):
        self.anchor = measured.copy()
        self.goal = [min(max(q, lo), hi) if not op_lo <= q <= op_hi else q
                     for q, (lo, hi), (op_lo, op_hi) in zip(measured, command_limits, limits)]

    def restart_trial(self, now):
        self.best = math.inf
        self.progress_at = now

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
        return [s + max(-min(rate, RECOVERY_RATE_DEG_S) * period,
                        min(min(rate, RECOVERY_RATE_DEG_S) * period, goal - s))
                if anchor != goal else s for s, goal, anchor, rate in
                zip(sent, self.goal, self.anchor, rates)]

    def deepen(self, limits):
        if self.margin >= RECOVERY_MAX_MARGIN_DEG:
            return False
        self.margin = min(RECOVERY_MAX_MARGIN_DEG, self.margin + MARGIN_STEP_DEG)
        changed = False
        for i, (anchor, goal, (lo, hi)) in enumerate(zip(self.anchor, self.goal, limits)):
            if anchor > hi:
                new = max((lo + hi) / 2, hi - self.margin)
            elif anchor < lo:
                new = min((lo + hi) / 2, lo + self.margin)
            else:
                continue
            changed |= new != goal
            self.goal[i] = new
        return changed

    def evaluate(self, measured, sent, intent, proposed, limits, command_limits, now):
        """Advance episode state, leaving all stop/write operations to the owner."""
        active = self.active_axes
        remaining = max((abs(q - g) for q, g, moving in zip(measured, self.goal, active) if moving), default=0.)
        self.remaining = remaining
        progressed = remaining < self.best - PROGRESS_DEG
        changed_input = (self.input is None
                         or any(abs(a - b) >= INPUT_CHANGE_THRESHOLD for a, b in zip(intent, self.input)))
        if changed_input:
            self.input = intent.copy()
        if progressed:
            self.best, self.progress_at = remaining, now
        recovered = all(lo - COMPLETION_TOLERANCE_DEG <= q <= hi + COMPLETION_TOLERANCE_DEG
                        for q, (lo, hi), moving in zip(measured, command_limits, active) if moving)
        if recovered and within_limits(measured, limits):
            return RecoveryAction.COMPLETE
        if self.blocked:
            if not (progressed or changed_input or proposed is not None):
                return RecoveryAction.WAIT
            self.blocked = False
            self.progress_at = now
        settled = all(abs(s - g) < JOINT_EPS_DEG for s, g, moving in zip(sent, self.goal, active) if moving)
        if settled and now - self.progress_at >= RECOVERY_DEEPEN_S and self.deepen(limits):
            self.remaining = max(abs(q - g) for q, g, moving in zip(measured, self.goal, active) if moving)
            self.best = self.remaining
            self.progress_at = now
        elif now - self.progress_at >= NO_PROGRESS_TIMEOUT_S:
            self.blocked = True
            return RecoveryAction.STALLED
        return RecoveryAction.ADVANCE


def check_coordination(proposed, automatic, sent, active, rates, period, limits, calibrated_limits):
    """Pi validation independent of the WSL candidate generator."""
    valid = within_limits(proposed, calibrated_limits)
    for i, (q, expected, last, moving, rate, (lo, hi)) in enumerate(zip(
            proposed, automatic, sent, active, rates, limits)):
        valid &= (abs(q - expected) <= JOINT_EPS_DEG if moving or i == 5 else
                  lo <= q <= hi and abs(q - last) <= min(rate, RECOVERY_RATE_DEG_S) * period + JOINT_EPS_DEG)
    if valid:
        return CoordinationResult.ACCEPT
    if (any(abs(q - last) > min(rate, RECOVERY_RATE_DEG_S) * period + JOINT_EPS_DEG
            for q, last, rate in zip(proposed, sent, rates)) or not within_limits(proposed, calibrated_limits)):
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
