"""Optional stationary correction policy. All distances are meters, times seconds."""
from dataclasses import dataclass
from math import dist, isfinite


@dataclass(frozen=True)
class CorrectionProfile:
    enter_m: float
    exit_m: float
    settle_s: float
    speed_m_s: float
    travel_m: float
    timeout_s: float

    def __post_init__(self):
        if not all(isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError("Correction settings must be finite and positive")
        if not self.exit_m < self.enter_m or self.exit_m > .002:
            raise ValueError("Require exit_m < enter_m and exit_m <= 0.002")


class StationaryCorrection:
    def __init__(self, profile):
        self.profile = profile
        self.reset()

    def reset(self):
        self.since = None
        self.active = self.exhausted = False
        self.travel = 0.
        self.best = float("inf")
        self.progress_at = None

    def ready(self, now, measured, desired):
        p = self.profile
        error = dist(measured, desired)
        if self.since is None:
            self.since = now
        if self.exhausted or now - self.since < p.settle_s:
            return False
        if error <= p.exit_m:
            self.active = False
            return False
        if not self.active and error < p.enter_m:
            return False
        if error < self.best - p.exit_m / 2:
            self.best, self.progress_at = error, now
        if self.progress_at is None:
            self.progress_at = now
        if now - self.progress_at >= p.timeout_s or self.travel >= p.travel_m:
            self.exhausted, self.active = True, False
            return False
        self.active = True
        return True

    def issued(self, before, after):
        self.travel += dist(before, after)
