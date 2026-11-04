"""Record a shadow self-collision check for sent SO-101 commands."""

from contextlib import contextmanager
import json
from math import ceil
from pathlib import Path
import sys
from time import perf_counter, time


GAP_M = 0.008
CLEARANCE_REFINEMENTS = 6  # Resolve the starting gap to a 0.125 mm interval.
STEP_DEG = 2.0
JAW_DEG_PER_UNIT = 1.1  # so101ik's normalized gripper to URDF angle conversion.


def assess_path(model, measured, sent):
    """Classify a sampled path using the minimum modeled point separation."""
    collision = model.collides(measured, gap=GAP_M)
    fraction = 0.0 if collision else None
    lower, upper = 0.0, GAP_M
    if collision:
        # Bracket the starting separation using the checker's distance threshold.
        for _ in range(CLEARANCE_REFINEMENTS):
            probe = (lower + upper) / 2
            if model.collides(measured, gap=probe):
                upper = probe
            else:
                lower = probe

    deltas = [end - begin for begin, end in zip(measured, sent)]
    travel = [*deltas[:-1], deltas[-1] * JAW_DEG_PER_UNIT]
    segments = max(1, ceil(max(map(abs, travel)) / STEP_DEG))
    status = "clear"
    threshold = lower if collision else GAP_M
    for index in range(1, segments + 1):
        progress = index / segments
        pose = [value + progress * delta for value, delta in zip(measured, deltas)]
        hit = model.collides(pose, gap=threshold)
        if hit:
            if collision:
                status = "worsens_clearance"
            else:
                collision, fraction = hit, progress
                status = "enters_clearance_zone"
            break
    else:
        if collision:
            status = (
                "no_measurable_improvement" if model.collides(sent, gap=upper)
                else "improves_clearance"
            )

    return {
        "collision": collision,
        "path_fraction": fraction,
        "path_status": status,
        "samples_checked": index + 1,
    }


class CollisionLog:
    def __init__(self, model, stream):
        self.model = model
        self.stream = stream
        self.enabled = True

    def record(self, measured, sent):
        if not self.enabled:
            return
        try:
            start = perf_counter()
            assessment = assess_path(self.model, measured, sent)
            entry = {
                "time_unix_s": time(),
                "measured_arm_deg": measured[:-1],
                "sent_arm_deg": sent[:-1],
                "measured_gripper_normalized": measured[-1],
                "sent_gripper_normalized": sent[-1],
                **assessment,
                "check_ms": (perf_counter() - start) * 1000,
            }
            self.stream.write(json.dumps(entry, allow_nan=False) + "\n")
        except Exception as exc:
            # A diagnostic failure must never affect which commands teleop sends.
            self.enabled = False
            print(f"Collision logging stopped: {exc}", file=sys.stderr)


@contextmanager
def open_collision_log(path: Path, calibration_file: Path):
    try:
        from so101ik import Model
    except ImportError as exc:
        raise RuntimeError("Collision logging requires so101ik. See the Raspberry Pi setup in README.md.") from exc

    # Use the same zero convention as PlaCo and isolate self-collisions from table checks.
    model = Model(calibration_file=calibration_file, offsets={})
    model.table_z = -1e9
    with path.open("x", encoding="utf-8") as stream:
        yield CollisionLog(model, stream)
