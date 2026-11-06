"""Compare the experimental hinge checker and exercise offline deadlines."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from time import perf_counter_ns
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

from robot_program import phase0_feasibility as f
from robot_program.phase0_hinge_collision import HINGE_BREAK_DISTANCE_M, REUSE_PADDING_M, HingeCollisionChecker
from scripts import phase0_collision_audit as audit


MARGIN_M = 0.002
DEADLINE_NS = 20_000_000


def cold_path(checker, *args, **kwargs):
    checker.reset_cache()
    return f.check_path(checker, *args, **kwargs)


def compare_paths(reference, checker, corpus):
    comparisons = {}
    for resolution in audit.STUDY_RESOLUTIONS_DEG:
        raw_ms = {"reference": [], "hinge": []}
        accepted = 0
        differences = []
        for group, start, end in corpus:
            begin = perf_counter_ns()
            actual = cold_path(checker, start, end, resolution, MARGIN_M)
            raw_ms["hinge"].append((perf_counter_ns() - begin) / 1e6)
            begin = perf_counter_ns()
            expected = f.check_path(reference, start, end, resolution, MARGIN_M)
            raw_ms["reference"].append((perf_counter_ns() - begin) / 1e6)
            accepted += actual["accepted"]
            if expected != actual:
                differences.append({"group": group, "start_deg": list(start), "end_deg": list(end),
                                    "reference": expected, "hinge": actual})
        comparisons[str(resolution)] = {"accepted": accepted, "differences": differences, "timing": {
            name: {**f.distribution(values), "raw_ms": values} for name, values in raw_ms.items()}}
        print(f"Compared {len(corpus)} paths at {resolution:g} degrees", flush=True)
    return comparisons


def compare_poses(reference, checker, fixtures, start, targets, exclusions):
    poses = [fixtures[name]["joints_deg"] for name in ("clear", "near", "colliding")]
    transition = fixtures["interior_collision"]
    poses.extend(f.path_samples(transition["start"], transition["end"], 0.5))
    for end in targets:
        poses.extend(f.path_samples(start, end, 0.125))
    for group in exclusions:
        for angle in group["angles_deg"]:
            pose = np.zeros(len(audit.JOINTS))
            pose[audit.JOINTS.index(group["joint"])] = angle
            poses.append(pose)
    checker.reset_cache()
    differences = []
    for pose in poses:
        expected = reference.query_with_margin(pose, MARGIN_M, all_hits=True)["geometry_pairs"]
        actual = checker.query_with_margin(pose, MARGIN_M, all_hits=True)["geometry_pairs"]
        if expected != actual:
            differences.append({"pose_deg": list(pose), "reference": expected, "hinge": actual})
    return {"poses": len(poses), "differences": differences}


def try_request(controller, checker, target, measured, current, *, clock_ns=perf_counter_ns):
    """Experiment only. Return a target after a fresh, completed check."""
    deadline = clock_ns() + DEADLINE_NS
    if not current():
        return {"accepted": False, "reason": "stale_or_superseded"}
    ik = controller.solve_pose(target, measured[:5])
    if not ik.accepted:
        return {"accepted": False, "reason": f"ik: {ik.reason}"}
    endpoint = [*ik.joints_deg, measured[5]]
    outcome = cold_path(checker, measured, endpoint, 0.5, MARGIN_M, deadline_ns=deadline, clock_ns=clock_ns)
    if outcome["accepted"]:
        if not current():
            return {"accepted": False, "reason": "stale_or_superseded"}
        if clock_ns() >= deadline:
            return {"accepted": False, "reason": "deadline"}
        outcome["command_deg"] = endpoint
    return outcome


def simulate_feedback(controller, checker, start, requests):
    scenarios = {}
    for name, gain in (("immediate", 1.), ("lagged", 0.35), ("fixed", 0.)):
        measured = np.array(start)
        rows = []
        for request in requests:
            begin = perf_counter_ns()
            outcome = try_request(controller, checker, request, measured, lambda: True)
            rows.append({"measured_deg": measured.tolist(), "ms": (perf_counter_ns() - begin) / 1e6, **outcome})
            if outcome["accepted"]:
                measured += gain * (np.asarray(outcome["command_deg"]) - measured)
        scenarios[name] = {"tracking_gain": gain, "issued": sum(row["accepted"] for row in rows), "requests": rows}

    # Deterministic faults. Change the simulated clock or freshness while a
    # native pose check is in progress. An in-flight query cannot be preempted.
    now, current = 0, True

    def inject(pose, margin):
        nonlocal now, current
        if fault == "late":
            now = DEADLINE_NS + 1
        if fault == "superseded":
            current = False
        return checker.query_with_margin(pose, margin)

    injected = SimpleNamespace(reset_cache=checker.reset_cache, query_with_margin=inject)
    faults = {}
    for fault in ("late", "stale_feedback", "superseded", "fresh_request"):
        now, current = 0, fault != "stale_feedback"
        target = requests[-1] if fault == "fresh_request" else requests[0]
        faults[fault] = try_request(controller, injected, target, np.array(start), lambda: current, clock_ns=lambda: now)
    return {"tracking": scenarios, "faults": faults}


def run_study(controller, reference):
    inventory = f.model_inventory()
    fixtures = json.loads(audit.FIXTURES.read_text(encoding="utf-8"))
    urdf = ET.parse(f.MODEL).getroot()
    jaw = urdf.find("joint[@name='gripper']/limit")
    limits = np.vstack((controller.limits_deg, np.rad2deg([float(jaw.attrib[k]) for k in ("lower", "upper")])))
    exclusions = audit.audit_excluded_pairs(reference, urdf, limits)
    corpus, boundaries = audit.build_corpus(reference, limits, fixtures["interior_collision"])
    geometry, added = audit.mesh_pair_mask(reference, exclusions)
    if len(geometry.collisionPairs) != 115 or len(added) != 30 or not set(audit.ELBOW_PAIRS) <= set(added):
        raise RuntimeError("The focused experimental mask has changed")
    reference.set_geometry(geometry)
    overrides = {pair: 0.00025 for pair in added if pair not in audit.ELBOW_PAIRS}
    reference.set_pair_margin_overrides(overrides)
    checker = HingeCollisionChecker(reference, added, MARGIN_M, overrides)
    with patch.object(controller, "solve_pose", wraps=controller.solve_pose) as solve:
        replay = f.retraction_replay(controller)
        requests = [call.args[0].copy() for call in solve.call_args_list]
    start, targets = audit.retraction_paths(replay)
    hinge_evidence, hinge_paths, hinge_boundaries = audit.study_hinges(reference, exclusions, limits, MARGIN_M)
    poses = compare_poses(reference, checker, fixtures, start, targets, exclusions)
    print(f"Compared {poses['poses']} full pair classifications", flush=True)
    paths = compare_paths(reference, checker, corpus + hinge_paths)
    timing = audit.study_timing(checker, controller, requests, start, targets, fixtures, MARGIN_M, path_check=cold_path)
    feedback = simulate_feedback(controller, checker, start, requests)
    checks = {
        "same_sampled_pose_and_path_outcomes": not poses["differences"] and all(not row["differences"] for row in paths.values()),
        "all_retraction_requests_still_clear": all(timing["cases"][f"replay_{i:02}"]["combined"]["accepted"] == 500
                                                  for i in range(1, 9)),
        "late_stale_and_superseded_requests_emit_no_target": all(
            not feedback["faults"][case]["accepted"] and "command_deg" not in feedback["faults"][case]
            for case in ("late", "stale_feedback", "superseded")) and feedback["faults"]["fresh_request"]["accepted"],
    }
    sources = (Path(__file__), Path(audit.__file__), Path(f.__file__),
               f.ROOT / "robot_program/phase0_hinge_collision.py", audit.FIXTURES, f.FIXTURES)
    return {"status": "OFFLINE VERIFIED" if all(checks.values()) else "OFFLINE CHECK FAILED",
            "phase1_gate": "NOT APPROVED", "timestamp_utc": datetime.now(timezone.utc).isoformat(),
            "environment": f.environment(), "model_sha256": inventory["urdf_sha256"],
            "source_sha256": {str(path.resolve().relative_to(f.ROOT)): f.sha256(path) for path in sources},
            "pair_count": len(geometry.collisionPairs), "adjacent_break_distance_m": HINGE_BREAK_DISTANCE_M,
            "reuse_padding_m": REUSE_PADDING_M, "default_margin_m": MARGIN_M,
            "overrides": [{"pair": pair, "margin_m": margin} for pair, margin in sorted(overrides.items())],
            "seed": audit.SEED, "seeded_boundaries": boundaries, "hinge_boundaries": hinge_boundaries,
            "hinge_evidence": hinge_evidence, "checks": checks, "pose_comparison": poses,
            "path_comparison": paths, "timing": timing, "feedback": feedback}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with f.hardware_forbidden():
        controller = f.offline_controller()
        reference = f.MarginCollisionChecker()
        try:
            report = run_study(controller, reference)
        finally:
            reference.close()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return 0 if all(report["checks"].values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
