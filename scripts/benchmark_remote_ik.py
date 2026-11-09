"""Offline real-solver comparison against a recorded Git worker revision. No serial I/O."""
import argparse
from collections import Counter
from contextlib import contextmanager
import inspect
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from time import perf_counter
from types import ModuleType

import numpy as np
from robot_program.lerobot_ik_controller import LeRobotPoseSolver
from robot_program.remote_ik_protocol import calibrated_follower, channel_limits, command_limits, kinematic_urdf
from scripts.remote_ik_worker import solve_intent


@contextmanager
def baseline_solver(source, revision):
    """Load the complete worker namespace, including helpers and dataclasses.

    This compares worker logic using the current kinematics adapter, as before.
    The module's main guard stays inactive: loading never starts a worker or bus.
    """
    name = "_remote_ik_benchmark_baseline"
    previous = sys.modules.get(name)
    module = ModuleType(name)
    sys.modules[name] = module
    try:
        exec(compile(source, f"baseline-worker-{revision}", "exec"), module.__dict__)
        yield module.solve_intent
    finally:
        if previous is None:
            sys.modules.pop(name, None)
        else:
            sys.modules[name] = previous


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-revision", default="HEAD")
    parser.add_argument("--check", action="store_true", help="Exit nonzero if offline progress/path gates fail")
    parser.add_argument("--arm-rate", type=float, default=200.)
    parser.add_argument("--recorded-trace", type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.arm_rate) or args.arm_rate <= 0:
        parser.error("arm rate must be positive")
    steps = [args.arm_rate / 50]*5 + [.6]
    revision = subprocess.check_output(["git", "rev-parse", args.baseline_revision], text=True).strip()
    source = subprocess.check_output(["git", "show", f"{revision}:scripts/remote_ik_worker.py"], text=True)
    limits = channel_limits(calibrated_follower("unused", Path("calibration").resolve()))
    report = {"baseline_revision": revision, "kind": "OFFLINE ideal tracking; physical acceptance unverified", "runs": []}
    with baseline_solver(source, revision) as baseline, TemporaryDirectory() as directory:
        baseline_has_rates = "joint_step_limits" in inspect.signature(baseline).parameters
        path = Path(directory) / "model.urdf"
        path.write_text(kinematic_urdf())
        poses = [("bent", [-4.7, -65.7, 53.8, 68.4, -20.1, 3.4]),
                              ("extended", [0., -25., 15., 20., 0., 3.4]),
                              ("tight_wrist", [-4.7, -65.7, 53.8, 94.4998, -20.1, 3.4])]
        if args.recorded_trace:
            trace = json.loads(args.recorded_trace.read_text(encoding="utf-8-sig"))
            eligible = [t for t in trace["ticks"] if t.get("issued") and t.get("issued_xyz_m")]
            poses.append(("recorded_max_x", max(eligible, key=lambda t: t["issued_xyz_m"][0])["issued"]))
        for name, initial in poses:
            for axes in ([1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0],
                         [1, 1, 1, 0, 0, 0], [0, 0, 0, 1, 1, 0]):
                for label, solve in (("restored", baseline), ("candidate", solve_intent)):
                    solver = LeRobotPoseSolver((command_limits(limits) if label == "candidate" else limits)[:5], urdf_path=path,
                                             position_tolerance_m=.00005 if label == "candidate" else .0005,
                                             orientation_weight=.0001 if label == "candidate" else .01)
                    sent = initial.copy()
                    desired = solver.kinematics.forward_kinematics(np.array(sent[:5]))[:3, 3].tolist()
                    start = desired.copy()
                    times, errors, progress, rotation = [], [], [], []
                    modes, mode_times = Counter(), {}
                    rejected = shortened = 0
                    for tick in range(400):
                        sign = 1 if (tick // 20) % 2 == 0 else -1
                        delta = [sign * a * (.002 if i < 3 else .4) for i, a in enumerate(axes)]
                        before_pose = solver.kinematics.forward_kinematics(np.array(sent[:5])).copy()
                        before = before_pose[:3, 3].copy()
                        at = perf_counter()
                        result = solve(solver, sent, delta, limits[5], sent=sent, desired_xyz_m=desired,
                                       **({"joint_step_limits": steps} if label == "candidate" or baseline_has_rates else {}))
                        times.append((perf_counter()-at)*1000)
                        mode = result.get("motion_status", "baseline")
                        modes[mode] += 1
                        mode_times.setdefault(mode, []).append(times[-1])
                        target = result["target"]
                        if target is None:
                            rejected += 1
                            progress.append(0.)
                            continue
                        fraction = max(1., *(abs(a-b)/r for a,b,r in zip(target, sent, steps)))
                        shortened += fraction > 1.
                        sent = [b+(a-b)/fraction for a,b in zip(target, sent)]
                        if fraction == 1.:
                            desired = result.get("accepted_xyz_m", result["requested_xyz_m"])
                        xyz = solver.kinematics.forward_kinematics(np.array(sent[:5]))[:3, 3].copy()
                        errors.append(float(np.linalg.norm(xyz-np.array(desired)))*1000)
                        after_pose = solver.kinematics.forward_kinematics(np.array(sent[:5])).copy()
                        angle = np.arccos(np.clip((np.trace(before_pose[:3,:3].T @ after_pose[:3,:3])-1)/2, -1, 1))
                        rotation.append(float(np.rad2deg(angle)))
                        progress.append(float(np.linalg.norm(xyz-before))*1000)
                    report["runs"].append({"pose": name, "axes": axes, "controller": label,
                        "travel_mm": sum(progress), "rotation_travel_deg": sum(rotation), "median_tick_ms": float(np.median(times)),
                        "p95_tick_ms": float(np.percentile(times,95)), "rejected_ticks": rejected,
                        "pi_shortened_ticks": shortened, "max_reference_error_mm": max(errors, default=0.),
                        "round_trip_error_mm": float(np.linalg.norm(xyz-np.array(start)))*1000})
                    report["runs"][-1].update(modes=dict(modes),
                        solver_p99_ms=float(np.percentile(times, 99)), solver_worst_ms=max(times),
                        ticks_over_20_ms=sum(t > 20 for t in times),
                        mode_timing_ms={mode: {"median": float(np.median(values)),
                                              "p95": float(np.percentile(values, 95)), "worst": max(values)}
                                        for mode, values in mode_times.items()})
        # Fixed corner fixtures deliberately exercise the fallback instead of
        # letting ideal tracking immediately leave the difficult posture.
        normal = command_limits(limits)
        corner = [normal[0][0], normal[1][1], normal[2][1], normal[3][1], normal[4][0], 3.4]
        blocked = [normal[0][1], normal[1][1], normal[2][1], normal[3][0], normal[4][1], 3.4]
        report["fallback_runs"] = []
        for name, q, delta in (("pitch", corner, [0., -.002, 0., 0., 0., 0.]),
                               ("nearby_path", corner, [-.002, 0., 0., 0., 0., 0.]),
                               ("blocked", blocked, [0., 0., .002, 0., 0., 0.])):
            for label, solve in (("restored", baseline), ("candidate", solve_intent)):
                solver = LeRobotPoseSolver((normal if label == "candidate" else limits)[:5], urdf_path=path)
                xyz = solver.kinematics.forward_kinematics(np.array(q[:5]))[:3, 3].copy()
                modes, durations, forwards = Counter(), [], []
                for _ in range(100):
                    at = perf_counter()
                    result = solve(solver, q, delta, limits[5], sent=q, desired_xyz_m=xyz.tolist(),
                                   **({"joint_step_limits": steps} if label == "candidate" or baseline_has_rates else {}))
                    durations.append((perf_counter() - at) * 1000)
                    modes[result.get("motion_status", "baseline")] += 1
                    movement = np.array(result["candidate_xyz_m"]) - xyz if result.get("candidate_xyz_m") else np.zeros(3)
                    forwards.append(float(movement @ np.array(delta[:3]) / .002) * 1000)
                report["fallback_runs"].append(dict(case=name, controller=label, modes=dict(modes),
                    median_forward_mm=float(np.median(forwards)), median_ms=float(np.median(durations)),
                    p95_ms=float(np.percentile(durations, 95)), p99_ms=float(np.percentile(durations, 99)),
                    worst_ms=max(durations), ticks_over_20_ms=sum(t > 20 for t in durations)))
    report["comparisons"] = []
    for before, after in zip(report["runs"][::2], report["runs"][1::2]):
        metric = "rotation_travel_deg" if any(after["axes"][3:5]) else "travel_mm"
        ratio = after[metric] / max(before[metric], 1e-12)
        report["comparisons"].append({"pose": after["pose"], "axes": after["axes"], "metric": metric,
                                      "progress_ratio": ratio,
                                      "passes_speed_floor": ratio >= .5,
                                      "passes_issued_path_error": after["max_reference_error_mm"] <= 2.})
    print(json.dumps(report, indent=2))
    if args.check and not all(c["passes_speed_floor"] and c["passes_issued_path_error"] for c in report["comparisons"]):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
