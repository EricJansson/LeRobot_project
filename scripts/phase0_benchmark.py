"""OFFLINE Phase 0 inspection and benchmarks; never starts a robot session."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
from queue import Queue, Empty
import subprocess
import sys
from threading import Thread
from time import perf_counter, perf_counter_ns, sleep

import numpy as np

from robot_program import phase0_feasibility as f


def write_report(path, result):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    lines = ["# Phase 0 offline benchmark", "", f"Status: **{result['status']}**", "",
             f"UTC run time: {result['timestamp_utc']}", "",
             "No motors, serial connections, calibration writes or torque operations.", "",
             "Experimental model/exclusions only; physical and continuous clearance unverified.", ""]
    if result.get("error"):
        lines.extend([f"Blocked: {result['error']}", ""])
    lines.extend(["| Case/stage | Median ms | p95 ms | p99 ms | Worst ms | >20 ms | Accepted/total |",
                  "|---|---:|---:|---:|---:|---:|---:|"])
    for name, row in result.get("benchmarks", {}).items():
        lines.append(f"| {name} | {row['median_ms']:.3f} | {row['p95_ms']:.3f} | {row['p99_ms']:.3f} | {row['worst_ms']:.3f} | {row['over_20ms']} | {row['accepted']}/{row['samples']} |")
    lines.extend(["", "See the accompanying JSON for raw samples, outcomes, model hashes, exclusions and dependencies.", ""])
    path.with_suffix(".md").write_text("\n".join(lines), encoding="utf-8")


def solver_benchmarks(args, result):
    with f.hardware_forbidden():
        controller = f.offline_controller(args.model)
        result["limits_deg"] = dict(zip(f.ARM_JOINTS, controller.limits_deg.tolist()))
        result["synthetic_collision_proof"] = f.synthetic_collision_proof()
        result["native_margin_proof"] = f.native_margin_proof()
        result["native_mesh_margin_proof"] = f.native_mesh_margin_proof()
        checker = (f.MarginCollisionChecker if args.backend == "native-margin" else f.CollisionChecker)(args.model)
        try:
            result["collision_policy"] = {
                "status": "EXPERIMENTAL, not hardware validated",
                "backend": args.backend,
                "included_link_pairs": checker.pairs, "excluded_link_pairs": checker.exclusions,
                "same_link": "Same-link geometry belongs to one rigid body; not self-tested",
                "jaw": "Sixth coordinate is a URDF degree angle, not normalized motor feedback",
                "distance_pair_count_at_zero": checker.query(np.zeros(6))["distance_pair_count"],
            }
            if args.mode == "discover":
                if args.backend != "native-margin":
                    raise ValueError("Use native-margin for bounded fixture discovery")
                fixtures = f.discover_robot_fixtures(controller, checker)
                result["robot_fixtures"] = fixtures
                if args.fixtures_output:
                    target = Path(args.fixtures_output)
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_text(json.dumps(fixtures, indent=2) + "\n", encoding="utf-8")
                return
            fixtures = json.loads(Path(args.robot_fixtures).read_text(encoding="utf-8"))
            result["robot_fixtures"] = fixtures
            result["fixture_sha256"] = {"ik": f.sha256(f.FIXTURES), "robot": f.sha256(args.robot_fixtures)}
            result["retraction_replay"] = f.retraction_replay(controller)
            cases = json.loads(f.FIXTURES.read_text(encoding="utf-8"))["cases"]
            cases.append({"name": "collision_near", "joints_deg": fixtures["near"]["joints_deg"][:5]})
            cases.append({"name": "retraction", "joints_deg": [0., 25., -12., 3., 0.],
                          "translation_m": [-0.016, 0., 0.]})
            if args.case:
                cases = [case for case in cases if case["name"] in args.case]
                if {case["name"] for case in cases} != set(args.case):
                    raise ValueError("Unknown case selection")
            def record(name, operation):
                result["benchmarks"][name] = f.measure(operation, args.warmup, args.repetitions)
                print(f"Completed {name}", flush=True)
            for case in cases:
                name = case["name"]
                if args.mode in ("all", "ik"):
                    record(f"{name}/ik", lambda case=case: f.ik_case(controller, case))
                q = [*case["joints_deg"], 0.]
                if name == "collision_near":
                    q = fixtures["near"]["joints_deg"]
                if args.mode in ("all", "collision"):
                    def collision(q=q):
                        value = checker.query(q)
                        return {"accepted": not value["colliding"],
                                "reason": "colliding" if value["colliding"] else "clear", **value}
                    record(f"{name}/collision", collision)
                if args.mode in ("all", "combined"):
                    for resolution in (0.5, 1., 2.):
                        for margin in (0., 0.002, 0.005):
                            def combined(case=case, q=q, resolution=resolution, margin=margin):
                                ik = f.ik_case(controller, case)
                                if not ik["accepted"]:
                                    return {**ik, "reason": f"ik: {ik['reason']}"}
                                end = [*ik["joints_deg"], q[5]]
                                return f.check_path(checker, q, end, resolution, margin)
                            record(f"{name}/combined/{resolution:g}deg/{margin*1000:g}mm", combined)
            if args.mode in ("all", "combined") and not args.case:
                for name in ("clear", "colliding"):
                    q = fixtures[name]["joints_deg"]
                    actual = checker.query(q)
                    if actual["colliding"] != (name == "colliding"):
                        raise RuntimeError(f"Saved robot {name} fixture no longer matches")
                transition = fixtures["interior_collision"]
                if checker.query(transition["start"])["colliding"] or checker.query(transition["end"])["colliding"]:
                    raise RuntimeError("Interior-collision fixture endpoints must be clear")
                for resolution in (0.5, 1., 2.):
                    for margin in (0., 0.002, 0.005):
                        operation = lambda resolution=resolution, margin=margin: f.check_path(
                            checker, transition["start"], transition["end"], resolution, margin)
                        record(f"interior_collision/path/{resolution:g}deg/{margin*1000:g}mm", operation)
                        if result["benchmarks"][f"interior_collision/path/{resolution:g}deg/{margin*1000:g}mm"]["accepted"]:
                            raise RuntimeError("Interior-colliding transition was accepted")
        finally:
            checker.close()


def ipc_benchmark(args):
    if sys.platform != "win32":
        raise RuntimeError("IPC benchmark must be launched by Windows Python")
    worker = """import sys,json,time
for line in sys.stdin:
 t=time.perf_counter_ns()
 value=json.loads(line)
 reply={'sequence':value['sequence'], 'worker_processing_ns':time.perf_counter_ns()-t}
 print(json.dumps(reply),flush=True)
"""
    lookup_start = perf_counter()
    executable = args.wsl_python
    if executable is None:
        home = subprocess.run(["wsl.exe", "-d", args.wsl_distro, "--", "printenv", "HOME"],
                              capture_output=True, text=True, encoding="utf-8", check=True, timeout=15).stdout.strip()
        if not home.startswith("/") or "\n" in home:
            raise RuntimeError("WSL did not return an absolute home directory")
        executable = home + "/.local/share/lerobot-phase0/venv/bin/python"
    lookup_ms = (perf_counter() - lookup_start) * 1000
    started = perf_counter()
    process = subprocess.Popen(["wsl.exe", "-d", args.wsl_distro, "--", executable, "-u", "-c", worker],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               text=True, encoding="utf-8", bufsize=1)
    replies = Queue()
    def reader():
        for line in process.stdout:
            replies.put(line)
        replies.put(None)
    Thread(target=reader, daemon=True).start()
    durations, processing, lateness, sends = [], [], [], []
    rescheduled = 0
    period = 1 / args.rate_hz if args.rate_hz else 0.
    next_tick = None
    try:
        for index in range(args.warmup + args.repetitions + 1):
            if next_tick is not None and period:
                sleep(max(0., next_tick - perf_counter()))
            late = max(0., perf_counter() - next_tick) * 1000 if next_tick is not None else 0.
            payload = {"protocol_version": 0, "session_id": "offline-echo-only", "sequence": index,
                       "joint_order": [*f.ARM_JOINTS, "gripper"], "joints_deg": [0, 25, -12, 3, 0],
                       "gripper_normalized": 20, "frame": "base_link"}
            encoded = json.dumps(payload, separators=(",", ":")) + "\n"
            t = perf_counter_ns()
            process.stdin.write(encoded)
            process.stdin.flush()
            try:
                line = replies.get(timeout=10)
            except Empty as exc:
                raise RuntimeError("WSL echo timed out") from exc
            if line is None:
                process.wait(timeout=5)
                raise RuntimeError(f"WSL echo exited before replying: {process.stderr.read()[:2048]}")
            reply = json.loads(line)
            elapsed = (perf_counter_ns() - t) / 1e6
            if reply["sequence"] != index:
                raise RuntimeError("Wrong IPC sequence")
            if index == 0:
                startup = (perf_counter() - started) * 1000
            elif index > args.warmup:
                durations.append(elapsed)
                processing.append(reply["worker_processing_ns"] / 1e6)
                lateness.append(late)
                sends.append(t / 1e9)
            # No backlog replay after late replies: schedule from now if behind.
            now = perf_counter()
            if period and next_tick is not None and now > next_tick + period:
                rescheduled += 1
            next_tick = max((next_tick or now) + period, now)
        return {**f.distribution(durations), "accepted": len(durations), "rejected": 0,
                "accepted_ratio": 1., "warmup": args.warmup, "raw_ms": durations,
                "worker_processing": f.distribution(processing), "raw_worker_ms": processing,
                "wake_lateness": f.distribution(lateness), "raw_wake_lateness_ms": lateness,
                "wake_over_one_period": sum(v >= period * 1000 for v in lateness) if period else None,
                "rescheduled_after_overrun_including_warmup": rescheduled,
                "reply_deadline_misses": sum(v > period * 1000 for v in durations) if period else None,
                "send_interval": f.distribution([1000 * (b - a) for a, b in zip(sends, sends[1:])]) if len(sends) > 1 else None,
                "effective_send_hz": (len(sends) - 1) / (sends[-1] - sends[0]) if len(sends) > 1 else None,
                "startup_ms": startup, "home_lookup_ms": lookup_ms,
                "rate_hz": args.rate_hz, "payload_bytes": len(encoded.encode()),
                "transport": "persistent stdio; host RTT, separately worker-local JSON parse time",
                "not_measured": "No IK/collision, Pi link or motors; no one-way latency claim"}
    finally:
        process.stdin.close()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        process.stdout.close()
        process.stderr.close()


def gamepad_benchmark(args):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame
    pygame.display.init()
    pygame.joystick.init()
    try:
        if not pygame.joystick.get_count():
            raise RuntimeError("No Windows gamepad detected; mapping/deadman UNVERIFIED")
        if sys.platform != "win32":
            raise RuntimeError("Physical gamepad polling benchmark requires Windows")
        device = pygame.joystick.Joystick(args.gamepad_index)
        device.init()
        def poll():
            pygame.event.pump()
            return {"accepted": True, "reason": "polled",
                    "axes": [device.get_axis(i) for i in range(device.get_numaxes())],
                    "buttons": [device.get_button(i) for i in range(device.get_numbuttons())],
                    "hats": [device.get_hat(i) for i in range(device.get_numhats())]}
        return {**f.measure(poll, args.warmup, args.repetitions), "device": device.get_name(),
                "sdl": list(pygame.get_sdl_version()), "mapping_deadman": "UNVERIFIED: owner button/stick exercise required"}
    finally:
        pygame.quit()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("inspect", "discover", "ik", "collision", "combined", "all", "ipc", "gamepad"), default="inspect")
    parser.add_argument("--backend", choices=("native-margin", "placo-distances"), default="native-margin")
    parser.add_argument("--case", action="append", help="Restrict IK/pose benchmarks to named cases")
    parser.add_argument("--model", type=Path, default=f.MODEL)
    parser.add_argument("--robot-fixtures", type=Path, default=f.ROOT / "tests/fixtures/phase0_robot_collisions.json")
    parser.add_argument("--fixtures-output", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--repetitions", type=int, default=500)
    parser.add_argument("--rate-hz", type=float, default=50.)
    parser.add_argument("--wsl-distro", default="Ubuntu")
    parser.add_argument("--wsl-python", help="Override the isolated interpreter in the WSL user's home")
    parser.add_argument("--gamepad-index", type=int, default=0)
    args = parser.parse_args(argv)
    if args.warmup < 0 or args.repetitions < 1 or not np.isfinite(args.rate_hz) or args.rate_hz < 0:
        parser.error("Use nonnegative warmup/rate and positive repetitions")
    result = {"status": "IN PROGRESS", "timestamp_utc": datetime.now(timezone.utc).isoformat(),
              "mode": args.mode, "environment": f.environment(), "benchmarks": {},
              "source_sha256": {str(path.relative_to(f.ROOT)): f.sha256(path) for path in (
                  Path(__file__).resolve(), f.ROOT / "robot_program/phase0_feasibility.py",
                  f.ROOT / "robot_program/lerobot_ik_controller.py")},
              "invocation": sys.argv if argv is None else argv,
              "hardware": "NOT RUN", "phase1_gate": "NOT APPROVED"}
    try:
        result["model"] = f.model_inventory(args.model)
        if args.mode == "ipc":
            result["benchmarks"]["windows_wsl/echo"] = ipc_benchmark(args)
        elif args.mode == "gamepad":
            result["benchmarks"]["windows/gamepad_poll"] = gamepad_benchmark(args)
        elif args.mode != "inspect":
            solver_benchmarks(args, result)
        result["status"] = "OFFLINE VERIFIED"
    except Exception as exc:
        result["status"] = "BLOCKED"
        result["error"] = f"{type(exc).__name__}: {exc}"
        print(result["error"], file=sys.stderr)
    write_report(args.output, result)
    return 0 if result["status"] == "OFFLINE VERIFIED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
