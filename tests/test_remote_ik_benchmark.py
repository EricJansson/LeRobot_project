"""Benchmark baselines retain their own helper globals after a commit."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from scripts.benchmark_remote_ik import baseline_solver
from scripts import remote_ik_worker


def test_baseline_keeps_helpers_constants_dataclasses_and_main_guard():
    source = '''
from dataclasses import dataclass
OFFSET = 7
@dataclass
class Result:
    value: int
def helper(value):
    return Result(value + OFFSET)
def solve_intent(value):
    return helper(value)
if __name__ == "__main__":
    raise AssertionError("Baseline main must not run")
'''
    with baseline_solver(source, "fixture") as solve:
        assert solve(5).value == 12
    with baseline_solver(source.replace("OFFSET = 7", "OFFSET = 9"), "other") as other:
        assert other(5).value == 14
        assert solve(5).value == 12


def test_new_worker_can_be_its_own_baseline_on_unreachable_input():
    source = Path(remote_ik_worker.__file__).read_text(encoding="utf-8")
    solver = SimpleNamespace(
        limits_deg=np.array([[-90., 90.]] * 5),
        kinematics=SimpleNamespace(forward_kinematics=lambda q: np.eye(4)),
        solve_pose=lambda pose, q, **kwargs: SimpleNamespace(
            accepted=False, reason="unreachable", joints_deg=q, pose=np.eye(4)),
    )
    with baseline_solver(source, "future-head") as solve:
        result = solve(solver, [0.]*6, [0.,0.,.002,0.,0.,0.], [0.,50.], joint_step_limits=[4.]*6)
    assert result["target"] is None
    assert result["search_attempts"] == 43
    assert result["reason"] == "No legal forward movement"
