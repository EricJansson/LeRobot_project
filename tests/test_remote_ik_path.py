"""Offline path and recovery regressions; never connect a motor bus."""
from types import SimpleNamespace
import numpy as np
import pytest

from robot_program.remote_ik_correction import CorrectionProfile, StationaryCorrection
from scripts.remote_ik_worker import solve_intent
from scripts.remote_ik_diagnostics import summarize


class CartesianSolver:
    """Nonlinear FK exposes errors hidden by joint-linear Cartesian fakes."""
    def __init__(self):
        self.kinematics = self

    def forward_kinematics(self, q):
        pose = np.eye(4)
        pose[:3, 3] = [q[0] / 100, q[1] / 100, q[2] / 100 + q[0] ** 2 / 10000]
        return pose

    def solve_pose(self, pose, seed):
        x, y, z = pose[:3, 3]
        q = np.array([x * 100, y * 100, (z - x*x) * 100, seed[3], seed[4]])
        return SimpleNamespace(accepted=x <= .1, reason="reach", joints_deg=q, pose=self.forward_kinematics(q))


@pytest.mark.parametrize("axis", [0, 1, 2])
@pytest.mark.parametrize("lag", [0., .2, 1.])
def test_ten_round_trips_with_lag_and_quantization(axis, lag):
    solver = CartesianSolver()
    sent = [0., 0., 10., 0., 0., 0.]
    measured = sent.copy()
    desired = [0., 0., .1]
    for cycle in range(10):
        for sign in (1, -1):
            for tick in range(20):
                delta = [0.] * 6
                delta[axis] = sign * .001
                result = solve_intent(solver, measured, delta, [0, 50], sent=sent,
                                      desired_xyz_m=desired, joint_step_limits=[1.] * 6)
                assert result["target"] is not None
                assert result["progress_fraction"] == 1.
                sent = result["target"]
                desired = result["accepted_xyz_m"]
                measured = [round(m + (1-lag)*(s-m), 2) for m, s in zip(measured, sent)]
                xyz = solver.forward_kinematics(sent)[:3, 3]
                assert xyz == pytest.approx(desired, abs=1e-9)
    assert desired == pytest.approx([0., 0., .1], abs=1e-9)


def test_cartesian_search_respects_rates_and_reverse_has_no_block_latch():
    solver = CartesianSolver()
    sent = [10., 0., 9., 0., 0., 0.]
    desired = [.1, 0., .1]
    blocked = solve_intent(solver, sent, [.002, 0, 0, 0, 0, 0], [0, 50], sent=sent,
                           desired_xyz_m=desired, joint_step_limits=[.1]*6)
    assert blocked["target"] is None
    reverse = solve_intent(solver, sent, [-.002, 0, 0, 0, 0, 0], [0, 50], sent=sent,
                           desired_xyz_m=desired, joint_step_limits=[.1]*6)
    assert reverse["target"][0] < sent[0]
    assert np.max(np.abs(np.array(reverse["target"]) - sent)) <= .1 + 1e-8
    assert reverse["candidate_xyz_m"][2] == pytest.approx(.1)


def test_recovery_is_bounded_without_adopting_sag_or_banking_input():
    solver = CartesianSolver()
    sent = [0., 0., 5., 0., 0., 0.]
    result = solve_intent(solver, sent, [.002, 0, 0, 0, 0, 0], [0, 50], sent=sent,
                          desired_xyz_m=[0, 0, .1], joint_step_limits=[1.]*6)
    assert result["requested_xyz_m"][2] == .1
    assert result["candidate_xyz_m"][2] <= .05100001
    assert result["recovering"]


def test_correction_hysteresis_timeout_and_new_input_reset():
    c = StationaryCorrection(CorrectionProfile(.003, .001, .3, .002, .01, .5))
    goal = [0, 0, .1]
    assert not c.ready(0, [0, 0, .09], goal)
    assert c.ready(.31, [0, 0, .09], goal)
    assert not c.ready(.82, [0, 0, .09], goal)
    assert c.exhausted
    c.reset()  # Operator input has priority and starts a new episode.
    assert not c.ready(1, [0, 0, .09], goal)
    assert c.ready(1.31, [0, 0, .09], goal)
    assert not c.ready(1.32, [0, 0, .0995], goal)
    assert not c.ready(1.33, [0, 0, .098], goal)
    c.issued([0, 0, 0], [0, 0, .01])
    assert not c.ready(1.4, [0, 0, .09], goal)


def test_diagnostics_aligns_measurement_with_prior_command():
    report = summarize({"ticks": [{"tick_s": 0, "ack_s": .02, "measured_xyz_m": [0, 0, 0],
                        "issued_before_xyz_m": [0, 0, .001], "issued_xyz_m": [0, 0, .1]}]})
    assert report["errors"]["measured_to_prior_issued_mm"] == 1.
    assert "unknown version" in report["provenance"]["classification"]


@pytest.mark.parametrize("lag,delay", [(0., .02), (.3, .04), (1., .06)])
def test_real_loop_worker_and_motor_session_with_delayed_or_stalled_feedback(monkeypatch, lag, delay):
    from threading import Event
    from robot_program import remote_ik_control as teleop
    from scripts.remote_ik_motor_service import MotorSession
    from robot_program.remote_ik_protocol import PROTOCOL_VERSION
    now = [0.]
    solver = CartesianSolver()

    class Bus:
        measured = [0., 0., 10., 0., 0., 0.]
        target = measured.copy()
        def read(self):
            self.measured = [round(m + (1-lag)*(s-m), 3) for m, s in zip(self.measured, self.target)]
            return self.measured.copy()
        def write(self, target):
            self.target = target.copy()
            return target.copy()
        arm = write

    motor = MotorSession(Bus(), limits=[[-90, 90]]*5+[[0, 50]], model="fake", token="token",
                         watchdog=.2, hz=50, arm_rate=100, gripper_rate=30, clock=lambda: now[0])
    fb = motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "token"})
    def command(op, **values):
        return motor.tick({"v": PROTOCOL_VERSION, "session": fb["session"], "seq": fb["ack"]+1,
                           "feedback_seq": fb["feedback_seq"], "op": op, **values})
    fb = command("arm")
    latest, stop, trace = teleop.LatestInput(clock=lambda: now[0]), Event(), []
    def publish(active=True, lb=True):
        latest.publish({"axes": dict(LX=0., LY=-.5 if active else 0., RX=0., RY=0., LT=0., RT=0.),
                        "buttons": dict(Start=False, Back=False, LB=lb), "dpad": (0, 0)})
    class Worker:
        def solve(self, measured, delta, *, sent, desired_xyz_m, joint_step_limits, correction, recovery_step_m):
            return solve_intent(solver, measured, delta, [0, 50], sent=sent, desired_xyz_m=desired_xyz_m,
                                joint_step_limits=joint_step_limits)
    count = 0
    def request(op, **values):
        nonlocal fb, count
        now[0] += delay
        fb = command(op, **values)
        count += 1
        if count >= 35:
            stop.set()
        else:
            publish(active=count not in (10, 20), lb=count != 25)
        return fb
    publish()
    loop = teleop.ControlLoop(Worker(), None, latest, stop, 1., "offline", trace=trace, clock=lambda: now[0])
    loop.session(SimpleNamespace(feedback=fb, request=request))
    assert len(trace) == 35
    assert all(t["desired_xyz_m"][2] == pytest.approx(.1) for t in trace)
    assert not any(t["rate_limited"] for t in trace)
    assert all(t["delta"][0] <= .002 for t in trace)
    assert trace[-1]["op"] == "move"  # Stall and delay never latch recovery.


def test_solver_tolerance_does_not_bank_unexecuted_boundary_progress():
    solver = CartesianSolver()
    original = solver.solve_pose
    def no_progress(pose, seed):
        result = original(pose, seed)
        result.joints_deg = seed.copy()
        result.pose = solver.forward_kinematics(seed)
        result.accepted = True
        return result
    solver.solve_pose = no_progress
    desired = [0., 0., .1]
    sent = [0., 0., 10., 0., 0., 0.]
    for _ in range(100):
        result = solve_intent(solver, sent, [.0001, 0, 0, 0, 0, 0], [0, 50], sent=sent,
                              desired_xyz_m=desired, joint_step_limits=[1.]*6)
        desired = result["accepted_xyz_m"]
    assert desired == [0., 0., .1]


def test_fresh_input_cancels_stationary_correction_after_solve(monkeypatch):
    from threading import Event
    from robot_program import remote_ik_control as teleop
    latest, stop, trace = teleop.LatestInput(clock=lambda: 1.), Event(), []
    pad = {"axes": dict(LX=0., LY=0., RX=0., RY=0., LT=0., RT=0.),
           "buttons": dict(Start=False, Back=False, LB=True), "dpad": (0, 0)}
    latest.publish(pad)
    fb = dict(armed=True, moving=False, measured=[0.]*6, sent=[0.]*6,
              hz=50, watchdog_s=.2, fault=False, reason="")
    operations = []
    class Worker:
        def solve(self, measured, delta, *, sent, desired_xyz_m, joint_step_limits, correction, recovery_step_m):
            if correction:
                latest.publish({**pad, "axes": {**pad["axes"], "LY": -1.}})
                return {"target": [1.]*6, "xyz_m": [0,0,0], "candidate_xyz_m": [0,0,.001]}
            return {"baseline_xyz_m": [0,0,.01], "xyz_m": [0,0,0]}
    def ready(self, *args):
        return True
    monkeypatch.setattr(StationaryCorrection, "ready", ready)
    def request(op, **values):
        operations.append(op)
        if len(operations) == 2:
            stop.set()
        return fb
    loop = teleop.ControlLoop(Worker(), None, latest, stop, 1., "offline", trace=trace,
                             correction_profile=CorrectionProfile(.003,.001,.1,.002,.01,.5), clock=lambda: 1.)
    loop.session(SimpleNamespace(feedback=fb, request=request))
    assert operations == ["poll", "poll"]
    assert trace[1]["correction"]


def test_issued_comparison_preserves_distinct_correction_tolerance():
    from robot_program.remote_ik_control import issued_arm_matches
    target = [100.] * 6
    issued = [100. + 5e-8] * 5 + [0.]
    assert not issued_arm_matches(issued, target)
    assert issued_arm_matches(issued, target, rel_tol=1e-9)
    assert not issued_arm_matches(None, target)
    assert issued_arm_matches(target[:5] + [0.], target)


def test_missing_accepted_xyz_is_rejected_before_motor_exchange():
    from threading import Event
    from robot_program.remote_ik_control import ControlLoop, LatestInput
    latest = LatestInput(clock=lambda: 1.)
    latest.publish({"axes": dict(LX=0., LY=-1., RX=0., RY=0., LT=0., RT=0.),
                    "buttons": dict(Start=False, Back=False, LB=True), "dpad": (0, 0)})
    worker = SimpleNamespace(solve=lambda *args, **kwargs: {"target": [0.]*6, "requested_xyz_m": [0.,0.,0.]})
    feedback = dict(armed=True, moving=False, measured=[0.]*6, sent=[0.]*6, hz=50,
                    watchdog_s=.2, fault=False, reason="")
    loop = ControlLoop(worker, None, latest, Event(), 1., "offline", clock=lambda: 1.)
    link = SimpleNamespace(feedback=feedback, request=lambda *args, **kwargs: pytest.fail("Motor exchange on malformed result"))
    with pytest.raises(ValueError, match="accepted_xyz_m"):
        loop.session(link)


def test_worker_reuses_achieved_pose_and_initial_measured_fk():
    solver = CartesianSolver()
    calls = []
    original = solver.forward_kinematics
    def forward(q):
        calls.append(np.asarray(q).copy())
        return original(q)
    solver.forward_kinematics = forward
    result = solve_intent(solver, [0.,0.,10.,0.,0.,0.], [.001,0,0,0,0,0], [0,50])
    assert result["target"] is not None
    assert len(calls) == 2  # Initial reference and the solver's achieved pose.



def test_correction_also_requires_accepted_xyz():
    from threading import Event
    from robot_program.remote_ik_control import ControlLoop, LatestInput
    latest = LatestInput(clock=lambda: 1.)
    latest.publish({"axes": dict(LX=0.,LY=0.,RX=0.,RY=0.,LT=0.,RT=0.),
                    "buttons": dict(Start=False,Back=False,LB=True), "dpad": (0,0)})
    loop = ControlLoop(None, None, latest, Event(), 1., "offline", clock=lambda: 1.)
    with pytest.raises(ValueError, match="accepted_xyz_m"):
        loop._validate_input("poll", {"target": [0.]*6}, latest.sample, latest.sample,
                             arm_input=False, correcting=True, freshness=.1)
