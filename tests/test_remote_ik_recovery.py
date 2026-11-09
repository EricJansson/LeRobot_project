"""Recovery rate, settling and recorded-limit regressions using fake feedback only."""

from types import SimpleNamespace
from threading import Event

import pytest

from robot_program.remote_ik_protocol import PROTOCOL_VERSION, command_limits, within_limits, recovery_step_limits
from robot_program.remote_ik_recovery import RecoveryAction, RecoveryState
from robot_program.remote_ik_control import ControlLoop, LatestInput
from scripts.remote_ik_motor_service import MotorSession


class Rig:
    def __init__(self, measured, limits=None):
        self.now = 0.
        self.measured = measured.copy()
        self.writes = []
        limits = limits or [[-90., 90.]] * 5 + [[0., 50.]]

        def write(target):
            self.writes.append(target.copy())
            return target.copy()

        bus = SimpleNamespace(read=lambda: self.measured.copy(), write=write, arm=write)
        self.motor = MotorSession(bus, limits=limits, calibrated_limits=[[-120., 120.]] * 5 + [[0., 100.]],
                                  model="test", token="test", watchdog=.2, hz=50,
                                  arm_rate=200, gripper_rate=30, clock=lambda: self.now)
        self.feedback = self.motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "test"})
        self.command("arm")

    def command(self, op="recover", *, intent=None, target=None):
        self.now += .02
        values = {"input": intent or [0.] * 6}
        if target is not None:
            values["target"] = target
        self.feedback = self.motor.tick({"v": PROTOCOL_VERSION, "op": op, "session": self.feedback["session"],
                                        "seq": self.feedback["ack"] + 1,
                                        "feedback_seq": self.feedback["feedback_seq"], **values})
        return self.feedback


def test_recovery_rate_doubles_for_arm_only_and_respects_lower_configured_rates():
    assert recovery_step_limits([200.] * 5 + [30.], .02) == [.2] * 5 + [.1]
    assert recovery_step_limits([2.] * 5 + [1.], .02) == [.04] * 5 + [.02]
    arm = Rig([91., 0., 0., 0., 0., 0.])
    assert arm.command()["sent"][0] == pytest.approx(90.8)
    jaw = Rig([0., 0., 0., 0., 0., 60.])
    assert jaw.command()["sent"][5] == pytest.approx(59.9)


@pytest.mark.parametrize("index,start", [(0, 115.), (5, 100.)])
def test_full_legal_destination_is_issued_before_declaring_no_measured_progress(index, start):
    pose = [0.] * 6
    pose[index] = start
    rig = Rig(pose)
    reached_goal = False
    for _ in range(700):
        fb = rig.command()
        assert not fb["recovery_blocked"] and not fb["fault"]
        # Fixed feedback until a complete legal goal is actually issued. This
        # takes more than one second at the recovery rate, particularly for the jaw.
        if fb["recovery_goal"] is not None and fb["sent"] == fb["recovery_goal"]:
            reached_goal = True
            assert rig.now > 1.
            rig.measured = fb["sent"].copy()
            break
    assert reached_goal
    for _ in range(20):
        fb = rig.command()
    assert not fb["recovery_required"] and fb["recovery_holding"]


def test_settling_interval_begins_when_goal_is_issued_not_when_episode_began():
    limits = [[-90., 90.]] * 5 + [[0., 50.]]
    state = RecoveryState()
    measured = [115., 0., 0., 0., 0., 0.]
    state.begin(measured, limits, command_limits(limits))
    state.restart_trial(0.)
    assert state.evaluate(measured, [110., 0., 0., 0., 0., 0.], [0.] * 6,
                          limits, 1.5) is RecoveryAction.ADVANCE
    sent = state.goal.copy()
    state.evaluate(measured, sent, [0.] * 6, limits, 6.)
    assert state.margin == 5.
    assert state.evaluate(measured, sent, [0.] * 6, limits, 6.99) is RecoveryAction.ADVANCE
    assert state.evaluate(measured, sent, [0.] * 6, limits, 7.01) is RecoveryAction.STALLED


def test_recorded_shoulder_and_elbow_excursion_reaches_legal_feedback_with_finite_deeper_goal():
    # feedback-test.json, second recovery, starts near tick 2714. The elbow stayed
    # around 96.835° while a goal only 2° inside its 96.829867° limit did not move it.
    limits = [[-90., 90.], [-100.000042857568, 100.], [-100., 96.82986737710912],
              [-95., 95.], [-90., 90.], [0., 50.]]
    pose = [-2.7252747252747254, -105.14285714285714, 96.83516483516483,
            69.71428571428571, -5.758241758241758, 1.7808219178082192]
    rig = Rig(pose, limits)
    elbow_released = False
    max_margin = 0.
    prior = pose.copy()
    for _ in range(600):
        fb = rig.command()
        max_margin = max(max_margin, fb["recovery_margin_deg"])
        assert not fb["recovery_blocked"] and not fb["fault"]
        assert all(abs(q - last) <= (.1 if i == 5 else .2) + 1e-8
                   for i, (q, last) in enumerate(zip(fb["sent"], prior)))
        prior = fb["sent"].copy()
        for index in (0, 3, 4, 5):
            assert fb["sent"][index] == pose[index]
        rig.measured[1] = fb["sent"][1]  # Shoulder tracks; elbow has a synthetic breakaway load.
        if pose[2] - fb["sent"][2] >= 2.5:
            elbow_released = True
        if elbow_released:
            rig.measured[2] = fb["sent"][2]
        if not fb["recovery_required"]:
            break
    assert not fb["recovery_required"]
    assert within_limits(rig.measured, limits)
    assert 2. < max_margin <= 5.
    # This synthetic load verifies command policy, not physical motor performance.


def test_fixed_destination_does_not_shift_while_a_different_joint_is_stalled():
    limits = [[-90., 90.]] * 5 + [[0., 50.]]
    state = RecoveryState()
    state.begin([91., 92., 0., 0., 0., 0.], limits, command_limits(limits))
    original = state.goal.copy()
    for tick in range(20):
        state.evaluate([85., 92., 0., 0., 0., 0.], original, [0.] * 6, limits, tick * .1)
    assert state.goal == original


def test_blocked_attempt_ignores_repeated_coordination_and_single_encoder_tick_noise():
    rig = Rig([0., 0., 0., 95.032967, 0., 0.])
    intent = [0., 0., 1., 0., 0., 0.]
    for _ in range(700):
        fb = rig.command(intent=intent)
    assert fb["recovery_blocked"] and fb["recovery_margin_deg"] == 5.
    count = len(rig.writes)
    for tick in range(100):
        rig.measured[3] = 95.032967 - (.087912 if tick % 2 else 0.)
        fb = rig.command(intent=intent, target=fb["recovery_target"])
        assert fb["recovery_blocked"]
    assert len(rig.writes) == count
    # A deliberate direction change permits one fresh, bounded attempt.
    fb = rig.command(intent=[0., 0., -1., 0., 0., 0.])
    assert not fb["recovery_blocked"]
    assert len(rig.writes) == count + 1


def test_new_recovery_episode_discards_the_previous_stall_and_input_state():
    limits = [[-90., 90.]] * 5 + [[0., 50.]]
    state = RecoveryState(blocked=True, input=[1.] * 6, best=0., margin=5., goal_issued_at=10.)
    state.begin([91., 0., 0., 0., 0., 0.], limits, command_limits(limits))
    assert not state.blocked and state.margin == 5. and state.input is None and state.goal_issued_at is None
    assert state.goal[0] == 85.


def test_completion_requires_sustained_measured_clearance_not_one_near_limit_sample():
    limits = [[-90., 90.]] * 5 + [[0., 50.]]
    state = RecoveryState()
    state.begin([90.01, 0., 0., 0., 0., 0.], limits, command_limits(limits))
    def evaluate(value, now):
        return state.evaluate([value, 0., 0., 0., 0., 0.], state.goal.copy(), [0.] * 6, limits, now)
    assert evaluate(89.6, 0.) is RecoveryAction.ADVANCE  # Old 0.4° clearance completed here.
    assert evaluate(88.9, .2) is RecoveryAction.ADVANCE
    assert evaluate(89.2, .35) is RecoveryAction.ADVANCE  # Rebound resets the settling interval.
    assert state.inside_since is None
    assert evaluate(88.8, .4) is RecoveryAction.ADVANCE
    assert evaluate(88.8, .699) is RecoveryAction.ADVANCE
    assert evaluate(88.8, .701) is RecoveryAction.COMPLETE


def test_recorded_load_offset_keeps_inward_goal_after_recovery_and_idle_rearm():
    # Latest feedback-test.json: at tick 2558, elbow feedback 96.30769°
    # replaced an inward command near 92.43°. Feedback rebounded to 96.835°.
    upper = 96.82986737710912
    limits = [[-90., 90.]] * 5 + [[0., 50.]]
    limits[2] = [-100., upper]
    pose = [0., 0., 96.83516483516483, 0., 0., 0.]
    rig = Rig(pose, limits)
    for _ in range(120):
        fb = rig.command()
        # Synthetic static load: measured pose stays about 3.9° above the goal.
        rig.measured[2] = min(pose[2], fb["sent"][2] + 3.9)
        if not fb["recovery_required"]:
            break
    assert not fb["recovery_required"] and fb["recovery_holding"]
    assert fb["sent"][2] == pytest.approx(upper - 5.)
    assert rig.measured[2] <= upper - 1.
    retained = fb["sent"].copy()
    count = len(rig.writes)
    for _ in range(60):
        fb = rig.command("poll")
        assert not fb["recovery_required"] and fb["sent"] == retained
    assert len(rig.writes) == count  # Completion and idle never recapture near-limit feedback.
    rig.command("disarm")
    fb = rig.command("arm")
    assert fb["armed"] and fb["recovery_holding"] and fb["sent"] == retained
    assert len(rig.writes) == count


def test_idle_then_fresh_input_uses_retained_recovery_reference_without_rollback():
    rig = Rig([90.1, 0., 0., 0., 0., 0.])
    for _ in range(100):
        feedback = rig.command()
        rig.measured[0] = min(90.1, feedback["sent"][0] + 3.9)
        if feedback["recovery_holding"]:
            break
    assert feedback["recovery_holding"]
    retained = feedback["sent"].copy()
    latest, stop = LatestInput(clock=lambda: rig.now), Event()
    neutral = {"axes": dict(LX=0., LY=0., RX=0., RY=0., LT=0., RT=0.),
               "buttons": dict(Start=False, Back=False, LB=False), "dpad": (0, 0)}
    latest.publish(neutral)
    calls, operations = [], []
    baseline = [retained[0] / 1000., 0., 0.]

    def solve(measured, delta, **values):
        calls.append(values)
        assert values["sent"] == retained
        if any(delta):
            assert values["desired_xyz_m"] == baseline
            target = retained.copy()
            target[1] += .01
            return dict(target=target, accepted_xyz_m=baseline, baseline_xyz_m=baseline, xyz_m=baseline)
        assert values["desired_xyz_m"] is None
        return dict(target=None, baseline_xyz_m=baseline, xyz_m=baseline)

    def request(op, **values):
        operations.append(op)
        if op == "move":
            assert values["target"][0] == retained[0]
            fb = rig.command("move", target=values["target"])
            stop.set()
            return fb
        fb = rig.command(op)
        latest.publish({**neutral, "axes": {**neutral["axes"], "LX": .5},
                        "buttons": {**neutral["buttons"], "LB": True}})
        return fb

    loop = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, 1., "Sim", clock=lambda: rig.now)
    loop.session(SimpleNamespace(feedback=rig.feedback, request=request))
    assert operations == ["poll", "move"]
    assert len(calls) == 2
    assert not rig.feedback["recovery_holding"]
