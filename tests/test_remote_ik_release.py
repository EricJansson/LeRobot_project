"""Offline torque-state and controller gesture regressions; no hardware access."""

from threading import Event
from types import SimpleNamespace
from importlib.metadata import version

import pytest

from robot_program.remote_ik_control import ControlLoop, LatestInput, MotionState
from robot_program.lerobot_ik_controller import DEFAULT_URDF
from robot_program.remote_ik_protocol import (
    CHANNELS, PROTOCOL_VERSION, calibrated_channel_limits, calibrated_follower, channel_limits,
)
from scripts.main_teleop_remote_ik import MotorLink
from scripts.remote_ik_motor_service import MotorSession, PiMotorBus, SimulatedBus


LIMITS = [[-90., 90.]] * 5 + [[0., 50.]]


def pad(*, start=False, back=False, lb=False, lx=0., trigger=0., dpad=(0, 0)):
    return {"axes": dict(LX=lx, LY=0., RX=0., RY=0., LT=trigger, RT=0.),
            "buttons": dict(Start=start, Back=back, LB=lb), "dpad": dpad}


class Fixture:
    def __init__(self, *, simulated=True, allow_motion=True):
        self.now = 0.
        self.bus = SimulatedBus(LIMITS)
        self.writes = []
        write = self.bus.write
        def record_write(target):
            self.writes.append(target.copy())
            return write(target)
        self.bus.write = record_write
        self.motor = MotorSession(self.bus, limits=LIMITS, model="fixture", token="test", watchdog=.2,
                                  hz=50, arm_rate=100, gripper_rate=50, simulated=simulated,
                                  allow_motion=allow_motion, clock=lambda: self.now)
        self.feedback = self.motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "test"})

    def request(self, op, **values):
        self.feedback = self.motor.tick({"v": PROTOCOL_VERSION, "op": op,
                                        "session": self.feedback["session"], "seq": self.feedback["ack"] + 1,
                                        "feedback_seq": self.feedback["feedback_seq"], **values})
        return self.feedback


def test_released_reconnect_never_writes_or_reenables_then_capture_uses_new_pose():
    rig = Fixture()
    assert rig.request("release_arm")["arm_torque"] == "released"
    assert rig.feedback["sent"] is None and not rig.feedback["armed"]
    rig.bus.measured = [12., -20., 30., 40., 5., 15.]
    rig.now += 1.
    rig.motor.tick(lost=True)
    rig.feedback = rig.motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "test"})
    assert rig.feedback["arm_torque"] == "released"
    for op in ("poll", "hold", "disarm", "arm", "move", "recover"):
        rig.request(op, target=[0.] * 6, input=[0.] * 6)
        assert not rig.feedback["armed"]
        assert rig.feedback["arm_torque"] == "released"
    assert not rig.writes
    captured = rig.request("capture")
    assert captured["arm_torque"] == "holding" and not captured["armed"]
    assert captured["sent"] == rig.bus.measured == rig.writes[-1]
    assert not captured["recovery_required"]
    assert rig.request("arm")["armed"]


def test_active_cannot_release_and_holding_does_not_recover():
    rig = Fixture()
    rig.request("arm")
    denied = rig.request("release_arm")
    assert not denied["armed"] and denied["arm_torque"] == "holding"
    rig.request("disarm")
    rig.request("recover", input=[1.] * 6)
    assert not rig.feedback["moving"]
    assert rig.request("release_arm")["arm_torque"] == "released"


@pytest.mark.parametrize("failure", ["stale", "fault", "observe"])
def test_release_cannot_bypass_feedback_fault_or_observe_gate(failure):
    rig = Fixture(simulated=False, allow_motion=failure != "observe")
    if failure == "stale":
        rig.now = .21
    elif failure == "fault":
        rig.motor.fault = "test fault"
    assert rig.request("release_arm")["arm_torque"] == "holding"
    assert not rig.writes


class RegisterBus:
    def __init__(self, *, enabled=True):
        self.torque = dict.fromkeys(CHANNELS, int(enabled))
        self.positions = dict(zip(CHANNELS, [1., 2., 3., 4., 5., 20.]))
        self.calls = []
        self.fail_channel = None
        self.ignore_channel = None

    def sync_read(self, register, motors=None, **kwargs):
        values = self.positions if register == "Present_Position" else self.torque
        return {channel: values[channel] for channel in (motors or CHANNELS)}

    def sync_write(self, register, values):
        self.calls.append((register, values.copy()))

    def enable_torque(self, *, num_retry):
        self.calls.append(("enable", None))
        self.torque = dict.fromkeys(CHANNELS, 1)

    def disable_torque(self, channel, *, num_retry):
        self.calls.append(("disable", channel))
        if channel == self.fail_channel:
            raise OSError("no reply")
        if channel != self.ignore_channel:
            self.torque[channel] = 0


def test_real_adapter_confirms_all_six_released_and_recaptures_before_enable():
    bus = RegisterBus()
    adapter = PiMotorBus(SimpleNamespace(bus=bus), __file__)
    adapter.read()
    assert adapter.torque_enabled is True
    adapter.release_arm()
    assert adapter.torque_enabled is False
    assert bus.torque["gripper"] == 0
    assert bus.calls == [("disable", channel) for channel in CHANNELS]
    pose = [10., 20., 30., 40., 50., 20.]
    adapter.arm(pose)
    assert adapter.torque_enabled is True
    assert bus.calls[-2:] == [("Goal_Position", dict(zip(CHANNELS, pose))), ("enable", None)]


@pytest.mark.parametrize("channel", ["shoulder_lift", "gripper"])
@pytest.mark.parametrize("failure", ["exception", "unconfirmed"])
def test_partial_release_is_fault_and_never_claims_released(failure, channel):
    registers = RegisterBus()
    if failure == "exception":
        registers.fail_channel = channel
    else:
        registers.ignore_channel = channel
    adapter = PiMotorBus(SimpleNamespace(bus=registers), __file__)
    rig = Fixture()
    rig.motor.bus = adapter
    result = rig.request("release_arm")
    assert result["fault"] and not result["armed"]
    assert "Could not confirm arm torque release" in result["reason"]
    assert result["arm_torque"] == "unknown"
    assert [value for op, value in registers.calls if op == "disable"] == list(CHANNELS)
    calls = registers.calls.copy()
    rig.request("capture")
    assert registers.calls == calls  # Faults never restore torque automatically.


def test_service_restart_reads_existing_release_without_torque_writes():
    registers = RegisterBus(enabled=False)
    adapter = PiMotorBus(SimpleNamespace(bus=registers), __file__)
    rig = Fixture()
    rig.motor.bus = adapter
    assert rig.request("poll")["arm_torque"] == "released"
    assert not registers.calls


def test_holding_outside_operating_limits_does_not_recover_automatically():
    rig = Fixture()
    rig.motor.calibrated_limits = [[-100., 100.]] * 5 + [[0., 50.]]
    rig.request("release_arm")
    rig.bus.measured[0] = 95.
    result = rig.request("capture")
    assert not result["armed"] and result["arm_torque"] == "holding"
    initial_writes = rig.writes.copy()
    for _ in range(10):
        result = rig.request("poll")
        assert not result["moving"]
    assert rig.writes == initial_writes


def test_recorded_released_shoulder_excursion_can_recapture_after_returning_in_range():
    # Compact measured-pose regression from feedback-test.json, ticks 2123/2124/2266/2874.
    # The original run latched its range fault even after the last pose returned in range.
    rig = Fixture()
    rig.motor.calibrated_limits = [[-120., 120.]] * 5 + [[0., 50.]]
    rig.motor.calibrated_limits[1] = [-105.14285714285714, 105.14285714285714]
    rig.request("release_arm")
    poses = [
        [-3.78021978021978, -104., 67.82417582417582, 74.37362637362638, -2.769230769230769, 1.5068493150684932],
        [-3.78021978021978, -106.72527472527473, 68.96703296703296, 71.82417582417582, -2.769230769230769, 1.5068493150684932],
        [-7.824175824175824, -103.2967032967033, 75.47252747252747, 64.43956043956044, -2.5054945054945055, 1.4383561643835616],
        [-4.131868131868132, -88.26373626373626, 74.50549450549451, 51.51648351648352, -2.5054945054945055, 1.8493150684931507],
    ]
    for index, pose in enumerate(poses):
        rig.bus.measured = pose.copy()
        feedback = rig.request("poll")
        assert not feedback["fault"] and feedback["arm_torque"] == "released"
        assert not feedback["moving"] and not feedback["recovery_required"]
        if index == 1:
            assert "shoulder_lift" in feedback["reason"]
            denied = rig.request("capture")
            assert denied["arm_torque"] == "released" and not rig.writes
            assert not denied["fault"]
        else:
            assert "inside calibrated" not in feedback["reason"]
    feedback = rig.request("capture")
    assert feedback["arm_torque"] == "holding" and not feedback["armed"] and not feedback["fault"]
    assert rig.writes == [poses[-1]]
    assert rig.request("arm")["armed"]


@pytest.mark.parametrize("channel_index", range(6))
def test_released_out_of_range_capture_never_writes_and_fresh_press_recovers(channel_index):
    rig = Fixture()
    rig.request("release_arm")
    rig.bus.measured[channel_index] = LIMITS[channel_index][1] + 10.
    for operation in ("poll", "capture", "disarm", "recover", "move"):
        feedback = rig.request(operation, target=[0.] * 6, input=[0.] * 6)
        assert not feedback["fault"] and not feedback["armed"] and not feedback["moving"]
        assert feedback["arm_torque"] == "released" and CHANNELS[channel_index] in feedback["reason"]
        assert not rig.writes
    rig.motor.tick(lost=True)
    rig.feedback = rig.motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "test"})
    assert not rig.feedback["fault"] and rig.feedback["arm_torque"] == "released"
    rig.bus.measured[channel_index] = (LIMITS[channel_index][0] + LIMITS[channel_index][1]) / 2
    # Returning in range only clears the warning; torque remains off until a new capture request.
    assert rig.request("poll")["arm_torque"] == "released"
    assert not rig.writes
    captured = rig.request("capture")
    assert not captured["fault"] and captured["arm_torque"] == "holding" and not captured["armed"]
    assert rig.writes == [rig.bus.measured]


@pytest.mark.parametrize("failure", ["read", "nonfinite", "powered_range"])
def test_real_faults_remain_latched_even_after_returning_in_range(failure):
    rig = Fixture()
    if failure != "powered_range":
        rig.request("release_arm")
    if failure == "read":
        read = rig.bus.read
        def fail_read():
            raise OSError("serial error")
        rig.bus.read = fail_read
        assert rig.request("poll")["fault"]
        rig.bus.read = read
    else:
        rig.bus.measured[0] = float("nan") if failure == "nonfinite" else 100.
        assert rig.request("poll")["fault"]
        rig.bus.measured[0] = 0.
    assert rig.request("capture")["fault"]
    assert not rig.writes


def test_startup_cannot_claim_released_when_only_gripper_is_still_powered():
    registers = RegisterBus(enabled=False)
    registers.torque["gripper"] = 1
    adapter = PiMotorBus(SimpleNamespace(bus=registers), __file__)
    rig = Fixture()
    rig.motor.bus = adapter
    assert rig.request("poll")["arm_torque"] == "unknown"
    assert not registers.calls


@pytest.mark.skipif(version("lerobot") != "0.6.1", reason="Remote IK targets LeRobot 0.6.1")
def test_calibrated_gripper_range_is_distinct_from_unchanged_command_range():
    # Construct only: no serial connection, configuration or calibration writes.
    follower = calibrated_follower("unused", DEFAULT_URDF.parent / "calibration")
    assert calibrated_channel_limits(follower)[5] == [0., 100.]
    assert channel_limits(follower)[5] == [0., 50.]


def test_manually_open_gripper_can_be_captured_without_closing_until_control_enabled():
    rig = Fixture()
    rig.motor.calibrated_limits = [*LIMITS[:5], [0., 100.]]
    rig.request("release_arm")
    rig.bus.measured[5] = 80.
    captured = rig.request("capture")
    assert not captured["fault"] and not captured["armed"]
    assert captured["sent"][5] == 80.
    writes = rig.writes.copy()
    for _ in range(5):
        assert not rig.request("poll")["moving"]
    assert rig.writes == writes
    assert rig.request("arm")["armed"]
    recovered = rig.request("recover", input=[0.] * 6)
    assert 50. < recovered["sent"][5] < 80.


@pytest.mark.parametrize("index", range(5))
@pytest.mark.parametrize("sign", [-1., 1.])
def test_explicit_near_boundary_capture_holds_legal_goal_then_recovers(index, sign):
    rig = Fixture()
    rig.request("release_arm")
    raw = sign * 90.2
    rig.bus.measured[index] = raw
    captured = rig.request("capture")
    assert captured["arm_torque"] == "holding" and not captured["armed"] and not captured["fault"]
    assert captured["boundary_capture"]
    assert captured["measured"][index] == raw  # Feedback must not be clamped or hidden.
    assert captured["sent"][index] == sign * 90.
    assert rig.writes[-1][index] == sign * 90.
    # A loaded servo may not immediately pull the measured joint inside calibration.
    rig.bus.measured[index] = raw
    count = len(rig.writes)
    for _ in range(20):
        holding = rig.request("poll")
        assert not holding["fault"] and not holding["moving"] and not holding["armed"]
    assert len(rig.writes) == count
    armed = rig.request("arm")
    assert armed["armed"] and not armed["fault"] and len(rig.writes) == count
    for _ in range(80):
        rig.now += .02
        recovered = rig.request("recover", input=[0.] * 6)
        assert not recovered["fault"]
        if not recovered["recovery_required"]:
            break
    assert not recovered["recovery_required"] and not recovered["boundary_capture"]
    assert recovered["recovery_holding"] and abs(recovered["measured"][index]) <= 89.
    assert all(all(lo <= q <= hi for q, (lo, hi) in zip(target, LIMITS)) for target in rig.writes)
    rig.bus.measured[index] = raw
    assert rig.request("poll")["fault"]  # The capture exception ends after successful recovery.


def test_recorded_retracted_shoulder_capture_no_longer_gets_stuck_released():
    rig = Fixture()
    rig.motor.calibrated_limits = [[-120., 120.]] * 5 + [[0., 50.]]
    rig.motor.calibrated_limits[1] = [-105.14285714285714, 105.14285714285714]
    rig.request("release_arm")
    # Latest feedback-test.json: blocked capture near tick 2762, one encoder tick outside.
    pose = [-11.516483516483516, -105.23076923076923, 96.74725274725274,
            48.879120879120876, 2.769230769230769, 1.7808219178082192]
    rig.bus.measured = pose.copy()
    # The recorded elbow lies outside this fixture's operating range, but within calibration.
    captured = rig.request("capture")
    assert not captured["fault"] and captured["arm_torque"] == "holding"
    assert captured["measured"] == pose
    assert captured["sent"][1] == rig.motor.calibrated_limits[1][0]
    assert captured["sent"][2] == pose[2]
    rig.bus.measured = pose.copy()
    rig.request("arm")
    for _ in range(200):
        rig.now += .02
        result = rig.request("recover", input=[0.] * 6)
        assert not result["fault"]
        if not result["recovery_required"]:
            break
    assert not result["recovery_required"] and result["recovery_holding"]


@pytest.mark.parametrize("kind", ["large", "gripper", "new_joint", "opposite_side", "grows"])
def test_boundary_capture_does_not_relax_general_calibration_or_fault_guards(kind):
    rig = Fixture()
    rig.request("release_arm")
    if kind in ("large", "gripper"):
        index, value = (0, 91.01) if kind == "large" else (5, 50.01)
        rig.bus.measured[index] = value
        denied = rig.request("capture")
        assert denied["arm_torque"] == "released" and not denied["fault"] and not rig.writes
        return
    rig.bus.measured[0] = 90.2
    rig.request("capture")
    count = len(rig.writes)
    if kind == "new_joint":
        rig.bus.measured[1] = 90.2
    elif kind == "opposite_side":
        rig.bus.measured[0] = -90.2
    else:
        rig.bus.measured[0] = 91.01
    fault = rig.request("poll")
    assert fault["fault"] and not fault["armed"]
    assert len(rig.writes) == count


@pytest.mark.parametrize("operation", ["disarm", "lost", "watchdog"])
def test_boundary_capture_stops_recovery_with_a_legal_hold_and_requires_rearm(operation):
    rig = Fixture()
    rig.request("release_arm")
    rig.bus.measured[0] = 90.2
    rig.request("capture")
    rig.request("arm")
    rig.request("recover", input=[0.] * 6)
    rig.bus.measured[0] = 90.2
    if operation == "lost":
        rig.motor.tick(lost=True)
        rig.feedback = rig.motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "test"})
    elif operation == "watchdog":
        rig.now += .21
        rig.request("poll")
    else:
        rig.request("disarm")
    assert not rig.feedback["armed"] and not rig.feedback["moving"] and not rig.feedback["fault"]
    assert rig.feedback["sent"][0] == 90.
    count = len(rig.writes)
    rig.request("poll")
    assert len(rig.writes) == count


@pytest.mark.parametrize("feedback", [dict(), dict(arm_torque="invalid"),
                                      dict(arm_torque="released", armed=True),
                                      dict(arm_torque="unknown", moving=True)])
def test_link_rejects_missing_or_inconsistent_torque_state(feedback):
    with pytest.raises(ValueError, match="torque state"):
        MotorLink.validate_torque_feedback(feedback)


def prepare(control, state, feedback, latest, seen, now):
    sample = latest.sample
    result = control._prepare_input(state, feedback, sample, seen, arm_interrupted=False,
                                    fresh=sample.gamepad is not None and now - sample.sampled_at < .1,
                                    elapsed=.02, period=.02, started=now)
    return result[0], sample


class Gesture:
    def __init__(self, *, armed=False):
        self.now = 0.
        self.latest = LatestInput(clock=lambda: self.now)
        self.latest.publish(pad())
        self.seen = self.latest.sample
        self.state = MotionState()
        self.feedback = dict(armed=armed, moving=armed, arm_torque="holding", fault=False)
        self.control = ControlLoop(None, None, self.latest, Event(), 1., "Sim", clock=lambda: self.now)

    def step(self, **buttons):
        self.now += .02
        self.latest.publish(pad(**buttons))
        op, self.seen = prepare(self.control, self.state, self.feedback, self.latest, self.seen, self.now)
        if op == "disarm":
            self.feedback.update(armed=False, moving=False)
        elif op == "release_arm":
            self.feedback["arm_torque"] = "released"
        return op

    def hold(self, seconds, **buttons):
        return [self.step(**buttons) for _ in range(round(seconds / .02))]


def test_release_requires_full_two_seconds_and_only_fires_once():
    gesture = Gesture()
    assert "release_arm" not in gesture.hold(2., back=True)
    assert gesture.state.release_progress == pytest.approx(.99)
    assert gesture.step(back=True) == "release_arm"
    assert "release_arm" not in gesture.hold(1., back=True)
    assert gesture.feedback["arm_torque"] == "released"


def test_stopping_active_control_requires_a_fresh_press_before_release():
    gesture = Gesture(armed=True)
    assert gesture.step(back=True) == "disarm"
    assert "release_arm" not in gesture.hold(3., back=True)
    gesture.step()
    assert "release_arm" in gesture.hold(2.04, back=True)


@pytest.mark.parametrize("cancel", [dict(), dict(back=True, lb=True), dict(back=True, lx=.1),
                                     dict(back=True, trigger=.1), dict(back=True, dpad=(0, 1)),
                                     dict(back=True, start=True)])
def test_cancelled_gesture_never_resumes_without_a_fresh_press(cancel):
    gesture = Gesture()
    gesture.hold(1., back=True)
    gesture.step(**cancel)
    # If the button was released, its next press can start a new full countdown.
    operations = gesture.hold(1.2, back=True)
    assert "release_arm" not in operations
    if cancel.get("back"):
        assert gesture.state.release_progress == 0.
        assert "release_arm" not in gesture.hold(2., back=True)


@pytest.mark.parametrize("disconnect", [False, True])
def test_input_loss_cancels_release_timer(disconnect):
    gesture = Gesture()
    gesture.hold(1., back=True)
    if disconnect:
        gesture.latest.publish(None)
    else:
        gesture.now += .2
    gesture.step(back=True)
    assert "release_arm" not in gesture.hold(3., back=True)
    assert gesture.state.release_progress == 0.


def test_reconnect_with_stop_button_held_cannot_start_countdown():
    gesture = Gesture()
    gesture.latest.publish(pad(back=True))
    gesture.seen = gesture.latest.sample  # New session consumes existing input edges.
    assert "release_arm" not in gesture.hold(3., back=True)


def test_start_capture_and_enable_require_separate_presses():
    gesture = Gesture()
    gesture.feedback["arm_torque"] = "released"
    assert gesture.step(start=True) == "capture"
    gesture.feedback["arm_torque"] = "holding"
    assert all(op == "poll" for op in gesture.hold(.5, start=True))
    gesture.step()
    assert gesture.step(start=True) == "arm"


def test_configurable_physical_mapping_preserves_raw_dashboard_input():
    latest = LatestInput(advance_button="Back", stop_button="Start")
    latest.publish(pad())
    physical = pad(start=True)
    latest.publish(physical)
    assert latest.sample.gamepad["buttons"]["Back"]
    assert not latest.sample.gamepad["buttons"]["Start"]
    assert latest.sample.start == 0 and latest.sample.stop == 1
    assert latest.raw_gamepad is physical and physical["buttons"]["Start"]


def test_stop_arriving_during_ik_cannot_be_reused_to_release():
    rig = Fixture()
    rig.request("arm")
    latest = LatestInput(clock=lambda: rig.now)
    latest.publish(pad(lb=True, lx=.5))
    stop = Event()
    operations = []

    def solve(*args, **kwargs):
        if not operations:
            latest.publish(pad(back=True))
        return {"target": [1.] * 6, "accepted_xyz_m": [0.] * 3}

    def request(op, **values):
        operations.append(op)
        feedback = rig.request(op, **values)
        rig.now += .02
        latest.publish(pad(back=True))
        if len(operations) >= 110:
            stop.set()
        return feedback

    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, 1., "Sim", clock=lambda: rig.now)
    control.session(SimpleNamespace(feedback=rig.feedback, request=request))
    assert operations[0] == "disarm"
    assert "release_arm" not in operations
    assert control.status["state"] == "Holding"


def test_full_control_loop_release_capture_and_enable_discard_old_targets():
    rig = Fixture()
    latest = LatestInput(clock=lambda: rig.now)
    latest.publish(pad())
    stop = Event()
    operations, states = [], []
    new_pose = [10., 20., 30., 40., 5., 20.]

    def solve(measured, delta, **kwargs):
        assert not any(delta)
        # No Released/captured reference may reuse the old commanded pose.
        assert kwargs["desired_xyz_m"] is None
        return {"xyz_m": [0.] * 3, "baseline_xyz_m": [0.] * 3}

    def request(op, **values):
        operations.append(op)
        result = rig.request(op, **values)
        states.append((result["arm_torque"], result["armed"]))
        rig.now += .02
        tick = len(operations)
        if tick <= 102:
            latest.publish(pad(back=True))
        elif tick == 103:
            rig.bus.measured = new_pose.copy()
            latest.publish(pad())
        elif tick in (104, 107):
            latest.publish(pad(start=True))
        else:
            latest.publish(pad())
        if tick == 108:
            stop.set()
        return result

    control = ControlLoop(SimpleNamespace(solve=solve), None, latest, stop, 1., "Sim", clock=lambda: rig.now)
    control.session(SimpleNamespace(feedback=rig.feedback, request=request))
    assert operations.count("release_arm") == 1
    assert operations.count("capture") == 1
    assert operations.count("arm") == 1
    assert operations.index("release_arm") < operations.index("capture") < operations.index("arm")
    assert rig.writes == [new_pose, new_pose]
    assert ("released", False) in states and ("holding", False) in states
    assert control.status["control_state"] == "Control enabled"


def test_stop_during_request_whose_watchdog_disarms_cannot_start_a_release():
    rig = Fixture()
    rig.request("arm")
    latest = LatestInput(clock=lambda: rig.now)
    latest.publish(pad())
    stop = Event()
    operations = []

    def request(op, **values):
        if not operations:
            rig.motor.hold(disarm=True)
            latest.publish(pad(back=True))
        operations.append(op)
        feedback = rig.request(op, **values)
        rig.now += .02
        latest.publish(pad(back=True))
        if len(operations) >= 110:
            stop.set()
        return feedback

    control = ControlLoop(SimpleNamespace(solve=lambda *args, **kwargs: {}), None,
                          latest, stop, 1., "Sim", clock=lambda: rig.now)
    control.session(SimpleNamespace(feedback=rig.feedback, request=request))
    assert "release_arm" not in operations
    assert control.status["state"] == "Holding"
