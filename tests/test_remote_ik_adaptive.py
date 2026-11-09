"""Offline adaptive motion and Pi recovery contracts. No serial or SSH."""
from threading import Event
from importlib.metadata import version
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock

import numpy as np
import pytest

from robot_program.lerobot_ik_controller import ARM_JOINTS, LeRobotPoseSolver
from robot_program.remote_ik_control import ControlLoop, LatestInput
from robot_program.remote_ik_protocol import (PROTOCOL_VERSION, calibrated_follower, channel_limits,
                                            command_limits, kinematic_urdf, limit_excess)
from scripts.remote_ik_motor_service import MotorSession
from scripts.remote_ik_worker import adapt_translation, solve_intent, solve_recovery


class AdaptiveSolver:
    def __init__(self, mode):
        self.mode, self.calls = mode, []
        self.limits_deg = np.array([[-89.5, 89.5]] * 5)
        self.kinematics = self

    def forward_kinematics(self, q):
        pose = np.eye(4)
        pose[:3, 3] = np.asarray(q[:3]) / 1000
        return pose

    def solve_pose(self, target, seed, *, position_only=False, approximate=False, **kwargs):
        self.calls.append((position_only, approximate))
        q = seed.copy()
        movement = target[:3, 3] - self.forward_kinematics(seed)[:3, 3]
        accepted = position_only and (self.mode == "pitch" or approximate)
        if self.mode == "slide":
            movement = movement.copy()
            movement[0] += .5 * movement[2]
            movement[2] *= .5
        elif self.mode == "backward":
            movement *= -1
        elif self.mode == "sideways":
            movement = np.array([movement[2], 0., 0.])
        elif self.mode == "overshoot":
            movement *= 2
        elif self.mode == "blocked":
            movement *= 0
        elif self.mode == "invalid":
            movement[:] = np.nan
        q[:3] += movement * 1000
        q[3] += .01
        return SimpleNamespace(accepted=accepted, reason="unreachable", joints_deg=q,
                               pose=self.forward_kinematics(q))


def solve(solver, *, measured=None, delta=None, desired=None, sent=None, steps=None):
    measured = [0.] * 5 + [10.] if measured is None else measured
    return solve_intent(solver, measured, [0., 0., .002, 0., 0., 0.] if delta is None else delta,
                        [0, 50], sent=measured if sent is None else sent, desired_xyz_m=desired,
                        joint_step_limits=[4.] * 6 if steps is None else steps)


@pytest.mark.parametrize("mode,status", [("pitch", "Adapting pitch"), ("slide", "Adapting path")])
def test_adaptation_uses_achieved_fk_and_bounded_forward_progress(mode, status):
    solver = AdaptiveSolver(mode)
    result = solve(solver)
    movement = np.array(result["candidate_xyz_m"])
    assert result["motion_status"] == status
    assert result["accepted_xyz_m"] == result["candidate_xyz_m"]
    assert np.linalg.norm(movement) <= .002 + 1e-8
    assert movement[2] >= np.linalg.norm(movement) * np.sqrt(.5) - 1e-10
    assert max(abs(np.array(result["target"][:5]))) <= 4.
    assert len(solver.calls) <= 35


@pytest.mark.parametrize("mode", ["backward", "sideways", "blocked", "invalid"])
def test_no_legal_forward_motion_remains_constrained(mode):
    result = solve(AdaptiveSolver(mode))
    assert result["target"] is None
    assert result["motion_status"] == "Armed—constrained"


def test_adaptation_rechecks_rates_and_discards_abandoned_line():
    solver = AdaptiveSolver("slide")
    first = solve(solver, desired=[0., 0., .1], steps=[.2] * 6)
    assert first["target"] is not None
    assert max(abs(np.array(first["target"][:5]))) <= .2 + 1e-8
    assert first["accepted_xyz_m"][2] < .001
    second = solve(solver, measured=first["target"], sent=first["target"],
                   desired=first["accepted_xyz_m"], delta=[0., 0., -.002, 0., 0., 0.])
    assert second["target"] is not None
    assert second["candidate_xyz_m"][2] < first["candidate_xyz_m"][2]
    assert second["accepted_xyz_m"] == second["candidate_xyz_m"]


def test_strict_solver_default_rejects_approximation_and_failure_is_not_a_candidate():
    kin = SimpleNamespace(joint_names=list(ARM_JOINTS), robot=SimpleNamespace(set_joint_limits=Mock()),
                          solver=SimpleNamespace(mask_dof=Mock(), enable_joint_limits=Mock()))
    kin.forward_kinematics = AdaptiveSolver("pitch").forward_kinematics
    kin.inverse_kinematics = Mock(return_value=np.array([0., 0., 1., 0., 0.]))
    solver = LeRobotPoseSolver([[-90, 90]] * 5, kinematics=kin)
    target = np.eye(4)
    target[2, 3] = .002
    strict = solver.solve_pose(target, np.zeros(5))
    assert not strict.accepted
    assert strict.joints_deg.tolist() == [0.] * 5
    approximate = solver.solve_pose(target, np.zeros(5), position_only=True, approximate=True, max_iterations=4)
    assert approximate.accepted
    assert approximate.pose[2, 3] == .001
    assert kin.inverse_kinematics.call_args.kwargs["orientation_weight"] == 0
    kin.inverse_kinematics.side_effect = RuntimeError("model failure")
    assert not solver.solve_pose(target, np.zeros(5), approximate=True).accepted


class Bus:
    def __init__(self):
        self.measured = [0.] * 6
        self.writes = []

    def read(self):
        return self.measured.copy()

    def write(self, target):
        self.writes.append(target.copy())
        return target.copy()

    arm = write


def rig():
    now, bus = [0.], Bus()
    limits = [[-90., 90.]] * 5 + [[0., 50.]]
    limits[3] = [-94.9998, 94.9998]
    motor = MotorSession(bus, limits=limits, calibrated_limits=[[-120., 120.]] * 5 + [[0., 50.]],
                         model="fixture", token="test", watchdog=.2, hz=50, arm_rate=200,
                         gripper_rate=30, clock=lambda: now[0])
    feedback = [motor.tick({"v": PROTOCOL_VERSION, "op": "hello", "token": "test"})]

    def command(op, **values):
        previous = feedback[0]
        now[0] += .02
        feedback[0] = motor.tick({"v": PROTOCOL_VERSION, "op": op, "seq": previous["ack"] + 1,
                                  "session": previous["session"], "feedback_seq": previous["feedback_seq"], **values})
        return feedback[0]
    command("arm")
    return motor, bus, now, feedback, command


@pytest.mark.parametrize("sign,other", [(1, False), (-1, False), (1, True)])
def test_reported_wrist_excursion_recovers_inward_without_disarming(sign, other):
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = sign * 95.1209
    if other:
        bus.measured[1] = -90.4
    fb = command("poll")
    assert fb["recovery_required"] and fb["armed"]
    assert fb["limit_excess"][3] == pytest.approx(.1211)
    previous_excess = sum(fb["limit_excess"])
    goal = None
    for _ in range(120):
        fb = command("recover", input=[0., 0., 1., 0., 0., 0.])
        assert fb["armed"] and not fb["fault"]
        if fb["recovery_goal"]:
            goal = goal or fb["recovery_goal"]
            assert fb["recovery_goal"] == goal
            assert abs(fb["sent"][3]) <= abs(bus.measured[3])
        bus.measured = fb["sent"].copy()
        excess = sum(limit_excess(bus.measured, motor.limits))
        assert excess <= previous_excess + 1e-8
        previous_excess = excess
        if not fb["recovery_required"]:
            break
    assert not fb["recovery_required"]
    assert abs(bus.measured[3]) <= 94.5998 + 1e-8
    assert goal[3] == pytest.approx(sign * 89.9998)


def test_stalled_recovery_holds_once_and_retries_only_on_progress_or_changed_input():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 100.  # Larger than 3 degrees, but inside calibration.
    for _ in range(600):
        fb = command("recover", input=[0.] * 6)
    assert fb["armed"] and fb["recovery_blocked"]
    count, goal = len(bus.writes), fb["recovery_goal"]
    for _ in range(5):
        command("recover", input=[0.] * 6)
    assert len(bus.writes) == count
    fb = command("recover", input=[0., 0., -1., 0., 0., 0.])
    assert fb["armed"] and not fb["recovery_blocked"]
    assert fb["recovery_goal"] == goal
    assert fb["sent"][3] < 100.


@pytest.mark.parametrize("operation", ["move", "bad_recover", "stale", "physical", "stop", "lost", "watchdog"])
def test_pi_independently_enforces_recovery_and_serious_stop_conditions(operation):
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.1209
    command("recover", input=[0.] * 6)
    if operation == "move":
        fb = command("move", target=[0.] * 6)
    elif operation == "bad_recover":
        fb = command("recover", input=[0.] * 6, target=[0.] * 6)
    elif operation == "stale":
        fb = command("recover", input=[0.] * 6, feedback_seq=-1)
    elif operation == "physical":
        bus.measured[3] = 121.
        count = len(bus.writes)
        fb = command("recover", input=[0.] * 6)
        assert len(bus.writes) == count
    elif operation == "stop":
        fb = command("disarm")
    elif operation == "lost":
        motor.tick(lost=True)
        assert not motor.armed
        return
    else:
        now[0] += .21
        fb = command("poll")
    assert fb["armed"] is (operation in ("move", "bad_recover"))
    assert not fb["moving"]


@pytest.mark.parametrize("lb,active", [(False, False), (False, True), (True, True)])
def test_control_loop_prioritizes_recovery_over_active_input_and_lb(lb, active):
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.1209
    command("poll")
    latest, stop, trace = LatestInput(clock=lambda: now[0]), Event(), []
    pad = {"axes": dict(LX=0., LY=0., RX=0., RY=-1. if active else 0., LT=0., RT=0.),
           "buttons": dict(Start=False, Back=False, LB=lb), "dpad": (0, 0)}
    latest.publish(pad)
    worker = SimpleNamespace(solve=Mock(side_effect=AssertionError("Recovery must not solve operator intent")))

    def request(op, **values):
        assert op == "recover"
        fb = command(op, **values)
        stop.set()
        return fb

    loop = ControlLoop(worker, None, latest, stop, 1., "Sim", trace=trace, clock=lambda: now[0])
    loop.session(SimpleNamespace(feedback=feedback[0], request=request))
    assert trace[0]["op"] == "recover"
    assert trace[0]["delta"] == [0.] * 6
    assert loop.status["state"] == "Recovering"


def test_normal_targets_reserve_half_degree_margin():
    motor, bus, now, feedback, command = rig()
    target = [0.] * 6
    target[3] = 94.8
    fb = command("move", target=target)
    assert fb["armed"] and not fb["moving"]
    assert motor.command_limits == command_limits(motor.limits)


def test_gripper_only_command_can_retain_an_arm_hold_inside_the_margin():
    motor, bus, now, feedback, command = rig()
    command("disarm")
    bus.measured[3] = 94.8
    command("arm")
    target = bus.measured.copy()
    target[5] = .5
    fb = command("move", target=target)
    assert fb["armed"] and fb["moving"]
    assert fb["sent"][3] == 94.8
    target[3] = 94.9
    fb = command("move", target=target)
    assert fb["armed"] and not fb["moving"]


def test_idle_worker_does_not_report_a_constraint():
    assert solve(AdaptiveSolver("blocked"), delta=[0.] * 6)["motion_status"] is None


def test_new_joint_excursion_does_not_recapture_the_existing_recovery_destination():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.1209
    fb = command("recover", input=[0.] * 6)
    wrist_goal = fb["recovery_goal"][3]
    bus.measured[1] = -90.2
    fb = command("recover", input=[0.] * 6)
    assert fb["armed"] and not fb["recovery_blocked"]
    assert fb["recovery_goal"][3] == wrist_goal
    assert fb["recovery_goal"][1] == -85.
    assert fb["sent"][1] > -90.2


def test_disarmed_recovery_request_never_writes():
    motor, bus, now, feedback, command = rig()
    command("disarm")
    bus.measured[3] = 95.1209
    count = len(bus.writes)
    fb = command("recover", input=[0.] * 6)
    assert not fb["armed"]
    assert len(bus.writes) == count


@pytest.mark.parametrize("tracking", [1., .4])
def test_recovery_with_quantized_lagging_feedback_stays_slow_and_completes(tracking):
    motor, bus, now, feedback, command = rig()
    motor.rates = [2.] * 5 + [30.]
    bus.measured[3] = 95.12
    last = None
    for _ in range(250):
        fb = command("recover", input=[0.] * 6)
        assert fb["armed"] and not fb["recovery_blocked"]
        if last is not None and fb["recovery_required"]:
            assert abs(fb["sent"][3] - last[3]) <= .04 + 1e-8
        last = fb["sent"].copy()
        bus.measured = [round(m + tracking * (s - m), 2) for m, s in zip(bus.measured, fb["sent"])]
        if not fb["recovery_required"]:
            break
    assert not fb["recovery_required"]
    assert motor.in_bounds(bus.measured)


@pytest.mark.parametrize("field", ["motion_policy", "v", "model", "calibrated_limits", "command_limits", "limits"])
def test_motor_link_refuses_incompatible_safety_metadata(monkeypatch, field):
    from scripts.main_teleop_remote_ik import MotorLink
    motor, bus, now, feedback, command = rig()
    fb = feedback[0]
    metadata = {name: fb[name] for name in ("v", "motion_policy", "model", "calibrated_limits", "command_limits", "limits")}
    metadata[field] = "incompatible"
    connection = MagicMock()
    connection.__enter__.return_value = connection
    monkeypatch.setattr("scripts.main_teleop_remote_ik.socket.create_connection", lambda *args: connection)
    link = MotorLink(1234, "test", metadata, True)
    monkeypatch.setattr(link, "exchange", lambda message: fb)
    with pytest.raises(RuntimeError, match="Update|differ"):
        with link:
            pytest.fail("Incompatible policy activated")
    assert connection.__exit__.called


def test_watchdog_is_not_masked_by_new_operating_limit_excursion():
    motor, bus, now, feedback, command = rig()
    command("move", target=[1.] * 6)
    bus.measured[3] = 95.1209
    now[0] += .21
    fb = command("poll")
    assert not fb["armed"] and not fb["moving"]
    assert "watchdog" in fb["reason"]


@pytest.mark.skipif(version("lerobot") != "0.6.1", reason="Real solver requires WSL LeRobot 0.6.1")
@pytest.mark.parametrize("axis,expected", [(0, "Adapting path"), (1, "Adapting pitch")])
def test_real_solver_has_useful_adaptive_escape_at_joint_limit_corner(tmp_path, axis, expected):
    from robot_program.lerobot_ik_controller import DEFAULT_URDF
    limits = command_limits(channel_limits(calibrated_follower("unused", DEFAULT_URDF.parent / "calibration")))
    path = tmp_path / "model.urdf"
    path.write_text(kinematic_urdf())
    solver = LeRobotPoseSolver(limits[:5], urdf_path=path)
    q = [limits[0][0], limits[1][1], limits[2][1], limits[3][1], limits[4][0], 3.4]
    xyz = solver.kinematics.forward_kinematics(np.array(q[:5]))[:3, 3].copy()
    delta = [0.] * 6
    delta[axis] = -.002
    result = solve_intent(solver, q, delta, limits[5], sent=q, desired_xyz_m=xyz.tolist(),
                          joint_step_limits=[4.] * 5 + [.6])
    assert result["motion_status"] == expected
    movement = np.array(result["candidate_xyz_m"]) - xyz
    assert movement[axis] < -.0001
    assert np.linalg.norm(movement) <= .002 + 1e-8
    assert -movement[axis] >= np.linalg.norm(movement) * np.sqrt(.5) - 1e-10


def test_recovery_ignores_analog_noise_but_retries_after_measured_progress():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.1209
    for _ in range(600):
        fb = command("recover", input=[0.] * 6)
    assert fb["recovery_blocked"]
    count = len(bus.writes)
    command("recover", input=[.01] * 6)
    assert len(bus.writes) == count
    bus.measured[3] -= .1
    fb = command("recover", input=[.01] * 6)
    assert not fb["recovery_blocked"] and fb["armed"]


def test_completed_recovery_uses_current_input_without_banked_cartesian_reference():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.1209
    command("poll")
    latest, stop, trace = LatestInput(clock=lambda: now[0]), Event(), []
    pad = {"axes": dict(LX=0., LY=0., RX=0., RY=-1., LT=0., RT=0.),
           "buttons": dict(Start=False, Back=False, LB=True), "dpad": (0, 0)}
    latest.publish(pad)
    seen_desired = []

    def worker_solve(measured, delta, **kwargs):
        seen_desired.append(kwargs["desired_xyz_m"])
        return dict(target=[0., 0., .1, 94.49, 0., 0.], accepted_xyz_m=[0., 0., .0001],
                    baseline_xyz_m=[0., 0., 0.], xyz_m=[0., 0., 0.], motion_status="Following path")

    def request(op, **values):
        fb = command(op, **values)
        bus.measured = fb["sent"].copy()
        latest.publish(pad)
        if op == "move":
            stop.set()
        assert len(trace) < 120
        return fb

    loop = ControlLoop(SimpleNamespace(solve=worker_solve), None, latest, stop, 1., "Sim", trace=trace,
                       clock=lambda: now[0])
    loop.session(SimpleNamespace(feedback=feedback[0], request=request))
    assert trace[-1]["op"] == "move"
    assert all(t["op"] == "recover" and not any(t["delta"]) for t in trace[:-1])
    assert seen_desired == [None]
    assert trace[-1]["arm_scale"] == .1


@pytest.mark.parametrize("sign", [1., -1.])
def test_recorded_wrist_stall_recovers_through_bounded_deeper_destination(sign):
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = sign * 95.032967
    margins = []
    previous = None
    for _ in range(200):
        fb = command("recover", input=[0.] * 6)
        margins.append(fb["recovery_margin_deg"])
        assert fb["armed"] and not fb["recovery_blocked"]
        if previous is not None:
            assert abs(fb["sent"][3] - previous) <= .2 + 1e-8
        previous = fb["sent"][3]
        # Synthetic dead zone reproduces failure to follow the observed 0.533°
        # retreat; this is a regression model, not a motor diagnosis.
        if abs(bus.measured[3] - fb["sent"][3]) >= .8:
            bus.measured[3] = fb["sent"][3]
        if not fb["recovery_required"]:
            break
    assert not fb["recovery_required"]
    assert max(margins) == 5.


def test_permanent_stall_exhausts_finite_retreat_search_and_holds_once():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.032967
    for _ in range(600):
        fb = command("recover", input=[0.] * 6)
    assert fb["recovery_blocked"] and fb["armed"]
    assert fb["recovery_margin_deg"] == 5.
    assert fb["recovery_goal"][3] == pytest.approx(89.9998)
    count = len(bus.writes)
    for _ in range(30):
        command("recover", input=[0.] * 6)
    assert len(bus.writes) == count


def test_coordinated_recovery_follows_input_without_changing_mandatory_retreat():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.032967
    fb = command("recover", input=[0., 0., 1., 0., 0., 0.])
    automatic = fb["recovery_target"]
    result = solve_recovery(AdaptiveSolver("pitch"), fb["sent"], [0., 0., .002, 0., 0., 0.],
                            automatic, fb["recovery_anchor"], fb["recovery_goal"], [.1]*6, motor.limits)
    assert result["target"] is not None
    assert result["target"][3] == automatic[3]
    assert result["target"][2] > fb["sent"][2]
    fb = command("recover", input=[0., 0., 1., 0., 0., 0.], target=result["target"])
    assert fb["recovery_coordinated"] and fb["armed"]
    elbow = fb["sent"][2]
    fb = command("recover", input=[0.] * 6)
    assert fb["sent"][2] == elbow  # Centering never returns to the old arm anchor.


@pytest.mark.parametrize("bad", ["faster_arm", "outward_wrist", "moving_jaw", "illegal_other_joint"])
def test_pi_rejects_coordinated_recovery_bypasses(bad):
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.032967
    fb = command("recover", input=[0.] * 6)
    proposed = fb["recovery_target"].copy()
    if bad == "faster_arm":
        proposed[2] += .3
    elif bad == "outward_wrist":
        proposed[3] = fb["sent"][3] + .01
    elif bad == "moving_jaw":
        proposed[5] += .01
    else:
        proposed[0] = 91.
    fb = command("recover", input=[0., 0., 1., 0., 0., 0.], target=proposed)
    assert fb["armed"] and not fb["recovery_coordinated"]
    assert fb["sent"] != proposed


@pytest.mark.parametrize("reverse", [False, True])
def test_control_loop_routes_fresh_recovery_coordination_and_discards_direction_changes(reverse):
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.032967
    command("recover", input=[0.] * 6)
    latest, stop, trace = LatestInput(clock=lambda: now[0]), Event(), []
    pad = {"axes": dict(LX=0., LY=0., RX=0., RY=-1., LT=0., RT=0.),
           "buttons": dict(Start=False, Back=False, LB=False), "dpad": (0, 0)}
    latest.publish(pad)

    def recover(measured, delta, **kwargs):
        result = solve_recovery(AdaptiveSolver("pitch"), kwargs["sent"], delta, kwargs["automatic"],
                                kwargs["anchor"], kwargs["goal"], kwargs["steps"], motor.limits)
        if reverse:
            latest.publish({**pad, "axes": {**pad["axes"], "RY": 1.}})
        return result

    def request(op, **values):
        assert op == "recover"
        assert ("target" in values) is (not reverse)
        fb = command(op, **values)
        stop.set()
        return fb

    worker = SimpleNamespace(recover=recover, solve=Mock(side_effect=AssertionError("No ordinary IK in recovery")))
    loop = ControlLoop(worker, None, latest, stop, 1., "Sim", trace=trace, clock=lambda: now[0])
    loop.session(SimpleNamespace(feedback=feedback[0], request=request))
    assert trace[0]["recovery_coordinated"] is (not reverse)


def test_integrated_recovery_unloads_stalled_wrist_through_fresh_coordinated_input():
    motor, bus, now, feedback, command = rig()
    bus.measured[3] = 95.032967
    command("poll")
    latest, stop, trace = LatestInput(clock=lambda: now[0]), Event(), []
    pad = {"axes": dict(LX=0., LY=0., RX=0., RY=-1., LT=0., RT=0.),
           "buttons": dict(Start=False, Back=False, LB=False), "dpad": (0, 0)}
    latest.publish(pad)

    def recover(measured, delta, **kwargs):
        return solve_recovery(AdaptiveSolver("pitch"), kwargs["sent"], delta, kwargs["automatic"],
                              kwargs["anchor"], kwargs["goal"], kwargs["steps"], motor.limits)

    def request(op, **values):
        assert op == "recover"
        fb = command(op, **values)
        # A synthetic load only releases the wrist after the other arm joints
        # move. Automatic wrist-only recovery cannot escape this fixture.
        for i in range(6):
            if i != 3 or fb["sent"][2] >= .2:
                bus.measured[i] = fb["sent"][i]
        latest.publish(pad)
        assert fb["armed"]
        if not fb["recovery_required"]:
            stop.set()
        assert len(trace) < 100
        return fb

    worker = SimpleNamespace(recover=recover, solve=Mock(side_effect=AssertionError("No ordinary IK in recovery")))
    loop = ControlLoop(worker, None, latest, stop, 1., "Sim", trace=trace, clock=lambda: now[0])
    loop.session(SimpleNamespace(feedback=feedback[0], request=request))
    assert any(t["recovery_coordinated"] for t in trace)
    assert motor.in_bounds(bus.measured)
    assert not motor.recovery_needed()


class LineEscapeSolver(AdaptiveSolver):
    """A locally reachable line whose vertical progress needs sideways travel."""
    def __init__(self, angle):
        super().__init__("line")
        self.line = np.array([np.sin(np.deg2rad(angle)), 0., np.cos(np.deg2rad(angle))])

    def solve_pose(self, target, seed, *, position_only=False, approximate=False, **kwargs):
        self.calls.append((position_only, approximate))
        requested = target[:3, 3] - self.forward_kinematics(seed)[:3, 3]
        movement = max(0., requested @ self.line) * self.line
        q = seed.copy()
        q[:3] += movement * 1000
        return SimpleNamespace(accepted=approximate or np.linalg.norm(movement-requested) <= .00005,
                               reason="No exact vertical path", joints_deg=q, pose=self.forward_kinematics(q))


@pytest.mark.parametrize("angle", [60., 70., 75.])
def test_wider_escape_makes_forward_progress_within_approved_step_and_angle(angle):
    solver = LineEscapeSolver(angle)
    result = solve(solver)
    assert result["target"] is not None
    assert result["motion_status"] == "Adapting path"
    assert result["path_bound_deg"] == 75.
    movement = np.array(result["candidate_xyz_m"])
    assert np.linalg.norm(movement) <= .002 + 1e-8
    assert movement[2] > .00025
    assert result["path_angle_deg"] == pytest.approx(angle)
    assert result["accepted_xyz_m"] == result["candidate_xyz_m"]
    assert len(solver.calls) <= 43


def test_useful_narrow_path_is_preferred_to_a_wider_escape():
    result = solve(LineEscapeSolver(40.))
    assert result["target"] is not None
    assert result["path_bound_deg"] == 45.
    assert result["path_angle_deg"] <= 45.


def test_narrow_useful_candidate_beats_a_closer_but_negligible_candidate():
    class NarrowPreferenceSolver(AdaptiveSolver):
        def solve_pose(self, target, seed, *, approximate=False, **kwargs):
            self.calls.append(approximate)
            q = seed.copy()
            q[:3] += np.array([.252, 0., .252]) if abs(target[0, 3]) > 1e-9 else [0., 0., .248]
            return SimpleNamespace(accepted=approximate, reason="reach", joints_deg=q,
                                   pose=self.forward_kinematics(q))
    solver = NarrowPreferenceSolver("preference")
    result = solve(solver)
    assert result["path_bound_deg"] == 45.
    assert result["candidate_xyz_m"][2] == pytest.approx(.000252)
    assert len(solver.calls) <= 35


@pytest.mark.parametrize("angle", [75.01, 80., 90., 110.])
def test_adaptation_never_accepts_beyond_75_degrees(angle):
    result = adapt_translation(LineEscapeSolver(angle), np.zeros(5), np.eye(4),
                               [0.,0.,.002,0.,0.,0.], 0., [4.]*6,
                               {"target":None, "search_attempts":9})
    assert result["target"] is None


def test_wider_escape_still_respects_pi_rate_allowance():
    result = solve(LineEscapeSolver(70.), steps=[.95]*6)
    assert result["target"] is not None
    assert result["path_bound_deg"] == 75.
    assert max(abs(np.array(result["target"][:5]))) <= .95 + 1e-8


@pytest.mark.skipif(version("lerobot") != "0.6.1", reason="Recorded pose replay requires WSL LeRobot 0.6.1")
def test_recorded_pure_z_stall_has_useful_bounded_75_degree_escape(tmp_path):
    from robot_program.lerobot_ik_controller import DEFAULT_URDF
    limits = command_limits(channel_limits(calibrated_follower("unused", DEFAULT_URDF.parent / "calibration")))
    path = tmp_path / "model.urdf"
    path.write_text(kinematic_urdf())
    solver = LeRobotPoseSolver(limits[:5], urdf_path=path)
    sent = [-2.6956948785061714, -30.766047521314526, 50.737633726667404,
            94.49984017946129, 1.0764298761276294, 1.5068493150684932]
    result = solve_intent(solver, sent, [0.,0.,.0020032500033266842,0.,-.13498461936478634,0.],
                          limits[5], sent=sent,
                          desired_xyz_m=[.09075262971990805,.0022863623097919947,.04481136636558508],
                          joint_step_limits=[4.]*5+[.6])
    assert result["target"] is not None
    movement = np.array(result["candidate_xyz_m"]) - np.array(result["issued_xyz_m"])
    assert result["path_bound_deg"] == 75.
    assert 45. < result["path_angle_deg"] <= 75.
    assert movement[2] > .0002
    assert np.linalg.norm(movement) <= .0020032500033266842 + 1e-8
