"""Remote input and motion orchestration, independent of Windows app startup."""

from contextlib import suppress
from dataclasses import dataclass
from math import isclose
import subprocess
from time import perf_counter as monotonic

from robot_program.remote_ik_correction import StationaryCorrection
from robot_program.remote_ik_protocol import RATE_HZ, finite_vector

ARM_START_RAMP_S = 0.3


@dataclass
class MotionState:
    desired_xyz_m: list | None = None
    arm_started: float | None = None
    correction: StationaryCorrection | None = None
    was_correcting: bool = False


@dataclass(frozen=True)
class GamepadSample:
    sampled_at: float
    gamepad: dict | None
    start: int = 0
    stop: int = 0
    release: int = 0
    disconnect: int = 0
    arm_idle: int = 0

    @property
    def lb(self):
        return bool(self.gamepad and self.gamepad["buttons"]["LB"])

    @property
    def neutral(self):
        return self.gamepad is not None and not any(self.gamepad["axes"].values()) and self.gamepad["dpad"] == (0, 0)


def motion_delta(gamepad, scale=1.):
    axes = gamepad["axes"]
    return [scale * value for value in (-.002 * axes["LY"], -.002 * axes["LX"], -.002 * axes["RY"],
                                        .4 * (axes["RT"] - axes["LT"]), .75 * axes["RX"],
                                        gamepad["dpad"][1])]


class LatestInput:
    def __init__(self, *, clock=monotonic):
        self.clock = clock
        self.sample = GamepadSample(self.clock(), None)

    def publish(self, gamepad):
        previous = self.sample
        before = previous.gamepad["buttons"] if previous.gamepad else {}
        after = gamepad["buttons"] if gamepad else {}
        start = (bool(before) and not before["Start"] and after.get("Start", False)
                 and not after["LB"] and not after["Back"]
                 and not any(gamepad["axes"].values()) and gamepad["dpad"] == (0, 0))
        # Retain a neutral edge even if another tap arrives before the next IK tick.
        arm_centered = (previous.gamepad is not None and gamepad is not None
                        and any(motion_delta(previous.gamepad)[:5])
                        and not any(motion_delta(gamepad)[:5]))
        self.sample = GamepadSample(
            self.clock(), gamepad,
            previous.start + start,
            previous.stop + (not before.get("Back", False) and after.get("Back", False)),
            previous.release + (previous.lb and not after.get("LB", False)),
            previous.disconnect + (previous.gamepad is not None and gamepad is None),
            previous.arm_idle + arm_centered,
        )


def issued_arm_matches(issued, target, *, rel_tol=0.):
    """Compare the five arm channels; the gripper is independent of XYZ."""
    return (finite_vector(issued, 6) and finite_vector(target, 6)
            and all(isclose(a, b, abs_tol=1e-8, rel_tol=rel_tol)
                    for a, b in zip(issued[:5], target[:5])))


class ControlLoop:
    def __init__(self, worker, connect, latest_input, stop, speed, mode, *, trace=None,
                 correction_profile=None, clock=monotonic):
        self.worker, self.connect, self.latest_input, self.stop = worker, connect, latest_input, stop
        self.speed, self.mode = speed, mode
        self.clock = clock
        self.trace = trace
        self.correction_profile = correction_profile
        self.status = {"connection": "Connecting", "state": "Disarmed", "reason": "", "mode": mode}
        self.error = None

    def run(self):
        while not self.stop.is_set():
            try:
                with self.connect(self.worker.metadata) as link:
                    try:
                        self.session(link)
                    finally:
                        with suppress(OSError, ValueError):
                            link.request("disarm")
            except (OSError, ValueError, subprocess.SubprocessError) as exc:
                self.status = {"connection": "Disconnected", "state": "Disarmed", "reason": str(exc),
                               "mode": self.mode}
                self.stop.wait(0.25)
            except Exception as exc:
                self.error = exc
                self.stop.set()

    def _prepare_input(self, state, feedback, sample, seen, *, arm_interrupted, fresh, elapsed, period, started):
        delta = [0.] * 6
        sent = None
        op = "poll"
        ramp_scale = None
        if (not fresh or sample.disconnect != seen.disconnect or sample.stop != seen.stop
                or sample.gamepad["buttons"]["Back"]):
            op = "disarm" if feedback["armed"] else "poll"
        elif feedback["armed"] and (not sample.lb or sample.release != seen.release
                or (state.arm_started is not None and arm_interrupted)):
            op = "hold" if feedback["moving"] else "poll"
        elif feedback["armed"] and sample.lb:
            if finite_vector(feedback["sent"], 6):
                sent = feedback["sent"]
            delta = motion_delta(sample.gamepad, self.speed * RATE_HZ * (elapsed if sent is not None else period))
            op = "hold" if feedback["moving"] else "poll"
        elif sample.start != seen.start and sample.neutral and not sample.lb:
            op = "arm"
        if arm_interrupted:
            state.arm_started = None

        arm_input = any(delta[:5])
        if arm_input:
            progress = 0. if state.arm_started is None else min(1., (started - state.arm_started) / ARM_START_RAMP_S)
            ramp_scale = .1 + .9 * progress * progress * (3. - 2. * progress)
            delta[:5] = [value * ramp_scale for value in delta[:5]]
        return op, delta, sent, arm_input, ramp_scale

    def _select_correction(self, state, feedback, sample, result, *, op, delta, arm_input,
                           fresh, started, period, joint_step_limits, measured):
        correcting = False
        if state.correction is not None:
            eligible = (op == "poll" and feedback["armed"] and fresh and sample.lb
                        and not any(delta) and state.desired_xyz_m is not None and result.get("xyz_m") is not None)
            if not eligible:
                # Ordinary LB release pauses correction; only operator motion or
                # a session reset grants a fresh bounded correction episode.
                if arm_input or not feedback["armed"]:
                    state.correction.reset()
                else:
                    state.correction.since = None
            elif state.correction.ready(started, result["xyz_m"], state.desired_xyz_m):
                correcting = True
                recovery_step_m = min(.001, state.correction.profile.speed_m_s * period,
                                      state.correction.profile.travel_m - state.correction.travel)
                result = self.worker.solve(
                    measured, delta, sent=feedback["sent"], desired_xyz_m=state.desired_xyz_m,
                    joint_step_limits=joint_step_limits, correction=correcting, recovery_step_m=recovery_step_m)
        if state.was_correcting and not correcting and feedback["moving"] and not any(delta) and op != "disarm":
            op = "hold"
        return op, result, correcting

    def _validate_input(self, op, result, sample, current, *, arm_input, correcting, freshness):
        current_delta = motion_delta(current.gamepad) if current.gamepad is not None else [0.] * 6
        if (self.stop.is_set() or current.gamepad is None or current.stop != sample.stop
                or current.disconnect != sample.disconnect
                or self.clock() - current.sampled_at >= freshness):
            op = "disarm"
        elif (result.get("target") is not None and current.lb and current.release == sample.release
              and self.clock() - sample.sampled_at < freshness
              and (not arm_input or current.arm_idle == sample.arm_idle)
              and ((not any(current_delta) and current.arm_idle == sample.arm_idle) if correcting
                   else (any(current_delta[:5]) if arm_input else current_delta[5]))):
            op = "move"
        elif op == "arm" and (current.lb or not current.neutral or current.gamepad["buttons"]["Back"]):
            op = "poll"

        if op == "move" and (arm_input or correcting) and not finite_vector(result.get("accepted_xyz_m"), 3):
            raise ValueError("Worker arm result requires finite accepted_xyz_m")
        return op

    def _acknowledge(self, state, feedback, result, *, op, correcting, arm_input, started):
        reset_reason = None
        exact_match = op == "move" and issued_arm_matches(feedback.get("sent"), result.get("target"))
        if correcting and op == "move" and result.get("candidate_xyz_m") is not None:
            # Count actual issued movement only when Pi confirms the exact arm target.
            if issued_arm_matches(feedback.get("sent"), result.get("target"), rel_tol=1e-9):
                state.correction.issued(result.get("issued_xyz_m", result["xyz_m"]), result["candidate_xyz_m"])
        issuance_diverged = op == "move" and finite_vector(feedback.get("sent"), 6) and not exact_match
        state.was_correcting = correcting and op == "move" and feedback["moving"]
        if arm_input and op == "move" and feedback["moving"] and feedback["armed"] and not feedback["fault"]:
            if state.arm_started is None:
                state.arm_started = started
        else:
            state.arm_started = None
        if not feedback["armed"] or feedback["fault"] or op == "disarm":
            if state.desired_xyz_m is not None:
                reset_reason = "disarmed or fault"
            state.desired_xyz_m = None
        else:
            if state.desired_xyz_m is None:
                state.desired_xyz_m = result.get("baseline_xyz_m")
                if state.desired_xyz_m is not None:
                    reset_reason = "fresh arm reference"
            # The Pi can shorten a move. Only advance XYZ for a fully issued arm target.
            if exact_match and feedback["moving"] and arm_input:
                state.desired_xyz_m = result["accepted_xyz_m"]
        return reset_reason, issuance_diverged

    def _record_trace(self, state, sample, feedback, result, *, started, received, op, ramp_scale,
                      delta, desired_before, reset_reason, ik_reason, issued_before, solve_finished,
                      issuance_diverged, correcting):
        if self.trace is not None:
            self.trace.append({
                "tick_s": started, "ack_s": received, "ack": feedback.get("ack"), "op": op,
                "input": sample.gamepad, "arm_idle": sample.arm_idle, "arm_scale": ramp_scale, "delta": delta,
                "desired_before_m": desired_before, "desired_xyz_m": state.desired_xyz_m, "xyz_reset": reset_reason,
                "requested_xyz_m": result.get("requested_xyz_m"), "ik_reason": ik_reason,
                "candidate": result.get("target"), "issued_before": issued_before, "issued": feedback["sent"],
                "candidate_xyz_m": result.get("candidate_xyz_m"),
                "progress_fraction": result.get("progress_fraction"), "recovering": result.get("recovering"),
                "session": feedback.get("session"), "feedback_seq": feedback.get("feedback_seq"),
                "solve_finished_s": solve_finished, "issuance_diverged": issuance_diverged,
                "rate_limited": feedback.get("rate_limited"), "sampled_before_command": True,
                "correction": correcting, "correction_exhausted": state.correction.exhausted if state.correction else False,
                "input_edges": {"neutral": sample.arm_idle, "release": sample.release},
                "measured": feedback["measured"], "pi_reason": feedback["reason"],
            })

    def _publish_status(self, state, sample, feedback, result, *, issuance_diverged, fresh, mode,
                        measured, feedback_count, received, connected_at, max_gap_ms):
        reason = ("Pi shortened the validated Cartesian target" if issuance_diverged
                  else feedback["reason"] or result.get("reason", ""))
        if not fresh and not reason:
            reason = "Connect a gamepad" if sample.gamepad is None else "Gamepad sample expired"
        display_state = ("Fault" if feedback["fault"] else "Moving" if feedback["moving"]
                         else "Armed" if feedback["armed"] else "Disarmed")
        self.status = {"connection": "Connected", "mode": mode, "state": display_state,
                       "reason": reason, "measured": measured, "sent": feedback["sent"],
                       "xyz_m": result.get("xyz_m"), "desired_xyz_m": state.desired_xyz_m,
                       "requested_xyz_m": result.get("requested_xyz_m"),
                       "candidate": result.get("target"),
                       "read_max_ms": feedback.get("read_max_ms"), "write_max_ms": feedback.get("write_max_ms"),
                       "feedback_hz": feedback_count / max(received - connected_at, 1e-6),
                       "feedback_max_gap_ms": max_gap_ms}

    def session(self, link):
        seen = self.latest_input.sample
        feedback = link.feedback
        mode = self.mode
        if not feedback.get("simulated", True):
            mode = "Real arm enabled" if feedback["motion_enabled"] else "Observe only"
        period = 1 / feedback["hz"]
        freshness = min(.1, feedback["watchdog_s"] / 2)
        connected_at = last_feedback = self.clock()
        previous_tick = connected_at
        feedback_count = 0
        max_gap_ms = 0.
        state = MotionState(correction=StationaryCorrection(self.correction_profile) if self.correction_profile else None)
        joint_step_limits = ([rate * period for rate in feedback["rates"]]
                             if finite_vector(feedback.get("rates"), 6) else None)
        while not self.stop.is_set():
            started = self.clock()
            # Use the command cadence, with at most two Pi cycles of input per
            # step so a slow round trip cannot turn into a large catch-up move.
            elapsed = min(2 * period, max(period, started - previous_tick))
            previous_tick = started
            sample = self.latest_input.sample
            arm_interrupted = sample.arm_idle != seen.arm_idle
            measured = feedback.get("measured")
            fresh = sample.gamepad is not None and started - sample.sampled_at < freshness
            desired_before, issued_before = state.desired_xyz_m, feedback["sent"]
            op, delta, sent, arm_input, ramp_scale = self._prepare_input(
                state, feedback, sample, seen, arm_interrupted=arm_interrupted,
                fresh=fresh, elapsed=elapsed, period=period, started=started)
            seen = sample
            result = {}
            correcting = False
            recovery_step_m = .001
            if state.was_correcting and fresh and sample.lb and not any(delta) and op != "disarm":
                op = "poll"
            if finite_vector(measured, 6) and (op in ("poll", "arm") or any(delta)):
                result = self.worker.solve(measured, delta, sent=sent if any(delta) else None,
                                           desired_xyz_m=state.desired_xyz_m, joint_step_limits=joint_step_limits,
                                           correction=correcting, recovery_step_m=recovery_step_m)
            op, result, correcting = self._select_correction(
                state, feedback, sample, result, op=op, delta=delta, arm_input=arm_input,
                fresh=fresh, started=started, period=period, joint_step_limits=joint_step_limits, measured=measured)
            # Recheck input after IK. A release and re-press while solving also
            # invalidates this candidate, even when LB is held again now.
            current = self.latest_input.sample
            op = self._validate_input(op, result, sample, current, arm_input=arm_input,
                                      correcting=correcting, freshness=freshness)

            values = {"target": result["target"]} if op == "move" else {}
            ik_reason = result.get("reason")
            solve_finished = self.clock()
            feedback = link.request(op, **values)
            reset_reason, issuance_diverged = self._acknowledge(
                state, feedback, result, op=op, correcting=correcting, arm_input=arm_input, started=started)
            received = self.clock()
            self._record_trace(state, sample, feedback, result, started=started, received=received,
                               op=op, ramp_scale=ramp_scale, delta=delta, desired_before=desired_before,
                               reset_reason=reset_reason, ik_reason=ik_reason, issued_before=issued_before,
                               solve_finished=solve_finished, issuance_diverged=issuance_diverged, correcting=correcting)
            feedback_count += 1
            max_gap_ms = max(max_gap_ms, (received - last_feedback) * 1000)
            last_feedback = received
            self._publish_status(state, sample, feedback, result, issuance_diverged=issuance_diverged,
                                 fresh=fresh, mode=mode, measured=measured, feedback_count=feedback_count,
                                 received=received, connected_at=connected_at, max_gap_ms=max_gap_ms)
