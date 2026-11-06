"""Pi-local motor owner and watchdog. Simulation is the default."""

import argparse
from contextlib import ExitStack
import hmac
import math
import os
from pathlib import Path
import secrets
import select
import signal
import socket
import subprocess
from threading import Event
from time import perf_counter as monotonic

from robot_program.lerobot_ik_controller import DEFAULT_URDF
from robot_program.remote_ik_protocol import (
    CHANNELS, MAX_FRAME, OFFLINE_WATCHDOG, PI_PORT, PROTOCOL_VERSION, RATE_HZ,
    calibrated_follower, channel_limits, decode_frame, encode_frame, finite_vector,
    kinematic_urdf, model_id,
)


class ProtocolError(ValueError):
    pass


class SimulatedBus:
    def __init__(self, limits):
        self.measured = [min(max(0., lo), hi) for lo, hi in limits]

    def read(self):
        return self.measured.copy()

    def arm(self, measured):
        return self.write(measured)

    def write(self, target):
        self.measured = target.copy()
        return self.measured.copy()


class PiMotorBus:
    def __init__(self, follower, port):
        self.follower = follower
        self.port = str(Path(port).resolve(strict=True))
        self.bus = follower.bus
        self.torque_enabled = False

    def __enter__(self):
        # The pinned Feetech SDK opens its actual pyserial descriptor in openPort.
        import fcntl
        from termios import TIOCEXCL

        handler = self.bus.port_handler
        original_open = handler.openPort

        def check_existing_owners():
            result = subprocess.run(["fuser", self.port], capture_output=True, text=True, timeout=2)
            if result.returncode not in (0, 1) or (result.returncode != 0 and result.stderr):
                raise RuntimeError(f"Could not check serial ownership of {self.port}: {result.stderr.strip()}")
            if any(int(pid) != os.getpid() for pid in result.stdout.split()):
                raise RuntimeError(f"Serial device already in use: {self.port}")

        def open_exclusive():
            if not original_open():
                return False
            fcntl.ioctl(handler.ser.fileno(), TIOCEXCL)
            check_existing_owners()
            return True

        check_existing_owners()
        handler.openPort = open_exclusive
        try:
            # Attach to the existing calibration and motor tuning without running
            # follower.connect(), whose configure() toggles torque and writes registers.
            self.bus.connect()
            if not self.follower.is_calibrated:
                raise RuntimeError("Pi calibration does not match the motors")
        except BaseException:
            self.__exit__()
            raise
        finally:
            handler.openPort = original_open
        return self

    def __exit__(self, *_):
        if self.bus.is_connected:
            self.bus.disconnect(disable_torque=False)

    def read(self):
        positions = self.bus.sync_read("Present_Position")
        return [float(positions[channel]) for channel in CHANNELS]

    def arm(self, measured):
        applied = self.write(measured)
        if not self.torque_enabled:
            self.bus.enable_torque(num_retry=1)
            self.torque_enabled = True
        return applied

    def write(self, target):
        self.bus.sync_write("Goal_Position", dict(zip(CHANNELS, target)))
        return target.copy()

    def relax(self):
        failures = []
        for channel in CHANNELS:
            try:
                self.bus.disable_torque(channel, num_retry=1)
            except Exception as exc:
                failures.append(f"{channel}: {exc}")
        if failures:
            raise RuntimeError("Could not release motor torque: " + ", ".join(failures))


class MotorSession:
    """Called only by the motor loop. Feedback timestamps are Pi-local."""

    def __init__(self, bus, *, limits, model, token, watchdog, hz, arm_rate, gripper_rate,
                 simulated=True, allow_motion=False, clock=monotonic):
        self.bus = bus
        self.limits = limits
        self.model = model
        self.token = token
        self.watchdog = watchdog
        self.period = 1 / hz
        self.rates = [arm_rate] * 5 + [gripper_rate]
        self.simulated = simulated
        self.motion_enabled = simulated or allow_motion
        self.relax_on_exit = False
        self.clock = clock
        self.read_max_ms = self.write_max_ms = None
        self.measured = self.sent = None
        self.sampled_at = self.feedback_at = self.last_move_at = float("-inf")
        self.feedback_seq = 0
        self.session = None
        self.sequence = -1
        self.armed = self.advancing = False
        self.fault = self.reason = ""

    def in_bounds(self, values):
        return (finite_vector(values, len(CHANNELS))
                and all(lo <= value <= hi for value, (lo, hi) in zip(values, self.limits)))

    def send_target(self, target):
        started = self.clock()
        sent = self.bus.write(target)
        self.write_max_ms = max(self.write_max_ms or 0., (self.clock() - started) * 1000)
        return sent

    def hold(self, reason="", *, disarm=False):
        if disarm:
            self.armed = False
        if reason:
            self.reason = reason
        if self.advancing and self.measured is not None:
            # One measured stop target. Avoid repeatedly chasing drift or sag.
            self.advancing = False
            self.sent = self.send_target(self.measured)

    def disconnect(self):
        self.session = None
        self.hold("controller disconnected", disarm=True)

    def shutdown(self, *, requested):
        self.tick(lost=True)
        if requested and self.relax_on_exit:
            self.bus.relax()

    def handle(self, request):
        if type(request.get("v")) is not int or request["v"] != PROTOCOL_VERSION:
            raise ProtocolError("Incompatible protocol version")
        op = request.get("op")
        if self.session is None:
            token = request.get("token")
            if (op != "hello" or not isinstance(token, str)
                    or not hmac.compare_digest(token.encode(), self.token.encode())):
                raise ProtocolError("Invalid credentials")
            self.session = secrets.token_hex(16)
            self.sequence = -1
            self.reason = ""
            return None

        seq = request.get("seq")
        if request.get("session") != self.session or type(seq) is not int or seq <= self.sequence:
            self.hold("wrong session or old command", disarm=True)
            return None
        self.sequence = seq
        if op == "poll":
            return None
        if op == "relax_on_exit":
            self.relax_on_exit = self.motion_enabled and not self.simulated
            if not self.relax_on_exit:
                self.reason = "Torque release requires real arm motion mode"
            return None
        if op in ("hold", "disarm"):
            self.hold(disarm=(op == "disarm"))
            return None
        if op not in ("arm", "move"):
            self.hold("unknown operation", disarm=True)
            return None
        if not self.motion_enabled:
            self.reason = "Observe only: motion is disabled on the Pi"
            return None
        age = self.clock() - self.feedback_at
        feedback_seq = request.get("feedback_seq")
        if (type(feedback_seq) is not int or feedback_seq != self.feedback_seq
                or age >= self.watchdog or self.measured is None or self.fault):
            self.hold("stale or unavailable motor feedback", disarm=True)
            return None
        if not self.in_bounds(self.measured):
            self.hold("measured pose is outside operating bounds", disarm=True)
            return None
        if op == "arm":
            if not self.armed:
                self.sent = self.bus.arm(self.measured)
                self.armed = True
                self.reason = ""
            return None
        target = request.get("target")
        if not self.armed or not self.in_bounds(target):
            self.hold("disarmed or joint target outside bounds")
            return None

        # Limit commanded velocity relative to the last issued target, independent
        # of tracking lag. A late tick cannot bank extra motion allowance.
        delta = [goal - last for goal, last in zip(target, self.sent)]
        fraction = max(1., *(abs(step) / (rate * self.period) for step, rate in zip(delta, self.rates)))
        bounded = [last + step / fraction for last, step in zip(self.sent, delta)]
        self.advancing = True
        self.last_move_at = self.clock()
        self.sent = self.send_target(bounded)
        self.reason = ""

    def tick(self, request=None, *, lost=False):
        try:
            started = self.clock()
            measured = self.bus.read()
            if not finite_vector(measured, len(CHANNELS)):
                raise ValueError("Motor bus returned incomplete or nonfinite feedback")
            self.measured, self.sampled_at = measured, self.clock()
            self.read_max_ms = max(self.read_max_ms or 0., (self.sampled_at - started) * 1000)
        except Exception as exc:
            self.measured, self.armed = None, False
            if not self.fault:
                self.fault = f"Motor read failed: {exc}"

        try:
            if lost:
                self.disconnect()
            if self.fault or self.advancing and self.clock() - self.last_move_at >= self.watchdog:
                self.hold("motion watchdog expired", disarm=True)
        except Exception as exc:
            self.armed = False
            self.fault = f"Motor hold failed: {exc}"
        if request is None:
            return None
        try:
            self.handle(request)
        except ProtocolError:
            raise
        except Exception as exc:
            self.armed = False
            self.fault = f"Motor command failed: {exc}"

        self.feedback_seq += 1
        self.feedback_at = self.sampled_at if self.measured is not None else float("-inf")
        return {"v": PROTOCOL_VERSION, "session": self.session, "ack": self.sequence,
                "feedback_seq": self.feedback_seq, "model": self.model, "limits": self.limits,
                "measured": self.measured, "sent": self.sent, "armed": self.armed,
                "moving": self.advancing, "reason": self.fault or self.reason,
                "fault": bool(self.fault),
                "simulated": self.simulated, "motion_enabled": self.motion_enabled,
                "relax_on_exit": self.relax_on_exit,
                "read_max_ms": self.read_max_ms, "write_max_ms": self.write_max_ms,
                "watchdog_s": self.watchdog, "hz": 1 / self.period}


def serve(listener, motor, stop):
    """One client and one outstanding request. All motor I/O runs on this thread."""
    client = None
    pending = None
    buffer = b""
    last_rx = next_tick = monotonic()
    lost = False

    def drop():
        nonlocal client, pending, buffer, lost
        if client is not None:
            client.close()
            client = pending = None
            buffer = b""
            lost = True

    try:
        while not stop.is_set():
            now = monotonic()
            if now >= next_tick:
                if client is not None and now - last_rx > max(0.5, motor.watchdog * 2):
                    drop()
                try:
                    response = motor.tick(pending, lost=lost)
                except ProtocolError:
                    drop()
                else:
                    lost = False
                    if response is not None:
                        try:
                            client.sendall(encode_frame(response))
                        except (OSError, ValueError):
                            drop()
                pending = None
                next_tick = max(next_tick + motor.period, monotonic())

            readers = [listener] if client is None else [listener, client]
            readable, _, _ = select.select(readers, [], [], max(0, next_tick - monotonic()))
            if client is not None and client in readable:
                try:
                    data = client.recv(MAX_FRAME + 1 - len(buffer))
                    if not data or pending is not None:
                        raise ValueError("Connection closed or commands pipelined")
                    buffer += data
                    if len(buffer) > MAX_FRAME:
                        raise ValueError("Oversized message")
                    if b"\n" in buffer:
                        pending = decode_frame(buffer)
                        buffer = b""
                        last_rx = monotonic()
                except (OSError, ValueError):
                    drop()
            if listener in readable:
                connection, _ = listener.accept()
                if client is None:
                    connection.setblocking(False)
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    client = connection
                    last_rx = monotonic()
                else:
                    connection.close()
    finally:
        drop()
        motor.shutdown(requested=stop.is_set())


def notify_ready():
    address = os.environ.get("NOTIFY_SOCKET")
    if address:
        if address.startswith("@"):
            address = "\0" + address[1:]
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as notifier:
            notifier.sendto(b"READY=1", address)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--token-file", type=Path, required=True)
    parser.add_argument("--listen-port", type=int, default=PI_PORT)
    parser.add_argument("--serial-port", help="Opt in to physical hardware using an explicit device path")
    parser.add_argument("--allow-motion", action="store_true", help="Enable supervised hardware commissioning motion")
    parser.add_argument("--calibration-dir", type=Path, default=DEFAULT_URDF.parent / "calibration")
    parser.add_argument("--hz", type=float)
    parser.add_argument("--watchdog-ms", type=float)
    parser.add_argument("--arm-rate", type=float, help="Maximum commanded arm-joint degrees per second")
    parser.add_argument("--gripper-rate", type=float, help="Maximum commanded gripper units per second")
    args = parser.parse_args()
    values = (args.hz, args.watchdog_ms, args.arm_rate, args.gripper_rate)
    if args.serial_port and Path(args.serial_port).parent != Path("/dev/serial/by-id"):
        parser.error("Hardware requires a stable /dev/serial/by-id/ device path")
    if args.allow_motion and not args.serial_port:
        parser.error("--allow-motion requires --serial-port")
    if args.allow_motion and any(value is None for value in values):
        parser.error("Hardware requires commissioned --hz, --watchdog-ms, --arm-rate and --gripper-rate")
    hz, watchdog_ms, arm_rate, gripper_rate = (
        default if value is None else value
        for value, default in zip(values, (RATE_HZ, OFFLINE_WATCHDOG * 1000, 120., 50.))
    )
    if not all(math.isfinite(value) and value > 0 for value in (hz, watchdog_ms, arm_rate, gripper_rate)):
        parser.error("Timing and rate settings must be finite and positive")
    if hz > RATE_HZ or not 2 / hz < watchdog_ms / 1000 <= 0.5:
        parser.error("Require at most 50 Hz and a watchdog longer than two cycles and at most 500 ms")
    if args.token_file.stat().st_mode & 0o077:
        parser.error("Token file must be private (chmod 600)")
    token = args.token_file.read_text(encoding="ascii").strip()
    if len(token) < 32:
        parser.error("Token must contain at least 32 characters")

    follower = calibrated_follower(args.serial_port or "unused", args.calibration_dir)
    limits = channel_limits(follower)
    fingerprint = model_id(kinematic_urdf(), args.calibration_dir)
    stop = Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    with ExitStack() as stack:
        listener = stack.enter_context(socket.socket())
        listener.bind(("127.0.0.1", args.listen_port))
        listener.listen(4)
        bus = (stack.enter_context(PiMotorBus(follower, args.serial_port))
               if args.serial_port else SimulatedBus(limits))
        motor = MotorSession(bus, limits=limits, model=fingerprint, token=token, watchdog=watchdog_ms / 1000,
                             hz=hz, arm_rate=arm_rate, gripper_rate=gripper_rate, simulated=not args.serial_port,
                             allow_motion=args.allow_motion)
        notify_ready()
        serve(listener, motor, stop)


if __name__ == "__main__":
    main()
