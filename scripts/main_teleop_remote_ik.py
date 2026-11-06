"""Windows gamepad, WSL2 LeRobot IK, and a Pi-owned SO-101 motor bus."""

import argparse
from contextlib import ExitStack, suppress
import ctypes
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
import re
import secrets
import socket
import subprocess
from threading import Event, Lock, Thread
from time import perf_counter as monotonic, sleep
import webbrowser

from controller.input.xinput import XInputGamepad
from robot_program.remote_ik_protocol import (
    CHANNELS, MAX_FRAME, OFFLINE_WATCHDOG, PI_PORT, PROTOCOL_VERSION, RATE_HZ,
    decode_frame, encode_frame, finite_vector,
)
from scripts.lerobot_web_dashboard import WebDashboard
from scripts.remote_ik_motor_service import MotorSession, SimulatedBus, serve


ROOT = Path(__file__).resolve().parents[1]
PI_UNIT = "lerobot-remote-ik"
PI_SIM_PORT = PI_PORT + 1
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=5"]
DEFAULT_WSL_PYTHON = ".local/share/lerobot-phase0/venv/bin/python"
VK_ESCAPE = 0x1B


def load_key_state():
    get_key_state = ctypes.WinDLL("user32.dll", winmode=0x800).GetAsyncKeyState
    get_key_state.argtypes = (ctypes.c_int,)
    get_key_state.restype = ctypes.c_short
    return get_key_state


class Worker:
    def __init__(self, distro, python):
        self.command = [
            "wsl.exe", "-d", distro, "--exec", "sh", "-c",
            'cd "$(wslpath -a -u "$1")" || exit; '
            'case "$2" in /*) py="$2";; *) py="$HOME/$2";; esac; '
            'exec "$py" -u -m scripts.remote_ik_worker',
            "remote-ik", str(ROOT), python,
        ]
        self.replies = Queue(maxsize=2)

    def __enter__(self):
        self.process = subprocess.Popen(self.command, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE)

        def read():
            try:
                while frame := self.process.stdout.readline(MAX_FRAME + 1):
                    self.replies.put_nowait(decode_frame(frame))
                error = RuntimeError("WSL2 IK worker exited")
            except (OSError, ValueError) as exc:
                error = RuntimeError(f"WSL2 IK worker output failed: {exc}")
            self.replies.put(error)

        self.reader = Thread(target=read, daemon=True)
        self.reader.start()
        try:
            self.metadata = self.receive(20)
        except BaseException:
            self.__exit__()
            raise
        return self

    def __exit__(self, *_):
        with suppress(OSError):
            self.process.stdin.close()
        try:
            self.process.wait(timeout=1)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=2)
        self.reader.join(timeout=1)
        self.process.stdout.close()

    def receive(self, timeout):
        try:
            reply = self.replies.get(timeout=timeout)
        except Empty as exc:
            raise RuntimeError("Timed out waiting for the WSL2 IK worker") from exc
        if isinstance(reply, Exception):
            raise reply
        return reply

    def solve(self, measured, delta):
        try:
            self.process.stdin.write(encode_frame({"measured": measured, "delta": delta}))
            self.process.stdin.flush()
        except OSError as exc:
            raise RuntimeError(f"WSL2 IK worker input failed: {exc}") from exc
        return self.receive(0.5)


class MotorLink:
    def __init__(self, port, token, metadata, simulated):
        self.port = port
        self.token = token
        self.metadata = metadata
        self.simulated = simulated

    def __enter__(self):
        self.resources = ExitStack()
        try:
            self.socket = self.resources.enter_context(socket.create_connection(("127.0.0.1", self.port), 0.5))
            self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.reader = self.resources.enter_context(self.socket.makefile("rb"))
            self.sequence = 0
            self.feedback = self.exchange({"op": "hello", "token": self.token})
            if (self.feedback.get("model") != self.metadata["model"]
                    or self.feedback.get("limits") != self.metadata["limits"]):
                raise RuntimeError("Pi and WSL2 URDF or calibration differ")
            if self.feedback.get("simulated") is not self.simulated:
                raise RuntimeError("Unexpected Pi motor service mode")
            if not isinstance(self.feedback.get("session"), str):
                raise ValueError("Motor service returned no session")
            self.session = self.feedback["session"]
            self.socket.settimeout(min(0.5, self.feedback["watchdog_s"] * 2))
            return self
        except BaseException:
            self.resources.close()
            raise

    def __exit__(self, *_):
        self.resources.close()

    def exchange(self, message):
        self.socket.sendall(encode_frame({"v": PROTOCOL_VERSION, **message}))
        response = decode_frame(self.reader.readline(MAX_FRAME + 1))
        if response.get("v") != PROTOCOL_VERSION:
            raise ValueError("Motor service protocol version differs")
        return response

    def request(self, op, **values):
        self.sequence += 1
        feedback = self.exchange({"op": op, "seq": self.sequence, "session": self.session,
                                  "feedback_seq": self.feedback["feedback_seq"], **values})
        if feedback.get("session") != self.session or feedback.get("ack") != self.sequence:
            raise ValueError("Motor service returned the wrong acknowledgement")
        self.feedback = feedback
        return feedback


class PiTunnel:
    def __init__(self, host, simulated):
        self.host = host
        self.simulated = simulated
        self.process = None
        self.closed = False
        self.lock = Lock()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        with self.lock:
            self.closed = True
            process = self.process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)

    def connect(self, metadata):
        if self.process is None or self.process.poll() is not None:
            unit = PI_UNIT + ("-sim.service" if self.simulated else ".service")
            mode = "sim" if self.simulated else "live"
            preflight = ("test -s ~/.config/lerobot-remote-ik/hardware.env || "
                         "{ echo 'Pi hardware is not commissioned. Use --pi-sim.' >&2; exit 1; }; "
                         if not self.simulated else "")
            remote = (f"{preflight}systemctl --user start {unit} && "
                      f"systemctl --user is-active --quiet {unit} && "
                      f"cat ~/.config/lerobot-remote-ik/{mode}.token")
            result = subprocess.run([*SSH, self.host, remote], capture_output=True, text=True, timeout=27)
            if result.returncode or len(result.stdout.strip()) < 32:
                detail = result.stderr.strip() or f"Check journalctl --user -u {unit} on the Pi"
                if result.returncode == 255:
                    raise OSError(detail)
                raise RuntimeError(detail)
            self.token = result.stdout.strip()
            with socket.socket() as candidate:
                candidate.bind(("127.0.0.1", 0))
                self.local_port = candidate.getsockname()[1]
            remote_port = PI_SIM_PORT if self.simulated else PI_PORT
            forward = f"127.0.0.1:{self.local_port}:127.0.0.1:{remote_port}"
            with self.lock:
                if self.closed:
                    raise OSError("SSH tunnel is closing")
                self.process = subprocess.Popen([*SSH, "-N", "-o", "ExitOnForwardFailure=yes",
                                                 "-o", "ServerAliveInterval=1", "-o", "ServerAliveCountMax=2",
                                                 "-L", forward, self.host], stdin=subprocess.DEVNULL)
        return MotorLink(self.local_port, self.token, metadata, self.simulated)


@dataclass(frozen=True)
class GamepadSample:
    sampled_at: float
    gamepad: dict | None
    start: int = 0
    stop: int = 0
    release: int = 0

    @property
    def lb(self):
        return bool(self.gamepad and self.gamepad["buttons"]["LB"])

    @property
    def neutral(self):
        return self.gamepad is not None and not any(self.gamepad["axes"].values()) and self.gamepad["dpad"] == (0, 0)

    def delta(self, scale):
        axes = self.gamepad["axes"]
        return [scale * value for value in (-.002 * axes["LY"], -.002 * axes["LX"], -.002 * axes["RY"],
                                            .4 * (axes["RT"] - axes["LT"]), .75 * axes["RX"],
                                            self.gamepad["dpad"][1])]


class LatestInput:
    def __init__(self):
        self.sample = GamepadSample(monotonic(), None)

    def publish(self, gamepad):
        previous = self.sample
        before = previous.gamepad["buttons"] if previous.gamepad else {}
        after = gamepad["buttons"] if gamepad else {}
        start = (bool(before) and not before["Start"] and after.get("Start", False)
                 and not after["LB"] and not after["Back"]
                 and not any(gamepad["axes"].values()) and gamepad["dpad"] == (0, 0))
        self.sample = GamepadSample(
            monotonic(), gamepad,
            previous.start + start,
            previous.stop + (not before.get("Back", False) and after.get("Back", False)),
            previous.release + (previous.lb and not after.get("LB", False)),
        )


class ControlLoop:
    def __init__(self, worker, connect, latest_input, stop, speed, mode):
        self.worker, self.connect, self.latest_input, self.stop = worker, connect, latest_input, stop
        self.speed, self.mode = speed, mode
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

    def session(self, link):
        seen = self.latest_input.sample
        feedback = link.feedback
        period = 1 / feedback["hz"]
        freshness = min(.1, feedback["watchdog_s"] / 2)
        while not self.stop.is_set():
            started = monotonic()
            sample = self.latest_input.sample
            measured = feedback.get("measured")
            fresh = sample.gamepad is not None and started - sample.sampled_at < freshness
            delta = [0.] * 6
            op = "poll"
            if not fresh or sample.stop != seen.stop or sample.gamepad["buttons"]["Back"]:
                op = "disarm" if feedback["armed"] else "poll"
            elif feedback["armed"] and (not sample.lb or sample.release != seen.release):
                op = "hold" if feedback["moving"] else "poll"
            elif feedback["armed"] and sample.lb:
                delta = sample.delta(self.speed * RATE_HZ * period)
                op = "hold" if feedback["moving"] else "poll"
            elif sample.start != seen.start and sample.neutral and not sample.lb:
                op = "arm"
            seen = sample

            result = {}
            if finite_vector(measured, 6) and (op == "poll" or any(delta)):
                result = self.worker.solve(measured, delta)
            # Recheck input after IK. A release and re-press while solving also
            # invalidates this candidate, even when LB is held again now.
            current = self.latest_input.sample
            if (self.stop.is_set() or current.gamepad is None or current.stop != sample.stop
                    or monotonic() - current.sampled_at >= freshness):
                op = "disarm"
            elif (result.get("target") is not None and current.lb and current.release == sample.release
                  and monotonic() - sample.sampled_at < freshness and any(current.delta(1.))):
                op = "move"
            elif op == "arm" and (current.lb or not current.neutral or current.gamepad["buttons"]["Back"]):
                op = "poll"

            values = {"target": result["target"]} if op == "move" else {}
            feedback = link.request(op, **values)
            reason = feedback["reason"] or result.get("reason", "")
            if not fresh and not reason:
                reason = "Connect a gamepad" if sample.gamepad is None else "Gamepad sample expired"
            state = ("Fault" if feedback["fault"] else "Moving" if feedback["moving"]
                     else "Armed" if feedback["armed"] else "Disarmed")
            self.status = {"connection": "Connected", "mode": self.mode, "state": state,
                           "reason": reason, "measured": measured, "sent": feedback["sent"],
                           "xyz_m": result.get("xyz_m"), "requested_xyz_m": result.get("requested_xyz_m")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--dry-run", action="store_true", help="Use a local simulated arm with real WSL2 IK")
    modes.add_argument("--pi-sim", action="store_true", help="Use the Pi’s simulated bus over SSH")
    parser.add_argument("--pi", default="eric@rpi", help="SSH destination")
    parser.add_argument("--index", type=int, choices=range(4), default=0, metavar="SLOT", help="XInput slot (0–3)")
    parser.add_argument("--wsl-distro", default="Ubuntu")
    parser.add_argument("--wsl-python", default=DEFAULT_WSL_PYTHON, help="Linux path, absolute or relative to WSL home")
    parser.add_argument("--dashboard-port", type=int, default=0)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--speed", type=float, default=.25, help="Fraction of nominal Cartesian input speed")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*", args.pi):
        parser.error("--pi must be user@host")
    if not 0 < args.speed <= 1 or not 0 <= args.dashboard_port <= 65535:
        parser.error("Invalid speed or dashboard port")

    stop = Event()
    try:
        gamepad = XInputGamepad(args.index)
        get_key_state = load_key_state()
        with ExitStack() as stack:
            worker = stack.enter_context(Worker(args.wsl_distro, args.wsl_python))
            dashboard = stack.enter_context(WebDashboard("127.0.0.1", args.dashboard_port))
            if args.dry_run:
                listener = stack.enter_context(socket.socket())
                listener.bind(("127.0.0.1", 0))
                listener.listen(4)
                token = secrets.token_hex(32)
                limits = worker.metadata["limits"]
                motor = MotorSession(SimulatedBus(limits), limits=limits, model=worker.metadata["model"], token=token,
                                     watchdog=OFFLINE_WATCHDOG, hz=RATE_HZ, arm_rate=120, gripper_rate=50)
                server = Thread(target=serve, args=(listener, motor, stop), daemon=True)
                server.start()
                stack.callback(server.join, 1)
                stack.callback(stop.set)

                def connect(metadata):
                    return MotorLink(listener.getsockname()[1], token, metadata, True)
            else:
                tunnel = stack.enter_context(PiTunnel(args.pi, args.pi_sim))
                connect = tunnel.connect

            mode = "Local simulation" if args.dry_run else "Pi simulation" if args.pi_sim else "SO-101 hardware"
            url = f"http://127.0.0.1:{dashboard.port}/"
            print(f"{mode} · Controller dashboard {url}", flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            print("Start arms with neutral controls. Hold LB to move. Back disarms. Esc stops and exits.", flush=True)
            latest = LatestInput()
            control = ControlLoop(worker, connect, latest, stop, args.speed, mode)
            thread = Thread(target=control.run, daemon=True)
            thread.start()
            try:
                next_dashboard = monotonic()
                while not stop.is_set():
                    # Windows reports a held key as a negative signed SHORT.
                    if get_key_state(VK_ESCAPE) < 0:
                        stop.set()
                        print("Esc pressed. Stopping teleop.", flush=True)
                        break
                    latest.publish(gamepad.read())
                    now = monotonic()
                    if now >= next_dashboard:
                        sample = latest.sample
                        status = control.status
                        measured = status.get("measured")
                        xyz = status.get("xyz_m")
                        dashboard.publish(dict(zip(CHANNELS[:5], measured[:5])) if measured else {},
                                          measured[5] if measured else None,
                                          dict(zip("xyz", xyz)) if xyz else {}, sample.lb,
                                          gamepad=sample.gamepad, remote=status)
                        next_dashboard = now + .1
                    sleep(1 / 120)
            finally:
                stop.set()
                thread.join(timeout=2)
            if control.error:
                raise control.error
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as exc:
        parser.exit(1, f"Remote IK: {exc}\n")


if __name__ == "__main__":
    main()
