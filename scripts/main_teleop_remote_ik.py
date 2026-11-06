"""Windows gamepad, WSL2 LeRobot IK, and a Pi-owned SO-101 motor bus."""

import argparse
from contextlib import ExitStack, suppress
import ctypes
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Queue
import re
import secrets
import shlex
import socket
import subprocess
import sys
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
HARDWARE_CONFIG = ROOT / "remote_ik.hardware.conf"
PI_UNIT = "lerobot-remote-ik"
PI_SIM_PORT = PI_PORT + 1
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=5"]
DEFAULT_WSL_PYTHON = ".local/share/lerobot-phase0/venv/bin/python"
VK_ESCAPE = 0x1B
TELEOP_ERRORS = (OSError, RuntimeError, ValueError, subprocess.SubprocessError)


def load_key_state():
    get_key_state = ctypes.WinDLL("user32.dll", winmode=0x800).GetAsyncKeyState
    get_key_state.argtypes = (ctypes.c_int,)
    get_key_state.restype = ctypes.c_short
    return get_key_state


def dashboard_hardware_settings(config):
    """Select the public settings from the snapshot sent to the Pi."""
    try:
        args = shlex.split(config.partition("=")[2])
        if len(args) == 1:
            args = shlex.split(args[0])
    except ValueError:
        return None
    fields = ("serial-port", "hz", "watchdog-ms", "arm-rate", "gripper-rate")
    settings = {"allow-motion": "--allow-motion" in args}
    for index, arg in enumerate(args):
        name, equals, value = arg.partition("=")
        if name.startswith("--") and name[2:] in fields:
            if equals:
                settings[name[2:]] = value
            elif index + 1 < len(args):
                settings[name[2:]] = args[index + 1]
    return settings


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

    def solve(self, measured, delta, *, sent=None):
        try:
            self.process.stdin.write(encode_frame({"measured": measured, "delta": delta, "sent": sent}))
            self.process.stdin.flush()
        except OSError as exc:
            raise RuntimeError(f"WSL2 IK worker input failed: {exc}") from exc
        return self.receive(0.5)


class MotorLink:
    def __init__(self, port, token, metadata, simulated, *, relax_on_exit=False):
        self.port = port
        self.token = token
        self.metadata = metadata
        self.simulated = simulated
        self.relax_on_exit = relax_on_exit

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
            if not self.simulated and type(self.feedback.get("motion_enabled")) is not bool:
                raise RuntimeError("Pi service needs an update for real arm mode. Run scripts.install_remote_ik_pi first.")
            if not isinstance(self.feedback.get("session"), str):
                raise ValueError("Motor service returned no session")
            self.session = self.feedback["session"]
            self.socket.settimeout(min(0.5, self.feedback["watchdog_s"] * 2))
            if self.relax_on_exit:
                if not self.feedback["motion_enabled"]:
                    raise RuntimeError("--relax-on-exit requires --allow-motion in remote_ik.hardware.conf")
                if self.request("relax_on_exit").get("relax_on_exit") is not True:
                    raise RuntimeError("Pi service needs an update for --relax-on-exit. Run scripts.install_remote_ik_pi.")
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
    def __init__(self, host, simulated, *, relax_on_exit=False):
        self.host = host
        self.simulated = simulated
        self.relax_on_exit = relax_on_exit
        self.unit = PI_UNIT + ("-sim.service" if simulated else ".service")
        self.mode = "sim" if simulated else "live"
        self.hardware_config = None
        self.hardware_settings = None
        self.invocation = ""
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
        if self.invocation:
            try:
                stopped = self.service_command(
                    f'invocation=$(systemctl --user show -p InvocationID --value {self.unit}); '
                    f'if test "$invocation" = "{self.invocation}"; then systemctl --user stop {self.unit}; '
                    f'test "$(systemctl --user show -p Result --value {self.unit})" = success; fi',
                    timeout=10,
                )
                if stopped.returncode:
                    raise RuntimeError(stopped.stderr.strip() or "The Pi reported a shutdown error.")
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                print(f"Could not confirm clean shutdown of {self.unit} on {self.host}. {exc}",
                      file=sys.stderr)

    def service_command(self, command, *, timeout, config=None):
        # Serialize startup and shutdown with other Windows launchers. Keep the
        # lock off the motor loop and release it before opening the SSH tunnel.
        remote = (f'set -e; umask 077; config_dir="$HOME/.config/lerobot-remote-ik"; '
                  f'exec 9>"$config_dir/{self.mode}.lock"; flock -x 9; {command}')
        return subprocess.run([*SSH, self.host, remote], input=config, capture_output=True,
                              encoding="utf-8", timeout=timeout)

    def load_hardware_config(self):
        try:
            contents = HARDWARE_CONFIG.read_text(encoding="utf-8-sig")
            imported = False
        except FileNotFoundError:
            result = self.service_command(
                'config="$config_dir/hardware.env"; '
                'if test -s "$config" && test "$(stat -c %a "$config")" = 600; then cat "$config"; '
                "else echo 'Create remote_ik.hardware.conf in the Windows checkout. "
                "See readme_files/REMOTE_IK_SETUP.md.' >&2; exit 1; fi",
                timeout=10,
            )
            if result.returncode:
                error = OSError if result.returncode == 255 else RuntimeError
                raise error(result.stderr.strip() or "Could not import hardware settings from the Pi.")
            contents = result.stdout
            imported = True
        except (OSError, UnicodeError) as exc:
            raise RuntimeError(f"Could not read {HARDWARE_CONFIG}: {exc}") from exc

        lines = [line.strip() for line in contents.splitlines() if line.strip() and not line.lstrip().startswith("#")]
        if len(lines) != 1 or not re.fullmatch(r'REMOTE_IK_HARDWARE_ARGS=[^\x00-\x1f]+', lines[0]):
            raise RuntimeError(f"{HARDWARE_CONFIG.name} must contain one REMOTE_IK_HARDWARE_ARGS setting. "
                               "See readme_files/REMOTE_IK_SETUP.md.")
        config = lines[0] + "\n"
        if imported:
            try:
                with HARDWARE_CONFIG.open("x", encoding="utf-8") as file:
                    file.write(config)
            except FileExistsError:
                return self.load_hardware_config()
            print(f"Copied Pi hardware settings to {HARDWARE_CONFIG}", flush=True)
        return config

    def start_service(self):
        sync = ""
        if not self.simulated:
            if self.hardware_config is None:
                self.hardware_config = self.load_hardware_config()
            size = len(self.hardware_config.encode("utf-8"))
            # Receive the settings as data. Only replace the private file once
            # the upload has finished, and before systemd reads it.
            sync = ('config="$config_dir/hardware.env"; '
                    'tmp=$(mktemp "$config.XXXXXXXX"); trap \'rm -f -- "$tmp"\' EXIT; '
                    'tr -d \'\\r\' > "$tmp"; '
                    f'if test "$(wc -c < "$tmp")" -ne {size}; then '
                    "echo 'Incomplete hardware settings upload.' >&2; exit 1; fi; "
                    'if ! cmp -s "$tmp" "$config" || test "$(stat -c %a "$config")" != 600; then '
                    'mv -fT -- "$tmp" "$config"; fi; ')
        active = (f'test "$invocation" = "{self.invocation}" || '
                  "{ echo 'Real arm service is already running on the Pi. Close its launcher or stop the "
                  "service with the arm supported before starting a new session.' >&2; exit 1; }; "
                  if not self.simulated else "")
        command = (f"if systemctl --user is-active --quiet {self.unit}; then "
                   f'invocation=$(systemctl --user show -p InvocationID --value {self.unit}); {active}'
                   f'if test "$invocation" != "{self.invocation}"; then invocation=; fi; '
                   f"else {sync}systemctl --user start {self.unit}; "
                   f"systemctl --user is-active --quiet {self.unit}; "
                   f"invocation=$(systemctl --user show -p InvocationID --value {self.unit}); fi; "
                   f'printf "%s\\n" "$invocation"; cat "$config_dir/{self.mode}.token"')
        result = self.service_command(command, timeout=27, config=self.hardware_config)
        invocation, _, token = result.stdout.partition("\n")
        if re.fullmatch(r"[a-f0-9]{32}", invocation):
            self.invocation = invocation
        if result.returncode or len(token.strip()) < 32:
            detail = result.stderr.strip() or f"Check journalctl --user -u {self.unit} on the Pi"
            if result.returncode == 255:
                raise OSError(detail)
            raise RuntimeError(detail)
        self.token = token.strip()
        if self.hardware_config is not None:
            self.hardware_settings = dashboard_hardware_settings(self.hardware_config)

    def connect(self, metadata):
        with self.lock:
            if self.closed:
                raise OSError("SSH tunnel is closing")
            if self.process is None or self.process.poll() is not None:
                self.start_service()
                with socket.socket() as candidate:
                    candidate.bind(("127.0.0.1", 0))
                    self.local_port = candidate.getsockname()[1]
                remote_port = PI_SIM_PORT if self.simulated else PI_PORT
                forward = f"127.0.0.1:{self.local_port}:127.0.0.1:{remote_port}"
                self.process = subprocess.Popen([*SSH, "-N", "-o", "ExitOnForwardFailure=yes",
                                                 "-o", "ServerAliveInterval=1", "-o", "ServerAliveCountMax=2",
                                                 "-L", forward, self.host], stdin=subprocess.DEVNULL)
        return MotorLink(self.local_port, self.token, metadata, self.simulated, relax_on_exit=self.relax_on_exit)


@dataclass(frozen=True)
class GamepadSample:
    sampled_at: float
    gamepad: dict | None
    start: int = 0
    stop: int = 0
    release: int = 0
    disconnect: int = 0

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
            previous.disconnect + (previous.gamepad is not None and gamepad is None),
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
        mode = self.mode
        if not feedback.get("simulated", True):
            mode = "Real arm enabled" if feedback["motion_enabled"] else "Observe only"
        period = 1 / feedback["hz"]
        freshness = min(.1, feedback["watchdog_s"] / 2)
        connected_at = last_feedback = monotonic()
        previous_tick = connected_at
        feedback_count = 0
        max_gap_ms = 0.
        while not self.stop.is_set():
            started = monotonic()
            # Use the command cadence, with at most two Pi cycles of input per
            # step so a slow round trip cannot turn into a large catch-up move.
            elapsed = min(2 * period, max(period, started - previous_tick))
            previous_tick = started
            sample = self.latest_input.sample
            measured = feedback.get("measured")
            fresh = sample.gamepad is not None and started - sample.sampled_at < freshness
            delta = [0.] * 6
            sent = None
            op = "poll"
            if (not fresh or sample.disconnect != seen.disconnect or sample.stop != seen.stop
                    or sample.gamepad["buttons"]["Back"]):
                op = "disarm" if feedback["armed"] else "poll"
            elif feedback["armed"] and (not sample.lb or sample.release != seen.release):
                op = "hold" if feedback["moving"] else "poll"
            elif feedback["armed"] and sample.lb:
                if feedback["moving"] and finite_vector(feedback["sent"], 6):
                    sent = feedback["sent"]
                delta = sample.delta(self.speed * RATE_HZ * (elapsed if sent is not None else period))
                op = "hold" if feedback["moving"] else "poll"
            elif sample.start != seen.start and sample.neutral and not sample.lb:
                op = "arm"
            seen = sample

            result = {}
            if finite_vector(measured, 6) and (op == "poll" or any(delta)):
                result = self.worker.solve(measured, delta, sent=sent if any(delta) else None)
            # Recheck input after IK. A release and re-press while solving also
            # invalidates this candidate, even when LB is held again now.
            current = self.latest_input.sample
            if (self.stop.is_set() or current.gamepad is None or current.stop != sample.stop
                    or current.disconnect != sample.disconnect
                    or monotonic() - current.sampled_at >= freshness):
                op = "disarm"
            elif (result.get("target") is not None and current.lb and current.release == sample.release
                  and monotonic() - sample.sampled_at < freshness and any(current.delta(1.))):
                op = "move"
            elif op == "arm" and (current.lb or not current.neutral or current.gamepad["buttons"]["Back"]):
                op = "poll"

            values = {"target": result["target"]} if op == "move" else {}
            feedback = link.request(op, **values)
            received = monotonic()
            feedback_count += 1
            max_gap_ms = max(max_gap_ms, (received - last_feedback) * 1000)
            last_feedback = received
            reason = feedback["reason"] or result.get("reason", "")
            if not fresh and not reason:
                reason = "Connect a gamepad" if sample.gamepad is None else "Gamepad sample expired"
            state = ("Fault" if feedback["fault"] else "Moving" if feedback["moving"]
                     else "Armed" if feedback["armed"] else "Disarmed")
            self.status = {"connection": "Connected", "mode": mode, "state": state,
                           "reason": reason, "measured": measured, "sent": feedback["sent"],
                           "xyz_m": result.get("xyz_m"), "requested_xyz_m": result.get("requested_xyz_m"),
                           "candidate": result.get("target"),
                           "read_max_ms": feedback.get("read_max_ms"), "write_max_ms": feedback.get("write_max_ms"),
                           "feedback_hz": feedback_count / max(received - connected_at, 1e-6),
                           "feedback_max_gap_ms": max_gap_ms}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--dry-run", action="store_true", help="Use a local simulated arm with real WSL2 IK")
    modes.add_argument("--pi-sim", action="store_true", help="Use the Pi’s simulated bus over SSH")
    modes.add_argument("--real-arm", action="store_true", help="Connect to the real robot arm on the Pi")
    modes.add_argument("--commission-ik", dest="real_arm", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--relax-on-exit", action="store_true", help="Release motor torque when the real arm service stops")
    parser.add_argument("--pi", default="eric@rpi", help="SSH destination")
    parser.add_argument("--index", type=int, choices=range(4), default=0, metavar="SLOT", help="XInput slot (0–3)")
    parser.add_argument("--wsl-distro", default="Ubuntu")
    parser.add_argument("--wsl-python", default=DEFAULT_WSL_PYTHON, help="Linux path, absolute or relative to WSL home")
    parser.add_argument("--dashboard-port", type=int, default=0)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--speed", type=float, default=.25, help="Fraction of nominal Cartesian input speed")
    args = parser.parse_args()
    if not (args.dry_run or args.pi_sim or args.real_arm):
        parser.error("Choose --dry-run, --pi-sim, or --real-arm. See readme_files/REMOTE_IK_SETUP.md.")
    if args.relax_on_exit and not args.real_arm:
        parser.error("--relax-on-exit requires --real-arm")
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
                tunnel = stack.enter_context(PiTunnel(args.pi, args.pi_sim, relax_on_exit=args.relax_on_exit))
                connect = tunnel.connect

            mode = "Local simulation" if args.dry_run else "Pi simulation" if args.pi_sim else "Real arm"
            url = f"http://127.0.0.1:{dashboard.port}/"
            print(f"{mode} · Controller dashboard {url}", flush=True)
            if not args.no_browser:
                webbrowser.open(url)
            print("Start arms with neutral controls. Hold LB to move. Back disarms. Esc stops and exits.", flush=True)
            if args.relax_on_exit:
                print("Support the arm before exiting. The motors will release their hold.", flush=True)
            latest = LatestInput()
            control = ControlLoop(worker, connect, latest, stop, args.speed, mode)
            thread = Thread(target=control.run, daemon=True)
            thread.start()
            try:
                next_dashboard = monotonic()
                while not stop.is_set():
                    # Windows reports a held key as a negative signed SHORT.
                    if get_key_state(VK_ESCAPE) < 0:
                        dashboard.exit_reason = "Esc pressed"
                        stop.set()
                        print("Esc pressed. Stopping teleop.", flush=True)
                        break
                    latest.publish(gamepad.read())
                    now = monotonic()
                    if now >= next_dashboard:
                        sample = latest.sample
                        status = dict(control.status)
                        if args.real_arm:
                            status["hardware_settings"] = tunnel.hardware_settings
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
                if isinstance(control.error, TELEOP_ERRORS):
                    dashboard.exit_reason = str(control.error) or "Control loop stopped"
                raise control.error
    except KeyboardInterrupt:
        pass
    except TELEOP_ERRORS as exc:
        parser.exit(1, f"Remote IK: {exc}\n")


if __name__ == "__main__":
    main()
