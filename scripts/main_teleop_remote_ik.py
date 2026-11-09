"""Windows gamepad, WSL2 LeRobot IK, and a Pi-owned SO-101 motor bus."""

import argparse
from collections import deque
from contextlib import ExitStack, suppress
import ctypes
import json
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

from controller.input.xinput import XInputGamepad
from robot_program.remote_ik_correction import CorrectionProfile
from robot_program import remote_ik_control
from robot_program.remote_ik_protocol import (
    CHANNELS, MAX_FRAME, MOTION_POLICY, OFFLINE_WATCHDOG, PI_PORT, PROTOCOL_VERSION, RATE_HZ,
    decode_frame, encode_frame, finite_vector,
)
from scripts.lerobot_web_dashboard import WebDashboard
from scripts.remote_ik_motor_service import MotorSession, SimulatedBus, serve
from scripts.remote_ik_deploy import (
    PI_SIM_PORT, PI_UNIT, ROOT, SSH, ensure_installed, locked_command, release_guard,
)


HARDWARE_CONFIG = ROOT / "remote_ik.hardware.conf"
DEFAULT_WSL_PYTHON = ".local/share/lerobot-phase0/venv/bin/python"
GAMEPAD_POLL_S = 1 / 250
TRACE_MAX_TICKS = 3000
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
            if (self.metadata.get("v") != PROTOCOL_VERSION
                    or self.metadata.get("motion_policy") != MOTION_POLICY):
                raise RuntimeError("Update Windows and WSL for adaptive limit recovery")
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

    def solve(self, measured, delta, *, sent, desired_xyz_m, joint_step_limits, correction, recovery_step_m):
        return self._request({
            "measured": measured, "delta": delta, "sent": sent, "desired_xyz_m": desired_xyz_m,
            "joint_step_limits": joint_step_limits, "recovery_step_m": recovery_step_m, "correction": correction,
        })

    def recover(self, measured, delta, *, sent, automatic, anchor, goal, steps):
        return self._request({"op": "recover", "measured": measured, "delta": delta, "sent": sent,
                              "automatic": automatic, "anchor": anchor, "goal": goal, "steps": steps})

    def _request(self, message):
        try:
            self.process.stdin.write(encode_frame(message))
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
            if (self.metadata.get("v") != PROTOCOL_VERSION
                    or self.metadata.get("motion_policy") != MOTION_POLICY
                    or self.feedback.get("motion_policy") != MOTION_POLICY):
                raise RuntimeError("Update Windows, WSL and Pi service for boundary capture and recovery (protocol 4)")
            if (self.feedback.get("model") != self.metadata["model"]
                    or any(self.feedback.get(key) != self.metadata.get(key)
                           for key in ("limits", "calibrated_limits", "command_limits"))):
                raise RuntimeError("Pi and WSL2 URDF or calibration differ")
            if self.feedback.get("simulated") is not self.simulated:
                raise RuntimeError("Unexpected Pi motor service mode")
            if not self.simulated and type(self.feedback.get("motion_enabled")) is not bool:
                raise RuntimeError("Pi service needs an update for real arm mode. Run scripts.install_remote_ik_pi first.")
            if not finite_vector(self.feedback.get("rates"), 6) or any(r <= 0 for r in self.feedback["rates"]):
                raise RuntimeError("Pi service needs rate metadata. Run scripts.install_remote_ik_pi first.")
            if not isinstance(self.feedback.get("session"), str):
                raise ValueError("Motor service returned no session")
            self.validate_torque_feedback(self.feedback)
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

    @staticmethod
    def validate_torque_feedback(feedback):
        if (feedback.get("arm_torque") not in ("holding", "released", "unknown")
                or feedback.get("arm_torque") != "holding" and (feedback.get("armed") or feedback.get("moving"))):
            raise ValueError("Motor service returned invalid arm torque state")

    def request(self, op, **values):
        self.sequence += 1
        feedback = self.exchange({"op": op, "seq": self.sequence, "session": self.session,
                                  "feedback_seq": self.feedback["feedback_seq"], **values})
        if feedback.get("session") != self.session or feedback.get("ack") != self.sequence:
            raise ValueError("Motor service returned the wrong acknowledgement")
        self.validate_torque_feedback(feedback)
        self.feedback = feedback
        return feedback


class PiTunnel:
    def __init__(self, host, simulated, *, relax_on_exit=False, fingerprint=None):
        self.host = host
        self.simulated = simulated
        self.relax_on_exit = relax_on_exit
        self.fingerprint = fingerprint
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
                    timeout=20,
                )
                if stopped.returncode:
                    raise RuntimeError(stopped.stderr.strip() or "The Pi reported a shutdown error.")
            except (OSError, RuntimeError, subprocess.SubprocessError) as exc:
                print(f"Could not confirm clean shutdown of {self.unit} on {self.host}. {exc}",
                      file=sys.stderr)

    def service_command(self, command, *, timeout, config=None):
        # Serialize startup and shutdown with other Windows launchers. Keep the
        # lock off the motor loop and release it before opening the SSH tunnel.
        remote = locked_command(command, mode=self.mode)
        return subprocess.run([*SSH, self.host, "sh -c " + shlex.quote(remote)], input=config, capture_output=True,
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
                timeout=15,
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
        guard = release_guard(self.fingerprint) if self.fingerprint is not None else ""
        command = (guard + f"if systemctl --user is-active --quiet {self.unit}; then "
                   f'invocation=$(systemctl --user show -p InvocationID --value {self.unit}); {active}'
                   f'if test "$invocation" != "{self.invocation}"; then invocation=; fi; '
                   f"else {sync}systemctl --user start {self.unit}; "
                   f"systemctl --user is-active --quiet {self.unit}; "
                   f"invocation=$(systemctl --user show -p InvocationID --value {self.unit}); fi; "
                   f'printf "%s\\n" "$invocation"; cat "$config_dir/{self.mode}.token"')
        result = self.service_command(command, timeout=40, config=self.hardware_config)
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




def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--dry-run", action="store_true", help="Use a local simulated arm with real WSL2 IK")
    modes.add_argument("--pi-sim", action="store_true", help="Use the Pi’s simulated bus over SSH")
    modes.add_argument("--real-arm", action="store_true", help="Connect to the real robot arm on the Pi")
    parser.add_argument("--relax-on-exit", action="store_true", help="Release motor torque when the real arm service stops")
    parser.add_argument("--pi", default="eric@rpi", help="SSH destination")
    parser.add_argument("--index", type=int, choices=range(4), default=0, metavar="SLOT", help="XInput slot (0–3)")
    parser.add_argument("--advance-button", choices=("Start", "Back"), default="Start",
                        help="XInput button for Holding/control enable; the other stops/releases (Start=Options, Back=Share/Create)")
    parser.add_argument("--wsl-distro", default="Ubuntu")
    parser.add_argument("--wsl-python", default=DEFAULT_WSL_PYTHON, help="Linux path, absolute or relative to WSL home")
    parser.add_argument("--dashboard-port", type=int, default=18743,
                        help="Dashboard port (default %(default)s; 0 selects a random port)")
    parser.add_argument("--trace", type=Path, metavar="PATH",
                        help="Save the last 3000 control ticks after stopping, overwriting PATH if it exists")
    parser.add_argument("--speed", type=float, default=1.0, help="Gamepad motion scale (default %(default)g)")
    parser.add_argument("--correction-profile", type=Path, help="Explicit commissioned stationary-correction JSON profile")
    args = parser.parse_args()
    correction_profile = None
    if args.correction_profile is not None:
        try:
            correction_profile = CorrectionProfile(**json.loads(args.correction_profile.read_text(encoding="utf-8")))
        except (OSError, ValueError, TypeError) as exc:
            parser.error(f"Invalid correction profile: {exc}")
    if not (args.dry_run or args.pi_sim or args.real_arm):
        parser.error("Choose --dry-run, --pi-sim, or --real-arm. See readme_files/REMOTE_IK_SETUP.md.")
    if args.relax_on_exit and not args.real_arm:
        parser.error("--relax-on-exit requires --real-arm")
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*", args.pi):
        parser.error("--pi must be user@host")
    if not 0 < args.speed <= 1 or not 0 <= args.dashboard_port <= 65535:
        parser.error("Invalid speed or dashboard port")

    stop = Event()
    trace = deque(maxlen=TRACE_MAX_TICKS) if args.trace is not None else None
    trace_model = None
    revision, dirty = None, None
    if trace is not None:
        revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True, text=True).stdout)
    try:
        gamepad = XInputGamepad(args.index)
        get_key_state = load_key_state()
        with ExitStack() as stack:
            try:
                dashboard = stack.enter_context(WebDashboard("127.0.0.1", args.dashboard_port))
            except OSError as exc:
                raise RuntimeError(f"Cannot start dashboard on 127.0.0.1:{args.dashboard_port}: {exc}. "
                                   "If the port is in use, stop the other server or choose --dashboard-port PORT.") from exc
            worker = stack.enter_context(Worker(args.wsl_distro, args.wsl_python))
            trace_model = worker.metadata["model"]
            if args.dry_run:
                listener = stack.enter_context(socket.socket())
                listener.bind(("127.0.0.1", 0))
                listener.listen(4)
                token = secrets.token_hex(32)
                limits = worker.metadata["limits"]
                motor = MotorSession(SimulatedBus(limits), limits=limits, model=worker.metadata["model"], token=token,
                                     calibrated_limits=worker.metadata["calibrated_limits"],
                                     watchdog=OFFLINE_WATCHDOG, hz=RATE_HZ, arm_rate=120, gripper_rate=50)
                server = Thread(target=serve, args=(listener, motor, stop), daemon=True)
                server.start()
                stack.callback(server.join, 1)
                stack.callback(stop.set)

                def connect(metadata):
                    return MotorLink(listener.getsockname()[1], token, metadata, True)
            else:
                fingerprint = ensure_installed(args.pi)
                tunnel = stack.enter_context(PiTunnel(args.pi, args.pi_sim, relax_on_exit=args.relax_on_exit,
                                                      fingerprint=fingerprint))
                connect = tunnel.connect

            mode = "Local simulation" if args.dry_run else "Pi simulation" if args.pi_sim else "Real arm"
            url = f"http://127.0.0.1:{dashboard.port}/"
            print(f"{mode} · Controller dashboard {url}", flush=True)
            advance_button = args.advance_button
            stop_button = "Back" if advance_button == "Start" else "Start"
            print(f"{advance_button}: Released → Holding → Control enabled (separate neutral presses). "
                  f"{stop_button}: stop control; fresh 2-second hold releases arm torque. Hold LB to move. Esc exits.", flush=True)
            if args.relax_on_exit:
                print("Support the arm before exiting. The motors will release their hold.", flush=True)
            latest = remote_ik_control.LatestInput(advance_button=advance_button, stop_button=stop_button)
            control = remote_ik_control.ControlLoop(worker, connect, latest, stop, args.speed, mode, trace=trace, correction_profile=correction_profile)
            thread = Thread(target=control.run, daemon=True)
            thread.start()
            try:
                next_dashboard = monotonic()
                next_gamepad = next_dashboard
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
                        status["advance_button"] = advance_button
                        status["stop_button"] = stop_button
                        if args.real_arm:
                            status["hardware_settings"] = tunnel.hardware_settings
                        measured = status.get("measured")
                        xyz = status.get("xyz_m")
                        dashboard.publish(dict(zip(CHANNELS[:5], measured[:5])) if measured else {},
                                          measured[5] if measured else None,
                                          dict(zip("xyz", xyz)) if xyz else {}, sample.lb,
                                          gamepad=latest.raw_gamepad, remote=status)
                        next_dashboard = now + .1
                    # Sample faster than the motor loop to catch short gaps between taps.
                    next_gamepad = max(now, next_gamepad + GAMEPAD_POLL_S)
                    sleep(max(0., next_gamepad - monotonic()))
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
    finally:
        if trace:
            try:
                args.trace.parent.mkdir(parents=True, exist_ok=True)
                with args.trace.open("w", encoding="utf-8") as output:
                    json.dump({"model": trace_model, "provenance": {"revision": revision, "dirty": dirty,
                               "controller": MOTION_POLICY, "speed": args.speed,
                               "correction_profile": vars(correction_profile) if correction_profile else None,
                               "hardware_config": HARDWARE_CONFIG.read_text() if args.real_arm and HARDWARE_CONFIG.exists() else None},
                               "ticks": list(trace.copy())}, output, indent=2)
                print(f"Remote IK trace saved to {args.trace.resolve()}", flush=True)
            except OSError as exc:
                parser.exit(1, f"Could not save remote IK trace: {exc}\n")


if __name__ == "__main__":
    main()
