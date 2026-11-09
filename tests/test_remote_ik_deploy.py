"""Offline deployment checks: mocked SSH plus real shell transactions in Linux."""

from contextlib import nullcontext
from io import BytesIO
import os
import subprocess
import sys
import tarfile
from time import monotonic, sleep
from types import SimpleNamespace

import pytest

from scripts import install_remote_ik_pi as installer
from scripts import main_teleop_remote_ik as teleop
from scripts import remote_ik_deploy as deploy


@pytest.fixture
def source(tmp_path, monkeypatch):
    root = tmp_path / "source"
    for name in deploy.FILES:
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes((deploy.ROOT / name).read_bytes())
    monkeypatch.setattr(deploy, "ROOT", root)
    return root


def test_fingerprint_ignores_metadata_and_unrelated_edits(source):
    before = deploy.build_deployment()
    path = source / deploy.FILES[0]
    os.utime(path, (1, 1))
    path.chmod(0o644)
    (source / "pc-only.py").write_text("changed", encoding="utf-8")
    assert deploy.build_deployment().fingerprint == before.fingerprint
    with tarfile.open(fileobj=BytesIO(before.archive)) as archive:
        assert archive.extractfile(deploy.FILES[0]).read() == path.read_bytes()


@pytest.mark.parametrize("change", ["source", "calibration", "urdf", "unit", "recipe", "file-list"])
def test_fingerprint_detects_deployment_changes(source, monkeypatch, change):
    before = deploy.build_deployment()
    original = (source / deploy.FILES[0]).read_bytes()
    if change in {"source", "calibration"}:
        path = source / (deploy.FILES[0] if change == "source" else "calibration/lerobot_arm.json")
        path.write_bytes(path.read_bytes() + b"\n")
    elif change == "urdf":
        monkeypatch.setattr(deploy, "kinematic_urdf", lambda: "changed model")
    elif change == "unit":
        monkeypatch.setattr(deploy, "service", lambda **kwargs: "changed unit")
    elif change == "recipe":
        monkeypatch.setattr(deploy, "INSTALL", deploy.INSTALL + "\n# new recipe\n")
    else:
        monkeypatch.setattr(deploy, "FILES", deploy.FILES[1:])
    assert deploy.build_deployment().fingerprint != before.fingerprint
    # The original archive is an immutable snapshot of the earlier bytes.
    with tarfile.open(fileobj=BytesIO(before.archive)) as archive:
        assert archive.extractfile("robot_program/__init__.py").read() == original


@pytest.mark.parametrize("force, replies", [
    (False, [b"current\n"]),
    (False, [b"install\n", b"installed\n"]),
    (False, [b"install\n", b"current\n"]),  # another launcher installed it first
    (True, [b"installed\n"]),
])
def test_ssh_uses_one_snapshot_and_preserves_custom_host(monkeypatch, capsys, force, replies):
    payload = deploy.Deployment("a" * 64, b"snapshot archive")
    builds, calls = [], []

    def build():
        builds.append(True)
        return payload

    def run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(returncode=0, stdout=replies[len(calls) - 1], stderr=b"")

    monkeypatch.setattr(deploy, "build_deployment", build)
    monkeypatch.setattr(deploy.subprocess, "run", run)
    assert deploy.ensure_installed("tester@custom-pi", force=force) == payload.fingerprint
    assert builds == [True]
    assert len(calls) == len(replies)
    for command, kwargs in calls:
        assert command[-2] == "tester@custom-pi"
        assert payload.fingerprint in command[-1]
        assert kwargs["input"] in (None, payload.archive)
    if replies[-1] == b"current\n":
        assert "Pi service is up to date" in capsys.readouterr().out
    else:
        assert calls[-1][1]["input"] == payload.archive


@pytest.mark.parametrize("failure", ["ssh", "install", "timeout", "missing-ssh", "invalid-reply", "false-success"])
def test_deployment_failures_are_fatal(monkeypatch, failure):
    monkeypatch.setattr(deploy, "build_deployment", lambda: deploy.Deployment("a" * 64, b"archive"))
    calls = []

    def run(*args, **kwargs):
        calls.append(kwargs)
        if failure == "timeout":
            raise subprocess.TimeoutExpired("ssh", 25)
        if failure == "missing-ssh":
            raise FileNotFoundError("ssh not found")
        if failure == "invalid-reply":
            return SimpleNamespace(returncode=0, stdout=b"unexpected", stderr=b"")
        if failure == "false-success":
            return SimpleNamespace(returncode=0, stdout=b"install\n", stderr=b"")
        if failure == "install" and len(calls) == 1:
            return SimpleNamespace(returncode=0, stdout=b"install\n", stderr=b"")
        return SimpleNamespace(returncode=255, stdout=b"", stderr=b"connection lost")

    monkeypatch.setattr(deploy.subprocess, "run", run)
    with pytest.raises((RuntimeError, subprocess.TimeoutExpired)):
        deploy.ensure_installed("eric@rpi")


def test_invalid_host_is_rejected_before_build_or_ssh(monkeypatch):
    monkeypatch.setattr(deploy, "build_deployment", lambda: pytest.fail("Built an invalid destination"))
    with pytest.raises(ValueError, match="user@host"):
        deploy.ensure_installed("user@host;bad")


def test_standalone_installer_forces_repair(monkeypatch):
    calls = []
    monkeypatch.setattr(installer, "ensure_installed", lambda *args, **kwargs: calls.append((args, kwargs)))
    monkeypatch.setattr(sys, "argv", ["installer", "--pi", "test@other"])
    installer.main()
    assert calls == [(("test@other",), {"force": True})]


@pytest.mark.parametrize("mode, fail", [("--pi-sim", False), ("--real-arm", False),
                                      ("--dry-run", False), ("--pi-sim", True)])
def test_launcher_deploys_before_control_and_stops_on_failure(monkeypatch, mode, fail):
    events = []
    metadata = {"model": "fixture", "limits": [[-90., 90.]] * 5 + [[0., 50.]],
                "calibrated_limits": [[-90., 90.]] * 5 + [[0., 50.]]}
    monkeypatch.setattr(sys, "argv", ["teleop", mode, "--pi", "test@other"])
    monkeypatch.setattr(teleop, "XInputGamepad", lambda index: None)
    monkeypatch.setattr(teleop, "load_key_state", lambda: lambda key: -1)
    monkeypatch.setattr(teleop, "WebDashboard", lambda *args: nullcontext(SimpleNamespace(port=18743)))
    monkeypatch.setattr(teleop, "Worker", lambda *args: nullcontext(SimpleNamespace(metadata=metadata)))
    monkeypatch.setattr(teleop, "MotorSession", lambda *args, **kwargs: None)
    monkeypatch.setattr(teleop, "serve", lambda *args: None)

    def ensure(host):
        assert mode != "--dry-run"
        assert host == "test@other"
        events.append("deploy")
        if fail:
            raise RuntimeError("installation failed")
        return "a" * 64

    def tunnel(host, simulated, **kwargs):
        assert kwargs["fingerprint"] == "a" * 64
        events.append("tunnel")
        return nullcontext(SimpleNamespace(connect=None, hardware_settings=None))

    def control(*args, **kwargs):
        events.append("control")
        return SimpleNamespace(run=lambda: None, error=None)

    monkeypatch.setattr(teleop, "ensure_installed", ensure)
    monkeypatch.setattr(teleop, "PiTunnel", tunnel)
    monkeypatch.setattr(teleop.remote_ik_control, "ControlLoop", control)
    if fail:
        with pytest.raises(SystemExit) as error:
            teleop.main()
        assert error.value.code == 1
        assert events == ["deploy"]
    else:
        teleop.main()
        assert events == (["control"] if mode == "--dry-run" else ["deploy", "tunnel", "control"])


@pytest.mark.parametrize("simulated, expected_port", [(True, deploy.PI_SIM_PORT), (False, deploy.PI_PORT)])
def test_reconnect_only_checks_release_without_deploying(monkeypatch, simulated, expected_port):
    tunnel = teleop.PiTunnel("eric@rpi", simulated, fingerprint="a" * 64)
    if not simulated:
        tunnel.hardware_config = 'REMOTE_IK_HARDWARE_ARGS="--serial-port /dev/serial/by-id/test"\n'
    starts = []
    forwards = []

    def service_command(command, **kwargs):
        starts.append(command)
        assert "installation_matches" in command
        assert "a" * 64 in command
        return SimpleNamespace(returncode=0, stdout="b" * 32 + "\n" + "c" * 64, stderr="")

    monkeypatch.setattr(tunnel, "service_command", service_command)
    monkeypatch.setattr(teleop, "ensure_installed", lambda *args: pytest.fail("Deployed on reconnect"))

    def popen(command, **kwargs):
        forwards.append(command[command.index("-L") + 1])
        return SimpleNamespace(poll=lambda: 1)

    monkeypatch.setattr(teleop.subprocess, "Popen", popen)
    tunnel.connect({})
    tunnel.connect({})
    assert len(starts) == 2
    assert len(forwards) == 2
    assert all(forward.endswith(f":127.0.0.1:{expected_port}") for forward in forwards)


@pytest.fixture
def pi(tmp_path):
    if sys.platform != "linux":
        pytest.skip("Real POSIX shell/flock/atomic symlink tests run under Linux/WSL")
    home = tmp_path / "pi home"
    home.mkdir()
    binaries = tmp_path / "bin"
    binaries.mkdir()
    state = tmp_path / "state"
    state.mkdir()
    fake_systemctl = binaries / "systemctl"
    fake_systemctl.write_text(f"#!{sys.executable}\n" + '''
import os, sys
from pathlib import Path
state = Path(os.environ["FAKE_STATE"])
args = sys.argv[1:]
with (state / "commands").open("a") as log:
    log.write(" ".join(args) + "\\n")
if "show" in args:
    if (state / "fail-show").exists():
        sys.exit(1)
    unit = args[-1]
    path = state / (unit + ".state")
    print(path.read_text() if path.exists() else "inactive")
elif "daemon-reload" in args:
    if (state / "fail-reload-always").exists():
        sys.exit(1)
    flag = state / "fail-reload-once"
    if flag.exists():
        flag.unlink()
        sys.exit(1)
elif "start" in args or "stop" in args:
    print("lifecycle executed")
else:
    sys.exit(1)
''', encoding="utf-8")
    fake_systemctl.chmod(0o700)
    python = home / "lerobot-env/bin/python"
    python.parent.mkdir(parents=True)
    python.write_text(f"#!{sys.executable}\n" + '''
import os, sys, time
from pathlib import Path
state = Path(os.environ["FAKE_STATE"])
if "calibrated_follower" in sys.argv[-1]:
    (state / "validated").touch()
    while (state / "block-validation").exists():
        time.sleep(.01)
    sys.exit(1 if (state / "fail-validation").exists() else 0)
os.execv(sys.executable, [sys.executable, *sys.argv[1:]])
''', encoding="utf-8")
    python.chmod(0o700)
    env = dict(os.environ, HOME=str(home), FAKE_STATE=str(state), PATH=str(binaries) + ":" + os.environ["PATH"])
    root = home / ".local/share/lerobot-remote-ik"
    units = home / ".config/systemd/user"
    config = home / ".config/lerobot-remote-ik"

    def shell(command, *, data=None, background=False):
        if background:
            process = subprocess.Popen(["sh", "-c", command], env=env, stdin=subprocess.PIPE,
                                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if data is not None:
                process.stdin.write(data)
                process.stdin.close()
                process.stdin = None
            return process
        return subprocess.run(["sh", "-c", command], input=data, env=env, capture_output=True, timeout=25)

    def install(payload, *, force=False, archive=None, background=False):
        return shell(deploy.locked_command(f"expected={payload.fingerprint}; force={int(force)}; " +
                                          deploy.MATCHES + deploy.INSTALL),
                     data=payload.archive if archive is None else archive, background=background)

    def check(payload):
        return shell(deploy.locked_command(f"expected={payload.fingerprint}; " + deploy.MATCHES +
                                           "\ninstallation_matches"))

    return SimpleNamespace(home=home, root=root, units=units, config=config, state=state,
                           shell=shell, install=install, check=check)


def assert_success(result, status=b"installed\n"):
    assert result.returncode == 0, result.stderr.decode()
    assert result.stdout == status


def test_shell_fresh_unchanged_legacy_and_repair(pi):
    payload = deploy.build_deployment()
    assert pi.check(payload).returncode != 0
    assert_success(pi.install(payload))
    first = (pi.root / "current").resolve()
    tokens = {p.name: p.read_bytes() for p in pi.config.glob("*.token")}
    (pi.config / "hardware.env").write_text("existing hardware settings", encoding="utf-8")
    assert_success(pi.install(payload), b"current\n")
    assert len(list((pi.root / "releases").iterdir())) == 1
    (first / ".deployment-sha256").unlink()  # pre-feature installation
    assert_success(pi.install(payload))
    assert (pi.root / "current").resolve() != first
    assert_success(pi.install(payload, force=True))
    assert {p.name: p.read_bytes() for p in pi.config.glob("*.token")} == tokens
    assert (pi.config / "hardware.env").read_text() == "existing hardware settings"
    assert pi.check(payload).returncode == 0
    assert not (pi.root / ".deployment-incomplete").exists()
    (pi.units / "lerobot-remote-ik.service").unlink()
    assert pi.check(payload).returncode != 0


def test_shell_installs_changed_payload_and_records_success(pi, source):
    before = deploy.build_deployment()
    assert_success(pi.install(before))
    old_release = (pi.root / "current").resolve()
    path = source / deploy.FILES[0]
    path.write_bytes(path.read_bytes() + b"\n# new implementation\n")
    after = deploy.build_deployment()
    assert_success(pi.install(after))
    current = (pi.root / "current").resolve()
    assert current != old_release
    assert (current / deploy.FILES[0]).read_bytes() == path.read_bytes()
    assert (current / ".deployment-sha256").read_text().strip() == after.fingerprint
    assert pi.check(before).returncode != 0
    assert pi.check(after).returncode == 0


@pytest.mark.parametrize("unit", ["lerobot-remote-ik.service", "lerobot-remote-ik-sim.service"])
@pytest.mark.parametrize("state", ["active", "activating", "deactivating", "reloading"])
def test_shell_refuses_busy_services_without_stopping(pi, unit, state):
    (pi.state / (unit + ".state")).write_text(state)
    result = pi.install(deploy.build_deployment())
    assert result.returncode != 0
    assert unit.encode() in result.stderr and state.encode() in result.stderr
    assert "stop" not in (pi.state / "commands").read_text()
    assert not pi.root.exists()


@pytest.mark.parametrize("failure", ["upload", "validation", "reload", "first-reload", "show"])
def test_shell_failure_preserves_previous_installation(pi, failure, monkeypatch):
    payload = deploy.build_deployment()
    if failure != "first-reload":
        assert_success(pi.install(payload))
    previous = os.readlink(pi.root / "current") if (pi.root / "current").exists() else None
    units = {p.name: p.read_bytes() for p in pi.units.glob("*.service")}
    old_service = deploy.service
    monkeypatch.setattr(deploy, "service", lambda **kwargs: old_service(**kwargs) + "\n# changed unit\n")
    candidate = deploy.build_deployment()
    if failure in {"reload", "first-reload"}:
        (pi.state / "fail-reload-once").touch()
    elif failure in {"validation", "show"}:
        (pi.state / ("fail-" + failure)).touch()
    result = pi.install(candidate, force=True, archive=candidate.archive[:100] if failure == "upload" else None)
    assert result.returncode != 0
    assert not (pi.root / ".deployment-incomplete").exists()
    assert {p.name: p.read_bytes() for p in pi.units.glob("*.service")} == units
    if previous:
        assert os.readlink(pi.root / "current") == previous
        assert pi.check(payload).returncode == 0
        assert len(list((pi.root / "releases").iterdir())) == 1
    else:
        assert not (pi.root / "current").is_symlink()
        assert not list((pi.root / "releases").iterdir())


def test_shell_failed_rollback_requires_repair_before_startup(pi):
    payload = deploy.build_deployment()
    assert_success(pi.install(payload))
    previous = (pi.root / "current").resolve()
    (pi.state / "fail-reload-always").touch()
    result = pi.install(payload, force=True)
    assert result.returncode != 0
    assert b"Rollback could not be completed" in result.stderr
    assert (pi.root / ".deployment-incomplete").exists()
    # Even the restored old release's matching fingerprint cannot bypass repair.
    assert (pi.root / "current").resolve() == previous
    assert (previous / ".deployment-sha256").read_text().strip() == payload.fingerprint
    assert pi.check(payload).returncode != 0
    guarded = pi.shell(deploy.locked_command(deploy.release_guard(payload.fingerprint) +
                                            "systemctl --user start lerobot-remote-ik.service", mode="live"))
    assert guarded.returncode != 0
    assert "start" not in (pi.state / "commands").read_text()
    (pi.state / "fail-reload-always").unlink()
    assert_success(pi.install(payload))  # identical content still repairs
    assert pi.check(payload).returncode == 0
    assert not (pi.root / ".deployment-incomplete").exists()


def test_shell_interrupted_activation_requires_repair(pi):
    payload = deploy.build_deployment()
    assert_success(pi.install(payload))
    # Model a process killed during activation, before it could run rollback.
    (pi.root / ".deployment-incomplete").touch()
    assert pi.check(payload).returncode != 0
    (pi.state / "lerobot-remote-ik-sim.service.state").write_text("active")
    result = pi.install(payload)
    assert result.returncode != 0
    assert b"lerobot-remote-ik-sim.service" in result.stderr
    assert (pi.root / ".deployment-incomplete").exists()
    (pi.state / "lerobot-remote-ik-sim.service.state").unlink()
    assert_success(pi.install(payload))
    assert pi.check(payload).returncode == 0


def test_shell_release_guard_prevents_starting_changed_release(pi):
    payload = deploy.build_deployment()
    assert_success(pi.install(payload))
    result = pi.shell(deploy.locked_command(deploy.release_guard("f" * 64) +
                                           "systemctl --user start lerobot-remote-ik.service", mode="live"))
    assert result.returncode != 0
    assert b"Exit and relaunch" in result.stderr
    assert "start" not in (pi.state / "commands").read_text()


def wait_for_file(path):
    deadline = monotonic() + 5
    while not path.exists():
        assert monotonic() < deadline, f"Timed out waiting for {path}"
        sleep(.01)


@pytest.mark.parametrize("concurrent", ["install", "start", "stop"])
def test_shell_serializes_installation_and_lifecycle(pi, concurrent):
    payload = deploy.build_deployment()
    block = pi.state / "block-validation"
    block.touch()
    first = pi.install(payload, background=True)
    second = None
    try:
        wait_for_file(pi.state / "validated")
        assert not list(pi.root.glob("releases/*/.deployment-sha256"))
        if concurrent == "install":
            second = pi.install(payload, background=True)
        else:
            second = pi.shell(deploy.locked_command(deploy.release_guard(payload.fingerprint) +
                               f"systemctl --user {concurrent} lerobot-remote-ik.service", mode="live"), background=True)
        with pytest.raises(subprocess.TimeoutExpired):
            second.wait(timeout=.15)
        block.unlink()
        output, error = first.communicate(timeout=10)
        assert first.returncode == 0, error.decode()
        assert output == b"installed\n"
        output, error = second.communicate(timeout=10)
        assert second.returncode == 0, error.decode()
        assert output == (b"current\n" if concurrent == "install" else b"lifecycle executed\n")
        assert len(list((pi.root / "releases").iterdir())) == 1
    finally:
        block.unlink(missing_ok=True)
        for process in (first, second):
            if process is not None:
                process.communicate(timeout=10)


def test_shell_deployment_lock_wait_is_bounded(pi):
    block = pi.state / "block-validation"
    block.touch()
    first = pi.install(deploy.build_deployment(), background=True)
    try:
        wait_for_file(pi.state / "validated")
        result = pi.shell(deploy.locked_command("echo should-not-run", mode="sim"))
        assert result.returncode != 0
        assert b"deployment lock is busy" in result.stderr
        assert not result.stdout
    finally:
        block.unlink()
        first.communicate(timeout=10)
