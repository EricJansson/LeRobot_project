"""Shared, content-addressed deployment of the isolated Pi motor services."""

from dataclasses import dataclass
import hashlib
from io import BytesIO
from pathlib import Path
import re
import shlex
import subprocess
import tarfile

from robot_program.remote_ik_protocol import PI_PORT, kinematic_urdf


ROOT = Path(__file__).resolve().parents[1]
PI_UNIT = "lerobot-remote-ik"
PI_SIM_PORT = PI_PORT + 1
SSH = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=5"]
FILES = (
    "robot_program/__init__.py",
    "robot_program/lerobot_compat.py",
    "robot_program/lerobot_ik_controller.py",
    "robot_program/remote_ik_protocol.py",
    "robot_program/remote_ik_recovery.py",
    "robot_program/so101_follower.py",
    "scripts/remote_ik_motor_service.py",
    "calibration/lerobot_arm.json",
)

# Every lifecycle command takes the deployment lock before its mode lock.
# Installation also takes the old mode locks so older launchers serialize with it.
REMOTE_PATHS = r'''
set -eu
umask 077
root="$HOME/.local/share/lerobot-remote-ik"
config_dir="$HOME/.config/lerobot-remote-ik"
units="$HOME/.config/systemd/user"
python="$HOME/lerobot-env/bin/python"
mkdir -p "$config_dir"
'''

MATCHES = r'''
installation_matches() {
    test ! -e "$root/.deployment-incomplete" || return 1
    test -f "$root/current/.deployment-sha256" || return 1
    test "$(cat "$root/current/.deployment-sha256")" = "$expected" || return 1
    for unit in lerobot-remote-ik.service lerobot-remote-ik-sim.service; do
        cmp -s "$root/current/$unit" "$units/$unit" || return 1
    done
    for mode in live sim; do
        test -s "$config_dir/$mode.token" || return 1
        test "$(stat -c %a "$config_dir/$mode.token")" = 600 || return 1
    done
}
'''

INSTALL = r'''
if test "$force" = 0 && installation_matches; then
    echo current
    exit 0
fi
test -x "$python" || { echo 'Expected ~/lerobot-env/bin/python' >&2; exit 1; }
for unit in lerobot-remote-ik.service lerobot-remote-ik-sim.service; do
    state=$(systemctl --user show -p ActiveState --value "$unit")
    case "$state" in
        inactive|failed) ;;
        *) echo "Cannot update $unit ($state). Close its launcher first; support the arm before stopping a live service." >&2; exit 1 ;;
    esac
done
if test -e "$root/current" && ! test -L "$root/current"; then
    echo 'Expected the current Pi release to be a symlink.' >&2
    exit 1
fi
mkdir -p "$root/releases" "$units"
release=$(mktemp -d "$root/releases/release.XXXXXXXX")
previous=$(readlink "$root/current" || true)
activating=0
committed=0
cleanup() {
    status=$?
    trap - EXIT HUP INT TERM
    set +e
    if test "$committed" = 0; then
        if test "$activating" = 1; then
            echo 'Pi activation failed; restoring the previous installation.' >&2
            restored=1
            if test -n "$previous"; then
                ln -s "$previous" "$release/rollback-link" &&
                    mv -Tf "$release/rollback-link" "$root/current" || restored=0
            else
                rm -f -- "$root/current" || restored=0
            fi
            for unit in lerobot-remote-ik.service lerobot-remote-ik-sim.service; do
                if test -f "$release/previous-units/$unit"; then
                    cp -p "$release/previous-units/$unit" "$units/$unit" || restored=0
                else
                    rm -f -- "$units/$unit" || restored=0
                fi
            done
            systemctl --user daemon-reload || restored=0
            if test "$restored" = 1; then
                rm -f -- "$root/.deployment-incomplete" || restored=0
            fi
            if test "$restored" = 0; then
                echo "Rollback could not be completed. Do not start teleop; inspect $release and repair the installation." >&2
                exit 1
            fi
        fi
        rm -rf -- "$release"
    fi
    exit "$status"
}
trap cleanup EXIT
trap 'exit 1' HUP INT TERM
tar -xzf - -C "$release"
cd "$release"
PYTHONDONTWRITEBYTECODE=1 "$python" -c 'from pathlib import Path; from scripts.remote_ik_motor_service import MotorSession, SimulatedBus; from robot_program.remote_ik_protocol import calibrated_follower, channel_limits; channel_limits(calibrated_follower("unused", Path("calibration").resolve()))' >&2
for mode in sim live; do
    token="$config_dir/$mode.token"
    if [ ! -s "$token" ]; then
        "$python" -c 'import secrets; print(secrets.token_hex(32))' > "$token"
    fi
    chmod 600 "$token"
done
mkdir "$release/previous-units"
for unit in lerobot-remote-ik.service lerobot-remote-ik-sim.service; do
    if test -e "$units/$unit"; then
        cp -p "$units/$unit" "$release/previous-units/$unit"
    fi
done
activating=1
: > "$root/.deployment-incomplete"
for unit in lerobot-remote-ik.service lerobot-remote-ik-sim.service; do
    install -m 600 "$release/$unit" "$units/$unit"
done
ln -s "$release" "$release/current-link"
mv -Tf "$release/current-link" "$root/current"
systemctl --user daemon-reload
printf '%s\n' "$expected" > "$release/.deployment-sha256.new"
mv -T "$release/.deployment-sha256.new" "$release/.deployment-sha256"
rm -f -- "$root/.deployment-incomplete"
committed=1
echo installed
'''


def locked_command(command, *, mode=None):
    """Build a bounded remote transaction; never hold locks on the motor loop."""
    locks = ('exec 8>"$config_dir/deployment.lock"; '
             'flock -x -w 5 8 || { echo "Pi deployment lock is busy; retry after the other launcher finishes." >&2; exit 1; }; ')
    modes = (mode,) if mode else ("live", "sim")
    for fd, name in zip((9, 7), modes):
        locks += (f'exec {fd}>"$config_dir/{name}.lock"; '
                  f'flock -x -w 5 {fd} || {{ echo "Pi {name} lifecycle lock is busy." >&2; exit 1; }}; ')
    return REMOTE_PATHS + locks + "\n" + command


def service(*, simulated):
    mode = "sim" if simulated else "live"
    port = PI_SIM_PORT if simulated else PI_PORT
    config = "%h/.config/lerobot-remote-ik"
    unit = ["[Unit]", f"Description=SO-101 remote IK motor service ({mode})"]
    if not simulated:
        unit.append(f"ConditionPathExists={config}/hardware.env")
    unit.extend(["", "[Service]", "Type=notify", "NoNewPrivileges=yes", "Restart=no", "TimeoutStartSec=20", "TimeoutStopSec=5",
                 "WorkingDirectory=%h/.local/share/lerobot-remote-ik/current"])
    if not simulated:
        unit.append(f"EnvironmentFile={config}/hardware.env")
    command = ("ExecStart=%h/lerobot-env/bin/python -u -m scripts.remote_ik_motor_service "
               f"--token-file {config}/{mode}.token --listen-port {port}")
    if not simulated:
        command += " $REMOTE_IK_HARDWARE_ARGS"
    return "\n".join([*unit, command, ""])


@dataclass(frozen=True)
class Deployment:
    fingerprint: str
    archive: bytes


def build_deployment():
    """Snapshot bytes once; hash content and recipe rather than archive metadata."""
    contents = {name: (ROOT / name).read_bytes() for name in FILES}
    contents["so101_new_calib.urdf"] = kinematic_urdf().encode()
    for suffix, simulated in (("", False), ("-sim", True)):
        contents[f"{PI_UNIT}{suffix}.service"] = service(simulated=simulated).encode()
    digest = hashlib.sha256()
    recipe = ("remote-ik-deployment-v1\n" + locked_command(MATCHES + INSTALL)).encode()
    digest.update(len(recipe).to_bytes(8, "big"))
    digest.update(recipe)
    output = BytesIO()
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for name, body in sorted(contents.items()):
            path = name.encode()
            digest.update(len(path).to_bytes(8, "big"))
            digest.update(path)
            digest.update(len(body).to_bytes(8, "big"))
            digest.update(body)
            member = tarfile.TarInfo(name)
            member.size = len(body)
            member.mode = 0o600
            archive.addfile(member, BytesIO(body))
    return Deployment(digest.hexdigest(), output.getvalue())


def bundle():
    """Compatibility helper for callers inspecting the installer archive."""
    return build_deployment().archive


def release_guard(fingerprint):
    if not re.fullmatch(r"[a-f0-9]{64}", fingerprint):
        raise ValueError("Invalid Pi deployment fingerprint")
    return (f"expected={fingerprint}; " + MATCHES + '\ninstallation_matches || { '
            'echo "Pi release changed or is incomplete. Exit and relaunch teleop to check for updates." >&2; exit 1; }; ')


def ensure_installed(host, *, force=False):
    """Check once at launch; force=True retains the standalone repair command."""
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*", host):
        raise ValueError("--pi must be user@host")
    deployment = build_deployment()

    def run(command, *, statuses, data=None, timeout=25):
        try:
            result = subprocess.run([*SSH, host, "sh -c " + shlex.quote(locked_command(command))],
                                    input=data, capture_output=True, timeout=timeout)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(f"Pi deployment on {host} timed out. Check SSH and the Pi service state, then relaunch.") from exc
        except OSError as exc:
            raise RuntimeError(f"Could not run SSH for Pi deployment on {host}: {exc}") from exc
        if result.returncode:
            detail = result.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(f"Pi deployment failed on {host}: {detail or 'SSH or installation failed'}")
        status = result.stdout.decode("utf-8", errors="replace").strip()
        if status not in statuses:
            raise RuntimeError(f"Unexpected Pi deployment response from {host}: {status!r}")
        return status

    print(f"Checking Pi service on {host} ({deployment.fingerprint[:12]})...", flush=True)
    expected = f"expected={deployment.fingerprint}; "
    if not force:
        status = run(expected + MATCHES + "\nif installation_matches; then echo current; else echo install; fi",
                     statuses={"current", "install"})
        if status == "current":
            print("Pi service is up to date", flush=True)
            return deployment.fingerprint
    print(f"Installing Pi service on {host}...", flush=True)
    status = run(expected + f"force={int(force)}; " + MATCHES + INSTALL,
                 statuses={"installed"} if force else {"installed", "current"}, data=deployment.archive, timeout=60)
    print("Pi service is up to date" if status == "current" else
          f"Installed Pi service ({deployment.fingerprint[:12]})", flush=True)
    return deployment.fingerprint
