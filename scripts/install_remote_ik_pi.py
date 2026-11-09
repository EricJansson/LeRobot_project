"""Install isolated Pi teleop services over SSH. Starts neither service."""

import argparse
from io import BytesIO
import re
import shlex
import subprocess
import tarfile

from scripts.main_teleop_remote_ik import PI_PORT, PI_SIM_PORT, PI_UNIT, ROOT, SSH
from robot_program.remote_ik_protocol import kinematic_urdf


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

INSTALL = r'''
set -eu
umask 077
root="$HOME/.local/share/lerobot-remote-ik"
config="$HOME/.config/lerobot-remote-ik"
units="$HOME/.config/systemd/user"
python="$HOME/lerobot-env/bin/python"
test -x "$python" || { echo 'Expected ~/lerobot-env/bin/python' >&2; exit 1; }
if systemctl --user is-active --quiet lerobot-remote-ik.service; then
    echo 'Stop the live Pi service with the arm supported before updating it.' >&2
    exit 1
fi
mkdir -p "$root/releases" "$config" "$units"
release=$(mktemp -d "$root/releases/release.XXXXXXXX")
tar -xzf - -C "$release"
for mode in sim live; do
    token="$config/$mode.token"
    if [ ! -s "$token" ]; then
        "$python" -c 'import secrets; print(secrets.token_hex(32))' > "$token"
    fi
    chmod 600 "$token"
done
cd "$release"
PYTHONDONTWRITEBYTECODE=1 "$python" -c 'from pathlib import Path; from robot_program.remote_ik_protocol import calibrated_follower, channel_limits; channel_limits(calibrated_follower("unused", Path("calibration").resolve()))'
systemctl --user stop lerobot-remote-ik-sim.service 2>/dev/null || true
install -m 600 "$release/lerobot-remote-ik.service" "$units/lerobot-remote-ik.service"
install -m 600 "$release/lerobot-remote-ik-sim.service" "$units/lerobot-remote-ik-sim.service"
ln -s "$release" "$release/current-link"
mv -Tf "$release/current-link" "$root/current"
systemctl --user daemon-reload
'''


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


def bundle():
    output = BytesIO()
    with tarfile.open(fileobj=output, mode="w:gz") as archive:
        for path in FILES:
            archive.add(ROOT / path, arcname=path)
        urdf = kinematic_urdf().encode()
        member = tarfile.TarInfo("so101_new_calib.urdf")
        member.size = len(urdf)
        member.mode = 0o600
        archive.addfile(member, BytesIO(urdf))
        for suffix, simulated in (("", False), ("-sim", True)):
            body = service(simulated=simulated).encode()
            member = tarfile.TarInfo(f"{PI_UNIT}{suffix}.service")
            member.size = len(body)
            member.mode = 0o600
            archive.addfile(member, BytesIO(body))
    return output.getvalue()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", default="eric@rpi")
    args = parser.parse_args()
    if not re.fullmatch(r"[a-zA-Z0-9_][a-zA-Z0-9_.-]*@[a-zA-Z0-9][a-zA-Z0-9.-]*", args.pi):
        parser.error("--pi must be user@host")
    result = subprocess.run([*SSH, args.pi, "sh -c " + shlex.quote(INSTALL)], input=bundle(), timeout=30)
    if result.returncode:
        parser.exit(result.returncode, "Pi installation failed.\n")
    print(f"Installed on {args.pi}. Run python.exe -m scripts.main_teleop_remote_ik --pi-sim to test.")


if __name__ == "__main__":
    main()
