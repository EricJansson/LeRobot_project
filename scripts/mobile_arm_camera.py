"""Start and check the Pi cameras used by Phone Vision touchscreen control."""

from contextlib import closing
from http.client import HTTPConnection, HTTPException
from ipaddress import IPv4Address, IPv4Network
import json
from pathlib import Path
import shlex
import subprocess
from time import monotonic, sleep

from scripts.remote_ik_deploy import SSH


LAN_NETWORKS = tuple(IPv4Network(cidr) for cidr in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))
CAMERA_UNIT = "lerobot-arm-camera.service"
CAMERA_PORT = 8080
RPI_CAMERA_UNIT = "lerobot-rpi-camera.service"
RPI_CAMERA_PORT = 8081


def _ssh(pi, command, *, data=None):
    result = subprocess.run([*SSH, "-4", pi, "sh -c " + shlex.quote(command)],
                            stdin=subprocess.DEVNULL if data is None else None, input=data,
                            capture_output=True, timeout=10)
    if result.returncode:
        detail = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(detail or f"Camera setup command failed on {pi}")
    return result.stdout.decode("utf-8", errors="replace").strip()


def _private_pi_address(pi):
    connection = _ssh(pi, 'printf "%s\\n" "$SSH_CONNECTION"').split()
    if len(connection) != 4:
        raise RuntimeError(f"Could not find the Pi's LAN address from SSH to {pi}")
    address = IPv4Address(connection[2])
    if not any(address in network for network in LAN_NETWORKS):
        raise RuntimeError(f"Use --pi with the Pi's private LAN IPv4 address for video (SSH reached {address})")
    return str(address)


def _camera_status(address):
    """Return None if µStreamer is unreachable, or whether its source is producing frames."""
    try:
        with closing(HTTPConnection(address, CAMERA_PORT, timeout=1)) as connection:
            connection.request("GET", "/state")
            response = connection.getresponse()
            if response.status != 200:
                return None
            state = json.loads(response.read(64 * 1024))
        source = state["result"]["source"]
        online = source["online"]
        fps = source["captured_fps"]
        if state["ok"] is True and type(online) is bool and type(fps) is int:
            return online and fps > 0
    except (OSError, HTTPException, ValueError, KeyError, TypeError):
        pass
    return None


def ensure_arm_camera(pi, device):
    """Return the verified MJPEG URL, starting a separate Pi user service if needed."""
    host = _private_pi_address(pi)
    url = f"http://{host}:{CAMERA_PORT}/stream?extra_headers=1"
    status = _camera_status(host)
    if status is None:
        device_setup = ""
        if device is None:
            device_setup = ('device=/dev/video0; '
                            'for candidate in /dev/v4l/by-id/usb-*Innomaker*-video-index0; do '
                            'if test -c "$candidate"; then device=$candidate; break; fi; done; ')
        device_arg = '"--device=$device"' if device is None else shlex.quote(f"--device={device}")
        args = shlex.join(["--format=MJPEG", "--encoder=HW", "--resolution=1280x720",
                           "--desired-fps=30", f"--host={host}", f"--port={CAMERA_PORT}", "--tcp-nodelay"])
        _ssh(pi, f"if ! systemctl --user is-active --quiet {CAMERA_UNIT}; then "
                 'binary=$(command -v ustreamer) || { echo "Install ustreamer on the Pi first." >&2; exit 1; }; '
                 f'{device_setup}systemd-run --user --collect --service-type=exec --unit={CAMERA_UNIT} '
                 f'"$binary" {device_arg} {args} '
                 f"|| systemctl --user is-active --quiet {CAMERA_UNIT}; fi")

    deadline = monotonic() + 5
    while status is not True and monotonic() < deadline:
        sleep(.25)
        status = _camera_status(host)
    if status is True:
        return url
    if status is False:
        detail = f"µStreamer is reachable at {host}:{CAMERA_PORT}, but {device or 'the USB camera'} is producing no video."
    else:
        detail = f"µStreamer did not answer at {host}:{CAMERA_PORT}."
    raise RuntimeError(f"{detail} On the Pi, check journalctl --user -u {CAMERA_UNIT} -n 20.")


def _rpi_camera_is_live(host):
    try:
        with closing(HTTPConnection(host, RPI_CAMERA_PORT, timeout=1)) as connection:
            connection.request("GET", "/health")
            response = connection.getresponse()
            return response.status == 200 and response.read(64) == b"lerobot-rpi-camera\n"
    except (OSError, HTTPException):
        return False


def ensure_rpi_camera(pi):
    """Start or reuse the CSI camera service and return its verified MJPEG URL."""
    host = _private_pi_address(pi)
    if not _rpi_camera_is_live(host):
        source = Path(__file__).with_name("pi_camera_stream.py").read_bytes()
        _ssh(pi, f"set -eu; if systemctl --user is-active --quiet {RPI_CAMERA_UNIT}; then exit 0; fi; "
                 '/usr/bin/python3 -c "import picamera2" || '
                 '{ echo "Check python3-picamera2 is installed on the Pi." >&2; exit 1; }; '
                 'dir="$HOME/.local/share/lerobot/mobile-camera"; mkdir -p -- "$dir"; '
                 'tmp=$(mktemp "$dir/pi_camera_stream.XXXXXXXX"); trap \'rm -f -- "$tmp"\' EXIT; '
                 'cat > "$tmp"; mv -f -- "$tmp" "$dir/pi_camera_stream.py"; '
                 f"systemd-run --user --collect --service-type=exec --unit={RPI_CAMERA_UNIT} "
                 f'/usr/bin/python3 "$dir/pi_camera_stream.py" {host} {RPI_CAMERA_PORT} '
                 f"|| systemctl --user is-active --quiet {RPI_CAMERA_UNIT}", data=source)

        deadline = monotonic() + 5
        while not _rpi_camera_is_live(host):
            if monotonic() >= deadline:
                raise RuntimeError(f"No live frames at {host}:{RPI_CAMERA_PORT}. On the Pi, stop any manual CSI "
                                   f"capture and check journalctl --user -u {RPI_CAMERA_UNIT} -n 20.")
            sleep(.25)

    return f"http://{host}:{RPI_CAMERA_PORT}/stream"
