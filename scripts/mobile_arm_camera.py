"""Start and check the Pi camera used by Phone Vision touchscreen control."""

from contextlib import closing
from http.client import HTTPConnection, HTTPException
from ipaddress import IPv4Address, IPv4Network
import json
import shlex
import subprocess
from time import monotonic, sleep

from scripts.remote_ik_deploy import SSH


LAN_NETWORKS = tuple(IPv4Network(cidr) for cidr in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16"))
CAMERA_UNIT = "lerobot-arm-camera.service"
CAMERA_PORT = 8080


def _ssh(pi, command):
    result = subprocess.run([*SSH, "-4", pi, "sh -c " + shlex.quote(command)], stdin=subprocess.DEVNULL,
                            capture_output=True, encoding="utf-8", errors="replace", timeout=10)
    if result.returncode:
        raise RuntimeError(result.stderr.strip() or f"Camera setup command failed on {pi}")
    return result.stdout.strip()


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
    connection = _ssh(pi, 'printf "%s\\n" "$SSH_CONNECTION"').split()
    if len(connection) != 4:
        raise RuntimeError(f"Could not find the Pi's LAN address from SSH to {pi}")
    address = IPv4Address(connection[2])
    if not any(address in network for network in LAN_NETWORKS):
        raise RuntimeError(f"Use --pi with the Pi's private LAN IPv4 address for video (SSH reached {address})")
    host = str(address)
    url = f"http://{host}:{CAMERA_PORT}/stream?extra_headers=1"
    status = _camera_status(host)
    if status is None:
        args = shlex.join([f"--device={device}", "--format=MJPEG", "--encoder=HW", "--resolution=1280x720",
                           "--desired-fps=30", f"--host={host}", f"--port={CAMERA_PORT}", "--tcp-nodelay"])
        _ssh(pi, f"if ! systemctl --user is-active --quiet {CAMERA_UNIT}; then "
                 'binary=$(command -v ustreamer) || { echo "Install ustreamer on the Pi first." >&2; exit 1; }; '
                 f'systemd-run --user --collect --service-type=exec --unit={CAMERA_UNIT} "$binary" {args} '
                 f"|| systemctl --user is-active --quiet {CAMERA_UNIT}; fi")

    deadline = monotonic() + 5
    while status is not True and monotonic() < deadline:
        sleep(.25)
        status = _camera_status(host)
    if status is True:
        return url
    if status is False:
        detail = f"µStreamer is reachable at {host}:{CAMERA_PORT}, but {device} is producing no video."
    else:
        detail = f"µStreamer did not answer at {host}:{CAMERA_PORT}."
    raise RuntimeError(f"{detail} On the Pi, check journalctl --user -u {CAMERA_UNIT} -n 20.")
