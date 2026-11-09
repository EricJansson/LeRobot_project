"""Shared, bounded messages and model metadata for PC-to-Pi teleop."""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from importlib.metadata import version
from pathlib import Path

from robot_program.lerobot_ik_controller import ARM_JOINTS, DEFAULT_URDF, GRIPPER_RANGE, _joint_limits


PROTOCOL_VERSION = 3
MOTION_POLICY = "adaptive-limits-v3"
COMMAND_MARGIN_DEG = .5
RECOVERY_RATE_DEG_S = 5.
RECOVERY_MAX_MARGIN_DEG = 2.
RECOVERY_DEEPEN_S = .5
MAX_FRAME = 4096
CHANNELS = (*ARM_JOINTS, "gripper")
PI_PORT = 8766
RATE_HZ = 50
OFFLINE_WATCHDOG = 0.15


def encode_frame(message):
    frame = json.dumps(message, allow_nan=False, separators=(",", ":")).encode() + b"\n"
    if len(frame) > MAX_FRAME:
        raise ValueError("Frame exceeds size limit")
    return frame


def decode_frame(frame):
    if not frame or len(frame) > MAX_FRAME or not frame.endswith(b"\n"):
        raise ValueError("Missing, incomplete, or oversized frame")
    message = json.loads(frame)
    if not isinstance(message, dict):
        raise ValueError("Expected a message object")
    return message


def finite_vector(values, size):
    return (isinstance(values, list) and len(values) == size
            and all(type(value) in (int, float) and abs(value) < math.inf for value in values))


def kinematic_urdf(source=DEFAULT_URDF):
    root = ET.parse(source).getroot()
    for link in root.findall("link"):
        for element in list(link):
            if element.tag in {"visual", "collision"}:
                link.remove(element)
    if root.find(".//mesh") is not None:
        raise ValueError("The IK model still references a mesh")
    return ET.canonicalize(ET.tostring(root), strip_text=True)


def model_id(urdf, calibration_dir):
    calibration = json.loads((Path(calibration_dir) / "lerobot_arm.json").read_text(encoding="utf-8"))
    canonical_calibration = json.dumps(calibration, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256((urdf + "\n" + canonical_calibration).encode()).hexdigest()


def calibrated_follower(port, calibration_dir):
    """Construct only. No serial access, torque changes or calibration writes."""
    if version("lerobot") != "0.6.1":
        raise RuntimeError("Remote IK requires LeRobot 0.6.1")
    from robot_program.lerobot_compat import SO101FollowerConfig
    from robot_program.so101_follower import SO101Follower

    return SO101Follower(SO101FollowerConfig(
        port=port, id="lerobot_arm", use_degrees=True, calibration_dir=calibration_dir,
    ))


def channel_limits(follower):
    return [*_joint_limits(follower.bus, DEFAULT_URDF).tolist(), list(GRIPPER_RANGE)]


def calibrated_channel_limits(follower):
    """Motor endpoints in bus units, before intersecting the URDF ranges."""
    bus = follower.bus
    ranges = []
    for name in ARM_JOINTS:
        motor, calibration = bus.motors[name], bus.calibration[name]
        ranges.append([float(bus._normalize({motor.id: endpoint})[motor.id])
                       for endpoint in (calibration.range_min, calibration.range_max)])
    return [*ranges, list(GRIPPER_RANGE)]


def command_limits(limits):
    return [[lo + COMMAND_MARGIN_DEG, hi - COMMAND_MARGIN_DEG] if i < 5 else [lo, hi]
            for i, (lo, hi) in enumerate(limits)]


def within_limits(values, limits):
    return (finite_vector(values, 6)
            and all(lo <= value <= hi for value, (lo, hi) in zip(values, limits)))


def limit_excess(values, limits):
    return [max(lo - value, 0., value - hi) for value, (lo, hi) in zip(values, limits)]
