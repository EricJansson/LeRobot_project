"""Receive Android Cartesian intent and marker poses on Windows."""

import asyncio
from concurrent.futures import Future
from contextlib import suppress
from http import HTTPStatus
from math import hypot, isfinite
from secrets import compare_digest
import socket
from struct import Struct
from threading import Lock, Thread
from time import perf_counter as monotonic

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from robot_program.remote_ik_control import LatestInput
from robot_program.remote_ik_protocol import OFFLINE_WATCHDOG


INPUT_PATH = "/api/v1/input"
INPUT_FRAME = Struct("!IBfff")  # uint32 sequence, uint8 flags, float32 X/Y/Z
ARM_INPUT_PATH = "/api/v2/input"
ARM_INPUT_FRAME = Struct("!IBfffffb")  # Sequence, flags, XYZ, roll, pitch, gripper.
POSE_PATH = "/api/v1/marker-pose"
POSE_FRAME = Struct("!IBffffff")  # uint32 sequence, uint8 flags, XYZ cm or normalized, RPY degrees
POSE_FRAME_HEADER = "X-Marker-Pose-Frame"
POSE_XYZ_UNITS_HEADER = "X-Marker-Pose-XYZ-Units"
CAMERA_AXES = "camera"
ROBOT_AXES = "robot-axes"
XYZ_CM = "cm"
XYZ_NORMALIZED = "normalized"
DEADMAN = 1
ADVANCE = 2
STOP = 4
TRACKING = 1
FRESHNESS_S = min(.1, OFFLINE_WATCHDOG / 2)
POSE_STALE_S = .5  # Dashboard receipt age, independent of the motion input deadline.
ARM_STATUS_INTERVAL_S = .1


def mobile_arm_status(control_status):
    if control_status is None:
        return "Monitoring only"
    if control_status.get("state") == "Fault":
        return "Fault"
    state = control_status.get("control_state")
    return state if state in ("Holding", "Control enabled", "Released") else "Unavailable"


class MobileInput:
    def __init__(self, *, clock=monotonic):
        self.clock = clock
        self.latest = LatestInput(clock=clock, allow_torque_release=False)
        self.lock = Lock()
        self.seq = None
        self.xyz = (0., 0., 0.)
        self.wrist = (0., 0.)
        self.gripper = 0
        self.arm_controls = False
        self.started = False

    def begin(self, *, arm_controls=False):
        with self.lock:
            self.seq = None
            self.arm_controls = arm_controls
            self.started = False

    def end(self):
        with self.lock:
            self.latest.publish(None)
            self.started = False

    def _expire(self):
        sample = self.latest.sample
        if sample.gamepad is not None and self.clock() - sample.sampled_at >= FRESHNESS_S:
            self.latest.publish(None)

    @property
    def sample(self):
        with self.lock:
            self._expire()
            return self.latest.sample

    def telemetry(self):
        with self.lock:
            self._expire()
            sample = self.latest.sample
            fresh = sample.gamepad is not None
            result = {"seq": self.seq, "fresh": fresh, "deadman": sample.lb,
                      "xyz": self.xyz if fresh else (0., 0., 0.)}
            if self.arm_controls:
                result.update(wrist=self.wrist if fresh else (0., 0.), gripper=self.gripper if fresh else 0)
            return result

    def publish(self, frame):
        codec = ARM_INPUT_FRAME if self.arm_controls else INPUT_FRAME
        if not isinstance(frame, bytes) or len(frame) != codec.size:
            raise ValueError(f"Expected a {codec.size}-byte binary input frame")
        seq, flags, x, y, z, *extra = codec.unpack(frame)
        xyz = (x, y, z)
        roll, pitch, gripper = extra if self.arm_controls else (0., 0., 0)
        if flags & ~(DEADMAN | ADVANCE | (STOP if self.arm_controls else 0)):
            raise ValueError("Unknown input flags")
        if not all(-1 <= axis <= 1 for axis in (*xyz, roll, pitch)) or gripper not in (-1, 0, 1):
            raise ValueError("Input axes must contain finite values in [-1, 1]")
        deadman, advance = bool(flags & DEADMAN), bool(flags & ADVANCE)
        stop = bool(flags & STOP)
        moving = any((*xyz, roll, pitch, gripper))
        if (advance or stop) and (flags not in (ADVANCE, STOP) or moving):
            raise ValueError("Start and Stop require a neutral input frame")

        with self.lock:
            self._expire()
            if self.seq is not None:
                if self.latest.sample.gamepad is None:
                    raise TimeoutError("Input expired")
                if seq <= self.seq:
                    return False
            elif self.arm_controls and (seq != 0 or flags or moving):
                raise ValueError("Expected a neutral sequence-zero input frame")

            if self.arm_controls:
                if stop:
                    self.started = False
                elif advance:
                    self.started = True
                deadman = deadman and self.started
            magnitude = max(1., hypot(*xyz))
            x, y, z = (axis / magnitude for axis in xyz)
            # Adapt Cartesian input to the existing IK control loop's input boundary.
            self.latest.publish({"name": "Phone", "axes": {"LY": -x, "LX": -y, "RY": -z,
                                                          "LT": max(0., -pitch), "RT": max(0., pitch), "RX": roll},
                                 "buttons": {"LB": deadman, "Start": advance, "Back": stop},
                                 "dpad": (0, gripper)})
            self.seq, self.xyz, self.wrist, self.gripper = seq, xyz, (roll, pitch), gripper
            return True


class MarkerPose:
    """Track marker status and retain the last detected pose for display."""

    def __init__(self, *, clock=monotonic):
        self.clock = clock
        self.lock = Lock()
        self.coordinate_frame = None
        self.xyz_units = None
        self.seq = None
        self.received_at = None
        self.last_pose = None
        self.tracking = False

    def begin(self, coordinate_frame=CAMERA_AXES, xyz_units=XYZ_CM):
        with self.lock:
            self.coordinate_frame = coordinate_frame
            self.xyz_units = xyz_units
            self.seq = self.received_at = self.last_pose = None
            self.tracking = False

    def end(self):
        with self.lock:
            self.coordinate_frame = self.xyz_units = self.last_pose = None
            self.tracking = False

    def publish(self, frame):
        if not isinstance(frame, bytes) or len(frame) != POSE_FRAME.size:
            raise ValueError("Expected a 29-byte binary marker pose")
        seq, flags, *values = POSE_FRAME.unpack(frame)
        if flags & ~TRACKING:
            raise ValueError("Unknown marker pose flags")
        if not all(isfinite(value) for value in values):
            raise ValueError("Marker pose values must be finite")
        with self.lock:
            if self.seq is not None and seq <= self.seq:
                return False
            self.seq = seq
            self.received_at = self.clock()
            self.tracking = bool(flags & TRACKING)
            if self.tracking:
                self.last_pose = tuple(values)
            return True

    def telemetry(self):
        with self.lock:
            if self.coordinate_frame is None or self.received_at is None:
                state = "Disconnected" if self.seq is not None else "Waiting for marker pose"
            elif self.clock() - self.received_at >= POSE_STALE_S:
                state = "Pose timed out"
            else:
                state = "Tracking" if self.tracking else "Marker lost"
            pose = self.last_pose
            return {"state": state, "seq": self.seq, "frame": self.coordinate_frame, "xyz_units": self.xyz_units,
                    "xyz_cm": pose[:3] if pose is not None else None,
                    "rpy_deg": pose[3:] if pose is not None else None}


class MobileGateway:
    def __init__(self, host, port, feed, token, *, arm_status=lambda: "Monitoring only"):
        self.host, self.port = host, port
        self.feed = feed
        self.marker_pose = MarkerPose()
        self.authorization = f"Bearer {token}".encode()
        self.arm_status = arm_status
        self._session_lock = Lock()
        self._phone_mode = None
        self.ready = Future()

    @property
    def active(self):
        with self._session_lock:
            return self._phone_mode is not None

    def telemetry(self):
        with self._session_lock:
            return {"phone_mode": self._phone_mode, "phone": self.feed.telemetry(),
                    "marker_pose": self.marker_pose.telemetry()}

    def _authorize(self, connection, request):
        if request.path not in (INPUT_PATH, ARM_INPUT_PATH, POSE_PATH):
            return connection.respond(HTTPStatus.NOT_FOUND, "Unknown endpoint\n")
        headers = request.headers.get_all("Authorization")
        if len(headers) != 1 or not compare_digest(headers[0].encode(), self.authorization):
            return connection.respond(HTTPStatus.UNAUTHORIZED, "Bearer token required\n")
        if request.path == POSE_PATH:
            frames = request.headers.get_all(POSE_FRAME_HEADER)
            if frames and frames != [ROBOT_AXES]:
                return connection.respond(HTTPStatus.BAD_REQUEST, "Invalid marker pose frame header\n")
            units = request.headers.get_all(POSE_XYZ_UNITS_HEADER)
            if units and units not in ([XYZ_CM], [XYZ_NORMALIZED]):
                return connection.respond(HTTPStatus.BAD_REQUEST, "Invalid marker pose XYZ units header\n")
        if self.active:
            return connection.respond(HTTPStatus.CONFLICT, "Phone input already connected\n")
        return None

    async def _receive(self, connection):
        if self.active:
            await connection.close(1008, "Phone input already connected")
            return
        is_pose = connection.request.path == POSE_PATH
        arm_controls = connection.request.path == ARM_INPUT_PATH
        with self._session_lock:
            if is_pose:
                feed = self.marker_pose
                headers = connection.request.headers
                feed.begin(headers.get(POSE_FRAME_HEADER, CAMERA_AXES), headers.get(POSE_XYZ_UNITS_HEADER, XYZ_CM))
            else:
                feed = self.feed
                feed.begin(arm_controls=arm_controls)
            self._phone_mode = "aruco" if is_pose else "arm" if arm_controls else "xyz"
        loop = asyncio.get_running_loop()
        # A pose viewer can wait for the marker. Only control has a receive deadline.
        deadline = None if is_pose else loop.time() + 2
        close_reason = None
        status_task = asyncio.create_task(self._report_arm_status(connection)) if arm_controls else None
        try:
            while True:
                if deadline is not None and loop.time() >= deadline:
                    raise TimeoutError("Input expired")
                async with asyncio.timeout_at(deadline):
                    frame = await connection.recv()
                if feed.publish(frame) and not is_pose:
                    deadline = loop.time() + FRESHNESS_S
        except ConnectionClosed:
            pass
        except (TimeoutError, ValueError) as exc:
            close_reason = str(exc) or "Input expired"
        finally:
            with self._session_lock:
                feed.end()
                self._phone_mode = None
            if status_task is not None:
                status_task.cancel()
                with suppress(asyncio.CancelledError, ConnectionClosed):
                    await status_task
        if close_reason is not None:
            await connection.close(1008, close_reason)

    async def _report_arm_status(self, connection):
        while True:
            await connection.send(f"arm-status:{self.arm_status()}")
            await asyncio.sleep(ARM_STATUS_INTERVAL_S)

    async def _serve(self):
        self.loop = asyncio.get_running_loop()
        self.stop = asyncio.Event()
        async with serve(self._receive, self.host, self.port, family=socket.AF_INET, origins=[None],
                         compression=None, max_size=POSE_FRAME.size, max_queue=1,
                         open_timeout=2, close_timeout=.1, process_request=self._authorize) as server:
            self.address, self.port = server.sockets[0].getsockname()
            self.ready.set_result(None)
            await self.stop.wait()

    def _run(self):
        try:
            asyncio.run(self._serve())
        except Exception as exc:
            if self.ready.done():
                raise
            self.ready.set_exception(exc)

    def __enter__(self):
        self.thread = Thread(target=self._run, name="Mobile input", daemon=True)
        self.thread.start()
        try:
            self.ready.result()
        except BaseException:
            self.ready.add_done_callback(self._stop_if_started)
            raise
        return self

    def _stop_if_started(self, ready):
        if ready.exception() is None:
            self._signal_stop()

    def _signal_stop(self):
        self.loop.call_soon_threadsafe(self.stop.set)

    def __exit__(self, *_):
        if self.thread.is_alive():
            self._signal_stop()
        self.thread.join()
