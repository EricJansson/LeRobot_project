"""Receive Android Cartesian intent and marker poses on Windows."""

import asyncio
from concurrent.futures import Future
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
TRACKING = 1
FRESHNESS_S = min(.1, OFFLINE_WATCHDOG / 2)
POSE_STALE_S = .5  # Dashboard receipt age, independent of the motion input deadline.


class MobileInput:
    def __init__(self, *, clock=monotonic):
        self.clock = clock
        self.latest = LatestInput(clock=clock)
        self.lock = Lock()
        self.seq = None
        self.xyz = (0., 0., 0.)

    def begin(self):
        with self.lock:
            self.seq = None

    def end(self):
        with self.lock:
            self.latest.publish(None)

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
            return {"seq": self.seq, "fresh": sample.gamepad is not None, "deadman": sample.lb,
                    "xyz": self.xyz if sample.gamepad is not None else (0., 0., 0.)}

    def publish(self, frame):
        if not isinstance(frame, bytes) or len(frame) != INPUT_FRAME.size:
            raise ValueError("Expected a 17-byte binary input frame")
        seq, flags, *xyz = INPUT_FRAME.unpack(frame)
        if flags & ~(DEADMAN | ADVANCE):
            raise ValueError("Unknown input flags")
        if not all(-1 <= axis <= 1 for axis in xyz):
            raise ValueError("XYZ must contain finite values in [-1, 1]")
        deadman, advance = bool(flags & DEADMAN), bool(flags & ADVANCE)
        if advance and (deadman or any(xyz)):
            raise ValueError("Advance requires neutral XYZ with deadman released")

        with self.lock:
            self._expire()
            if self.seq is not None:
                if self.latest.sample.gamepad is None:
                    raise TimeoutError("Input expired")
                if seq <= self.seq:
                    return False
            magnitude = max(1., hypot(*xyz))
            x, y, z = (axis / magnitude for axis in xyz)
            # Adapt Cartesian input to the existing IK control loop's input boundary.
            self.latest.publish({"name": "Phone", "axes": {"LY": -x, "LX": -y, "RY": -z,
                                                          "LT": 0., "RT": 0., "RX": 0.},
                                 "buttons": {"LB": deadman, "Start": advance, "Back": False}, "dpad": (0, 0)})
            self.seq, self.xyz = seq, tuple(xyz)
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
    def __init__(self, host, port, feed, token):
        self.host, self.port = host, port
        self.feed = feed
        self.marker_pose = MarkerPose()
        self.authorization = f"Bearer {token}".encode()
        self.active = False
        self.ready = Future()

    def _authorize(self, connection, request):
        if request.path not in (INPUT_PATH, POSE_PATH):
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
        self.active = True
        is_pose = connection.request.path == POSE_PATH
        if is_pose:
            feed = self.marker_pose
            headers = connection.request.headers
            feed.begin(headers.get(POSE_FRAME_HEADER, CAMERA_AXES), headers.get(POSE_XYZ_UNITS_HEADER, XYZ_CM))
        else:
            feed = self.feed
            feed.begin()
        loop = asyncio.get_running_loop()
        # A pose viewer can wait for the marker. Only control has a receive deadline.
        deadline = None if is_pose else loop.time() + 2
        close_reason = None
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
            feed.end()
            self.active = False
        if close_reason is not None:
            await connection.close(1008, close_reason)

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
