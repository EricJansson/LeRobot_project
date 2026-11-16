"""Receive the latest Android Cartesian intent on the Windows PC."""

import asyncio
from concurrent.futures import Future
from http import HTTPStatus
from math import hypot
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
DEADMAN = 1
ADVANCE = 2
FRESHNESS_S = min(.1, OFFLINE_WATCHDOG / 2)


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


class MobileGateway:
    def __init__(self, host, port, feed, token):
        self.host, self.port = host, port
        self.feed = feed
        self.authorization = f"Bearer {token}".encode()
        self.active = False
        self.ready = Future()

    def _authorize(self, connection, request):
        if request.path != INPUT_PATH:
            return connection.respond(HTTPStatus.NOT_FOUND, "Unknown endpoint\n")
        headers = request.headers.get_all("Authorization")
        if len(headers) != 1 or not compare_digest(headers[0].encode(), self.authorization):
            return connection.respond(HTTPStatus.UNAUTHORIZED, "Bearer token required\n")
        if self.active:
            return connection.respond(HTTPStatus.CONFLICT, "Phone input already connected\n")
        return None

    async def _receive(self, connection):
        if self.active:
            await connection.close(1008, "Phone input already connected")
            return
        self.active = True
        self.feed.begin()
        loop = asyncio.get_running_loop()
        deadline = loop.time() + 2  # Allow the first frame to arrive after pairing.
        close_reason = None
        try:
            while True:
                if loop.time() >= deadline:
                    raise TimeoutError("Input expired")
                async with asyncio.timeout_at(deadline):
                    frame = await connection.recv()
                if self.feed.publish(frame):
                    deadline = loop.time() + FRESHNESS_S
        except ConnectionClosed:
            pass
        except (TimeoutError, ValueError) as exc:
            close_reason = str(exc) or "Input expired"
        finally:
            self.feed.end()
            self.active = False
        if close_reason is not None:
            await connection.close(1008, close_reason)

    async def _serve(self):
        self.loop = asyncio.get_running_loop()
        self.stop = asyncio.Event()
        async with serve(self._receive, self.host, self.port, family=socket.AF_INET, origins=[None],
                         compression=None, max_size=INPUT_FRAME.size, max_queue=1,
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
