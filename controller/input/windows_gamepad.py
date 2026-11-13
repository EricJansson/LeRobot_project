"""Select a Windows gamepad and rediscover it after input is lost."""

from contextlib import ExitStack, contextmanager
from time import monotonic

from controller.input.xinput import XInputGamepad


PROBE_INTERVAL_S = 1.


@contextmanager
def open_gamepad(backend, index):
    """Yield a polling function that returns None while input is unavailable."""
    if backend != "auto" and index is None:
        index = 0
    slots = range(4) if index is None else (index,)
    xinput_pads = [XInputGamepad(slot) for slot in slots if slot < 4 and backend != "sdl"]

    with ExitStack() as stack:
        read_selected = None
        if backend == "sdl":
            from controller.input.sdl_gamepad import open_sdl_gamepad

            read_selected = stack.enter_context(open_sdl_gamepad(index)).read
        next_probe = 0.

        def read():
            nonlocal read_selected, next_probe
            if read_selected is not None:
                sample = read_selected()
                if sample is not None:
                    return sample
                read_selected = None
                next_probe = monotonic() + PROBE_INTERVAL_S
                # Publish input loss before closing the old handle or probing another device.
                return None

            now = monotonic()
            if now < next_probe:
                return None
            next_probe = now + PROBE_INTERVAL_S
            stack.close()

            for pad in xinput_pads:
                sample = pad.read()
                if sample is not None:
                    read_selected = pad.read
                    return sample

            if backend == "xinput":
                return None

            from controller.input.sdl_gamepad import NoSDLGamepad, open_sdl_gamepad

            try:
                pad = stack.enter_context(open_sdl_gamepad(index))
            except NoSDLGamepad:
                return None
            sample = pad.read()
            if sample is not None:
                read_selected = pad.read
            return sample

        yield read
