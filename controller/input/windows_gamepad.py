"""Select a Windows gamepad backend and keep it for the teleop session."""

from contextlib import ExitStack, contextmanager
from time import monotonic

from controller.input.xinput import XInputGamepad


@contextmanager
def open_gamepad(backend, index):
    """Yield a polling function that returns None while input is unavailable."""
    if backend == "xinput":
        yield XInputGamepad(0 if index is None else index).read
        return

    with ExitStack() as stack:
        if backend == "sdl":
            yield _open_sdl(stack, 0 if index is None else index).read
            return

        slots = range(4) if index is None else (index,)
        xinput_pads = [XInputGamepad(slot) for slot in slots if slot < 4]
        read_selected = None
        next_probe = 0.

        def read():
            nonlocal read_selected, next_probe
            # Once selected, leave controller loss handling to the teleop input loop.
            if read_selected is not None:
                return read_selected()
            now = monotonic()
            if now < next_probe:
                return None
            next_probe = now + 1.

            for pad in xinput_pads:
                sample = pad.read()
                if sample is not None:
                    read_selected = pad.read
                    print(f"Gamepad: {sample['name']}", flush=True)
                    return sample

            from controller.input.sdl_gamepad import NoSDLGamepad

            try:
                pad = _open_sdl(stack, index)
            except NoSDLGamepad:
                return None
            read_selected = pad.read
            return read_selected()

        yield read


def _open_sdl(stack, index):
    from controller.input.sdl_gamepad import open_sdl_gamepad

    pad = stack.enter_context(open_sdl_gamepad(index))
    print(f"Gamepad: {pad.name} (SDL index {pad.index}). Relaunch if disconnected.", flush=True)
    return pad
