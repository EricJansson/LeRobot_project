"""Keep Pygame's import output quiet for headless teleop."""

from contextlib import contextmanager
import os
import warnings


@contextmanager
def quiet_pygame_import():
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="pkg_resources is deprecated as an API",
            category=UserWarning,
            module=r"^pygame\.pkgdata$",
        )
        yield
