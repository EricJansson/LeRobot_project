"""Use SDL's device mapping for Sony controllers on Linux; retain other mappings."""

import sys

import pygame


def _sdl_mapping(joystick):
    from pygame._sdl2 import controller

    controller.init()
    return controller.Controller.from_joystick(joystick).get_mapping()


def select_labels(joystick, axis_labels, button_labels):
    """Return raw-index labels without changing the caller's default dictionaries."""
    name = joystick.get_name().lower()
    if not sys.platform.startswith("linux") or not any(
        token in name for token in ("sony", "dualshock")
    ):
        return dict(axis_labels), dict(button_labels)

    try:
        mapping = _sdl_mapping(joystick)
        axes = _indices(mapping, {
            "leftx": "LX", "lefty": "LY", "rightx": "RX", "righty": "RY",
            "lefttrigger": "LT", "righttrigger": "RT",
        }, "a", joystick.get_numaxes())
        buttons = _indices(mapping, {
            "a": "Cross", "b": "Circle", "x": "Square", "y": "Triangle",
            "leftshoulder": "LB", "rightshoulder": "RB", "back": "Back",
            "start": "Start", "leftstick": "L3", "rightstick": "R3",
        }, "b", joystick.get_numbuttons())
    except (pygame.error, KeyError, ValueError) as exc:
        raise RuntimeError(
            f"Cannot determine a complete SDL input mapping for {joystick.get_name()!r}. "
            "Verify the controller mapping before robot control."
        ) from exc
    return axes, buttons


def _indices(mapping, labels, prefix, count):
    result = {}
    for key, label in labels.items():
        binding = mapping[key]
        if not binding.startswith(prefix) or not binding[1:].isdigit():
            raise ValueError(f"Unsupported SDL binding: {key}={binding}")
        index = int(binding[1:])
        if index >= count or index in result:
            raise ValueError(f"Invalid SDL index: {key}={binding}")
        result[index] = label
    return result
