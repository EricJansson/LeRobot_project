"""Replay the Pi's Sony input readings without a joystick or robot attached."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("pygame")

from controller.input import device_mapping, gamepad_mapper


DEFAULT_AXES = {0: "LX", 1: "LY", 2: "RX", 3: "RY", 4: "LT", 5: "RT"}
DEFAULT_BUTTONS = {
    0: "Cross", 1: "Circle", 2: "Square", 3: "Triangle", 4: "LB", 5: "RB",
    6: "Back", 7: "Start", 8: "L3", 9: "R3",
}
# Linux Sony layout represented in the installed SDL mapping database.
SONY_MAPPING = {
    "leftx": "a0", "lefty": "a1", "lefttrigger": "a2",
    "rightx": "a3", "righty": "a4", "righttrigger": "a5",
    "a": "b0", "b": "b1", "x": "b3", "y": "b2",
    "leftshoulder": "b4", "rightshoulder": "b5", "back": "b8", "start": "b9",
    "leftstick": "b11", "rightstick": "b12",
}


class FakeJoystick:
    def __init__(self, name="Sony Computer Entertainment Wireless Controller"):
        self.name = name
        self.axes = [0.004, 0.004, -1.0, 0.004, 0.004, -1.0]
        self.pressed = set()

    def init(self):
        pass

    def get_name(self):
        return self.name

    def get_numaxes(self):
        return 6

    def get_numbuttons(self):
        return 14

    def get_numhats(self):
        return 1

    def get_axis(self, index):
        return self.axes[index]

    def get_button(self, index):
        return index in self.pressed

    def get_hat(self, index):
        return (0, 0)


@pytest.fixture
def sony_gamepad(monkeypatch):
    js = FakeJoystick()
    monkeypatch.setattr(device_mapping, "sys", SimpleNamespace(platform="linux"))
    monkeypatch.setattr(device_mapping, "_sdl_mapping", lambda _: dict(SONY_MAPPING))
    monkeypatch.setattr(gamepad_mapper.pygame, "init", lambda: None)
    monkeypatch.setattr(gamepad_mapper.pygame.joystick, "init", lambda: None)
    monkeypatch.setattr(gamepad_mapper.pygame.joystick, "get_count", lambda: 1)
    monkeypatch.setattr(gamepad_mapper.pygame.joystick, "Joystick", lambda _: js)
    monkeypatch.setattr(gamepad_mapper.pygame.event, "get", lambda: [])
    monkeypatch.setattr(gamepad_mapper.pygame.time, "Clock", Mock)
    gp = gamepad_mapper.Gamepad(
        axis_labels=DEFAULT_AXES, button_labels=DEFAULT_BUTTONS,
        deadman_button="LB", deadzone=0.10,
    )
    state = {label: 0.0 for label in DEFAULT_AXES.values()}
    buttons = []
    gp.set_bindings(gamepad_mapper.Bindings(
        axes={name: lambda label, value: state.__setitem__(label, value) for name in state},
        buttons_down={name: lambda label, value: buttons.append(label) for name in DEFAULT_BUTTONS.values()},
    ))
    return gp, js, state, buttons


def test_neutral_sony_readings_stay_zero_while_lb_held(sony_gamepad):
    gp, js, state, buttons = sony_gamepad
    js.pressed = {4}
    for _ in range(20):
        assert gp.step()
        assert all(value == 0.0 for value in state.values())
    assert buttons == ["LB"]


def test_trigger_is_not_wrist_roll_or_back(sony_gamepad):
    gp, js, state, buttons = sony_gamepad
    js.pressed = {4, 6}
    js.axes[2] = 1.0
    gp.step()
    assert state == {"LX": 0.0, "LY": 0.0, "RX": 0.0, "RY": 0.0, "LT": 1.0, "RT": 0.0}
    assert "Back" not in buttons
    js.axes[2] = -1.0
    js.axes[3] = 0.8
    gp.step()
    assert state["LT"] == 0.0
    assert state["RX"] == 0.8
    js.pressed.clear()
    gp.step()
    assert all(value == 0.0 for value in state.values())


@pytest.mark.parametrize("platform,name", [
    ("win32", "Sony Computer Entertainment Wireless Controller"),
    ("linux", "Other gamepad"),
])
def test_existing_mappings_unchanged_elsewhere(monkeypatch, platform, name):
    monkeypatch.setattr(device_mapping, "sys", SimpleNamespace(platform=platform))
    query = Mock(side_effect=AssertionError("SDL mapping should not be queried"))
    monkeypatch.setattr(device_mapping, "_sdl_mapping", query)
    axes, buttons = device_mapping.select_labels(FakeJoystick(name), DEFAULT_AXES, DEFAULT_BUTTONS)
    assert axes == DEFAULT_AXES and buttons == DEFAULT_BUTTONS
    assert axes is not DEFAULT_AXES and buttons is not DEFAULT_BUTTONS
    query.assert_not_called()


@pytest.mark.parametrize("binding", [None, "a99", "a0", "-a2"])
def test_incomplete_or_unsupported_sony_mapping_is_rejected(monkeypatch, binding):
    monkeypatch.setattr(device_mapping, "sys", SimpleNamespace(platform="linux"))
    mapping = dict(SONY_MAPPING)
    if binding is None:
        del mapping["lefttrigger"]
    else:
        mapping["lefttrigger"] = binding
    monkeypatch.setattr(device_mapping, "_sdl_mapping", lambda _: mapping)
    with pytest.raises(RuntimeError, match="complete SDL input mapping"):
        device_mapping.select_labels(FakeJoystick(), DEFAULT_AXES, DEFAULT_BUTTONS)
