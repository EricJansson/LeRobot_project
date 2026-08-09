"""
dashboard_panels.py

Reusable pygame panels for the Controller Dashboard.

Currently provides:
  - `ManualJointPanel`: lets the user type hard per-motor goal angles (degrees),
    validates them live against the calibrated motor limits, and sends the goal
    to the robot owner (teleop_ik_control) via a small JSON command file.

Two IPC files separate the directions of communication:

  Command file   (dashboard -> teleop): statuses "active" / "muted" / "pending".
      - "active"  : controller input is live (default)
      - "muted"   : a text field is focused -> teleop must ignore gamepad
      - "pending" : "joints" holds the 6 goal angles -> teleop should run

  Telemetry file (teleop -> dashboard): statuses "telemetry" / "done".
      - "telemetry": live "joints" so the panel tracks the arm's true position
      - "done"     : implies the "pending" command was applied; "joints" holds
                     the achieved angles (also re-populates the fields)

This module never touches the serial port - teleop owns the hardware.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import pygame

# Ordered like RobotArmController.JOINT_NAMES
DEFAULT_MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)

# ---------------------------------------------------------------- IPC helpers

def runtime_dir() -> Path:
    """A project-local runtime folder (kept out of the OS temp dir).

    The system temp dir (tempfile.gettempdir()) caused intermittent
    PermissionError when the two processes raced on os.replace, so the IPC
    files now live inside the project where we have predictable access.
    """
    d = Path(__file__).resolve().parent.parent / ".runtime"
    d.mkdir(parents=True, exist_ok=True)
    return d


def default_command_file() -> Path:
    """File path used by the dashboard to send commands to teleop."""
    return runtime_dir() / "lerobot_dashboard_cmd.json"


def default_telemetry_file() -> Path:
    """File path used by teleop to stream live positions back to the dashboard."""
    return runtime_dir() / "lerobot_dashboard_telemetry.json"


def write_json(path, payload: dict) -> None:
    """Write a JSON payload, best-effort and race-tolerant.

    An atomic-ish tmp+rename is used when possible. On Windows a transient
    PermissionError can occur if the peer process is momentarily reading the
    file (os.replace can't overwrite an open destination). We retry briefly and,
    if that still fails, fall back to a plain write so the caller never crashes.
    """
    data = dict(payload)
    data["ts"] = time.time()
    path = Path(path)
    tmp = path.with_name(f".{path.name}.{time.time_ns()}.tmp")

    def _write(f):
        json.dump(data, f)

    # Try an atomic replace, with a few quick retries to ride out file locks.
    for attempt in range(5):
        try:
            with open(tmp, "w") as f:
                _write(f)
            os.replace(tmp, path)
            return
        except OSError:
            try:
                os.remove(tmp)
            except OSError:
                pass
            if attempt == 4:
                break
            time.sleep(0.004)
        except Exception:
            try:
                os.remove(tmp)
            except OSError:
                pass
            raise

    # Last resort: direct (non-atomic) write so we degrade gracefully.
    try:
        with open(path, "w") as f:
            _write(f)
    except OSError:
        pass


def read_json(path) -> Optional[dict]:
    """Read a JSON file, or None if missing / corrupt."""
    try:
        with open(path, "r") as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError, OSError):
        return None


# Kept for backwards-compatibility with earlier naming.
write_command = write_json
read_command = read_json


# ---------------------------------------------------------------- calibration

def read_motor_limits(
    calibration_path: Optional[Path] = None,
) -> Dict[str, Tuple[float, float]]:
    """
    Read per-motor (degree_min, degree_max) from the existing calibration JSON.

    Uses the project's calibration file. No new file is created; this is just a
    read so we never drift from a reconfigured calibration.
    """
    if calibration_path is None:
        root = Path(__file__).resolve().parent.parent
        calibration_path = root / "calibration" / "lerobot_arm_with_degrees.json"
    else:
        calibration_path = Path(calibration_path)

    with open(calibration_path, "r") as f:
        config = json.load(f)

    limits: Dict[str, Tuple[float, float]] = {}
    for m in config.get("motors", []):
        name = m["name"]
        limits[name] = (float(m["degree_min"]), float(m["degree_max"]))
    return limits


def motor_names_from_limits(limits: Dict[str, Tuple[float, float]]) -> List[str]:
    """Order motors by the canonical JOINT order, keeping only those with limits."""
    ordered = [n for n in DEFAULT_MOTOR_NAMES if n in limits]
    ordered += [n for n in limits if n not in DEFAULT_MOTOR_NAMES]
    return ordered


# ---------------------------------------------------------------- the panel

class ManualJointPanel:
    """
    Six text fields (one per motor) + Act button + live validation.

    - Fields are continuously driven by live "telemetry" joint positions, so
      they always show the arm's current angles (until the user edits one).
    - Focusing a field clears it (the previous value is remembered). If the
      field is left empty on blur, the remembered value is restored.
    - Act is only enabled when every field parses AND is within its calibrated
      [min, max]. On activation it writes a "pending" command.
    - `focused` is True while a field is focused -> caller should mute the
      controller. The panel draws a prominent ACTIVE / MUTED indicator.
    """

    FRAME_X, FRAME_Y = 590, 455
    FRAME_W, FRAME_H = 400, 215

    def __init__(
        self,
        cmd_file: Path,
        telemetry_file: Path,
        limits: Dict[str, Tuple[float, float]],
        font,
        small_font,
        panel_rect=pygame.Rect(590, 455, 400, 215),
    ) -> None:
        self.cmd_file = Path(cmd_file)
        self.telemetry_file = Path(telemetry_file)
        self.font = font
        self.small = small_font
        self.panel_rect = pygame.Rect(panel_rect)

        self.names: List[str] = motor_names_from_limits(limits)
        self.limits = limits

        # text buffers + validity + whether the field is live-driven (unedited)
        self.buffers: Dict[str, str] = {n: "" for n in self.names}
        self.valid: Dict[str, bool] = {n: False for n in self.names}
        self.auto: Dict[str, bool] = {n: True for n in self.names}
        # Whether a field's text is "selected" (next typed char overwrites it).
        self.select: Dict[str, bool] = {n: False for n in self.names}

        self.focus_index: Optional[int] = None
        self.last_status = ""
        self.last_status_color = (180, 180, 185)

        # layout
        self.field_height = 20
        self.row_h = 22
        self.name_w = 118
        self.field_w = 68

        self._lazy_build_rects()

    # ---------------------------------------------------------- geometry
    def _lazy_build_rects(self) -> None:
        px, py = self.panel_rect.topleft
        self.field_rects: Dict[str, pygame.Rect] = {}
        self.rowoffsets: Dict[str, int] = {}
        x0 = px + 8
        y0 = py + 30

        for i, name in enumerate(self.names):
            ry = y0 + i * self.row_h
            self.field_rects[name] = pygame.Rect(x0 + self.name_w, ry, self.field_w, self.field_height)
            self.rowoffsets[name] = (x0, ry)

        # Act button (+ status text beside it)
        self.act_rect = pygame.Rect(px + 8, y0 + len(self.names) * self.row_h + 8, 110, 26)

    # ---------------------------------------------------------- public state
    @property
    def focused(self) -> bool:
        """True if a text field currently has focus (controller should mute)."""
        return self.focus_index is not None

    def _parse(self, name: str) -> Optional[float]:
        try:
            return float(self.buffers[name].strip() or "nan")
        except ValueError:
            return None

    def _in_range(self, name: str, value: float) -> bool:
        lo, hi = self.limits[name]
        return lo <= value <= hi

    def _revalidate(self, name: str) -> None:
        parsed = self._parse(name)
        self.valid[name] = parsed is not None and self._in_range(name, parsed)

    @property
    def all_valid(self) -> bool:
        return bool(self.names) and all(self.valid[n] for n in self.names)

    # ---------------------------------------------------------- position updates
    def apply_positions(self, joints: Dict[str, float], force: bool = False) -> None:
        """
        Sync the displayed values with live hardware positions.

        Only fields that are not being manually edited (auto=True) are updated,
        unless `force` is True (used after a "done" ack) in which case all are
        driven again by the achieved angles.
        """
        for name in self.names:
            if name not in joints:
                continue
            if force or self.auto[name]:
                self.buffers[name] = f"{joints[name]:.1f}"
                self._revalidate(name)
        if force:
            for name in self.names:
                if name in joints:
                    self.auto[name] = True

    # ---------------------------------------------------------- focus handling
    def _focus(self, i: int) -> None:
        self._commit_focus()
        self.focus_index = i
        name = self.names[i]
        # Select the current text so the next typed character overwrites it.
        # The value itself is NOT cleared: if the user clicks in and does nothing
        # (or clicks away), the original value stays exactly as it was.
        self.select[name] = True
        self.auto[name] = False  # stop live-tracking while this field is edited
        self._revalidate(name)
        self.last_status = ""

    def _blur(self) -> None:
        self._commit_focus()
        self.focus_index = None

    def _commit_focus(self) -> None:
        """Called when leaving the currently-focused field (blur / tab / new focus)."""
        if self.focus_index is None:
            return
        name = self.names[self.focus_index]
        # Keep whatever is in the field; just stop "selecting" it.
        self.select[name] = False
        if self.buffers[name].strip() == "":
            # Nothing entered -> fall back to live-tracking again.
            self.auto[name] = True
        self._revalidate(name)

    def _do_input(self, name: str, ch: str) -> None:
        buf = self.buffers[name]
        if self.select[name]:
            # Overwrite the selected text with the first typed char.
            self.buffers[name] = ch
            self.select[name] = False
        else:
            self.buffers[name] = buf + ch
        self.auto[name] = False
        self._revalidate(name)

    def _do_backspace(self, name: str) -> None:
        if self.select[name]:
            # Backspace with all text selected -> clear the field.
            self.buffers[name] = ""
            self.select[name] = False
        else:
            self.buffers[name] = self.buffers[name][:-1]
        self.auto[name] = False
        self._revalidate(name)

    # ---------------------------------------------------------- event handling
    def handle_event(self, event) -> None:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = event.pos
            for i, name in enumerate(self.names):
                if self.field_rects[name].collidepoint(pos):
                    self._focus(i)
                    return
            # click elsewhere (empty space or Act) -> blur first
            self._blur()
            if self.act_rect.collidepoint(pos):
                self._on_act()
            return

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_TAB:
                if self.focus_index is None:
                    self._focus(0)
                else:
                    self._focus((self.focus_index + 1) % len(self.names))
                return
            if self.focus_index is None:
                return
            name = self.names[self.focus_index]
            if event.key == pygame.K_BACKSPACE:
                self._do_backspace(name)
            elif event.key == pygame.K_RETURN:
                self._blur()
                self._on_act()
                return
            else:
                return

        if event.type == pygame.TEXTINPUT and self.focus_index is not None:
            name = self.names[self.focus_index]
            for ch in event.text:
                if ch in "0123456789+-.e ":
                    self._do_input(name, ch)

    def _on_act(self) -> None:
        if not self.all_valid:
            self.last_status = "Invalid values"
            self.last_status_color = (240, 90, 90)
            return

        joints = {name: self._parse(name) for name in self.names}
        write_command(self.cmd_file, {"status": "pending", "joints": joints})
        self.last_status = "SENT"
        self.last_status_color = (90, 200, 140)

    def handle_status(self) -> None:
        """
        Read the telemetry file: live positions (idle) or a "done" ack.
        Called by the dashboard every frame.
        """
        data = read_command(self.telemetry_file)
        if not data:
            return
        joints = data.get("joints") or {}
        if data.get("status") == "done":
            self.apply_positions(joints, force=True)
            self.last_status = "DONE"
            self.last_status_color = (90, 200, 140)
        else:
            self.apply_positions(joints)

    # ---------------------------------------------------------- drawing
    def draw(self, surf) -> None:
        # frame
        pygame.draw.rect(surf, (30, 30, 36), self.panel_rect, border_radius=10)
        pygame.draw.rect(surf, (90, 170, 255), self.panel_rect, width=1, border_radius=10)

        px, py = self.panel_rect.topleft

        # ACTIVE / MUTED indicator (prominent, drawn at the top of the panel)
        muted_now = self.focused
        if muted_now:
            badge_color = (200, 120, 40)   # amber
            badge_text = "CONTROLLER: MUTED"
        else:
            badge_color = (60, 160, 110)   # green
            badge_text = "CONTROLLER: ACTIVE"
        badge = pygame.Rect(px + 168, self.act_rect.y, 224, self.act_rect.height)
        pygame.draw.rect(surf, badge_color, badge, border_radius=8)
        self._draw_text(surf, badge_text, badge.center, self.font, (255, 255, 255), align="center")

        self._draw_text(surf, "Manual Motor Goals", (px + 8, py + 7), self.font, (90, 170, 255))

        for i, name in enumerate(self.names):
            x0, ry = self.rowoffsets[name]
            lo, hi = self.limits[name]
            label_name = f"{name}"
            label_lo_hi = f"Min: {int(lo):>4}    Max: {int(hi):>3}"
            self._draw_text(surf, label_name, (x0, ry + 3), self.small, (170, 170, 175))

            x_lo_hi = self.field_rects[name].x + 84
            self._draw_text(surf, label_lo_hi, (x_lo_hi, ry + 3), self.small, (170, 170, 175))

            rect = self.field_rects[name]
            focused = i == self.focus_index
            if self.valid[name]:
                color = (90, 200, 140)
            elif self.buffers[name].strip() == "":
                color = (80, 80, 88)
            else:
                color = (240, 90, 90)
            pygame.draw.rect(surf, (45, 45, 52), rect, border_radius=5)
            pygame.draw.rect(surf, color, rect, width=(2 if focused else 1), border_radius=5)
            txt = self.buffers[name]
            img = self.small.render(txt, True, (230, 230, 235))
            surf.blit(img, (rect.x + 5, rect.y + 3))

        # Act button
        if self.all_valid:
            btn_color = (70, 150, 220)
        else:
            btn_color = (60, 60, 68)
        pygame.draw.rect(surf, btn_color, self.act_rect, border_radius=8)
        self._draw_text(surf, "ACT", self.act_rect.center, self.font, (255, 255, 255), align="center")

        # Status text (sent/done/invalid)
        status_x = self.act_rect.right + 10
        self._draw_text(surf, self.last_status, (status_x, self.act_rect.centery - 8), self.small, self.last_status_color)

    def _draw_text(self, surf, text, pos, font, color=(230, 230, 235), align="topleft"):
        img = font.render(text, True, color)
        r = img.get_rect(**{align: pos})
        surf.blit(img, r)
        return r


def build_manual_panel(cmd_file, telemetry_file, limits, font, small) -> Optional[ManualJointPanel]:
    """Small factory used by the dashboard. Requires command + telemetry paths and limits."""
    if not cmd_file or not telemetry_file or not limits:
        return None
    return ManualJointPanel(
        cmd_file=cmd_file,
        telemetry_file=telemetry_file,
        limits=limits,
        font=font,
        small_font=small,
    )
