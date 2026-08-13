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


def default_plan_cmd_file() -> Path:
    """File path used by the dashboard's plan panel to send plan-edit commands.

    This is a SEPARATE file from the main command file so the plan panel's
    "plan_cmd" writes never collide with the mute/active/pending status traffic
    that the manual-motor panel writes on every frame.
    """
    return runtime_dir() / "lerobot_dashboard_plan_cmd.json"


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

PANEL_MARGIN = 16  # gap from the window edges


def _default_panel_rect(window_size: Optional[Tuple[int, int]] = None) -> pygame.Rect:
    """Compute a panel rect anchored to the bottom-right corner of the window."""
    w, h = window_size if window_size else (1020, 700)
    return pygame.Rect(
        w - ManualJointPanel.FRAME_W - PANEL_MARGIN,
        h - ManualJointPanel.FRAME_H - PANEL_MARGIN,
        ManualJointPanel.FRAME_W,
        ManualJointPanel.FRAME_H,
    )


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
        panel_rect=None,
        window_size: Optional[Tuple[int, int]] = None,
    ) -> None:
        self.cmd_file = Path(cmd_file)
        self.telemetry_file = Path(telemetry_file)
        self.font = font
        self.small = small_font

        # Anchor to the bottom-right corner of the window by default.
        self._window_size = tuple(window_size) if window_size else (1020, 700)
        if panel_rect is None:
            panel_rect = _default_panel_rect(self._window_size)
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
    def reposition(self, window_size: Tuple[int, int]) -> None:
        """Re-anchor the panel to the bottom-right corner of a (possibly resized) window."""
        self._window_size = tuple(window_size)
        self.panel_rect = _default_panel_rect(self._window_size)
        self._lazy_build_rects()

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


def build_manual_panel(cmd_file, telemetry_file, limits, font, small, window_size=None, panel_rect=None) -> Optional[ManualJointPanel]:
    """Small factory used by the dashboard. Requires command + telemetry paths and limits."""
    if not cmd_file or not telemetry_file or not limits:
        return None
    return ManualJointPanel(
        cmd_file=cmd_file,
        telemetry_file=telemetry_file,
        limits=limits,
        font=font,
        small_font=small,
        panel_rect=panel_rect,
        window_size=window_size,
    )


# ---------------------------------------------------------------------------
# Plan panel (teleop_plan_control)
# ---------------------------------------------------------------------------
# Displays the recorded plan as a table of PlanActions (one row per action).
# Each action is either a MOVE (six motor joint angles + a 0..1 speed) or a
# PAUSE (a duration_s wait). Pauses are added/edited independently from MOVE
# steps, always here (never from the controller at record time). The panel lets
# the user delete any action, edit a MOVE speed or a PAUSE duration, save/load
# the plan as JSON, add a pause, and clear the plan.
#
# The teleop process owns the plan and the serial port, so the panel does not
# mutate anything directly — it sends "plan_cmd" commands (with a unique cmd_id)
# through the plan-command file and reads the live plan back through the
# telemetry file.

# Canonical joint display columns (short labels)
_PLAN_COLUMNS = (
    ("shoulder_pan",  "Sh.Pan"),
    ("shoulder_lift", "Sh.Lift"),
    ("elbow_flex",    "Elbow"),
    ("wrist_flex",    "W.Flex"),
    ("wrist_roll",    "W.Roll"),
    ("gripper",       "Grip"),
)

# How many actions fit on a page (fixed rows, paginated)
PLAN_ROWS_PER_PAGE = 10


class PlanPanel:
    """
    A paginated table of plan actions (MOVE / PAUSE) plus action buttons.

    Layout (anchored below the ManualJointPanel):
        Header:  "Plan Actions"   count   mode badge
        Column headers row
        5 action rows (each with a Del button and an Edit button)
        Footer row:  < page >   [Save] [Load] [Clear All] [Add Pause]

    Data model (PlanAction protocol from teleop_plan_control):
        Each action dict has:
            id, type ("move"|"pause"),
            and for MOVE: joints {name: deg} + speed
            and for PAUSE: duration_s

    Interaction:
        - MOVE rows show the 6 joint angles and a Speed column.
        - PAUSE rows show a "PAUSE" badge spanning the joint columns plus its
          duration; the Edit button edits the pause duration.
        - Del removes the selected action via "delete_action".
        - Edit opens an inline field: speed for MOVE, duration_s for PAUSE.
        - Add Pause appends a PAUSE action ("add_pause").
        - Save / Load / Clear All send the matching "plan_cmd" action.
        - Every command carries a unique monotonic cmd_id; the panel waits for a
          matching "plan_cmd_ack" before sending another (one in flight at a time).
    """

    ROW_H = 20
    HEADER_H = 24
    FOOTER_H = 30

    def __init__(self, cmd_file, telemetry_file, plan_path, plan_cmd_file, font, small, panel_rect=None):
        self.cmd_file = Path(cmd_file)
        self.telemetry_file = Path(telemetry_file)
        self.plan_path = Path(plan_path) if plan_path else None
        # Plan commands go to a SEPARATE file so they never collide with the
        # mute/active/pending status traffic in the main command file.
        self.plan_cmd_file = Path(plan_cmd_file) if plan_cmd_file else Path(cmd_file)
        self.font = font
        self.small = small

        # Live plan data, refreshed from telemetry every frame (PlanAction dicts).
        self.actions: List[dict] = []
        self.mode = "IDLE"
        self.playback_index = -1

        # UI state
        self.page = 0
        self.selected_index: Optional[int] = None   # global action index
        self.editing: Optional[int] = None          # global action index being edited
        self.edit_value = ""
        self.status_text = ""
        self.status_color = (180, 180, 185)
        self._status_ts = 0.0

        # Plan-command dedup / ack: each command gets a unique monotonic id and
        # the panel holds at most one in flight until teleop acks it.
        self._next_cmd_id = 0
        self._pending_cmd_id: Optional[int] = None

        # geometry (filled by build_rects)
        self.panel_rect = pygame.Rect(panel_rect)
        self._rects: Dict[str, pygame.Rect] = {}

        self._build_rects()

    # ---------------------------------------------------------- geometry
    @property
    def page_count(self) -> int:
        return max(1, (len(self.actions) + PLAN_ROWS_PER_PAGE - 1) // PLAN_ROWS_PER_PAGE)

    def _clamp_page(self) -> None:
        self.page = max(0, min(self.page, self.page_count - 1))

    def page_actions(self) -> List[Tuple[int, dict]]:
        """Return (global_index, action) for the rows on the current page."""
        self._clamp_page()
        start = self.page * PLAN_ROWS_PER_PAGE
        end = start + PLAN_ROWS_PER_PAGE
        return [(i, self.actions[i]) for i in range(start, min(end, len(self.actions)))]

    def _build_rects(self) -> None:
        px, py = self.panel_rect.topleft
        w, h = self.panel_rect.size
        self._rects = {}
        x0 = px + 8
        y0 = py + self.HEADER_H + 8

        # Columns: index + type badge + 6 joint cols + speed + duration + Del/Edit
        self.idx_w = 28
        self.type_w = 48
        self.joint_w = 56
        self.spd_w = 40
        self.dur_w = 48
        self.btn_w = 36

        self.table_x = x0
        self.idx_x = self.table_x
        self.type_x = self.idx_x + self.idx_w
        self.joint_x0 = self.type_x + self.type_w
        self.spd_x = self.joint_x0 + 6 * self.joint_w
        self.dur_x = self.spd_x + self.spd_w
        self.btn_x = self.dur_x + self.dur_w
        self.edit_x = self.btn_x + self.btn_w + 6

        # Row rects for the visible rows (positions must match draw's row math:
        # drawn rows start one ROW_H below y0, i.e. y0 + (1 + i) * ROW_H).
        self.row_rects: List[Tuple[int, pygame.Rect]] = []
        for i in range(PLAN_ROWS_PER_PAGE):
            ry = y0 + (1 + i) * self.ROW_H
            self.row_rects.append((i, pygame.Rect(x0, ry, w - 16, self.ROW_H)))

        # Per-row Del / Edit buttons
        self.del_rects: List[pygame.Rect] = []
        self.edit_rects: List[pygame.Rect] = []
        for i in range(PLAN_ROWS_PER_PAGE):
            ry = y0 + i * self.ROW_H
            self.del_rects.append(pygame.Rect(self.btn_x, ry + 2 + self.ROW_H, self.btn_w, self.ROW_H - 4))
            self.edit_rects.append(pygame.Rect(self.edit_x, ry + 2 + self.ROW_H, self.btn_w, self.ROW_H - 4))

        # Footer buttons anchored to the BOTTOM of the panel box.
        fy = py + h - self.FOOTER_H - 8
        self.footer_y = fy
        self.prev_rect = pygame.Rect(x0, fy, 30, 24)
        self.page_rect = pygame.Rect(x0 + 34, fy, 46, 24)
        self.next_rect = pygame.Rect(x0 + 84, fy, 30, 24)

        bx = x0 + 130
        self.save_rect = pygame.Rect(bx, fy, 62, 24)
        self.load_rect = pygame.Rect(bx + 66, fy, 62, 24)
        self.clear_rect = pygame.Rect(bx + 132, fy, 78, 24)
        self.add_pause_rect = pygame.Rect(bx + 214, fy, 86, 24)

        # Status text + inline edit field. The status (_flash) text is anchored
        # to the footer row so it lines up horizontally with the Save/Load buttons.
        self.edit_rect = pygame.Rect(x0 + 160, y0 + PLAN_ROWS_PER_PAGE * self.ROW_H + 24, 70, 20)
        self.status_pos = (x0 + 460, fy + 4)

    def reposition(self, window_size: Tuple[int, int]) -> None:
        """Re-anchor to the bottom-left corner of a (possibly resized) window.

        The panel keeps its fixed size but stays 16px from the bottom-left.
        """
        w, h = window_size
        self.panel_rect = pygame.Rect(16, h - self.panel_rect.height - 16,
                                      self.panel_rect.width, self.panel_rect.height)
        self._build_rects()

    # ---------------------------------------------------------- IPC
    def _send_plan_cmd(self, action: str, **extra) -> None:
        """Send a plan command with a unique cmd_id (one in flight at a time).

        If a previous command has not been acknowledged yet, this request is
        dropped rather than clobbering the unprocessed command in the file.
        """
        if self._pending_cmd_id is not None:
            self._flash("busy (cmd pending)", (200, 160, 60))
            return
        self._next_cmd_id += 1
        cmd_id = self._next_cmd_id
        payload = {"status": "plan_cmd", "action": action, "cmd_id": cmd_id}
        payload.update(extra)
        if action in ("save", "load") and self.plan_path is not None:
            payload.setdefault("file", str(self.plan_path))
        write_command(self.plan_cmd_file, payload)
        self._pending_cmd_id = cmd_id

    def _check_ack(self) -> None:
        """Recognize a matching plan_cmd_ack and clear the in-flight command."""
        if self._pending_cmd_id is None:
            return
        data = read_command(self.plan_cmd_file)
        if not data:
            return
        if (data.get("status") == "plan_cmd_ack"
                and data.get("cmd_id") == self._pending_cmd_id):
            self._pending_cmd_id = None

    def _flash(self, text: str, color) -> None:
        self.status_text = text
        self.status_color = color
        self._status_ts = time.time()

    # ---------------------------------------------------------- update
    def handle_status(self) -> None:
        """Read the plan-command ACK + telemetry each frame."""
        self._check_ack()
        data = read_command(self.telemetry_file)
        if not data:
            return
        self.actions = list(data.get("plan_actions") or [])
        self.mode = data.get("mode", "IDLE")
        self.playback_index = int(data.get("play_index", -1))
        self._clamp_page()

    # ---------------------------------------------------------- events
    def _row_global_index(self, row_pos: int) -> Optional[int]:
        page_items = self.page_actions()
        if 0 <= row_pos < len(page_items):
            return page_items[row_pos][0]
        return None

    def _begin_edit(self, global_idx: int) -> None:
        if 0 <= global_idx < len(self.actions):
            self.editing = global_idx
            self.selected_index = global_idx
            a = self.actions[global_idx]
            if a.get("type") == "pause":
                self.edit_value = f"{float(a.get('duration_s', 1.0)):.2f}"
            else:
                self.edit_value = f"{float(a.get('speed', 0.3)):.2f}"

    def _commit_edit(self) -> None:
        if self.editing is None or not (0 <= self.editing < len(self.actions)):
            self.editing = None
            return
        a = self.actions[self.editing]
        if a.get("type") == "pause":
            try:
                dur = max(0.0, float(self.edit_value.strip() or "1.0"))
            except ValueError:
                dur = 1.0
            self._send_plan_cmd("set_action", index=self.editing,
                                fields={"duration_s": dur})
            self._flash("pause updated", (90, 200, 140))
        else:
            try:
                speed = max(0.01, min(1.0, float(self.edit_value.strip() or "0.3")))
            except ValueError:
                speed = 0.3
            self._send_plan_cmd("set_action", index=self.editing,
                                fields={"speed": speed})
            self._flash("speed updated", (90, 200, 140))
        self.editing = None

    def handle_event(self, event) -> None:
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            pos = event.pos

            # Editing text field (click into it)
            if self.editing is not None and self.edit_rect.collidepoint(pos):
                return

            # Row selection + Del/Edit buttons
            for row_pos, (_gi, _a) in enumerate(self.page_actions()):
                gidx = self._row_global_index(row_pos)
                if gidx is None:
                    continue
                if self.del_rects[row_pos].collidepoint(pos):
                    self._send_plan_cmd("delete_action", index=gidx)
                    self._flash("action deleted", (240, 90, 90))
                    self.selected_index = None
                    self.editing = None
                    return
                if self.edit_rects[row_pos].collidepoint(pos):
                    self._begin_edit(gidx)
                    return
                if self.row_rects[row_pos][1].collidepoint(pos):
                    self.selected_index = gidx
                    return

            # Pagination
            if self.prev_rect.collidepoint(pos):
                self.page = max(0, self.page - 1)
                return
            if self.next_rect.collidepoint(pos):
                self.page = min(self.page_count - 1, self.page + 1)
                return

            # Footer actions
            if self.save_rect.collidepoint(pos):
                self._send_plan_cmd("save")
                self._flash("save requested", (90, 200, 140))
                return
            if self.load_rect.collidepoint(pos):
                self._send_plan_cmd("load")
                self._flash("load requested", (90, 200, 140))
                return
            if self.clear_rect.collidepoint(pos):
                self._send_plan_cmd("clear")
                self._flash("plan cleared", (240, 90, 90))
                self.selected_index = None
                self.editing = None
                return
            if self.add_pause_rect.collidepoint(pos):
                # Append a PAUSE at the end (index -1 -> after last action).
                self._send_plan_cmd("add_pause", index=-1, duration_s=1.0)
                self._flash("pause added", (90, 200, 140))
                return

        # Commit edit on Enter (Escape quits the window, handled by the dashboard).
        if event.type == pygame.KEYDOWN and self.editing is not None:
            if event.key == pygame.K_RETURN:
                self._commit_edit()

    def handle_textinput(self, event) -> None:
        if self.editing is not None:
            self.edit_value = _apply_edit_char(self.edit_value, event.text)

    def handle_backspace(self) -> None:
        if self.editing is not None:
            self.edit_value = self.edit_value[:-1]

    # ---------------------------------------------------------- drawing
    def _draw_text(self, surf, text, pos, font, color=(230, 230, 235), align="topleft"):
        img = font.render(text, True, color)
        r = img.get_rect(**{align: pos})
        surf.blit(img, r)
        return r

    def draw(self, surf) -> None:
        px, py = self.panel_rect.topleft
        w, h = self.panel_rect.size

        # Frame
        pygame.draw.rect(surf, (30, 30, 36), self.panel_rect, border_radius=10)
        pygame.draw.rect(surf, (90, 170, 255), self.panel_rect, width=1, border_radius=10)

        # Header
        self._draw_text(surf, "Plan Actions", (px + 8, py + 5), self.font, (90, 170, 255))
        self._draw_text(surf, f"Actions: {len(self.actions)}",
                        (px + 150, py + 8), self.small, (170, 170, 175))

        # Mode badge
        mode_color = {"PLAY": (120, 200, 90), "IDLE": (200, 160, 60)}.get(self.mode, (180, 180, 185))
        badge = pygame.Rect(px + w - 130, py + 5, 122, 20)
        pygame.draw.rect(surf, mode_color, badge, border_radius=8)
        self._draw_text(surf, f"Mode: {self.mode}", badge.center, self.small, (20, 20, 24), align="center")

        # Column headers
        chy = py + self.HEADER_H
        self._draw_text(surf, "#", (self.idx_x, chy + 2), self.small, (150, 150, 158))
        self._draw_text(surf, "Type", (self.type_x, chy + 2), self.small, (150, 150, 158))
        for ci, (_key, label) in enumerate(_PLAN_COLUMNS):
            x = self.joint_x0 + ci * self.joint_w
            self._draw_text(surf, label, (x, chy + 2), self.small, (150, 150, 158))
        self._draw_text(surf, "Spd", (self.spd_x, chy + 2), self.small, (150, 150, 158))
        self._draw_text(surf, "Pause", (self.dur_x, chy + 2), self.small, (150, 150, 158))
        self._draw_text(surf, "Act", (self.btn_x, chy + 2), self.small, (150, 150, 158))

        # Rows
        y0 = py + self.HEADER_H + 8
        for row_pos, (gi, a) in enumerate(self.page_actions()):
            ry = y0 + (1 + row_pos) * self.ROW_H
            is_selected = (gi == self.selected_index)
            is_playing = (gi == self.playback_index)
            is_pause = a.get("type") == "pause"

            if is_playing:
                row_col = (45, 70, 110)
            elif is_selected:
                row_col = (45, 55, 78)
            else:
                row_col = (40, 40, 46)
            pygame.draw.rect(surf, row_col,
                             (px + 6, ry, w - 12, self.ROW_H - 1), border_radius=4)

            # Global index number
            self._draw_text(surf, str(gi), (self.idx_x + 2, ry + 2), self.small,
                            (230, 230, 235))

            # Type badge: MOVE (blue) or PAUSE (amber)
            type_color = (150, 190, 110) if not is_pause else (200, 160, 60)
            self._draw_text(surf, "MOVE" if not is_pause else "PAUSE",
                            (self.type_x + 2, ry + 2), self.small, type_color)

            if is_pause:
                # PAUSE: no joint/speed cells; show duration across the joints area.
                dur = float(a.get("duration_s", 0.0))
                self._draw_text(surf, f"wait {dur:.1f}s",
                                (self.joint_x0 + 2, ry + 2), self.small, (200, 200, 205))
            else:
                # MOVE: joint values (one decimal)
                joints = a.get("joints") or {}
                for ci, (key, _label) in enumerate(_PLAN_COLUMNS):
                    val = float(joints.get(key, 0.0))
                    x = self.joint_x0 + ci * self.joint_w
                    self._draw_text(surf, f"{val:+.1f}", (x + 2, ry + 2), self.small,
                                    (230, 230, 235))
                self._draw_text(surf, f"{float(a.get('speed', 0.3)):.1f}",
                                (self.spd_x + 2, ry + 2), self.small, (200, 200, 205))

            # Del / Edit buttons
            self._draw_button(surf, self.del_rects[row_pos], "Del", (180, 90, 90))
            self._draw_button(surf, self.edit_rects[row_pos], "Edit", (90, 140, 210))

        # Editing inline field (if editing an action)
        if self.editing is not None:
            a = self.actions[self.editing] if 0 <= self.editing < len(self.actions) else None
            if a is not None:
                label = "Pause(s):" if a.get("type") == "pause" else "Speed:"
                self._draw_text(surf, label,
                                (self.edit_rect.x - 64, self.edit_rect.y + 3),
                                self.small, (170, 170, 175), align="topright")
                self._draw_box(surf, self.edit_rect, self.edit_value, True)
                self._draw_text(surf, "[Enter] apply",
                                (self.edit_rect.right + 6, self.edit_rect.y + 3),
                                self.small, (150, 150, 158))

        # Footer: pagination + action buttons
        fy = self.footer_y
        self._draw_button(surf, self.prev_rect, "<", (70, 70, 80))
        self._draw_button(surf, self.next_rect, ">", (70, 70, 80))
        self._draw_text(surf, f"{self.page + 1}/{self.page_count}",
                        self.page_rect.center, self.small, (200, 200, 205), align="center")

        self._draw_button(surf, self.save_rect, "Save", (70, 150, 90))
        self._draw_button(surf, self.load_rect, "Load", (90, 140, 210))
        self._draw_button(surf, self.clear_rect, "Clear All", (200, 90, 90))
        self._draw_button(surf, self.add_pause_rect, "Add Pause", (160, 130, 70))

        # Status text
        if self.status_text:
            self._draw_text(surf, self.status_text, self.status_pos, self.small, self.status_color)
            if time.time() - self._status_ts > 2.0:
                self.status_text = ""

    def _draw_button(self, surf, rect, label, color):
        pygame.draw.rect(surf, color, rect, border_radius=6)
        self._draw_text(surf, label, rect.center, self.small, (255, 255, 255), align="center")

    def _draw_box(self, surf, rect, text, _active):
        pygame.draw.rect(surf, (50, 50, 58), rect, border_radius=5)
        pygame.draw.rect(surf, (90, 170, 255), rect, width=1, border_radius=5)
        img = self.small.render(text, True, (230, 230, 235))
        surf.blit(img, (rect.x + 4, rect.y + 2))


def _apply_edit_char(buffer: str, text: str) -> str:
    """Append only numeric-ish characters to an edit buffer."""
    for ch in text:
        if ch in "0123456789.":
            buffer += ch
    return buffer


def build_plan_panel(cmd_file, telemetry_file, plan_path, plan_cmd_file, font, small, panel_rect=None) -> Optional["PlanPanel"]:
    """Small factory used by the dashboard for the plan panel."""
    if not cmd_file or not telemetry_file:
        return None
    return PlanPanel(
        cmd_file=cmd_file,
        telemetry_file=telemetry_file,
        plan_path=plan_path,
        plan_cmd_file=plan_cmd_file,
        font=font,
        small=small,
        panel_rect=panel_rect,
    )

