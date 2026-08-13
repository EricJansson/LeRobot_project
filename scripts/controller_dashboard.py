"""
Controller Dashboard (pygame)
- Visualizes axes (sticks + triggers), buttons, and D-pad in a tidy window.
- Works with most controllers; includes nice labels for Xbox 360 on Windows.
- Quit with ESC or close the window.
"""

import argparse
import time
import warnings
import pygame

try:
    from dashboard_panels import (
        build_manual_panel,
        build_plan_panel,
        default_plan_cmd_file,
        default_telemetry_file,
        read_command,
        read_motor_limits,
        write_command,
    )
except ModuleNotFoundError:  # in case it is run as a package module
    from scripts.dashboard_panels import (
        build_manual_panel,
        build_plan_panel,
        default_plan_cmd_file,
        default_telemetry_file,
        read_command,
        read_motor_limits,
        write_command,
    )

# Optional: silence setuptools/pkg_resources deprecation warning from pygame
warnings.filterwarnings("ignore", message="pkg_resources is deprecated.*", category=UserWarning, module="pygame.pkgdata")

# ---- Config ----
WINDOW_W, WINDOW_H = 720, 480
# Taller window used when the plan panel is enabled.
WINDOW_H_PLAN = 760
# Plan panel geometry (bottom-left of the window).
PLAN_PANEL_W = 680
PLAN_PANEL_H = 320
BG = (18, 18, 20)
FG = (230, 230, 235)
MUTED = (140, 140, 150)
ACCENT = (90, 170, 255)
OK = (90, 200, 140)
WARN = (255, 180, 0)
ERR = (240, 90, 90)

FPS = 90
DEADZONE = 0.08
# If the controller briefly drops out (USB/driver hiccup), wait this long for it
# to reconnect before giving up and closing the dashboard.
CONTROLLER_RECONNECT_S = 1.5

# --- Controller mapping (edit these to match your gamepad) ---
# Give friendly labels to the *indices* reported by pygame/SDL for YOUR controller.
# Start with this guess (works for many generic/PS-style pads), then adjust after you test:
AXIS_LABELS = {
    0: "LX",   # Left stick X
    1: "LY",   # Left stick Y
    2: "RX",   # Right stick X   <-- you reported RX moving when pushing right stick up/down, so we'll fix later if needed
    3: "RY",   # Right stick Y
    4: "LT",   # Left trigger (0..1 after normalization)
    5: "RT",   # Right trigger (0..1 after normalization)
}
BUTTON_LABELS = {
    # Face buttons (PlayStation-style names)
    0: "Cross",     # (A)
    1: "Circle",    # (B)
    2: "Square",    # (X)
    3: "Triangle",  # (Y)

    4: "LB",
    5: "RB",
    6: "Back",
    7: "Start",
    8: "L3",
    9: "R3",        # Left stick click
    10: "Guide (Unused?)" # Rename to "Unused" if it never lights up
}

TRIGGER_NAMES = {"LT", "RT"}

def is_trigger_label(lbl: str) -> bool:
    return lbl in TRIGGER_NAMES

def norm_trigger(raw: float) -> float:
    # SDL often reports triggers in [-1..+1]; map to [0..1]
    return (raw + 1.0) * 0.5

def clamp(v, lo, hi): return max(lo, min(hi, v))

def draw_text(surf, text, pos, font, color=FG, align="topleft"):
    img = font.render(text, True, color)
    r = img.get_rect(**{align: pos})
    surf.blit(img, r)
    return r

def draw_stick_circle(surf, x, y, r, xv, yv, clicked=False):
    # xv,yv in [-1..1]; draw dot position
    pygame.draw.circle(surf, (35, 35, 40), (x, y), r, width=0)
    pygame.draw.line(surf, MUTED, (x-r, y), (x+r, y), 1)
    pygame.draw.line(surf, MUTED, (x, y-r), (x, y+r), 1)
    # deadzone ring
    dzr = int(r * DEADZONE)
    if dzr > 0:
        pygame.draw.circle(surf, (55, 55, 60), (x, y), dzr, width=1)
    # dot
    px = int(x + clamp(xv, -1, 1) * (r-3))
    py = int(y + clamp(yv, -1, 1) * (r-3))
    pygame.draw.circle(surf, ACCENT, (px, py), 5)
    # highlight on L3/R3 click: brighter ring + inner glow
    if clicked:
        pygame.draw.circle(surf, OK, (x, y), r - 2, width=4)
        glow = pygame.Surface((r*2, r*2), pygame.SRCALPHA)
        pygame.draw.circle(glow, (90, 200, 140, 60), (r, r), r)
        surf.blit(glow, (x - r, y - r))
    else:
        pygame.draw.circle(surf, MUTED, (x, y), r, width=1)

def draw_button_pill(surf, x, y, w, h, label, pressed, font, off_color=(50, 50, 55)):
    rect = pygame.Rect(x, y, w, h)
    color = OK if pressed else off_color
    pygame.draw.rect(surf, color, rect, border_radius=8)
    if not pressed:
        pygame.draw.rect(surf, MUTED, rect, width=1, border_radius=8)
    draw_text(surf, label, rect.center, font, FG, align="center")
    return rect

def draw_trigger_bar(surf, x, y, label, value, font, bar_w=14, bar_h=60):
    # Vertical bar that fills upward; value in [0..1].
    pygame.draw.rect(surf, (35, 35, 40), (x, y, bar_w, bar_h), border_radius=4)
    fill_h = int(bar_h * clamp(value, 0, 1))
    pygame.draw.rect(surf, ACCENT, (x, y + bar_h - fill_h, bar_w, fill_h), border_radius=4)
    draw_text(surf, label, (x + bar_w//2, y + bar_h + 6), font, MUTED, align="midtop")

def draw_face_pill(surf, cx, cy, radius, shape_idx, pressed):
    # Circular face button with a PlayStation-style shape glyph inside.
    color = OK if pressed else (50, 50, 55)
    pygame.draw.circle(surf, color, (cx, cy), radius)
    if not pressed:
        pygame.draw.circle(surf, MUTED, (cx, cy), radius, width=1)

    # Glyph color: dark on green-fill (pressed), light otherwise
    gcol = BG if pressed else FG
    s = radius * 0.55

    if shape_idx == 3:      # Triangle (drawn a bit smaller to stay inside)
        kh = s * 0.8
        pts = [(cx, cy - kh - 1), (cx - kh, cy + s*0.8 - 1), (cx + kh, cy + s*0.8 - 1)]
        pygame.draw.polygon(surf, gcol, pts, width=2)
    elif shape_idx == 2:    # Square
        sq = pygame.Rect(0, 0, int(s*2), int(s*2))
        sq.center = (cx, cy)
        pygame.draw.rect(surf, gcol, sq, width=2, border_radius=2)
    elif shape_idx == 1:    # Circle
        pygame.draw.circle(surf, gcol, (cx, cy), int(s), width=2)
    else:                   # Cross (drawn a bit smaller to stay inside)
        kh = s * 0.75
        pygame.draw.line(surf, gcol, (cx - kh, cy - kh), (cx + kh, cy + kh), 2)
        pygame.draw.line(surf, gcol, (cx - kh, cy + kh), (cx + kh, cy - kh), 2)

def draw_face_diamond(surf, cx, cy, r, button_states, radius=11):
    # PlayStation-style diamond. button_states indices:
    #   0 Cross (top), 1 Circle (right), 2 Square (left), 3 Triangle (top)
    layout = [
        (3, 0, -1),   # Triangle   -> top
        (2, -1, 0),   # Square     -> left
        (1, 1, 0),    # Circle     -> right
        (0, 0, 1),    # Cross      -> bottom
    ]
    for idx, dx, dy in layout:
        if idx < 0 or idx >= len(button_states):
            continue
        px = int(cx + dx * r)
        py = int(cy + dy * r)
        draw_face_pill(surf, px, py, radius, idx, bool(button_states[idx]))




def draw_dpad(surf, x, y, hat_xy, font, size=22):
    # hat_xy is (x,y) in {-1,0,1}
    up = pygame.Rect(x+size, y, size, size)
    down = pygame.Rect(x+size, y+2*size, size, size)
    left = pygame.Rect(x, y+size, size, size)
    right = pygame.Rect(x+2*size, y+size, size, size)
    center = pygame.Rect(x+size, y+size, size, size)

    for rect in (up, down, left, right, center):
        pygame.draw.rect(surf, (45, 45, 50), rect, border_radius=5)
        pygame.draw.rect(surf, MUTED, rect, width=1, border_radius=5)

    # highlight pressed
    hx, hy = hat_xy
    if hy > 0: pygame.draw.rect(surf, ACCENT, up, border_radius=5)
    if hy < 0: pygame.draw.rect(surf, ACCENT, down, border_radius=5)
    if hx < 0: pygame.draw.rect(surf, ACCENT, left, border_radius=5)
    if hx > 0: pygame.draw.rect(surf, ACCENT, right, border_radius=5)

    # draw_text(surf, "D-Pad", (x+size*1.5, y+3*size+10), font, MUTED, align="midtop")

def main():
    global DEADZONE
    ap = argparse.ArgumentParser(description="Visual controller dashboard")
    ap.add_argument("-i", "--index", type=int, default=0, help="Joystick index (default 0)")
    ap.add_argument("--deadzone", type=float, default=DEADZONE, help="Axis deadzone for sticks")
    ap.add_argument("--cmd-file", type=str, default=None,
                    help="Path to the JSON command file used to talk to teleop (enables manual motor panel).")
    ap.add_argument("--telemetry-file", type=str, default=str(default_telemetry_file()),
                    help="Path to the telemetry file teleop streams positions back through.")
    ap.add_argument("--plan-file", type=str, default=None,
                    help="Path to the plan JSON file. Enables the plan panel "
                         "(requires teleop_plan_control running).")
    ap.add_argument("--plan-cmd-file", type=str, default=None,
                    help="Path to the plan-command file used by the plan panel. "
                         "Defaults to a project-local runtime file.")
    args = ap.parse_args()

    DEADZONE = args.deadzone

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controllers detected.")
        return 1

    if args.index < 0 or args.index >= pygame.joystick.get_count():
        print(f"Invalid joystick index {args.index}. Use one of: 0..{pygame.joystick.get_count()-1}")
        return 1

    js = pygame.joystick.Joystick(args.index)
    js.init()

    name = js.get_name()
    na, nb, nh = js.get_numaxes(), js.get_numbuttons(), js.get_numhats()

    # When the plan panel is enabled we need a taller window so it can sit below
    # the manual-motor panel without overlapping the controller visual.
    plan_mode = bool(args.plan_file)
    win_h = WINDOW_H_PLAN if plan_mode else WINDOW_H

    screen = pygame.display.set_mode((WINDOW_W, win_h), pygame.RESIZABLE)
    pygame.display.set_caption("Controller Dashboard")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("consolas,menlo,monospace", 18)
    small = pygame.font.SysFont("consolas,menlo,monospace", 15)
    title = pygame.font.SysFont("consolas,menlo,monospace", 22, bold=True)

    # Optional manual-motor panel (active whenever teleop passes --cmd-file).
    # In plan mode we anchor it to the top-right so it never overlaps the plan
    # panel at the bottom of the (taller) window.
    limits = {}
    if args.cmd_file:
        try:
            limits = read_motor_limits()
        except Exception as e:  # pragma: no cover - depends on calibration file presence
            print(f"Warning: could not load motor limits for manual panel: {e}")
    manual_panel = build_manual_panel(
        cmd_file=args.cmd_file,
        telemetry_file=args.telemetry_file,
        limits=limits,
        font=font,
        small=small,
        panel_rect=(pygame.Rect(WINDOW_W - 416, 90, 400, 215) if plan_mode else None),
        window_size=(WINDOW_W, win_h),
    )
    if args.cmd_file:
        write_command(args.cmd_file, {"status": "active"})
    if manual_panel is not None:
        pygame.key.start_text_input()
    elif args.cmd_file:
        print(
            "[dashboard] WARNING: manual motor panel disabled "
            "because motor limits could not be loaded."
        )

    # Optional plan panel (active whenever teleop passes --plan-file). It is
    # anchored to the bottom-left, below the manual panel. Plan commands are
    # sent through a separate file so they don't collide with mute/active.
    plan_panel = build_plan_panel(
        cmd_file=args.cmd_file,
        telemetry_file=args.telemetry_file,
        plan_path=args.plan_file,
        plan_cmd_file=args.plan_cmd_file or str(default_plan_cmd_file()),
        font=font,
        small=small,
        panel_rect=pygame.Rect(16, win_h - PLAN_PANEL_H - 16, PLAN_PANEL_W, PLAN_PANEL_H),
    )

    # Tracks whether the controller should be muted (teleop ignores gamepad while true).
    _panel_muted = False

    last_hat = (0, 0)

    # Track the live window size (changes on resize).
    win_w, win_h = WINDOW_W, win_h

    # main loop
    running = True
    controller_lost_at = None   # set when the joystick drops; grace window before exit
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: running = False
            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE: running = False
            if e.type == pygame.JOYDEVICEREMOVED and e.instance_id == js.get_instance_id():
                # Don't exit immediately: USB/driver dropouts can be momentary.
                if controller_lost_at is None:
                    controller_lost_at = time.time()
                    print(f"[dashboard] controller removed (id={e.instance_id}); "
                          f"waiting {CONTROLLER_RECONNECT_S}s for reconnect...")
            if e.type == pygame.JOYHATMOTION:
                last_hat = e.value
            if e.type == pygame.VIDEORESIZE:
                win_w, win_h = e.w, e.h
                try:
                    screen = pygame.display.set_mode((win_w, win_h), pygame.RESIZABLE)
                except pygame.error as er:
                    print(f"[dashboard] WARNING: resize failed: {er}")
                if manual_panel is not None:
                    if plan_mode:
                        # Keep the manual panel at the top-right so it never
                        # overlaps the plan panel (which is bottom-left).
                        manual_panel.panel_rect = pygame.Rect(win_w - 416, 90, 400, 215)
                        manual_panel._lazy_build_rects()
                    else:
                        manual_panel.reposition((win_w, win_h))
                if plan_panel is not None:
                    plan_panel.reposition((win_w, win_h))
            if manual_panel is not None:
                manual_panel.handle_event(e)
            if plan_panel is not None:
                if e.type == pygame.KEYDOWN and e.key == pygame.K_BACKSPACE:
                    plan_panel.handle_backspace()
                elif e.type == pygame.TEXTINPUT:
                    plan_panel.handle_textinput(e)
                else:
                    plan_panel.handle_event(e)

        # If the controller dropped out, try to recover it within the grace window.
        if controller_lost_at is not None:
            elapsed = time.time() - controller_lost_at
            if elapsed > CONTROLLER_RECONNECT_S:
                print("[dashboard] controller did not reconnect; closing dashboard.")
                running = False
                break
            # Attempt to re-acquire the same device index.
            recovered = False
            try:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > args.index:
                    js = pygame.joystick.Joystick(args.index)
                    js.init()
                    name = js.get_name()
                    na, nb, nh = js.get_numaxes(), js.get_numbuttons(), js.get_numhats()
                    recovered = True
            except pygame.error as er:
                print(f"[dashboard] WARNING: controller recovery error: {er}")
            if recovered:
                controller_lost_at = None
                print(f"[dashboard] controller reconnected: {name}")
            else:
                # Nothing recovered yet; keep looping to wait for the grace window.
                pygame.display.flip()
                clock.tick(FPS)
                continue

        # Live reads (polling) - guarded so a mid-poll disconnect can't crash us.
        try:
            axes = [js.get_axis(i) for i in range(na)]
            buttons = [js.get_button(i) for i in range(nb)]
            hats = [js.get_hat(i) for i in range(nh)]
        except pygame.error as er:
            print(f"[dashboard] WARNING: joystick read error: {er}")
            # Treat like a dropped controller and enter the grace window.
            if controller_lost_at is None:
                controller_lost_at = time.time()
                print(f"[dashboard] controller read failed; waiting {CONTROLLER_RECONNECT_S}s for reconnect...")
            pygame.display.flip()
            clock.tick(FPS)
            continue
        if nh >= 1:
            last_hat = hats[0]

        # Normalize values and assign labels from AXIS_LABELS
        ax_named = {}
        for i, raw in enumerate(axes):
            label = AXIS_LABELS.get(i, f"AX{i}")
            if is_trigger_label(label):
                ax_named[label] = norm_trigger(raw)
            else:
                ax_named[label] = 0.0 if abs(raw) < DEADZONE else raw


        # Layout
        screen.fill(BG)
        draw_text(screen, "Controller Dashboard", (16, 12), title, FG)
        draw_text(screen, f"Device: {name}", (16, 44), small, MUTED)
        draw_text(screen, f"Axes:{na} Buttons:{nb} Hats:{nh}  |  Deadzone:{DEADZONE:.2f}", (16, 66), small, MUTED)

        # Pull labeled values with safe fallbacks
        lx = ax_named.get("LX", axes[0] if na > 0 else 0.0)
        ly = ax_named.get("LY", axes[1] if na > 1 else 0.0)
        rx = ax_named.get("RX", axes[2] if na > 2 else 0.0)
        ry = ax_named.get("RY", axes[3] if na > 3 else 0.0)
        # Triggers
        lt = ax_named.get("LT", norm_trigger(axes[4] if na > 4 else -1.0))
        rt = ax_named.get("RT", norm_trigger(axes[5] if na > 5 else -1.0))


        # ---- Controller-style input visualization (compact box) ----
        BX, BY, BW, BH = 16, 86, 170, 200
        box_rect = pygame.Rect(BX, BY, BW, BH)
        pygame.draw.rect(screen, (26, 26, 30), box_rect, border_radius=10)
        pygame.draw.rect(screen, ACCENT, box_rect, width=1, border_radius=10)

        # Pressed states (read once)
        lb_pressed = bool(buttons[4] if len(buttons) > 4 else False)
        rb_pressed = bool(buttons[5] if len(buttons) > 5 else False)
        back_pressed = bool(buttons[6] if len(buttons) > 6 else False)
        start_pressed = bool(buttons[7] if len(buttons) > 7 else False)
        l3_pressed = bool(buttons[8] if len(buttons) > 8 else False)
        r3_pressed = bool(buttons[9] if len(buttons) > 9 else False)

        # Top row: triggers at corners, LB/RB on each half
        draw_trigger_bar(screen, BX + 4,   BY + 10, "LT", lt, small, bar_w=12, bar_h=36)
        draw_trigger_bar(screen, BX + BW - 16, BY + 10, "RT", rt, small, bar_w=12, bar_h=36)
        draw_button_pill(screen, BX + 18, BY + 8, 48, 14, "LB", lb_pressed, small)
        draw_button_pill(screen, BX + BW - 66, BY + 8, 48, 14, "RB", rb_pressed, small)

        # Center row: Back/Start just below LB/RB
        draw_button_pill(screen, BX + 28, BY + 30, 48, 14, "Back", back_pressed, small)
        draw_button_pill(screen, BX + BW - 76, BY + 30, 48, 14, "Start", start_pressed, small)

        # Middle row: D-Pad (left), face diamond (right), horizontally aligned
        draw_dpad(screen, BX + 18, BY + 66, last_hat if nh >= 1 else (0, 0), small, size=18)
        draw_face_diamond(screen, BX + BW - 50, BY + 93, 20, buttons, radius=11)

        # Bottom row: sticks, below their respective D-Pad / diamond
        draw_stick_circle(screen, BX + 45, BY + 160, 26, lx, ly, clicked=l3_pressed)
        draw_stick_circle(screen, BX + BW - 50, BY + 160, 26, rx, ry, clicked=r3_pressed)

        # Legend / hints
        draw_text(screen, "ESC to quit • Close window to quit", (16, win_h-28), small, MUTED)

        # Manual motor panel (right side), plus controller mute signalling.
        # We write the command file EVERY frame (unthrottled) so teleop's
        # dashboard-alive check (a fresh timestamp) never fires falsely.
        if manual_panel is not None:
            # Consume any "done" ack (re-populates fields) BEFORE writing fresh
            # mute/active status, so the ack is never clobbered in the same frame.
            manual_panel.handle_status()
            manual_panel.draw(screen)

            want_muted = manual_panel.focused
            # Don't clobber an in-flight "pending" goal with a mute/active status;
            # that must stay put until teleop acks it with "done".
            command_in_flight = (
                read_command(args.cmd_file) or {}
            ).get("status") == "pending"
            if not command_in_flight:
                try:
                    write_command(args.cmd_file, {"status": "muted" if want_muted else "active"})
                except Exception as er:
                    print(f"[dashboard] WARNING: failed to write status file: {er}")
                _panel_muted = want_muted


        # Plan panel (bottom-left). It reads its data from the telemetry file
        # and sends edits through the separate plan-command file.
        if plan_panel is not None:
            plan_panel.handle_status()
            plan_panel.draw(screen)


        pygame.display.flip()
        clock.tick(FPS)

    # On clean exit, make sure we leave the controller unmuted for teleop.
    if manual_panel is not None and _panel_muted:
        try:
            write_command(args.cmd_file, {"status": "active"})
        except Exception as er:
            print(f"[dashboard] WARNING: failed to write active status on exit: {er}")

    pygame.quit()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
