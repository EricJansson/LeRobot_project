"""
Controller Dashboard (pygame)
- Visualizes axes (sticks + triggers), buttons, and D-pad in a tidy window.
- Works with most controllers; includes nice labels for Xbox 360 on Windows.
- Quit with ESC or close the window.
"""

import argparse
import math
import time
import warnings
import pygame

# Optional: silence setuptools/pkg_resources deprecation warning from pygame
warnings.filterwarnings("ignore", message="pkg_resources is deprecated.*", category=UserWarning, module="pygame.pkgdata")

# ---- Config ----
WINDOW_W, WINDOW_H = 1020, 700
BG = (18, 18, 20)
FG = (230, 230, 235)
MUTED = (140, 140, 150)
ACCENT = (90, 170, 255)
OK = (90, 200, 140)
WARN = (255, 180, 0)
ERR = (240, 90, 90)

FPS = 90
DEADZONE = 0.08
AXIS_BAR_W = 220
AXIS_BAR_H = 10

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
# Optional layout rows (indices) – adjust as you like. Any buttons not listed here will be appended after.
BUTTON_ROWS = [
    [4, 5],          # Shoulders: LB, RB
    [0, 1, 2, 3],    # Face: Cross, Circle, Square/MR, Triangle/ML
    [8, 9],          # Stick clicks: L3, R3
    [6, 7, 10],      # System: Back, Start, Guide/Unused
]
HIDDEN_BUTTONS = {10}


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

def draw_axis_bar(surf, x, y, label, value, is_trigger, font):
    # value: sticks in [-1..1], triggers in [0..1]
    label_color = FG
    pygame.draw.rect(surf, (35, 35, 40), (x, y, AXIS_BAR_W, AXIS_BAR_H), border_radius=4)

    if is_trigger:
        # 0..1 fill to the right
        fill_w = int(AXIS_BAR_W * clamp(value, 0, 1))
        pygame.draw.rect(surf, ACCENT, (x, y, fill_w, AXIS_BAR_H), border_radius=4)
        val_txt = f"{value:0.3f}"
    else:
        # center at zero, negative fills left, positive fills right
        cx = x + AXIS_BAR_W // 2
        zero_rect = pygame.Rect(x, y, AXIS_BAR_W, AXIS_BAR_H)
        pygame.draw.line(surf, MUTED, (cx, y-3), (cx, y+AXIS_BAR_H+3), 1)

        v = clamp(value, -1, 1)
        if v >= 0:
            fill = pygame.Rect(cx, y, int((AXIS_BAR_W//2) * v), AXIS_BAR_H)
        else:
            fill = pygame.Rect(cx + int((AXIS_BAR_W//2) * v), y, int((AXIS_BAR_W//2) * -v), AXIS_BAR_H)
        pygame.draw.rect(surf, ACCENT, fill, border_radius=4)
        val_txt = f"{value:+0.3f}"

    draw_text(surf, f"{label}", (x, y-20), font, label_color)
    draw_text(surf, val_txt, (x+AXIS_BAR_W+10, y-6), font, MUTED)

def draw_stick_circle(surf, x, y, r, label, xv, yv, font):
    # xv,yv in [-1..1]; draw dot position
    pygame.draw.circle(surf, (35, 35, 40), (x, y), r, width=0)
    pygame.draw.circle(surf, MUTED, (x, y), r, width=1)
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
    draw_text(surf, label, (x, y + r + 8), font, MUTED, align="midtop")
    draw_text(surf, f"({xv:+.2f}, {yv:+.2f})", (x, y + r + 28), font, FG, align="midtop")

def draw_buttons_grid(surf, x, y, button_states, font, button_labels, button_rows=None, hidden_buttons=None):
    if hidden_buttons is None:
        hidden_buttons = set()
    pad_x = 8
    pad_y = 10
    pill_w, pill_h = 96, 26  # room for "Triangle / ML"

    drawn = set()

    def draw_row(row_indices, row_y):
        bx = x
        for idx in row_indices:
            if idx in hidden_buttons or idx < 0 or idx >= len(button_states):
                continue
            pressed = bool(button_states[idx])
            label = button_labels.get(idx, f"B{idx}")
            rect = pygame.Rect(bx, row_y, pill_w, pill_h)
            color = OK if pressed else (50, 50, 55)
            border = 0 if pressed else 1
            pygame.draw.rect(surf, color, rect, border_radius=8)
            if border: pygame.draw.rect(surf, MUTED, rect, width=1, border_radius=8)
            draw_text(surf, label, rect.center, font, FG, align="center")
            bx += pill_w + pad_x
            drawn.add(idx)

    # Draw configured rows
    row_y = y
    if button_rows:
        for row in button_rows:
            draw_row(row, row_y)
            row_y += pill_h + pad_y

    # Any remaining buttons not in the rows (append)
    leftover = [i for i in range(len(button_states)) if i not in drawn and i not in hidden_buttons]
    if leftover:
        draw_row(leftover, row_y)




def draw_dpad(surf, x, y, hat_xy, font):
    # hat_xy is (x,y) in {-1,0,1}
    size = 22
    # base squares
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

    draw_text(surf, "D-Pad", (x+size*1.5, y+3*size+10), font, MUTED, align="midtop")
    draw_text(surf, f"{hat_xy}", (x+size*1.5, y+3*size+30), font, FG, align="midtop")

def main():
    global DEADZONE
    ap = argparse.ArgumentParser(description="Visual controller dashboard")
    ap.add_argument("-i", "--index", type=int, default=0, help="Joystick index (default 0)")
    ap.add_argument("--deadzone", type=float, default=DEADZONE, help="Axis deadzone for sticks")
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

    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Controller Dashboard")
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("consolas,menlo,monospace", 18)
    small = pygame.font.SysFont("consolas,menlo,monospace", 15)
    title = pygame.font.SysFont("consolas,menlo,monospace", 22, bold=True)

    last_hat = (0, 0)

    # main loop
    running = True
    while running:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: running = False
            if e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE: running = False
            if e.type == pygame.JOYDEVICEREMOVED and e.instance_id == js.get_instance_id(): running = False
            if e.type == pygame.JOYHATMOTION:
                last_hat = e.value

        # Live reads (polling)
        axes = [js.get_axis(i) for i in range(na)]
        buttons = [js.get_button(i) for i in range(nb)]
        hats = [js.get_hat(i) for i in range(nh)]
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

        # Left column: sticks as circles
        # Pull labeled values with safe fallbacks
        lx = ax_named.get("LX", axes[0] if na > 0 else 0.0)
        ly = ax_named.get("LY", axes[1] if na > 1 else 0.0)
        rx = ax_named.get("RX", axes[2] if na > 2 else 0.0)
        ry = ax_named.get("RY", axes[3] if na > 3 else 0.0)
        # Triggers
        lt = ax_named.get("LT", norm_trigger(axes[4] if na > 4 else -1.0))
        rt = ax_named.get("RT", norm_trigger(axes[5] if na > 5 else -1.0))


        draw_stick_circle(screen, 150, 160, 70, "Left Stick", lx, ly, small)
        draw_stick_circle(screen, 150, 360, 70, "Right Stick", rx, ry, small)

        # Middle: axis bars (sticks + triggers)
        colx = 300
        y = 120
        # Sticks
        draw_axis_bar(screen, colx, y,   "LX", lx, False, small); y += 34
        draw_axis_bar(screen, colx, y,   "LY", ly, False, small); y += 34
        draw_axis_bar(screen, colx, y,   "RX", rx, False, small); y += 34
        draw_axis_bar(screen, colx, y,   "RY", ry, False, small); y += 48
        draw_axis_bar(screen, colx, y,   "LT", lt, True, small); y += 34
        draw_axis_bar(screen, colx, y,   "RT", rt, True, small); y += 48

        # Right: buttons grid + D-pad
        draw_text(screen, "Buttons", (590, 100), small, MUTED)
        draw_buttons_grid(screen, 590, 120, buttons, small, button_labels=BUTTON_LABELS, button_rows=BUTTON_ROWS, hidden_buttons=HIDDEN_BUTTONS)

        draw_text(screen, "D-Pad", (590, 300), small, MUTED)
        draw_dpad(screen, 590, 320, last_hat if nh >= 1 else (0, 0), small)

        # Legend / hints
        draw_text(screen, "ESC to quit • Close window to quit", (16, WINDOW_H-28), small, MUTED)



        # Raw indices viewer (helps calibrate mappings)
        raw_y = 480
        draw_text(screen, "Raw axes:", (16, raw_y), small, MUTED)
        raw_y += 20
        for i, raw in enumerate(axes):
            t = f"AX{i}: {raw:+.3f}"
            draw_text(screen, t, (16, raw_y), small, FG)
            raw_y += 18

        raw_y += 8
        draw_text(screen, "Raw buttons:", (16, raw_y), small, MUTED); raw_y += 20
        for i, val in enumerate(buttons):
            t = f"B{i}: {'1' if val else '0'}"
            draw_text(screen, t, (16 + (i%12)*60, raw_y + (i//12)*18), small, FG)




        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
