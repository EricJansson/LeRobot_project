// ===== Canvas & state =====
const cv = document.getElementById("cv");
const ctx = cv.getContext("2d");

function resizeCanvas() {
    const dpr = Math.max(1, window.devicePixelRatio || 1);
    const cssW = cv.clientWidth;
    const cssH = cv.clientHeight;
    cv.width = Math.max(1, Math.floor(cssW * dpr));
    cv.height = Math.max(1, Math.floor(cssH * dpr));
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    draw();
}
window.addEventListener("resize", resizeCanvas);

// Calculations for L0 based on real arm geometry
// --- Real arm geometry (cm) -> pixels ---
const CM_TO_PX = 12;                 // scale: 12 px per cm (tweak if needed)
const ARM_A_CM = 11.3;               // first leg (cm)
const ARM_B_CM = 3.8;                // perpendicular leg (cm)

const ARM_L0_CM = Math.hypot(ARM_A_CM, ARM_B_CM);
const ARM_L1_CM = 13.5;
const ARM_L2_CM = 17.0;

// pixel lengths for drawing
const ARM_A = ARM_A_CM * CM_TO_PX;
const ARM_B = ARM_B_CM * CM_TO_PX;
// which side to place the perpendicular (+1 = CCW/left of link, -1 = CW/right)
const MOUNT_SIGN = 1;

const ARM_HEIGHT_CM = 11;

const CANVAS_SIZE_X = 1400;
const CANVAS_SIZE_Y = 700;

// Base position of the arm
const base = {
    x: CANVAS_SIZE_X / 2,
    y: CANVAS_SIZE_Y - (CM_TO_PX * ARM_HEIGHT_CM)
};

// const L0 = 180;
const L0 = ARM_L0_CM * CM_TO_PX;
const L1 = ARM_L1_CM * CM_TO_PX;
const L2 = ARM_L2_CM * CM_TO_PX;


// ===== Arm state =====
// --- L0 sprite (scaled to the link length) ---
const L0_IMG_URL = "images/L0.png";     // <-- your path
const L0_IMG_NATIVE = { w: 580, h: 260 };  // original pixels
const L0_IMG_ASPECT = L0_IMG_NATIVE.h / L0_IMG_NATIVE.w; // ≈ 0.4483
const L0_IMG_SCALE = 1.434;  // 1.0 = current size, >1 bigger, <1 smaller

// Width of the drawn sprite = kinematic link length (L0), height keeps aspect
const L0_IMG_SIZE = {
    w: Math.round(L0 * L0_IMG_SCALE),
    h: Math.round(L0 * L0_IMG_ASPECT * L0_IMG_SCALE)
};

// Anchor: pivot near the left edge, slightly below vertical center (tweak as needed)
const L0_IMG_ANCHOR = { u: 0.159, v: 0.284 };

// Extra offset in local (rotated) space; small downwards nudge (tweak as needed)
const L0_IMG_OFFSET = { x: 0, y: 0 };

// Preload
const L0_IMG = new Image();
L0_IMG.onload = () => draw();
L0_IMG.src = L0_IMG_URL;

// --- L1 sprite (scaled to the second link length) ---
const L1_IMG_URL = "images/L1.png";     // <-- your actual path
const L1_IMG_NATIVE = { w: 580, h: 260 };  // replace with real size if different
const L1_IMG_ASPECT = L1_IMG_NATIVE.h / L1_IMG_NATIVE.w;

const L1_IMG_SCALE = 1.2; // start same as L0, tweak later
const L1_IMG_SIZE = {
    w: Math.round(L1 * L1_IMG_SCALE),
    h: Math.round(L1 * L1_IMG_SCALE * L1_IMG_ASPECT)
};

const L1_IMG_ANCHOR = { u: 0.081, v: 0.567 }; // tweak per image
const L1_IMG_OFFSET = { x: 0, y: 0 };

const L1_IMG = new Image();
L1_IMG.onload = () => draw();
L1_IMG.src = L1_IMG_URL;

// --- L2 sprite (scaled to the third link length) ---
const L2_IMG_URL = "images/L2.png";     // <-- your actual path
const L2_IMG_NATIVE = { w: 580, h: 260 };  // replace with real size if different
const L2_IMG_ASPECT = L2_IMG_NATIVE.h / L2_IMG_NATIVE.w;

const L2_IMG_SCALE = 1.2; // start same as others
const L2_IMG_SIZE = {
    w: Math.round(L2 * L2_IMG_SCALE),
    h: Math.round(L2 * L2_IMG_SCALE * L2_IMG_ASPECT)
};

const L2_IMG_ANCHOR = { u: 0.085, v: 0.567 }; // tweak per image
const L2_IMG_OFFSET = { x: 0, y: 0 };

const L2_IMG = new Image();
L2_IMG.onload = () => draw();
L2_IMG.src = L2_IMG_URL;
// ===== Arm state =====


// Limits (deg) — tune as needed
const ANG0_MIN_DEG = 13, ANG0_MAX_DEG = 203;  // joint 0 (base)
const ANG1_MIN_DEG = -20, ANG1_MAX_DEG = 160; // joint 1 (relative to J0)
const ANG2_MIN_DEG = -93, ANG2_MAX_DEG = 93; // joint 2 (relative to J1)

// Joint angles (radians)
let angle0Rad = 0;  // J0 local (also global for first link)
let angle1Rad = 0;  // J1 local (global phi1 = angle0 + angle1)
let angle2Rad = 0; // J2 local (global phi2 = angle0 + angle1 + angle2)

// Canvas styling
const HANDLE_R = 4;
const ARM_LINE_WIDTH = 8;

// UI
const angle0Range = document.getElementById("angle0Range");
const angle0Out = document.getElementById("angle0Out");
const angle1Range = document.getElementById("angle1Range");
const angle1Out = document.getElementById("angle1Out");
const angle2Range = document.getElementById("angle2Range");
const angle2Out = document.getElementById("angle2Out");

const showKinematicEl = document.getElementById("showKinematic");
const showHardwareEl = document.getElementById("showHardware");
let showKinematic = showKinematicEl.checked;
let showHardware = showHardwareEl.checked;

const viewModeEl = document.getElementById("viewMode");
let viewMode = "both";

angle0Range.min = String(ANG0_MIN_DEG);
angle0Range.max = String(ANG0_MAX_DEG);
angle0Range.step = "1";
angle0Range.value = "0";

angle1Range.min = String(ANG1_MIN_DEG);
angle1Range.max = String(ANG1_MAX_DEG);
angle1Range.step = "1";
angle1Range.value = "0";

angle2Range.min = String(ANG2_MIN_DEG);
angle2Range.max = String(ANG2_MAX_DEG);
angle2Range.step = "1";
angle2Range.value = "0";

function clamp(x, a, b) { return Math.max(a, Math.min(b, x)); }
function degToRad(d) { return (d * Math.PI) / 180; }
function radToDeg(r) { return (r * 180) / Math.PI; }

// Setters (rounded label, slider sync, redraw)
function setAngle0FromDeg(deg) {
    const c = clamp(deg, ANG0_MIN_DEG, ANG0_MAX_DEG);
    angle0Rad = degToRad(c);
    angle0Out.textContent = (Math.round(c * 10) / 10).toFixed(1);
    if (angle0Range.value !== String(Math.round(c))) angle0Range.value = String(Math.round(c));
    draw();
}

function setAngle1FromDeg(deg) {
    const c = clamp(deg, ANG1_MIN_DEG, ANG1_MAX_DEG);
    angle1Rad = degToRad(c);
    angle1Out.textContent = (Math.round(c * 10) / 10).toFixed(1);
    if (angle1Range.value !== String(Math.round(c))) angle1Range.value = String(Math.round(c));
    draw();
}

function setAngle2FromDeg(deg) {
    const c = clamp(deg, ANG2_MIN_DEG, ANG2_MAX_DEG);
    angle2Rad = degToRad(c);
    angle2Out.textContent = (Math.round(c * 10) / 10).toFixed(1);
    if (angle2Range.value !== String(Math.round(c))) angle2Range.value = String(Math.round(c));
    draw();
}


angle0Range.addEventListener("input", () => setAngle0FromDeg(parseInt(angle0Range.value, 10)));
angle1Range.addEventListener("input", () => setAngle1FromDeg(parseInt(angle1Range.value, 10)));
angle2Range.addEventListener("input", () => setAngle2FromDeg(parseInt(angle2Range.value, 10)));


showKinematicEl.addEventListener("change", () => {
    showKinematic = !!showKinematicEl.checked;
    draw();
});

showHardwareEl.addEventListener("change", () => {
    showHardware = !!showHardwareEl.checked;
    draw();
});

viewModeEl.addEventListener("change", () => {
    viewMode = viewModeEl.value; // "lines" | "graphics" | "both"
    draw();
});


// FK helpers (CCW on screen: y -= sin)
function getJointPositions() {
    const phi0 = angle0Rad;
    const phi1 = angle0Rad + angle1Rad;
    const phi2 = angle0Rad + angle1Rad + angle2Rad;

    const p0 = { x: base.x, y: base.y };
    const p1 = {
        x: p0.x + Math.cos(phi0) * L0,
        y: p0.y - Math.sin(phi0) * L0,
    };
    const p2 = {
        x: p1.x + Math.cos(phi1) * L1,
        y: p1.y - Math.sin(phi1) * L1,
    };
    const p3 = {
        x: p2.x + Math.cos(phi2) * L2,
        y: p2.y - Math.sin(phi2) * L2,
    };

    return { p0, p1, p2, p3, phi0, phi1, phi2 };
}

/*          DRAW         */

// Draw the real L-shaped arm (right-angle motor mount) over link 0
function drawRightAngleMountOverlay(origin /* p0 */, phi0, lineColor = "#22c55e") {
    // Uses globals: ARM_A, ARM_B, MOUNT_SIGN, ctx
    const alpha = -Math.atan2(MOUNT_SIGN * ARM_B, ARM_A);

    // CCW screen math: y -= sin()
    const dir1 = { x: Math.cos(phi0 + alpha), y: -Math.sin(phi0 + alpha) };
    const dir2 = { x: Math.cos(phi0 + alpha + MOUNT_SIGN * Math.PI / 2), y: -Math.sin(phi0 + alpha + MOUNT_SIGN * Math.PI / 2) };

    const pA = { x: origin.x + dir1.x * ARM_A, y: origin.y + dir1.y * ARM_A };
    const pB = { x: pA.x + dir2.x * ARM_B, y: pA.y + dir2.y * ARM_B };

    ctx.beginPath();
    ctx.moveTo(origin.x, origin.y);
    ctx.lineTo(pA.x, pA.y);
    ctx.lineTo(pB.x, pB.y);
    ctx.lineWidth = ARM_LINE_WIDTH;
    ctx.strokeStyle = lineColor; // green
    ctx.stroke();
}

// Draw a link sprite at 'origin' rotated by global angle 'phi' (CCW math).
function drawLinkSprite(origin, phi, img, size, anchor, offsetPx) {
    if (!img || !img.complete) return;

    ctx.save();
    ctx.translate(origin.x, origin.y);

    // Canvas positive rotation is clockwise (because y grows down).
    // Our phi is CCW in math space, so rotate by -phi to align visually.
    ctx.rotate(-phi);

    // apply extra tweak offset (in the rotated local space)
    ctx.translate(offsetPx.x, offsetPx.y);

    // move so that the chosen anchor lands on the joint (origin)
    ctx.translate(-anchor.u * size.w, -anchor.v * size.h);

    // draw at chosen size (independent of the image's intrinsic size)
    ctx.drawImage(img, 0, 0, size.w, size.h);
    ctx.restore();
}



function strokeSegment(pA, pB, width, color) {
    ctx.beginPath();
    ctx.moveTo(pA.x, pA.y);
    ctx.lineTo(pB.x, pB.y);
    ctx.lineWidth = ARM_LINE_WIDTH;
    ctx.strokeStyle = color;
    ctx.stroke();
}

function drawHandle(p, r, fill = "#7dd3fc") {
    ctx.beginPath();
    ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#0f172a";
    ctx.stroke();
}

function draw() {
    ctx.clearRect(0, 0, cv.width, cv.height);
    const { p0, p1, p2, p3, phi0, phi1, phi2 } = getJointPositions();

    if (viewMode === "graphics" || viewMode === "both") {
        // Link 0
        drawLinkSprite(p0, phi0, L0_IMG, L0_IMG_SIZE, L0_IMG_ANCHOR, L0_IMG_OFFSET);
        // Link 1
        drawLinkSprite(p1, phi1, L1_IMG, L1_IMG_SIZE, L1_IMG_ANCHOR, L1_IMG_OFFSET);
        // Link 2
        drawLinkSprite(p2, phi2, L2_IMG, L2_IMG_SIZE, L2_IMG_ANCHOR, L2_IMG_OFFSET);
    }
    if (viewMode === "lines" || viewMode === "both") {
        // Link 0 (with conditional)
        if (showKinematic) strokeSegment(p0, p1, 6, "#7dd3fc");
        if (showHardware) drawRightAngleMountOverlay(p0, phi0, "#7dd3fc");
        // Link 1
        strokeSegment(p1, p2, 6, "#38bdf8");
        // Link 2
        strokeSegment(p2, p3, 6, "#0ea5e9");
    }

    // joints
    ctx.fillStyle = "#e5e7eb";
    for (const p of [p0, p1, p2, p3]) {
        ctx.beginPath(); ctx.arc(p.x, p.y, 6, 0, Math.PI * 2); ctx.fill();
    }

    drawHandle(p1, HANDLE_R, "#bd3333ff");   // J0 handle
    drawHandle(p2, HANDLE_R, "#bd3333ff");   // J1 handle
    drawHandle(p3, HANDLE_R, "#bd3333ff");   // J2 handle
}

// expose for other files
window.resizeCanvas = resizeCanvas;
window.setAngle0FromDeg = setAngle0FromDeg;
window.setAngle1FromDeg = setAngle1FromDeg;
window.setAngle2FromDeg = setAngle2FromDeg;
window.getJointPositions = getJointPositions;
window.base = base;
window.cv = cv;
window.HANDLE_R = HANDLE_R;

// expose limits so interactions.js can clamp
window.ANG0_MIN_DEG = ANG0_MIN_DEG;
window.ANG0_MAX_DEG = ANG0_MAX_DEG;
window.ANG1_MIN_DEG = ANG1_MIN_DEG;
window.ANG1_MAX_DEG = ANG1_MAX_DEG;
window.ANG2_MIN_DEG = ANG2_MIN_DEG;
window.ANG2_MAX_DEG = ANG2_MAX_DEG;
window.angle0Rad = angle0Rad;
window.angle1Rad = angle1Rad;
window.angle2Rad = angle2Rad;
Object.defineProperty(window, "angle0Rad", { get: () => angle0Rad });
Object.defineProperty(window, "angle1Rad", { get: () => angle1Rad });
Object.defineProperty(window, "angle2Rad", { get: () => angle2Rad });

