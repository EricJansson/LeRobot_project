// ===== Canvas & state =====
const cv = document.getElementById("cv");
const ctx = cv.getContext("2d");

function resizeCanvas() {
    const w = cv.clientWidth;
    const h = cv.clientHeight;
    cv.width = w;
    cv.height = h;
    // Reset transform so 1 unit = 1 pixel
    ctx.setTransform(1, 0, 0, 1, 0, 0);
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

const BASE_COL_HEIGHT = ARM_HEIGHT_CM * CM_TO_PX; // height of column from table to arm base

function getTableY() {
    // table is at the bottom of the base column
    return base.y + BASE_COL_HEIGHT;
}

const L0 = ARM_L0_CM * CM_TO_PX;
const L1 = ARM_L1_CM * CM_TO_PX;
const L2 = ARM_L2_CM * CM_TO_PX;


// --- Base sprite (column from table to arm base) ---
const BASE_IMG_URL = "images/robot_base.png";
const BASE_IMG_ASPECT = (450 / 450); // aspect ratio of native image
const BASE_IMG_SCALE = 1.14;

const BASE_IMG_SIZE = {
    h: Math.round(BASE_COL_HEIGHT * BASE_IMG_SCALE),
    w: Math.round((BASE_COL_HEIGHT * BASE_IMG_SCALE) / BASE_IMG_ASPECT)
};

// Anchor bottom-center at the arm base joint
const BASE_IMG_ANCHOR = { u: 0.2, v: 0.116 };

const BASE_IMG = new Image();
BASE_IMG.onload = () => draw();
BASE_IMG.src = BASE_IMG_URL;


// ===== Arm state =====
// --- L0 sprite (scaled to the link length) ---
const L0_IMG_URL = "images/L0.png";
const L0_IMG_ASPECT = (260 / 580); // aspect ratio
const L0_IMG_SCALE = 1.434;

const L0_IMG_SIZE = {
    w: Math.round(L0 * L0_IMG_SCALE),
    h: Math.round(L0 * L0_IMG_ASPECT * L0_IMG_SCALE)
};

const L0_IMG_ANCHOR = { u: 0.159, v: 0.284 };

// Preload
const L0_IMG = new Image();
L0_IMG.onload = () => draw();
L0_IMG.src = L0_IMG_URL;

// --- L1 sprite (scaled to the second link length) ---
const L1_IMG_URL = "images/L1.png";
const L1_IMG_ASPECT = (260 / 580); // aspect ratio
const L1_IMG_SCALE = 1.2;

const L1_IMG_SIZE = {
    w: Math.round(L1 * L1_IMG_SCALE),
    h: Math.round(L1 * L1_IMG_SCALE * L1_IMG_ASPECT)
};

const L1_IMG_ANCHOR = { u: 0.081, v: 0.567 };

const L1_IMG = new Image();
L1_IMG.onload = () => draw();
L1_IMG.src = L1_IMG_URL;

// --- L2 sprite (scaled to the third link length) ---
const L2_IMG_URL = "images/L2.png";
const L2_IMG_ASPECT = (260 / 580); // aspect ratio
const L2_IMG_SCALE = 1.2;

const L2_IMG_SIZE = {
    w: Math.round(L2 * L2_IMG_SCALE),
    h: Math.round(L2 * L2_IMG_SCALE * L2_IMG_ASPECT)
};

const L2_IMG_ANCHOR = { u: 0.085, v: 0.567 };

const L2_IMG = new Image();
L2_IMG.onload = () => draw();
L2_IMG.src = L2_IMG_URL;
// ===== Arm state =====


// Limits (deg) — tune as needed
const ANG0_MIN_DEG = 1, ANG0_MAX_DEG = 206;  // joint 0 (base)
const ANG1_MIN_DEG = -20, ANG1_MAX_DEG = 160; // joint 1 (relative to J0)
const ANG2_MIN_DEG = -93, ANG2_MAX_DEG = 93; // joint 2 (relative to J1)

// ===== Unified Application State =====
const state = {
    // Joint angles (radians)
    angles: {
        j0: 0,  // J0 local (also global for first link)
        j1: 0,  // J1 local (global phi1 = angle0 + angle1)
        j2: 0   // J2 local (global phi2 = angle0 + angle1 + angle2)
    },
    // End effector target position (cm)
    endEffector: {
        x: -16.0,
        y: 6.1
    },
    // Stick control state
    stick: {
        active: false,
        dx: 0,
        dy: 0
    },
    // Display and UI flags
    display: {
        showKinematic: true,
        showHardware: true,
        viewMode: "both",
        blockTableHit: true
    }
};

// Canvas styling
const HANDLE_R = 4;
const ARM_LINE_WIDTH = 8;

// UI element references
const angle0Range = document.getElementById("angle0Range");
const angle0Out = document.getElementById("angle0Out");
const angle1Range = document.getElementById("angle1Range");
const angle1Out = document.getElementById("angle1Out");
const angle2Range = document.getElementById("angle2Range");
const angle2Out = document.getElementById("angle2Out");

const showKinematicEl = document.getElementById("showKinematic");
const showHardwareEl = document.getElementById("showHardware");
const viewModeEl = document.getElementById("viewMode");
const eeOut = document.getElementById("eeOut");
const blockTableHitEl = document.getElementById("blockTableHit");

// Initialize display state from UI elements
state.display.showKinematic = showKinematicEl.checked;
state.display.showHardware = showHardwareEl.checked;
state.display.viewMode = viewModeEl.value;
state.display.blockTableHit = blockTableHitEl.checked;



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


function makeAngleSetter({ minDeg, maxDeg, stateKey, outEl, rangeEl }) {
    return function setAngleFromDegGeneric(deg) {
        const c = Math.max(minDeg, Math.min(maxDeg, deg));
        const prevRad = state.angles[stateKey];
        const newRad = (c * Math.PI) / 180;

        // Tentatively apply new angle
        state.angles[stateKey] = newRad;

        // If we block on table hit and the end effector goes below the table, revert
        if (state.display.blockTableHit && !isEndEffectorAboveTable()) {
            state.angles[stateKey] = prevRad;
            return; // reject this move
        }

        // Otherwise, accept and update UI
        outEl.textContent = (Math.round(c * 10) / 10).toFixed(1);
        const rounded = String(Math.round(c));
        if (rangeEl.value !== rounded) rangeEl.value = rounded;

        // NEW: keep EE state consistent with angles
        syncEndEffectorFromFK();

        draw();
    };
}

const setAngle0FromDeg = makeAngleSetter({
    minDeg: ANG0_MIN_DEG, maxDeg: ANG0_MAX_DEG,
    stateKey: "j0",
    outEl: angle0Out, rangeEl: angle0Range
});

const setAngle1FromDeg = makeAngleSetter({
    minDeg: ANG1_MIN_DEG, maxDeg: ANG1_MAX_DEG,
    stateKey: "j1",
    outEl: angle1Out, rangeEl: angle1Range
});

const setAngle2FromDeg = makeAngleSetter({
    minDeg: ANG2_MIN_DEG, maxDeg: ANG2_MAX_DEG,
    stateKey: "j2",
    outEl: angle2Out, rangeEl: angle2Range
});


angle0Range.addEventListener("input", () => setAngle0FromDeg(parseInt(angle0Range.value, 10)));
angle1Range.addEventListener("input", () => setAngle1FromDeg(parseInt(angle1Range.value, 10)));
angle2Range.addEventListener("input", () => setAngle2FromDeg(parseInt(angle2Range.value, 10)));


showKinematicEl.addEventListener("change", () => {
    state.display.showKinematic = !!showKinematicEl.checked;
    draw();
});

showHardwareEl.addEventListener("change", () => {
    state.display.showHardware = !!showHardwareEl.checked;
    draw();
});

viewModeEl.addEventListener("change", () => {
    state.display.viewMode = viewModeEl.value; // "lines" | "graphics" | "both"
    draw();
});

blockTableHitEl.addEventListener("change", () => {
    state.display.blockTableHit = blockTableHitEl.checked;
});

function solveIK_EndEffector({ xCm, yCm, phiDegOpt = null, elbow = "auto" }) {
    // Target is specified in your displayed coordinate system:
    // xCm: right from base
    // yCm: height above table (since eeOut adds ARM_HEIGHT_CM)

    // Convert target EE position to "arm math space" relative to base:
    // In math space: +x right, +y up (NOT screen y)
    const tx = xCm * CM_TO_PX;
    const ty = (yCm - ARM_HEIGHT_CM) * CM_TO_PX; // remove base height offset

    // Choose desired end-effector global orientation phi2 (deg)
    const { phi2 } = getJointPositions();
    const currentPhi2Deg = radToDeg(phi2);
    const phi2Deg = (phiDegOpt === null || Number.isNaN(phiDegOpt))
        ? currentPhi2Deg
        : phiDegOpt;

    const phi2Rad = degToRad(phi2Deg);

    // Wrist target (joint p2): p2 = p3 - L2 * dir(phi2)
    const wx = tx - Math.cos(phi2Rad) * L2;
    const wy = ty - Math.sin(phi2Rad) * L2;

    // 2-link IK for L0, L1 reaching (wx, wy)
    const r2 = wx * wx + wy * wy;
    const r = Math.sqrt(r2);

    // Basic reachability check
    const maxReach = L0 + L1;
    const minReach = Math.abs(L0 - L1);

    if (r > maxReach + 1e-6 || r < minReach - 1e-6) {
        console.warn(`❌ IK FAILED: Target out of reach`, {
            targetX_cm: xCm.toFixed(2),
            targetY_cm: yCm.toFixed(2),
            distance_px: r.toFixed(2),
            maxReach_px: maxReach.toFixed(2),
            minReach_px: minReach.toFixed(2),
            reason: r > maxReach ? "too far" : "too close"
        });
        return { ok: false, reason: "Target out of reach", anglesDeg: null };
    }

    // Law of cosines for angle1 (local joint between L0 and L1)
    let c1 = (r2 - L0 * L0 - L1 * L1) / (2 * L0 * L1);
    c1 = clamp(c1, -1, 1);

    const s1_pos = Math.sqrt(Math.max(0, 1 - c1 * c1));
    const s1_neg = -s1_pos;

    // Two solutions for angle1
    const a1_up = Math.atan2(s1_pos, c1);
    const a1_down = Math.atan2(s1_neg, c1);

    // angle0 from geometry: atan2(wy,wx) - atan2(L1*sin(a1), L0 + L1*cos(a1))
    function angle0For(a1) {
        const k1 = L0 + L1 * Math.cos(a1);
        const k2 = L1 * Math.sin(a1);
        return Math.atan2(wy, wx) - Math.atan2(k2, k1);
    }

    const a0_up = angle0For(a1_up);
    const a0_down = angle0For(a1_down);

    // angle2 to hit desired phi2
    function angle2For(a0, a1) {
        return phi2Rad - a0 - a1;
    }

    const a2_up = angle2For(a0_up, a1_up);
    const a2_down = angle2For(a0_down, a1_down);

    // Convert to degrees
    const candUp = {
        a0: radToDeg(a0_up),
        a1: radToDeg(a1_up),
        a2: radToDeg(a2_up),
        tag: "up",
    };
    const candDown = {
        a0: radToDeg(a0_down),
        a1: radToDeg(a1_down),
        a2: radToDeg(a2_down),
        tag: "down",
    };

    // Pick solution
    const cur0 = radToDeg(state.angles.j0);
    const cur1 = radToDeg(state.angles.j1);
    const cur2 = radToDeg(state.angles.j2);

    function score(c) {
        return (
            3.0 * angleDistDeg(c.a0, cur0) + // base matters most
            1.5 * angleDistDeg(c.a1, cur1) +
            1.0 * angleDistDeg(c.a2, cur2)
        );
    }

    // Ensure memory exists BEFORE using it
    if (state._lastIKTag === undefined) {
        state._lastIKTag = candDown.tag; // or candUp, your default
    }

    let chosen = candDown;
    if (elbow === "up") {
        chosen = candUp;
    } else if (elbow === "down") {
        chosen = candDown;
    } else {
        const sUp = score(candUp);
        const sDown = score(candDown);

        // hysteresis: don't switch unless clearly better
        if (candUp.tag !== state._lastIKTag && sUp < sDown * 0.85) {
            chosen = candUp;
        } else if (candDown.tag !== state._lastIKTag && sDown < sUp * 0.85) {
            chosen = candDown;
        } else {
            chosen = state._lastIKTag === "up" ? candUp : candDown;
        }
    }

    // Update memory AFTER decision
    state._lastIKTag = chosen.tag;

    return { ok: true, reason: "", anglesDeg: chosen };
}

function applyIK_EndEffector(xCm, yCm, phiDegOpt, elbowMode, statusEl) {
    const res = solveIK_EndEffector({ xCm, yCm, phiDegOpt, elbow: elbowMode });

    if (!res.ok) {
        if (statusEl) statusEl.textContent = res.reason;
        return false;
    }

    const { a0, a1, a2 } = res.anglesDeg;

    /*
    console.log(`✅ IK SUCCESS: Applied solution`, {
        targetX_cm: xCm.toFixed(2),
        targetY_cm: yCm.toFixed(2),
        solution: res.anglesDeg.tag,
        angles_deg: { a0: a0.toFixed(2), a1: a1.toFixed(2), a2: a2.toFixed(2) }
    });*/

    // IMPORTANT: apply via your existing setters (keeps limits + table blocking)
    setAngle0FromDeg(a0);
    setAngle1FromDeg(a1);
    setAngle2FromDeg(a2);

    if (statusEl) statusEl.textContent = `ok (${res.anglesDeg.tag})`;
    return true;
}

function projectToReachableEE(xCm, yCm) {
    // convert to arm math space (same as IK)
    const tx = xCm * CM_TO_PX;
    const ty = (yCm - ARM_HEIGHT_CM) * CM_TO_PX;

    const r = Math.hypot(tx, ty);
    if (r < 1e-6) return { xCm, yCm };

    const maxReach = L0 + L1;
    const minReach = Math.abs(L0 - L1);

    const rClamped = clamp(r, minReach, maxReach);

    if (rClamped === r) {
        return { xCm, yCm }; // already legal
    }

    const scale = rClamped / r;

    const px = tx * scale;
    const py = ty * scale;

    return {
        xCm: px / CM_TO_PX,
        yCm: (py / CM_TO_PX) + ARM_HEIGHT_CM
    };
}


// FK helpers (CCW on screen: y -= sin)
function getJointPositions() {
    const phi0 = state.angles.j0;
    const phi1 = state.angles.j0 + state.angles.j1;
    const phi2 = state.angles.j0 + state.angles.j1 + state.angles.j2;

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

function isEndEffectorAboveTable() {
    const { p3 } = getJointPositions();
    const tableY = getTableY();
    // smaller y = visually higher (since screen Y goes down)
    return p3.y <= tableY;
}

function syncEndEffectorFromFK() {
    const { p3 } = getJointPositions();

    const x_px = p3.x - base.x;          // right +
    const y_px = base.y - p3.y;          // up +

    state.endEffector.x = x_px / CM_TO_PX;
    state.endEffector.y = (y_px / CM_TO_PX) + ARM_HEIGHT_CM;
}

function angleDistDeg(a, b) {
    let d = a - b;
    d = ((d + 180) % 360) - 180; // wrap to [-180, 180]
    return Math.abs(d);
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
function drawLinkSprite(origin, phi, img, size, anchor) {
    if (!img || !img.complete) return;

    ctx.save();
    ctx.translate(origin.x, origin.y);
    ctx.rotate(-phi); // Rotate: -phi because canvas y grows down
    ctx.translate(-anchor.u * size.w, -anchor.v * size.h);
    ctx.drawImage(img, 0, 0, size.w, size.h);
    ctx.restore();
}

// Draw the robot base at the arm base joint (no rotation; column is vertical)
function drawBaseSprite(origin, img, size, anchor) {
    if (!img || !img.complete) return;

    ctx.save();
    ctx.translate(origin.x, origin.y);
    ctx.translate(-anchor.u * size.w, -anchor.v * size.h);
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

    // End effector info (relative to base joint, in cm)
    if (eeOut) {
        // use CM_TO_PX if you want physical units
        const x_px = p3.x - base.x;       // right is positive
        const y_px = base.y - p3.y;       // up is positive (invert screen Y)
        const x_cm = x_px / CM_TO_PX;
        const y_cm = (y_px / CM_TO_PX) + ARM_HEIGHT_CM;
        eeOut.textContent = `x=${x_cm.toFixed(1)} cm, y=${(y_cm.toFixed(1))} cm`;
    }

    // --- Base column from table to arm base ---
    const baseBottom = { x: base.x, y: base.y + BASE_COL_HEIGHT };

    if (state.display.viewMode === "graphics" || state.display.viewMode === "both") {
        // base column
        drawBaseSprite(base, BASE_IMG, BASE_IMG_SIZE, BASE_IMG_ANCHOR);
        // Link 0
        drawLinkSprite(p0, phi0, L0_IMG, L0_IMG_SIZE, L0_IMG_ANCHOR);
        // Link 1
        drawLinkSprite(p1, phi1, L1_IMG, L1_IMG_SIZE, L1_IMG_ANCHOR);
        // Link 2
        drawLinkSprite(p2, phi2, L2_IMG, L2_IMG_SIZE, L2_IMG_ANCHOR);
    }
    if (state.display.viewMode === "lines" || state.display.viewMode === "both") {
        // base column
        strokeSegment(baseBottom, base, 6, "#9ca3af");   // grey column line
        // Link 0 (with conditional)
        if (state.display.showKinematic) strokeSegment(p0, p1, 6, "#7dd3fc");
        if (state.display.showHardware) drawRightAngleMountOverlay(p0, phi0, "#7dd3fc");
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
window.state = state;
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

