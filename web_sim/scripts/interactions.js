function getCanvasPoint(e) {
  const r = cv.getBoundingClientRect();
  return { x: e.clientX - r.left, y: e.clientY - r.top };
}

// angle around origin (deg), CCW; screen Y inverted
function angleDegFromPointer(pt, origin) {
  const dx = pt.x - origin.x;
  const dy = origin.y - pt.y;
  return (Math.atan2(dy, dx) * 180) / Math.PI; // [-180, 180]
}

// keep continuity near reference to avoid wrap jumps
function normalizeToNearest(rawDeg, referenceDeg) {
  const k = Math.round((referenceDeg - rawDeg) / 360);
  return rawDeg + 360 * k;
}

let dragging = false;
let dragKind = null; // 'J0' or 'J1'

cv.addEventListener("pointermove", (e) => {
  const pt = getCanvasPoint(e);
  const { p1, p2, p3 } = getJointPositions(); // was { p1, p2 }

  // hover
  if (!dragging) {
    const nearP1 = Math.hypot(pt.x - p1.x, pt.y - p1.y) <= HANDLE_R + 4;
    const nearP2 = Math.hypot(pt.x - p2.x, pt.y - p2.y) <= HANDLE_R + 4;
    const nearP3 = Math.hypot(pt.x - p3.x, pt.y - p3.y) <= HANDLE_R + 4;

    if (nearP3) cv.style.cursor = "grab";
    else if (nearP2) cv.style.cursor = "grab";
    else if (nearP1) cv.style.cursor = "grab";
    else cv.style.cursor = "default";

    return;
  }

  if (dragKind === "J0") {
    // rotate first link around base using pointer angle at p0
    const { p0 } = getJointPositions();
    let target = angleDegFromPointer(pt, p0);
    const current0 = (angle0Rad * 180) / Math.PI;
    target = normalizeToNearest(target, current0);
    target = clamp(target, ANG0_MIN_DEG, ANG0_MAX_DEG);
    setAngle0FromDeg(target);
  } else if (dragKind === "J1") {
    // rotate second link around p1, adjusting J1's LOCAL angle
    const { p1 } = getJointPositions();

    // Desired GLOBAL phi1 from pointer:
    let phi1Deg = angleDegFromPointer(pt, p1);
    const currentPhi1 = ((angle0Rad + angle1Rad) * 180) / Math.PI;
    phi1Deg = normalizeToNearest(phi1Deg, currentPhi1);

    // Convert to LOCAL J1 angle: angle1 = phi1 - angle0
    const angle0Deg = (angle0Rad * 180) / Math.PI;
    let j1Deg = phi1Deg - angle0Deg;

    // Clamp to J1 limits
    j1Deg = clamp(j1Deg, ANG1_MIN_DEG, ANG1_MAX_DEG);

    setAngle1FromDeg(j1Deg);
  } else if (dragKind === "J2") {
    // rotate third link around p2, adjusting J2's LOCAL angle
    const { p2 } = getJointPositions();

    // Desired GLOBAL phi2 from pointer:
    let phi2Deg = angleDegFromPointer(pt, p2);
    const currentPhi2 = ((angle0Rad + angle1Rad + angle2Rad) * 180) / Math.PI;
    phi2Deg = normalizeToNearest(phi2Deg, currentPhi2);

    // LOCAL J2: angle2 = phi2 - (angle0 + angle1)
    const base12Deg = ((angle0Rad + angle1Rad) * 180) / Math.PI;
    let j2Deg = phi2Deg - base12Deg;

    // clamp and set
    j2Deg = clamp(j2Deg, ANG2_MIN_DEG, ANG2_MAX_DEG);
    setAngle2FromDeg(j2Deg);
  }

});

cv.addEventListener("pointerdown", (e) => {
  e.preventDefault();
  const pt = getCanvasPoint(e);
  const { p1, p2, p3 } = getJointPositions();
  const d1 = Math.hypot(pt.x - p1.x, pt.y - p1.y);
  const d2 = Math.hypot(pt.x - p2.x, pt.y - p2.y);
  const d3 = Math.hypot(pt.x - p3.x, pt.y - p3.y);

  if (d3 <= HANDLE_R + 4) {
    dragKind = "J2";
  } else if (d2 <= HANDLE_R + 4) {
    dragKind = "J1";
  } else if (d1 <= HANDLE_R + 4) {
    dragKind = "J0";
  } else {
    dragKind = null;
  }


  dragging = !!dragKind;
  if (dragging) {
    cv.setPointerCapture(e.pointerId);
    cv.style.cursor = "grabbing";

    // apply once on down
    if (dragKind === "J0") {
      const { p0 } = getJointPositions();
      let target = angleDegFromPointer(getCanvasPoint(e), p0);
      const current0 = (angle0Rad * 180) / Math.PI;
      target = normalizeToNearest(target, current0);
      target = clamp(target, ANG0_MIN_DEG, ANG0_MAX_DEG);
      setAngle0FromDeg(target);
    } else if (dragKind === "J1") {
      const { p1 } = getJointPositions();
      let phi1Deg = angleDegFromPointer(getCanvasPoint(e), p1);
      const currentPhi1 = ((angle0Rad + angle1Rad) * 180) / Math.PI;
      phi1Deg = normalizeToNearest(phi1Deg, currentPhi1);
      const angle0Deg = (angle0Rad * 180) / Math.PI;
      let j1Deg = phi1Deg - angle0Deg;
      j1Deg = clamp(j1Deg, ANG1_MIN_DEG, ANG1_MAX_DEG);
      setAngle1FromDeg(j1Deg);
    } else if (dragKind === "J2") {
      const { p2 } = getJointPositions();
      let phi2Deg = angleDegFromPointer(getCanvasPoint(e), p2);
      const currentPhi2 = ((angle0Rad + angle1Rad + angle2Rad) * 180) / Math.PI;
      phi2Deg = normalizeToNearest(phi2Deg, currentPhi2);
      const base12Deg = ((angle0Rad + angle1Rad) * 180) / Math.PI;
      let j2Deg = phi2Deg - base12Deg;
      j2Deg = clamp(j2Deg, ANG2_MIN_DEG, ANG2_MAX_DEG);
      setAngle2FromDeg(j2Deg);
    }

  }
});

cv.addEventListener("pointerup", (e) => {
  dragging = false;
  dragKind = null;
  cv.style.cursor = "default";
  try { cv.releasePointerCapture(e.pointerId); } catch { }
});

cv.addEventListener("pointercancel", () => {
  dragging = false;
  dragKind = null;
  cv.style.cursor = "default";
});
