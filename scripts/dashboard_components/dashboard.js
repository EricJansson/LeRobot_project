const byId = id => document.getElementById(id);
const readingsFields = document.querySelectorAll('[data-value]');
const cartesianAxes = ['x', 'y', 'z'];

function setText(element, value) {
  if (element.textContent !== value) element.textContent = value;
}

function renderConnectivity(reason, offlineLabel = 'Offline') {
  byId('connectivity').dataset.online = !reason;
  setText(byId('connectivity-label'), reason ? offlineLabel : 'Online');
  const detail = byId('connectivity-detail');
  setText(detail, reason ? ` · ${reason}` : '');
  detail.hidden = !reason;
}

function renderArmStatus(tone, label, detail) {
  byId('status-card').dataset.tone = tone;
  setText(byId('status'), label);
  const detailField = byId('status-detail');
  setText(detailField, detail ?? '');
  detailField.hidden = !detail;
}

function releaseProgress(value) {
  return Number.isFinite(value) ? Math.max(0, Math.min(1, value)) : 0;
}

function renderRelease(progress, label, visible) {
  byId('release-gesture').hidden = !visible;
  byId('release-fill').setAttribute('stroke-dashoffset', String(100 * (1 - progress)));
  const countdown = byId('release-countdown');
  countdown.setAttribute('aria-valuenow', String(Math.round(progress * 100)));
  setText(countdown, label);
}

function renderReadings(readings) {
  for (const element of readingsFields) {
    const key = element.dataset.value;
    let value;
    if (cartesianAxes.includes(key)) {
      const meters = readings?.xyz_m?.[key];
      if (Number.isFinite(meters)) {
        const mm = meters * 1000;
        value = `${mm > 0 ? '+' : ''}${mm.toFixed(1)} mm`;
      }
    } else {
      const position = key === 'gripper' ? readings?.gripper_normalized : readings?.joints_deg?.[key];
      if (Number.isFinite(position)) value = position.toFixed(1) + (key === 'gripper' ? '' : '°');
    }
    setText(element, value ?? '—');
    element.dataset.measured = value !== undefined;
  }
}

function formatReading(value, suffix = '') {
  return Number.isFinite(value) ? value.toFixed(1) + suffix : '—';
}

function formatVector(values, scale = 1) {
  if (!values?.length) return '—';
  return values.map(value => Number.isFinite(value) ? (value * scale).toFixed(1) : '—').join(', ');
}

function renderDiagnostics(remote) {
  setText(byId('requested-xyz'), formatVector(remote?.requested_xyz_m, 1000));
  setText(byId('ik-candidate'), formatVector(remote?.candidate));
  setText(byId('motor-target'), formatVector(remote?.sent));
  setText(byId('feedback-rate'), formatReading(remote?.feedback_hz, ' Hz'));
  setText(byId('feedback-gap'), formatReading(remote?.feedback_max_gap_ms, ' ms'));
  setText(byId('pi-read'), formatReading(remote?.read_max_ms, ' ms'));
  setText(byId('pi-write'), formatReading(remote?.write_max_ms, ' ms'));
}
