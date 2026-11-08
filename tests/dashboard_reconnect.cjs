// Execute the shipped dashboard script with a small DOM and deterministic network.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    textContent: '', dataset: {}, hidden: false,
    querySelectorAll: () => [], setAttribute() {}, classList: { toggle() {} },
  });
  return elements.get(id);
}
const value = element('reading');
value.dataset.value = 'shoulder_pan';
const streams = [];
const timers = [];
let session = 'first';
let readings = { age_s: 0, joints_deg: { shoulder_pan: 12 }, xyz_m: {}, gamepad_enabled: true };
let offline = false;
let now = 100;
let updateStatus;
class EventSource {
  constructor() { this.listeners = {}; this.closed = false; streams.push(this); }
  addEventListener(name, callback) { this.listeners[name] = callback; }
  close() { this.closed = true; }
  stopped(reason) { this.listeners.stopped({ data: JSON.stringify(reason) }); }
}
const context = vm.createContext({
  document: {
    getElementById: element,
    querySelectorAll: () => [value],
  },
  performance: { now: () => now },
  EventSource, AbortController,
  setInterval(callback) { updateStatus = callback; },
  setTimeout(callback, delay) { timers.push({ callback, delay }); return callback; },
  clearTimeout(callback) {
    const index = timers.findIndex(timer => timer.callback === callback);
    if (index >= 0) timers.splice(index, 1);
  },
  fetch: async () => {
    if (offline) throw new Error('Connection refused');
    return { ok: true, headers: { get: () => session }, json: async () => readings };
  },
});
const html = fs.readFileSync(path.join(__dirname, '../scripts/lerobot_web_dashboard.html'), 'utf8');
assert.doesNotMatch(html, /Last sample|id="age"|ageLabel|#age\s*\{/);
assert.doesNotMatch(html, /Motion allowed|id="remote-status"|id="remote-target"|id="remote-timing"|SO-101 · LeRobot IK · Read-only/);
assert.match(html, /<details id="diagnostics"[^>]*hidden>/);
assert.match(html, /id="connectivity"[^>]*role="status"[^>]*aria-atomic="true"/);
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
const settle = () => new Promise(resolve => setImmediate(resolve));
async function nextPoll() {
  assert.equal(timers.length, 1, 'Exactly one polling timer must remain');
  const timer = timers.shift();
  await timer.callback();
  await settle();
}

(async () => {
  // The first HTTP response has not arrived yet.
  updateStatus();
  assert.equal(element('connectivity-label').textContent, 'Offline');
  assert.match(element('connectivity-detail').textContent, /Waiting for telemetry/);
  assert.equal(element('status').textContent, 'Control unavailable');
  await settle();
  assert.equal(value.textContent, '12.0°');
  assert.equal(element('connectivity-label').textContent, 'Online');
  assert.equal(element('connectivity-detail').hidden, true);
  assert.equal(element('status').textContent, 'Control enabled');
  assert.equal(element('arm-mode').hidden, true);
  assert.equal(element('diagnostics').hidden, true);
  assert.equal(streams.length, 1);

  // A failed request does not discard a still-fresh sample or flicker offline.
  offline = true;
  now += 1000;
  await nextPoll();
  assert.equal(element('connectivity-label').textContent, 'Online');
  assert.equal(element('lb-status').textContent, 'LB held');
  now += 2000;
  updateStatus();
  assert.equal(element('connectivity-label').textContent, 'Online', 'Exactly 3 seconds is fresh');
  now += 1;
  updateStatus();
  assert.equal(element('connectivity-label').textContent, 'Offline');
  assert.match(element('connectivity-detail').textContent, /Telemetry unavailable/);
  assert.equal(element('status').textContent, 'Control unavailable');
  assert.equal(element('lb-status').textContent, 'LB unavailable');
  assert.equal(element('gamepad').dataset.live, false);
  assert.equal(value.textContent, '12.0°', 'Last measured readings remain visible');
  offline = false;
  await nextPoll();
  assert.equal(element('connectivity-label').textContent, 'Online');
  assert.equal(streams[0].closed, true);
  assert.equal(streams.length, 2);

  // A healthy HTTP connection cannot make an old producer sample fresh.
  readings.age_s = 3.001;
  await nextPoll();
  assert.equal(element('connectivity-label').textContent, 'Offline');
  readings.age_s = 3;
  await nextPoll();
  assert.equal(element('connectivity-label').textContent, 'Online');
  readings.age_s = 0;
  readings.gamepad = { name: 'Mock gamepad', deadzone: 0.1, axes: {}, buttons: {}, dpad: [0, 0] };
  readings.remote = { connection: 'Connected', state: 'Armed', mode: 'Real arm enabled', reason: 'Ready for movement input' };
  await nextPoll();
  assert.equal(element('arm-mode').textContent, 'Arm mode: Physical');
  assert.equal(element('diagnostics').hidden, false);
  assert.equal(element('feedback-rate').textContent, '—');
  assert.equal(element('ik-candidate').textContent, '—');
  readings.remote.requested_xyz_m = [0.2267, -0.0043, 0.0591];
  readings.remote.sent = [2, 13.4, -11.2, 95, -8.6, 1.5];
  readings.remote.feedback_hz = 28.8;
  readings.remote.feedback_max_gap_ms = 100.1;
  readings.remote.read_max_ms = 6.5;
  readings.remote.write_max_ms = 3.5;
  await nextPoll();
  assert.equal(element('requested-xyz').textContent, '226.7, -4.3, 59.1');
  assert.equal(element('motor-target').textContent, '2.0, 13.4, -11.2, 95.0, -8.6, 1.5');
  assert.equal(element('feedback-rate').textContent, '28.8 Hz');
  assert.equal(element('feedback-gap').textContent, '100.1 ms');
  assert.equal(element('pi-read').textContent, '6.5 ms');
  assert.equal(element('pi-write').textContent, '3.5 ms');
  for (const mode of ['Local simulation', 'Pi simulation', 'Real arm']) {
    readings.remote.mode = mode;
    await nextPoll();
    assert.match(element('arm-mode').textContent, mode === 'Real arm' ? /Physical/ : /Simulation/);
  }
  assert.equal(element('status').textContent, 'Armed');
  assert.equal(element('status-detail').hidden, true);
  assert.equal(element('gamepad').dataset.live, true);
  readings.remote.state = 'Moving';
  await nextPoll();
  assert.equal(element('status').textContent, 'Moving');
  assert.equal(element('status-detail').hidden, true);
  readings.remote.state = 'Fault';
  readings.remote.reason = 'Motor read failed';
  await nextPoll();
  assert.equal(element('connectivity-label').textContent, 'Online', 'Faults are separate from connectivity');
  assert.equal(element('status').textContent, 'Motor fault');
  assert.equal(element('status-detail').textContent, 'Motor read failed');
  assert.equal(element('status-card').dataset.tone, 'red');
  readings.remote.state = 'Disarmed';
  readings.remote.reason = '';
  readings.remote.mode = 'Observe only';
  await nextPoll();
  assert.equal(element('status').textContent, 'Disarmed');
  assert.match(element('status-detail').textContent, /Observe only.*motion disabled/);
  assert.equal(element('arm-mode').textContent, 'Arm mode: Physical · Observe only');
  readings.remote.reason = 'Start requires neutral controls';
  await nextPoll();
  assert.equal(element('status-detail').textContent, 'Start requires neutral controls');

  for (const connection of ['Connecting', 'Disconnected']) {
    readings.remote.connection = connection;
    readings.remote.state = 'Moving';
    readings.remote.reason = 'Pi transport unavailable';
    await nextPoll();
    assert.equal(element('connectivity-label').textContent, 'Offline');
    assert.match(element('connectivity-detail').textContent,
      connection === 'Connecting' ? /Connecting to Pi/ : /Pi disconnected/);
    assert.match(element('connectivity-detail').textContent, /Pi transport unavailable/);
    assert.equal(element('status').textContent, 'Control unavailable');
    assert.equal(element('diagnostics').hidden, true);
    assert.equal(element('arm-mode').textContent, 'Arm mode: Physical · Observe only');
    assert.equal(element('lb-status').textContent, 'LB unavailable');
    assert.equal(element('gamepad').dataset.live, false);
    assert.equal(value.textContent, '12.0°');
  }
  readings.remote.connection = 'Connected';
  readings.remote.state = 'Armed';
  await nextPoll();
  assert.equal(element('connectivity-label').textContent, 'Online');
  assert.equal(element('status').textContent, 'Armed');
  assert.equal(element('gamepad').dataset.live, true);

  streams[0].stopped('Old connection ended');
  assert.equal(element('status').textContent, 'Armed');
  streams[1].stopped('Esc pressed');
  assert.equal(element('status').textContent, 'Program stopped');
  assert.equal(element('status-detail').textContent, 'Esc pressed');
  assert.equal(element('connectivity-label').textContent, 'Offline');
  assert.match(element('connectivity-detail').textContent, /Program stopped/);
  // A final response from the stopping server must not erase its exit reason.
  await nextPoll();
  assert.equal(element('status').textContent, 'Program stopped');
  offline = true;
  await nextPoll();
  assert.equal(element('status-detail').textContent, 'Esc pressed');
  assert.equal(timers[0].delay, 1000);

  offline = false;
  session = 'second';
  readings = null;
  await nextPoll();
  assert.equal(element('status').textContent, 'Control unavailable');
  assert.match(element('connectivity-detail').textContent, /Waiting for telemetry/);
  assert.equal(value.textContent, '—', 'Old readings must clear before the first new sample');
  assert.equal(element('arm-mode').hidden, true);
  assert.equal(element('diagnostics').hidden, true);
  assert.equal(streams.length, 3);
  readings = { age_s: 0, joints_deg: { shoulder_pan: 25 }, gamepad_enabled: false };
  await nextPoll();
  assert.equal(value.textContent, '25.0°');
  assert.equal(element('status').textContent, 'Standing by');
  assert.equal(element('connectivity-label').textContent, 'Online');

  // Unexpected loss and recovery, without a graceful stopped event.
  offline = true;
  now += 3001;
  await nextPoll();
  assert.equal(element('status').textContent, 'Control unavailable');
  assert.equal(element('connectivity-label').textContent, 'Offline');
  offline = false;
  session = 'third';
  readings = null;
  await nextPoll();
  assert.equal(value.textContent, '—');
  assert.equal(streams[2].closed, true);
  assert.equal(streams.length, 4);
  // A late event from an old connection cannot stop the new session.
  streams[2].stopped('Old session ended');
  assert.equal(element('status').textContent, 'Control unavailable');
  streams[3].stopped('Ctrl+C pressed');
  assert.equal(element('status-detail').textContent, 'Ctrl+C pressed');
})().catch(error => { console.error(error); process.exitCode = 1; });
