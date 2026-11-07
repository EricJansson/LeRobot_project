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
  performance: { now: () => 100 },
  EventSource, AbortController,
  setInterval() {},
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
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
const settle = () => new Promise(resolve => setImmediate(resolve));
async function nextPoll() {
  assert.equal(timers.length, 1, 'Exactly one polling timer must remain');
  const timer = timers.shift();
  await timer.callback();
  await settle();
}

(async () => {
  await settle();
  assert.equal(value.textContent, '12.0°');
  assert.equal(streams.length, 1);
  streams[0].stopped('Esc pressed');
  assert.equal(element('status').textContent, 'Program stopped');
  assert.equal(element('status-detail').textContent, 'Esc pressed');
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
  assert.equal(element('status').textContent, 'Waiting for telemetry');
  assert.equal(value.textContent, '—', 'Old readings must clear before the first new sample');
  assert.equal(element('age').hidden, true);
  assert.equal(streams.length, 2);
  readings = { age_s: 0, joints_deg: { shoulder_pan: 25 }, gamepad_enabled: false };
  await nextPoll();
  assert.equal(value.textContent, '25.0°');
  assert.equal(element('status').textContent, 'Standing by');

  // Unexpected loss and recovery, without a graceful stopped event.
  offline = true;
  await nextPoll();
  assert.equal(element('status').textContent, 'Telemetry unavailable');
  offline = false;
  session = 'third';
  readings = null;
  await nextPoll();
  assert.equal(value.textContent, '—');
  assert.equal(streams[1].closed, true);
  assert.equal(streams.length, 3);
  // A late event from an old connection cannot stop the new session.
  streams[1].stopped('Old session ended');
  assert.equal(element('status').textContent, 'Waiting for telemetry');
  streams[2].stopped('Ctrl+C pressed');
  assert.equal(element('status-detail').textContent, 'Ctrl+C pressed');
})().catch(error => { console.error(error); process.exitCode = 1; });
