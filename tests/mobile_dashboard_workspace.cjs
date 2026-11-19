// Exercise the shipped mobile dashboard with deterministic telemetry and form input.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const elements = new Map();
function element(id) {
  if (!elements.has(id)) elements.set(id, {
    textContent: '', dataset: {}, hidden: false, value: '', checked: false,
    listeners: {}, addEventListener(name, callback) { this.listeners[name] = callback; },
    querySelectorAll: () => [], querySelector: selector => element(selector), setAttribute() {},
  });
  return elements.get(id);
}
const saved = {version: 1, x_min_cm: 10, x_max_cm: 30, y_min_cm: -10, y_max_cm: 10,
  height_min_cm: 5, height_max_cm: 20, table_z_cm: -2};
const fields = Object.fromEntries(Object.entries(saved).filter(([key]) => key !== 'version')
  .map(([key, value]) => [key, {value: String(value)}]));
const form = element('workspace-form');
form.elements = {namedItem: name => fields[name]};
form.reportValidity = () => true;
class FormData {
  *[Symbol.iterator]() { for (const [key, field] of Object.entries(fields)) yield [key, field.value]; }
}
const timers = [];
let session = 'first';
let offline = false;
let rejectedSave = false;
let submitted;
const remote = {mode: 'Input only', phone_mode: 'aruco', marker_pose: {state: 'Tracking'},
  robot_workspace: {workspace: saved, error: ''},
  aruco_control: {alignment_cm: 5.4, marker_adjustment_cm: [-2, 3, -4], tool_in_workspace: true,
    measured_xyz_m: [.2, 0, .1], target_xyz_m: [.22, -.03, .14],
    workspace_m: {min: [.1, -.1, .03], max: [.3, .1, .18]}, reason: 'Align marker'}};
const context = vm.createContext({
  document: {getElementById: element, querySelectorAll: () => []},
  FormData, AbortSignal, setTimeout: callback => timers.push(callback),
  fetch: async (url, options = {}) => {
    if (offline) throw new Error('Offline');
    if (options.method === 'POST') {
      assert.equal(url, '/api/robot-workspace');
      assert.equal(options.headers['X-Dashboard-Session'], session);
      submitted = JSON.parse(options.body);
      return {ok: !rejectedSave, json: async () => ({}), text: async () => 'Invalid workspace'};
    }
    return {ok: true, headers: {get: () => session}, json: async () => ({age_s: 0, remote})};
  },
});
const html = fs.readFileSync(0, 'utf8');
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1], context);
const settle = () => new Promise(resolve => setImmediate(resolve));
async function poll() { await timers.shift()(); await settle(); }

(async () => {
  await settle();
  assert.equal(fields.x_max_cm.value, 30);
  assert.equal(element('button').disabled, false);
  assert.equal(element('alignment-x-adjustment').textContent, '-2.0 · backward');
  assert.equal(element('alignment-y-adjustment').textContent, '+3.0 · left');
  assert.equal(element('alignment-z-adjustment').textContent, '-4.0 · down');
  assert.equal(element('alignment-z-bounds').textContent, '3.0 to 18.0');
  assert.match(element('workspace-height-summary').textContent, /3.0 to 18.0/);
  remote.aruco_control.marker_adjustment_cm = [2, -3, 4];
  await poll();
  assert.equal(element('alignment-x-adjustment').textContent, '+2.0 · forward');
  assert.equal(element('alignment-y-adjustment').textContent, '-3.0 · right');
  assert.equal(element('alignment-z-adjustment').textContent, '+4.0 · up');
  remote.aruco_control.enabled = true;
  await poll();
  assert.equal(element('alignment-z-adjustment').textContent, '+4.0');
  assert.match(element('alignment-help').textContent, /following error/);
  assert.doesNotMatch(element('aruco-alignment').textContent, /2 cm/);
  remote.aruco_control.enabled = false;
  remote.aruco_control.marker_adjustment_cm = [-2, 3, -4];
  remote.aruco_control.tool_in_workspace = false;
  remote.aruco_control.measured_xyz_m[2] = .02;
  await poll();
  assert.equal(element('alignment-z').dataset.outside, true);
  assert.equal(element('alignment-z-adjustment').textContent, '-4.0', 'Do not direct the marker outside the box');
  assert.match(element('alignment-help').textContent, /alignment is impossible/);
  remote.aruco_control.marker_adjustment_cm = null;
  remote.aruco_control.alignment_cm = null;
  await poll();
  assert.equal(element('alignment-z-adjustment').textContent, '—', 'Lost tracking clears stale directions');
  assert.equal(element('alignment-z-target').textContent, '—');
  fields.x_max_cm.value = '35';
  await poll();
  assert.equal(fields.x_max_cm.value, '35', 'Polling must not replace a draft edit');
  element('workspace-reviewed').checked = true;
  await form.listeners.submit({preventDefault() {}});
  assert.equal(submitted.x_max_cm, 35);
  assert.equal(submitted.version, 1);
  assert.equal(element('workspace-reviewed').checked, false);
  assert.match(element('workspace-message').textContent, /saved/);
  rejectedSave = true;
  await form.listeners.submit({preventDefault() {}});
  assert.match(element('workspace-message').textContent, /Invalid workspace/);
  offline = true;
  await poll();
  assert.equal(element('button').disabled, true);
  session = 'second';
  offline = false;
  await poll();
  assert.equal(fields.x_max_cm.value, 30, 'A new server session reloads the saved configuration');
  assert.equal(element('workspace-reviewed').checked, false);
  assert.equal(element('button').disabled, false);
  console.log('Mobile workspace editor checks passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
