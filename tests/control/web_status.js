// Run: node tests/control/web_status.js
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../../main/control.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
new vm.Script(script); // Also check the complete embedded page's JavaScript syntax.
const status = script.slice(script.indexOf('async function status(){'), script.indexOf("$('upload').onclick"));

async function check(flags, expected, disabled, changeSession = false) {
  const elements = {};
  const context = {
    session: 'current', connected: true, version: 1,
    axes: {surge: 70, yaw: 20, heave: 0},
    keys: new Set(['forward']), pointers: new Map([[1, 'left']]),
    $: id => elements[id] || (elements[id] = {}), showHeld() {},
  };
  context.enabled = value => {context.connected = value;};
  context.fetch = async () => {
    if (changeSession) context.session = 'newly-acquired';
    return {ok: true, json: async () => ({
      last_written: {tail_front_deg: 90, tail_rear_deg: 90,
        pectoral_left_deg: 90, pectoral_right_deg: 90,
        left_motor_pct: 0, right_motor_pct: 0},
      lease_remaining_ms: 0, ...flags,
    })};
  };
  await vm.runInNewContext(status + '\nstatus()', context);
  assert.match(elements.connection.textContent, expected);
  assert.equal(context.connected, !disabled);
  if (disabled) {
    assert.equal(context.keys.size, 0);
    assert.equal(context.pointers.size, 0);
    assert.deepEqual(Object.values(context.axes), [0, 0, 0]);
    assert.equal(context.version, 2);
  }
}

(async () => {
  await check({output_fault: true, failsafe_active: true}, /输出故障已锁定/, true);
  await check({output_recovery: true, failsafe_active: true}, /连续写入异常/, true);
  await check({control_rearm_required: true}, /重新启用控制/, true);
  await check({ota_locked: true}, /升级期间/, true);
  await check({output_warning: true}, /输出写入异常/, false);
  await check({failsafe_active: true}, /3 秒未续期/, false);
  await check({soft_stopping: true}, /舵机和螺旋桨正在缓停/, false);
  await check({}, /控制已连接/, false);
  await check({control_rearm_required: true}, /重新启用控制/, false, true);
  console.log('Web safety status: 9 cases passed');
})().catch(error => {console.error(error); process.exitCode = 1;});
