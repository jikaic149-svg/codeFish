"""Headless Edge check using loopback HTTP fixtures; never opens hardware ports."""
import copy
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import socket

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT / 'tests')]
import diagnostics
import server
from test_robotfish import Target as ControlTarget, HEALTH
from test_realtime import RAW


class Target(ControlTarget):
    protocol_version = 'HTTP/1.1'
    reads = 0
    connections = set()
    delay = 0
    motion_unavailable = False
    lose_next_target_reply = False
    stop_reply_failures = 0

    def reply(self, data, status=200):
        if self.path == '/api/v1/control' and data.get('accepted') is True:
            action = self.controls[-1]['action']
            if action == 'target' and type(self).lose_next_target_reply:
                type(self).lose_next_target_reply = False
                type(self).motion_unavailable = True
                self.close_connection = True
                self.connection.shutdown(socket.SHUT_RDWR)
                return
            if action == 'stop' and type(self).stop_reply_failures:
                type(self).stop_reply_failures -= 1
                return super().reply({'error': 'control_timeout'}, 503)
        return super().reply(data, status)

    def do_GET(self):
        if self.path == '/get_ip':
            return self.reply(dict(HEALTH, lora_protocol=2))
        if self.path == '/motion_status':
            type(self).reads += 1
            self.connections.add(self.client_address[1])
            if self.motion_unavailable:
                return self.reply({'error': 'fixture_unavailable'}, 503)
            raw = copy.deepcopy(self.motion)
            raw.update(lora_protocol=2, lora={'healthy': False, 'accepted': 0},
                       preferred_control='wifi', imu=copy.deepcopy(RAW['imu']))
            raw['imu']['frames'] = int(time.monotonic() * 300)
            raw['imu']['rx_bytes'] = raw['imu']['frames'] * 11
            time.sleep(self.delay)
            return self.reply(raw)
        super().do_GET()


def verify_recovery(page):
    acquired = sum(entry['action'] == 'acquire' for entry in Target.controls)
    for lose_reply in (False, True):
        Target.lose_next_target_reply = lose_reply
        page.evaluate('document.activeElement.blur()')
        page.keyboard.down('w')
        if not lose_reply:
            page.wait_for_function('robotState?.control?.axes?.surge === 35')
            Target.motion_unavailable = True
        page.wait_for_function('controlSuspended && wifiArmed')
        assert not server.device_client.CLIENT._sessions[server.STATION.host].get('password') is None
        page.keyboard.up('w')
        before = len(Target.controls)
        page.wait_for_timeout(350)
        assert len(Target.controls) == before, 'Old held input was transmitted during an outage'
        Target.motion_unavailable = False
        page.wait_for_function('wifiControlReady() && !controlSuspended', timeout=6000)
        restored = Target.controls[before:]
        assert restored and all(entry['action'] == 'stop' for entry in restored), restored
        page.wait_for_timeout(350)
        assert Target.controls[-1]['action'] == 'stop'
        assert sum(entry['action'] == 'acquire' for entry in Target.controls) == acquired

    # Failed STOP acknowledgments must have a finite recovery budget.
    Target.motion_unavailable = True
    page.wait_for_function('controlSuspended && wifiArmed')
    Target.stop_reply_failures = 3
    Target.motion_unavailable = False
    page.wait_for_function('recoveryAttempts === 3 && !recoveryPending', timeout=7000)
    before = len(Target.controls)
    page.wait_for_timeout(1200)
    assert len(Target.controls) == before, 'Recovery exceeded its retry budget'
    assert page.locator('#control-stop').is_enabled()
    page.locator('#control-stop').click()
    page.wait_for_function('wifiControlReady() && !controlSuspended')
    assert sum(entry['action'] == 'acquire' for entry in Target.controls) == acquired

    # A different controller is a permanent session change, not a reconnect trigger.
    Target.motion['wifi']['session'] = 'another-operator'
    page.wait_for_function('!wifiArmed')
    before = len(Target.controls)
    page.wait_for_timeout(500)
    assert len(Target.controls) == before


def main():
    Target.reset()
    # Fault injection must not inspect the user's Windows network during this test.
    server.device_client.capture_network_context = lambda: None
    server.device_client.CLIENT.discover_robotfish = lambda candidates=(): None
    target = ThreadingHTTPServer(('127.0.0.1', 0), Target)
    web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    server.PORT = web.server_port
    for http in (target, web):
        threading.Thread(target=http.serve_forever, daemon=True).start()
    server.STATION.start()
    host = f'127.0.0.1:{target.server_port}'
    with tempfile.TemporaryDirectory() as directory:
        diagnostics.start(directory)
        try:
            with sync_playwright() as playwright:
                browser = playwright.chromium.launch(channel='msedge', headless=True)
                page = browser.new_page(viewport={'width': 1280, 'height': 900})
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto(f'http://127.0.0.1:{web.server_port}/')
                page.locator('[data-go="network"]').first.click()
                page.evaluate('document.getElementById("connection-advanced").open=true')
                page.locator('#robot-host').fill(host)
                page.wait_for_function('host => robotState?.host === host && robotIdentity?.firmware === "robotfish"', arg=host)
                page.locator('[data-page="overview"]').click()
                page.wait_for_function('robotState?.imu?.valid === true')
                start = Target.reads
                page.wait_for_timeout(1200)
                reads = Target.reads - start
                assert 8 <= reads <= 17, reads
                assert len(Target.connections) == 1, Target.connections
                metrics = page.evaluate('''() => {
                  const observer=new MutationObserver(()=>{});
                  observer.observe(document.body,{childList:true,subtree:true,attributes:true});
                  const start=performance.now();for(let i=0;i<100;i++)renderRobot();
                  const elapsed_ms=performance.now()-start,dom_mutations=observer.takeRecords().length;
                  observer.disconnect();return {renders:100,elapsed_ms,dom_mutations};
                }''')
                assert metrics['dom_mutations'] < 3000, metrics

                # Exercise the real UI control path, with a loopback fake fish.
                page.locator('[data-go="network"]').first.click()
                page.evaluate('document.getElementById("device-session").open=true')
                page.locator('#robot-password').fill('test-user-input')
                page.locator('#robot-remember').uncheck()
                page.locator('#robot-login').click()
                page.wait_for_function('wifiControlReady()')
                page.locator('[data-page="overview"]').click()
                page.locator('#robot-control-mode').select_option('2')
                page.wait_for_function('robotState?.control?.mode === 2 && !controlConfigPending')
                page.evaluate('document.activeElement.blur()')
                page.keyboard.down('w')
                page.wait_for_function('robotState?.control?.axes?.surge === 35')
                page.keyboard.up('w')
                page.wait_for_function('robotState?.control?.axes?.surge === 0')
                page.locator('#control-stop').click()
                page.wait_for_function('document.getElementById("robot-command-result").textContent === "已全停"')
                assert Target.controls[-1]['action'] == 'stop'
                verify_recovery(page)

                page.locator('[data-page="data"]').click()
                page.locator('#capture-toggle').click()
                page.wait_for_function('recorded.count >= 1')
                unchanged = page.evaluate('''() => {
                  renderDataTable();const observer=new MutationObserver(()=>{});
                  observer.observe($('data-rows'),{childList:true,subtree:true});
                  for(let i=0;i<100;i++)renderDataTable();
                  const count=observer.takeRecords().length;observer.disconnect();return count;
                }''')
                assert unchanged == 0, unchanged
                page.locator('#capture-toggle').click()
                page.locator('[data-page="overview"]').click()

                Target.delay = .65
                page.wait_for_function('robotState?.imu?.valid === false', timeout=5000)
                assert page.locator('#imu-yaw').inner_text() == '—'
                assert page.locator('#imu-state').inner_text() == '数据过期'
                Target.delay = 0
                page.wait_for_function('robotState?.imu?.valid === true', timeout=5000)

                # Diagnostic events are persisted without adding UI journal rows.
                page.evaluate('''() => window.dispatchEvent(new ErrorEvent('error',
                  {message:'realtime-browser-check',filename:'http://local/check.js',lineno:7}))''')
                page.wait_for_timeout(1800)
                assert page.locator('#log-drawer-body').is_hidden()
                assert not page.evaluate('state.logs.some(row=>row.message.includes("realtime-browser-check"))')
                assert not errors, errors
                browser.close()
            diagnostics.stop()
            text = (Path(directory) / 'workstation.jsonl').read_text(encoding='utf-8')
            rows = [json.loads(line) for line in text.splitlines()]
            assert any(row['event'] == 'browser' and row.get('message') == 'realtime-browser-check' for row in rows)
            assert any(row['event'] == 'telemetry' and row['imu']['valid'] is False for row in rows)
            assert any(row['event'] == 'device_http' and row['path'] == '/motion_status' and row['elapsed_ms'] >= 600 for row in rows)
            assert 'test-user-input' not in text and server.TOKEN not in text
            print(json.dumps(dict(single_tab_reads_per_1_2s=reads, telemetry_connections=len(Target.connections),
                rendering=metrics, unchanged_table_mutations=unchanged,
                checks='control, recording, telemetry outage, lost ACK, no goal replay, bounded recovery, takeover rejection, slow/stale response, silent logs passed')))
        finally:
            diagnostics.stop()
            for connection in server.device_client.CLIENT._telemetry_connections.values():
                connection.close()
            for http in (target, web):
                http.shutdown()
                http.server_close()


if __name__ == '__main__':
    main()
