"""Headless Edge: no address/password input for a factory-configured fake fish."""
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
from unittest.mock import patch

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT / 'tests')]
import diagnostics
import robotfish
import server
from credentials import KeyStore
from verify_realtime import Target as TelemetryTarget


class Target(TelemetryTarget):
    CONTROL_PASSWORD = robotfish.DEFAULT_CONTROL_PASSWORD


def verify_delayed_logout(page, url, host, keys, *, automatic):
    pending = []
    endpoint = 'auto-connect' if automatic else 'login'
    page.route('**/api/device/' + endpoint, lambda route: pending.append((route, route.fetch())))
    if automatic:
        page.goto(url)
    else:
        page.locator('[data-go="network"]').first.click()
        page.locator('#connection-advanced').evaluate('(element) => element.open = true')
        page.locator('#robot-password').fill(Target.CONTROL_PASSWORD)
        page.evaluate('window.loginAttempt = robotLogin(); void 0')
    for _ in range(100):
        if pending:
            break
        page.wait_for_timeout(20)
    assert pending, 'The simulated login response was not intercepted'
    try:
        page.locator('[data-go="network"]').first.click()
        page.locator('#connection-advanced').evaluate('(element) => element.open = true')
        page.locator('#forget-key').click()
        page.wait_for_function('document.getElementById("robot-login-state").textContent.includes("已退出")')
        assert host not in server.device_client.CLIENT._sessions
        assert not keys.has(host)
    finally:
        route, response = pending[0]
        route.fulfill(response=response)
    if automatic:
        page.wait_for_function('!autoConnectPending')
    else:
        page.evaluate('window.loginAttempt')
    assert not page.evaluate('wifiArmed'), 'A delayed login response restored page authorization'
    assert not server.device_client.CLIENT.identity(host)['authenticated']
    assert not keys.has(host)


def main():
    Target.reset()
    target = ThreadingHTTPServer(('127.0.0.1', 0), Target)
    web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    server.PORT = web.server_port
    for http in (target, web):
        threading.Thread(target=http.serve_forever, daemon=True).start()
    server.STATION.start()
    host, url = f'127.0.0.1:{target.server_port}', f'http://127.0.0.1:{web.server_port}/'
    with tempfile.TemporaryDirectory() as directory:
        keys = KeyStore(Path(directory) / 'keys.json')
        diagnostics.start(directory)
        try:
            with patch('device_client.gateways', return_value=[host]), patch('device_client.KEYS', keys), \
                    patch('server.KEYS', keys), sync_playwright() as playwright:
                browser = playwright.chromium.launch(channel='msedge', headless=True)
                errors = []
                page = browser.new_page()
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto(url)
                page.wait_for_function('wifiControlReady()', timeout=8000)
                session = page.evaluate('controlSession')
                assert page.locator('#robot-host').input_value() == host
                assert page.locator('#robot-password').input_value() == ''
                assert [entry['action'] for entry in Target.controls] == ['acquire']
                page.locator('[data-go="network"]').first.click()
                assert page.locator('#robot-host').is_hidden()
                assert page.locator('#robot-password').is_hidden()
                assert not keys.path.exists(), 'The factory key need not be saved to disk'

                second = browser.new_page()
                second.goto(url)
                second.wait_for_function('wifiControlReady()')
                assert second.evaluate('controlSession') == session
                assert second.evaluate('controlClientId') != page.evaluate('controlClientId')
                assert [entry['action'] for entry in Target.controls] == ['acquire']
                awaitable = '''async () => {
                  await configureControl('mode', 2);
                  desiredAxes={surge:35,yaw:0,heave:0};inputVersion++;
                  await pumpTarget();desiredAxes={surge:0,yaw:0,heave:0};
                }'''
                page.evaluate(awaitable)
                assert Target.motion['remote_active']
                second.evaluate('stopControl()')
                assert Target.controls[-1]['action'] == 'stop'
                assert not Target.motion['remote_active']
                assert second.evaluate('wifiArmed')
                second.evaluate('configureControl("mode", 3)')
                page.evaluate('configureControl("mode", 2)')
                assert page.evaluate('wifiArmed')
                assert sum(entry['action'] == 'acquire' for entry in Target.controls) == 1
                assert all('client_id' not in entry for entry in Target.controls)
                second.close()

                # A real takeover must not trigger automatic reacquisition.
                before = len(Target.controls)
                Target.session, Target.sequence = 'another-operator', 0
                Target.motion['wifi']['session'] = 'another-operator'
                page.wait_for_function('!wifiArmed')
                page.wait_for_timeout(4200)
                assert len(Target.controls) == before
                page.close()

                # A newly opened page must also respect another idle controller.
                visitor = browser.new_page()
                visitor.goto(url)
                visitor.wait_for_function('autoConnectAttempted && !autoConnectPending')
                assert not visitor.evaluate('wifiArmed')
                assert Target.session == 'another-operator'
                assert len(Target.controls) == before
                visitor.locator('[data-go="network"]').first.click()
                visitor.locator('#robot-login').click()
                visitor.wait_for_function('wifiControlReady()')
                assert len(Target.controls) == before + 1
                assert Target.controls[-1]['action'] == 'acquire'
                visitor.close()

                # A changed password is entered once and then reused across page/service sessions.
                server.device_client.CLIENT.forget(host)
                Target.reset()
                Target.CONTROL_PASSWORD = 'tiny-key'
                custom = browser.new_page()
                custom.goto(url)
                custom.wait_for_function('autoConnectError && document.getElementById("connection-advanced").open')
                custom.locator('[data-go="network"]').first.click()
                custom.locator('#robot-password').fill('tiny-key')
                custom.locator('#robot-login').click()
                custom.wait_for_function('wifiControlReady()')
                assert keys.has(host)
                assert 'tiny-key' not in keys.path.read_text(encoding='utf-8')
                custom.close()
                server.device_client.CLIENT.forget(host)
                Target.reset()  # A restarted fish has no foreign control session.
                remembered = browser.new_page()
                remembered.goto(url)
                remembered.wait_for_function('wifiControlReady()')
                assert remembered.locator('#robot-password').input_value() == ''
                assert not remembered.evaluate('document.getElementById("connection-advanced").open')
                verify_delayed_logout(remembered, url, host, keys, automatic=False)
                remembered.close()
                Target.reset()
                Target.CONTROL_PASSWORD = robotfish.DEFAULT_CONTROL_PASSWORD
                delayed = browser.new_page()
                verify_delayed_logout(delayed, url, host, keys, automatic=True)
                delayed.close()
                assert not errors, errors
                browser.close()
            diagnostics.stop()
            log = (Path(directory) / 'workstation.jsonl').read_text(encoding='utf-8')
            assert robotfish.DEFAULT_CONTROL_PASSWORD not in log and 'tiny-key' not in log
            print(json.dumps({'checks': 'zero-input connection, independent page sequencing and STOP, idle-session ownership, explicit takeover, no takeover loop, one-time custom key, DPAPI reuse, delayed manual/automatic login cannot undo logout, quiet secret-free logs passed'}))
        finally:
            diagnostics.stop()
            for connection in server.device_client.CLIENT._telemetry_connections.values():
                connection.close()
            for http in (target, web):
                http.shutdown()
                http.server_close()


if __name__ == '__main__':
    main()
