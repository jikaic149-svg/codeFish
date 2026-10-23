"""Optional UI check with installed Playwright; only loopback test targets."""
import importlib.util
from http.server import ThreadingHTTPServer
from pathlib import Path
import sys
import threading

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import server

spec = importlib.util.spec_from_file_location('fixture', ROOT / 'tests/test_robotfish.py')
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)


def verify_device_page(browser, target):
    fixture.Target.reset()
    page = browser.new_page(viewport={'width': 390, 'height': 844},
        extra_http_headers={'X-OTA-Password': 'test-user-input'})
    errors = []
    page.on('pageerror', lambda error: errors.append(str(error)))
    page.goto(f'http://127.0.0.1:{target.server_port}/')
    page.wait_for_function("document.getElementById('outputs').textContent.includes('91.0')")
    assert page.locator('[data-axis], #connect, #mode').count() == 0
    assert '115200' not in page.locator('#imu').inner_text()  # No fabricated sensor metadata.
    fixture.Target.motion['imu'] = dict(uart_ready=True, baud=115200, frames=1,
        angles_deg=dict(valid=True, x=45, y=-45, z=90, age_ms=20))
    page.wait_for_function("document.getElementById('imu').textContent.includes('45.0, -45.0, 90.0')")
    assert '115200' in page.locator('#imu').inner_text()
    saved = page.locator('#outputs').inner_text()
    page.route('**/motion_status', lambda route: route.abort())
    page.wait_for_function("document.getElementById('connection').textContent.includes('状态读取失败')")
    assert page.locator('#outputs').inner_text() == saved
    page.unroute('**/motion_status')
    page.wait_for_function("document.getElementById('connection').textContent.includes('Wi-Fi 已连接')")
    page.locator('details').evaluate('(element) => element.open = true')
    page.locator('#firmware').set_input_files({
        'name': 'robotfish.bin', 'mimeType': 'application/octet-stream', 'buffer': fixture.image()})
    page.locator('#upload').click()
    page.wait_for_function("document.getElementById('ota-result').textContent.includes('固件已提交')")
    assert fixture.Target.uploads == [fixture.image()]
    assert not fixture.Target.controls
    assert '/api/v1/control' not in fixture.Target.paths
    assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth + 1')
    assert not errors, errors
    page.screenshot(path=str(ROOT.parent / 'build-host' / 'device-wifi-verified.png'), full_page=True)
    page.close()
    print('Device browser OK: actual telemetry HTML; valid/absent IMU; outage and recovery; simulated OTA; no motion requests; 390px.')


def main():
    fixture.Target.reset()
    server.device_client.CLIENT.discover_robotfish = lambda candidates=(): None
    target = ThreadingHTTPServer(('127.0.0.1', 0), fixture.Target)
    web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    server.PORT = web.server_port
    for service in (target, web):
        threading.Thread(target=service.serve_forever, daemon=True).start()
    server.STATION.start()
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(channel='msedge', headless=True)
            page = browser.new_page(viewport={'width': 1280, 'height': 900})
            errors = []
            page.on('pageerror', lambda error: errors.append(str(error)))
            page.goto(f'http://127.0.0.1:{web.server_port}/')
            page.locator('[data-go="network"]').first.click()
            page.evaluate('document.getElementById("connection-advanced").open=true')
            page.locator('#robot-host').fill(f'127.0.0.1:{target.server_port}')
            page.wait_for_function("document.body.dataset.device === 'robotfish'")
            page.locator('[data-page="overview"]').click()
            page.wait_for_function("document.getElementById('robot-tail1').textContent.includes('91')")
            assert page.locator('#robot-tail1').inner_text() == '91°'
            assert page.locator('#robot-left-esc').inner_text() == '40%'
            assert page.locator('[data-axis="forward"]').is_disabled()
            page.locator('[data-go="network"]').first.click()
            page.evaluate("document.getElementById('device-session').open=true")
            page.locator('#robot-password').fill('test-user-input')
            page.locator('#robot-remember').uncheck()
            page.locator('#robot-login').click()
            page.wait_for_function("document.getElementById('robot-login-state').textContent.includes('控制会话已连接')")
            assert fixture.Target.controls[0]['action'] == 'acquire'
            page.locator('[data-page="overview"]').click()
            page.wait_for_function("!document.querySelector('[data-axis=forward]').disabled")
            assert page.locator('#control-source').input_value() == 'wifi'
            page.locator('#robot-control-mode').select_option('2')
            page.wait_for_function("document.getElementById('control-configuration-state').textContent.includes('已应用')")
            assert fixture.Target.motion['mode'] == 2
            page.evaluate("document.activeElement.blur()")
            begin = len(fixture.Target.controls)
            page.keyboard.down('f')
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('-100')")
            page.wait_for_timeout(3300)
            held = [entry for entry in fixture.Target.controls[begin:] if entry['action'] == 'target']
            assert len(held) >= 8, held
            assert all(entry['axes'] == {'surge': 0, 'yaw': 0, 'heave': -100} for entry in held), held
            assert held[-1]['received_at'] - held[0]['received_at'] >= 3, held
            assert fixture.Target.motion['remote_active'] and not fixture.Target.motion['failsafe_active']
            page.keyboard.up('f')
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('浮沉 0')")
            assert fixture.Target.controls[-1]['axes'] == {'surge': 0, 'yaw': 0, 'heave': 0}
            page.keyboard.down('w')
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('前后 35')")
            page.evaluate("window.dispatchEvent(new Event('blur'))")
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('前后 0')")
            page.keyboard.up('w')
            page.keyboard.down('w')
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('前后 35')")
            page.keyboard.press('Space')
            page.wait_for_function("document.getElementById('robot-command-result').textContent.includes('已全停')")
            page.keyboard.up('w')
            assert fixture.Target.controls[-1]['action'] == 'stop'
            pointer = page.locator('[data-axis="front-left"]')
            pointer.hover()
            page.mouse.down()
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('前后 35') && document.getElementById('control-axis-value').textContent.includes('转向 -40')")
            page.mouse.up()
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('前后 0')")
            page.evaluate("document.activeElement.blur()")
            page.keyboard.down('w')
            page.wait_for_function("document.getElementById('control-axis-value').textContent.includes('前后 35')")
            fixture.Target.deny_control = True
            page.wait_for_function("document.querySelector('[data-axis=forward]').disabled")
            stopped_at = len(fixture.Target.controls)
            page.wait_for_timeout(500)
            assert len(fixture.Target.controls) == stopped_at
            page.keyboard.up('w')
            fixture.Target.deny_control = False
            accepted = [entry.get('sequence', 0) for entry in fixture.Target.controls if entry['action'] != 'acquire']
            assert accepted == sorted(set(accepted)), accepted
            page.locator('[data-page="debug"]').click()
            assert page.locator('#robotfish-maintenance').is_visible()
            assert not page.locator('#dev-connect').is_visible()
            page.locator('#release-file').set_input_files({
                'name': 'robotfish.bin', 'mimeType': 'application/octet-stream', 'buffer': fixture.image()})
            page.wait_for_function("!document.getElementById('upgrade-probe').disabled")
            page.locator('#release-password').fill('test-user-input')
            page.on('dialog', lambda dialog: dialog.accept())
            page.locator('#upgrade-probe').click()
            page.wait_for_function("document.getElementById('release-status').textContent.includes('升级完成')", timeout=15000)
            assert fixture.Target.uploads == [fixture.image()]
            assert not any('/stream' in path or '/capture' in path for path in fixture.Target.paths)
            assert not errors, errors
            output = ROOT.parent / 'build-host'
            output.mkdir(exist_ok=True)
            page.screenshot(path=str(output / 'upper-ota-verified.png'), full_page=True)
            page.set_viewport_size({'width': 390, 'height': 844})
            assert page.locator('#upgrade-probe').is_visible()
            assert page.evaluate('document.documentElement.scrollWidth <= window.innerWidth + 1')
            print('Browser OK: authenticated Wi-Fi acquire; >3s down-axis renewal; keyup/blur/full-stop; pointer axes; conflict locks until reconnect; six channels; OTA confirmed new slot; 390px layout.')
            page.close()
            verify_device_page(browser, target)
            browser.close()
    finally:
        for service in (target, web):
            service.shutdown()
            service.server_close()


if __name__ == '__main__':
    main()
