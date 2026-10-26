"""Real-browser check of the S3 console against a loopback-only mock device."""
import argparse
import base64
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import struct
import threading
import time

from playwright.sync_api import sync_playwright

MOTION = dict(mode=3, source='wifi', safety_state='WAIT_COMMAND', wifi_control_allowed=True,
    control_protocol=1, remote_active=False, ota_locked=False, output_fault=False,
    lora=dict(healthy=False, accepted=128, rejected=0),
    wifi=dict(session='', sequence=0, accepted=0, rejected=0),
    last_written=dict(tail_front_deg=91.5, tail_rear_deg=92.0, pectoral_left_deg=90,
        pectoral_right_deg=90, left_motor_pct=24, right_motor_pct=-18),
    imu=dict(uart_ready=True, baud=115200, frames=128, checksum_errors=0,
        angles_deg=dict(valid=True, x=3.2, y=-2.1, z=12.5, age_ms=20)),
    attitude_control_supported=True,
    attitude_control=dict(enabled=False, reason='disabled', yaw_pid_supported=True,
        yaw_calibrated=True, body_valid=True, heading_valid=True, age_ms=20,
        maximum_age_ms=200, body_roll_deg=3.2, body_pitch_deg=-2.1, body_yaw_deg=12.5,
        heading_locked=False, pid=dict(kp=2.5, ki=.02, kd=.1)))


def image(chip=9):
    data = bytearray(384)
    data[0] = 0xe9
    struct.pack_into('<H', data, 12, chip)
    struct.pack_into('<I', data, 32, 0xabcd5432)
    data[48:48 + len(b'test-console')] = b'test-console'
    data[80:89] = b'robotfish'
    return bytes(data)


class Device(BaseHTTPRequestHandler):
    motion = copy.deepcopy(MOTION)
    controls = []
    uploads = []
    started = time.monotonic() - 100
    html = b''
    version = 'test-console'

    def log_message(self, *_):
        pass

    def reply(self, data, status=200, mime='application/json'):
        payload = data if isinstance(data, bytes) else json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(payload)))
        self.end_headers()
        try:
            self.wfile.write(payload)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def authorized(self):
        expected = 'Basic ' + base64.b64encode(b'test-user:test-password').decode()
        if self.headers.get('Authorization') == expected:
            return True
        self.send_response(401)
        self.send_header('WWW-Authenticate', 'Basic realm="RobotFish"')
        self.send_header('Content-Length', '0')
        self.end_headers()
        return False

    def do_GET(self):
        if self.path == '/':
            if self.authorized():
                self.reply(self.html, mime='text/html; charset=utf-8')
        elif self.path == '/motion_status':
            self.reply(dict(self.motion, uptime_ms=int((time.monotonic() - self.started) * 1000)))
        elif self.path == '/get_ip':
            self.reply(dict(version=self.version, runtime_ready=True, ota_confirmed=True))
        elif self.path == '/favicon.ico':
            self.reply(b'', mime='image/x-icon')
        else:
            self.reply(dict(error='not_found'), 404)

    def do_POST(self):
        if not self.authorized():
            return
        data = self.rfile.read(int(self.headers.get('Content-Length', 0)))
        cls = type(self)
        if self.path == '/ota_upload':
            cls.uploads.append(data)
            cls.started = time.monotonic()
            self.reply(dict(success=True))
            return
        if self.path != '/api/v1/control':
            return self.reply(dict(error='not_found'), 404)
        c = json.loads(data)
        w = cls.motion['wifi']
        if c['action'] == 'acquire':
            w.update(session=c['session'], sequence=0)
        elif c['session'] != w['session']:
            return self.reply(dict(error='control_session_changed'), 409)
        elif c['sequence'] <= w['sequence']:
            return self.reply(dict(error='stale_sequence'), 409)
        elif c['action'] != 'stop' and cls.motion['lora']['healthy']:
            return self.reply(dict(error='lora_primary_active'), 409)
        else:
            w['sequence'] = c['sequence']
        if c['action'] == 'yaw_pid':
            cls.motion['attitude_control']['pid'] = c['pid']
        elif c['action'] in ('attitude_on', 'attitude_off'):
            cls.motion['attitude_control']['enabled'] = c['action'] == 'attitude_on'
        elif c['action'] == 'stop':
            cls.motion['attitude_control']['enabled'] = False
        elif c['action'] == 'mode':
            cls.motion['mode'] = c['mode']
        w['accepted'] += 1
        cls.controls.append(c)
        self.reply(dict(accepted=True, session=w['session'], sequence=w['sequence']))


def wait_control(page, action, axes=None):
    for _ in range(60):
        if Device.controls and Device.controls[-1]['action'] == action:
            if axes is None or Device.controls[-1].get('axes') == axes:
                return
        page.wait_for_timeout(50)
    raise AssertionError((action, axes, Device.controls[-4:]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--html', type=Path, default=Path(__file__).resolve().parents[2] / 'main/control.html')
    parser.add_argument('--out', type=Path, default=Path(__file__).resolve().parents[2] / 'build-host/s3-console')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    Device.html = args.html.read_bytes()
    http = ThreadingHTTPServer(('127.0.0.1', 0), Device)
    threading.Thread(target=http.serve_forever, daemon=True).start()
    base = f'http://127.0.0.1:{http.server_port}'
    result = dict(viewports=[], errors=[], controls=0)
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(channel='msedge', headless=True)
            context = browser.new_context(viewport=dict(width=1440, height=1100),
                http_credentials=dict(username='test-user', password='test-password'))
            page = context.new_page()
            page.on('pageerror', lambda error: result['errors'].append(str(error)))
            page.goto(base)
            page.wait_for_function("document.querySelector('#connection').textContent==='Wi-Fi 已连接'")
            assert not Device.controls, 'Loading the console must not acquire control'
            assert page.locator('[data-axis]').first.is_disabled()
            page.locator('#connect').click()
            page.wait_for_function("!document.querySelector('[data-axis]').disabled")
            page.locator('#connect').blur()
            page.keyboard.down('w')
            wait_control(page, 'target', dict(surge=35, yaw=0, heave=0))
            page.keyboard.down('d')
            wait_control(page, 'target', dict(surge=35, yaw=40, heave=0))
            page.keyboard.up('w')
            page.keyboard.up('d')
            wait_control(page, 'target', dict(surge=0, yaw=0, heave=0))
            button = page.locator('[data-axis=forward]')
            button.hover()
            page.mouse.down()
            wait_control(page, 'target', dict(surge=35, yaw=0, heave=0))
            page.evaluate("window.dispatchEvent(new Event('blur'))")
            page.mouse.up()
            wait_control(page, 'stop')
            page.locator('#tab-tuning').click()
            page.locator('#pid-kp').fill('3.125')
            page.wait_for_timeout(650)
            assert page.locator('#pid-kp').input_value() == '3.125'
            page.locator('#pid-apply').click()
            page.wait_for_function("document.querySelector('#pid-result').textContent.includes('回读确认')")
            assert Device.motion['attitude_control']['pid']['kp'] == 3.125
            before = len(Device.controls)
            page.locator('#pid-kp').fill('101')
            page.locator('#pid-apply').click()
            page.wait_for_timeout(150)
            assert len(Device.controls) == before, 'Invalid PID must not reach the device'
            page.locator('#pid-read').click()
            page.locator('#attitude-toggle').click()
            page.wait_for_function("document.querySelector('#attitude-toggle').textContent==='关闭辅助'")
            page.locator('#global-stop').click()
            wait_control(page, 'stop')
            Device.motion['lora']['healthy'] = True
            page.wait_for_function("document.querySelector('#control-hint').textContent.includes('LoRa 正在主控')")
            assert page.locator('#pid-apply').is_disabled()
            assert page.locator('#global-stop').is_enabled()
            Device.motion['lora']['healthy'] = False
            page.locator('#tab-control').click()
            page.wait_for_function("!document.querySelector('[data-axis]').disabled")
            # Arrow navigation in the tablist must not also drive the fish.
            page.locator('#tab-tuning').focus()
            before = len(Device.controls)
            page.keyboard.press('ArrowLeft')
            page.wait_for_timeout(200)
            assert len(Device.controls) == before
            saved = page.locator('#outputs').inner_text()
            page.route('**/motion_status', lambda route: route.fulfill(status=503, json=dict(error='offline')))
            page.wait_for_function("document.querySelector('#connection').textContent.includes('状态读取失败')")
            assert page.locator('#outputs').inner_text() == saved
            assert page.locator('[data-axis]').first.is_disabled()
            page.unroute('**/motion_status')
            page.wait_for_function("document.querySelector('#connection').textContent==='Wi-Fi 已连接'")
            for width in (1440, 1024, 768, 720, 390, 375, 320):
                page.set_viewport_size(dict(width=width, height=1050))
                for tab in ('control', 'tuning', 'update'):
                    page.locator('#tab-' + tab).click()
                    page.wait_for_timeout(30)
                    assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'), (width, tab)
                result['viewports'].append(width)
            page.set_viewport_size(dict(width=1440, height=1100))
            page.locator('#tab-control').click()
            page.screenshot(path=str(args.out / 'desktop.png'), full_page=True)
            page.locator('#tab-update').click()
            page.locator('#firmware').set_input_files(dict(name='wrong-chip.bin', mimeType='application/octet-stream', buffer=image(0)))
            page.wait_for_function("document.querySelector('#ota-result').textContent.includes('不是 RobotFish')")
            assert page.locator('#upload').is_disabled()
            page.locator('#firmware').set_input_files(dict(name='robotfish.bin', mimeType='application/octet-stream', buffer=image()))
            page.wait_for_function("!document.querySelector('#upload').disabled")
            page.locator('#upload').click()
            page.wait_for_function("document.querySelector('#ota-result').textContent.includes('重新上线')")
            assert Device.uploads == [image()]
            assert page.locator('[data-axis]').first.is_disabled()
            page.screenshot(path=str(args.out / 'upgrade.png'), full_page=True)
            context.close()
            # Native touch events verify pointer capture and two-finger combinations.
            mobile = browser.new_context(viewport=dict(width=390, height=844), is_mobile=True, has_touch=True,
                http_credentials=dict(username='test-user', password='test-password'))
            page = mobile.new_page()
            page.on('pageerror', lambda error: result['errors'].append(str(error)))
            page.goto(base)
            page.wait_for_function("!document.querySelector('#connect').disabled")
            page.locator('#connect').tap()
            page.wait_for_function("!document.querySelector('[data-axis]').disabled")
            cdp = mobile.new_cdp_session(page)
            points = []
            for index, axis in enumerate(('forward', 'up')):
                box = page.locator(f'[data-axis={axis}]').bounding_box()
                points.append(dict(x=box['x'] + box['width'] / 2, y=box['y'] + box['height'] / 2, id=index + 1))
            cdp.send('Input.dispatchTouchEvent', dict(type='touchStart', touchPoints=points))
            wait_control(page, 'target', dict(surge=35, yaw=0, heave=50))
            cdp.send('Input.dispatchTouchEvent', dict(type='touchEnd', touchPoints=[]))
            wait_control(page, 'target', dict(surge=0, yaw=0, heave=0))
            page.screenshot(path=str(args.out / 'mobile.png'), full_page=True)
            mobile.close()
            browser.close()
        assert not result['errors'], result['errors']
        result['controls'] = len(Device.controls)
        (args.out / 'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print('PASS: authenticated S3 console; keyboard/mouse/native multi-touch; PID limits/readback; LoRa priority; stale data; OTA/reboot; 320–1440 px; zero browser errors.')
    finally:
        http.shutdown()
        http.server_close()


if __name__ == '__main__':
    main()
