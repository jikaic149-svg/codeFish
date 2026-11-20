"""Exercise the data workspace with loopback telemetry and no motion commands."""
import copy
import csv
from http.server import ThreadingHTTPServer
import io
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
from test_robotfish import Target as ControlTarget, HEALTH
from test_realtime import RAW


class Target(ControlTarget):
    CONTROL_PASSWORD = robotfish.DEFAULT_CONTROL_PASSWORD
    roll = 179
    unavailable = False
    stale = False

    def do_GET(self):
        if self.path == '/get_ip':
            return self.reply(dict(HEALTH, lora_protocol=2))
        if self.path == '/motion_status':
            if self.unavailable:
                return self.reply({'error': 'fixture_offline'}, 503)
            raw = copy.deepcopy(self.motion)
            raw.update(lora_protocol=2, lora={'healthy': False}, preferred_control='wifi', imu=copy.deepcopy(RAW['imu']))
            raw['imu']['angles_deg'].update(x=self.roll, age_ms=700 if self.stale else 10)
            return self.reply(raw)
        return super().do_GET()


def main():
    Target.reset()
    target = ThreadingHTTPServer(('127.0.0.1', 0), Target)
    web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    server.PORT = web.server_port
    server.STATION.start()
    for http in (target, web):
        threading.Thread(target=http.serve_forever, daemon=True).start()
    host = f'127.0.0.1:{target.server_port}'
    with tempfile.TemporaryDirectory() as directory:
        keys = KeyStore(Path(directory) / 'keys.json')
        diagnostics.start(directory)
        try:
            with patch('device_client.gateways', return_value=[host]), patch('device_client.KEYS', keys), \
                    patch('server.KEYS', keys), patch('device_client.capture_network_context', return_value=None), \
                    sync_playwright() as playwright:
                browser = playwright.chromium.launch(channel='msedge', headless=True)
                page = browser.new_page(viewport={'width': 1440, 'height': 1000}, accept_downloads=True)
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto(f'http://127.0.0.1:{web.server_port}/#data')
                page.wait_for_function('wifiControlReady()')
                page.wait_for_function('liveRows.length >= 3')
                assert not page.evaluate('recorded.recording') and page.evaluate('recorded.count') == 0
                assert page.locator('#chart-group').input_value() == 'imu'
                assert page.locator('#chart-empty').is_hidden(), 'Live plot requires recording'
                assert page.locator('#raw-telemetry-panel').is_hidden(), 'F405 panel shown for single-S3 fish'
                assert page.locator('#chart-group option[value=battery]').is_disabled()
                assert page.locator('#data-live-roll').inner_text() == '179.00°'
                for format in ('csv','json'):
                    with page.expect_download() as download:
                        page.locator('#data-synchronous-'+format).click()
                    payload=Path(download.value.path()).read_bytes()
                    paired=list(csv.DictReader(io.StringIO(payload.decode('utf-8-sig')))) if format=='csv' else json.loads(payload)['samples']
                    assert paired and paired[0]['tail1_command_deg'] not in (None,'')
                    assert paired[0]['roll_deg'] not in (None,'')
                assert not page.locator('.capture-overview').is_visible()
                roll=page.locator('#chart-legend [data-series=roll_deg]')
                roll.click()
                assert roll.get_attribute('aria-pressed')=='false'
                roll.click()
                assert roll.get_attribute('aria-pressed')=='true'
                Target.roll = -179
                page.wait_for_function('liveRows.at(-1)?.roll_deg === -179')
                continuous = page.evaluate('plotSeries(chartRows(), "roll_deg").filter(Number.isFinite)')
                assert max(continuous) - min(continuous) < 3, continuous
                page.locator('#chart-freeze').click()
                frozen = page.evaluate('JSON.stringify(chartFrozen)')
                Target.roll = -170
                page.wait_for_function('liveRows.at(-1)?.roll_deg === -170')
                assert page.evaluate('JSON.stringify(chartFrozen)') == frozen
                assert page.locator('#data-live-roll').inner_text() == '-170.00°'
                page.locator('#chart-freeze').click()
                Target.stale = True
                page.wait_for_function('liveRows.at(-1)?.imu_valid === false')
                assert page.locator('#data-live-roll').inner_text() == '—'
                assert page.evaluate('liveRows.at(-1).roll_deg') is None
                Target.stale = False
                page.wait_for_function('liveRows.at(-1)?.imu_valid === true')
                continuous = page.evaluate('plotSeries(chartRows(), "roll_deg").filter(Number.isFinite)')
                assert max(continuous) - min(continuous) < 15, 'A missing sample reset the continuous-angle reference'
                page.locator('#capture-toggle').click()
                page.wait_for_function('recorded.recording && recorded.count >= 2')
                assert page.evaluate('Number.isFinite(recorded.rows.at(-1).latency_ms)')
                assert page.evaluate('recorded.rows.at(-1).battery_v') is None
                count = page.evaluate('recorded.count')
                page.locator('[data-page=overview]').click()
                page.wait_for_function('(count) => recorded.count > count', arg=count)
                page.locator('[data-page=data]').click()
                page.locator('#capture-toggle').click()
                page.wait_for_function('!recorded.recording && !capturePending')
                page.locator('#chart-source').select_option('recorded')
                assert page.locator('#chart-empty').is_hidden()
                assert 'Roll' in page.locator('#data-rows').locator('xpath=../..').inner_text()
                for format in ('csv', 'json'):
                    with page.expect_download() as download:
                        page.locator('#data-export-' + format).click()
                    payload = Path(download.value.path()).read_bytes()
                    samples = list(csv.DictReader(io.StringIO(payload.decode('utf-8-sig')))) if format == 'csv' else json.loads(payload)['samples']
                    assert len(samples) >= 2
                    assert all(float(row['roll_deg']) == -170 for row in samples)
                    assert all(row['latency_ms'] not in (None, '') for row in samples)
                with page.expect_download() as download:
                    page.locator('#chart-export').click()
                assert Path(download.value.path()).read_bytes().startswith(b'\x89PNG\r\n\x1a\n')
                page.locator('#chart-source').select_option('live')
                Target.unavailable = True
                page.wait_for_function('robotState === null && liveRows.at(-1)?.valid === false')
                assert page.locator('#data-live-roll').inner_text() == '—'
                assert page.evaluate('recorded.count') >= 2
                Target.unavailable = False
                page.wait_for_function('liveRows.at(-1)?.imu_valid === true')
                page.on('dialog', lambda dialog: dialog.accept())
                page.locator('#capture-clear').click()
                page.wait_for_function('recorded.count === 0 && !capturePending')
                assert page.locator('#chart-empty').is_hidden(), 'Clearing saved records broke live plot'
                assert page.locator('#data-export-csv').is_disabled()
                page.evaluate('''() => {
                  chartFrozen=[0,9000,10000].map((ms,i)=>({time:new Date(ms).toISOString(),roll_deg:i,pitch_deg:0,yaw_deg:0}));drawChart();
                }''')
                page.locator('#data-chart').scroll_into_view_if_needed()
                box=page.locator('#data-chart').bounding_box()
                page.mouse.move(box['x']+55+.9*(box['width']-77),box['y']+90)
                page.wait_for_function('chartHover === 1')
                page.evaluate('chartFrozen=null;chartHover=-1;drawChart()')
                page.locator('[data-page=data]').click()
                output = ROOT.parent / 'build/checks/host-data'
                output.mkdir(parents=True, exist_ok=True)
                page.screenshot(path=str(output / 'data-desktop.png'), full_page=True)
                page.set_viewport_size({'width': 390, 'height': 844})
                page.screenshot(path=str(output / 'data-mobile.png'), full_page=True)
                assert page.evaluate('document.documentElement.scrollWidth <= innerWidth + 1')
                assert not any(c['action'] in ('target', 'mode', 'attitude_on') for c in Target.controls)
                assert not errors, errors
                page.locator('#station-exit').click()
                page.wait_for_url('blob:**')
                assert page.locator('h1').inner_text() == '上位机已退出'
                browser.close()
            print(json.dumps({'checks': 'live preview before recording, +/-180 continuous plot, stale/offline gaps, freeze/resume, background capture, CSV/JSON/PNG, clear, mobile layout, no motion commands passed'}))
        finally:
            diagnostics.stop()
            for connection in server.device_client.CLIENT._telemetry_connections.values():
                connection.close()
            for http in (target, web):
                http.shutdown()
                http.server_close()


if __name__ == '__main__':
    main()
