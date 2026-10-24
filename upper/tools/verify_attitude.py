"""Real UI with loopback fixtures only; no Wi-Fi change or physical actuation."""
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import sys
import tempfile
import threading
from unittest.mock import patch

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'tests')]
import diagnostics
import robotfish
import server
from credentials import KeyStore
from test_robotfish import HEALTH
from verify_realtime import Target as TelemetryTarget


class Target(TelemetryTarget):
    CONTROL_PASSWORD = robotfish.DEFAULT_CONTROL_PASSWORD
    calibrated = False
    supported = True
    fault = ''
    heading = 30
    held_heading = None

    def do_GET(self):
        if self.path == '/get_ip':
            return self.reply(dict(HEALTH, version='2.6.1-yaw-hold', lora_protocol=2, attitude_control_supported=self.supported,
                                   yaw_pid_supported=self.supported,
                                   attitude_control_calibrated=self.calibrated))
        if self.path == '/motion_status':
            enabled = self.motion.get('attitude_control', {}).get('enabled', False)
            active = enabled and self.calibrated and self.motion['remote_target']['surge_pct'] >= 0 and not self.fault
            manual_yaw = self.motion['remote_target']['yaw_pct'] != 0
            yaw_active = active and not manual_yaw
            self.motion['remote_active'] = yaw_active or any(self.motion['remote_target'].get(key,0)
                for key in ('surge_pct','yaw_pct','heave_pct'))
            if not yaw_active:
                type(self).held_heading = None
            elif self.held_heading is None:
                type(self).held_heading = self.heading
            reason = self.fault or ('manual_yaw' if active and manual_yaw else 'active' if active else 'waiting_motion' if enabled else 'disabled')
            self.motion['attitude_control_supported'] = self.supported
            self.motion['attitude_control'] = dict(enabled=enabled, reason=reason,
                yaw_pid_supported=self.supported,pid=self.motion.get('yaw_pid',dict(kp=1,ki=.02,kd=.1)),pid_period_ms=100,
                pitch_calibrated=self.calibrated, roll_calibrated=self.calibrated, yaw_calibrated=self.calibrated,
                body_valid=self.calibrated and not self.fault, pitch_active=False, roll_active=False,
                age_ms=300 if self.fault == 'imu_stale' else 0, maximum_age_ms=200,
                body_roll_deg=12, body_pitch_deg=-8,
                body_yaw_deg=self.heading, heading_valid=self.calibrated and not self.fault,
                heading_locked=yaw_active, heading_target_deg=self.held_heading, manual_yaw=manual_yaw,
                yaw_error_deg=self.held_heading-self.heading if yaw_active else 0,
                yaw_active=yaw_active, yaw_correction_pct=max(-20, min(20, self.held_heading-self.heading)) if yaw_active else 0,
                pitch_correction_pct=8.2 if active else 0, roll_correction_pct=-12.4 if active else 0)
        return super().do_GET()


def main():
    Target.reset()
    Target.motion['remote_target']=dict(surge_pct=0,yaw_pct=0,heave_pct=0)
    target = ThreadingHTTPServer(('127.0.0.1', 0), Target)
    web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
    server.PORT = web.server_port
    for http in (target, web):
        threading.Thread(target=http.serve_forever, daemon=True).start()
    host = '127.0.0.1:' + str(target.server_port)
    with tempfile.TemporaryDirectory() as directory:
        keys = KeyStore(Path(directory) / 'keys.json')
        diagnostics.start(directory)
        try:
            with patch('device_client.gateways', return_value=[host]), patch('device_client.KEYS', keys), \
                    patch('server.KEYS', keys), patch('device_client.capture_network_context', return_value=None), \
                    sync_playwright() as playwright:
                browser = playwright.chromium.launch(channel='msedge', headless=True)
                page = browser.new_page()
                errors = []
                page.on('pageerror', lambda error: errors.append(str(error)))
                page.goto('http://127.0.0.1:' + str(web.server_port) + '/')
                page.wait_for_function('wifiControlReady()')
                page.wait_for_function('document.getElementById("robot-gate-note").textContent.includes("不因3秒没有变化而停止")')
                toggle = page.locator('#robot-attitude-enabled')
                toggle.evaluate('element => { for(let p=element.parentElement;p;p=p.parentElement)if(p.tagName==="DETAILS")p.open=true; }')
                assert toggle.is_disabled() and not toggle.is_checked()
                page.evaluate('configureControl("attitude", true)')
                assert page.evaluate('wifiControlReady()'), 'Calibration error removed manual authorization'
                assert not any(entry['action'].startswith('attitude_') for entry in Target.controls)
                Target.calibrated = True  # Synthetic confirmed profile, never applied to real hardware.
                page.evaluate('pollIdentity()')
                page.wait_for_function('!document.getElementById("robot-attitude-enabled").disabled')
                for key,value in [('kp','1.2'),('ki','0.025'),('kd','0.12')]:page.locator('#robot-yaw-'+key).fill(value)
                page.locator('#robot-yaw-apply').click()
                page.wait_for_function('robotState?.attitude_control?.pid?.kp === 1.2 && !controlConfigPending && yawPidSubmitted === null')
                assert not toggle.is_checked() and not Target.motion['remote_active']
                with page.expect_download() as download:page.locator('#robot-yaw-history').click()
                assert 'confirmed' in Path(download.value.path()).read_text(encoding='utf-8-sig')
                toggle.check()
                page.wait_for_function('robotState?.attitude_control?.enabled && !controlConfigPending')
                page.wait_for_function('robotState?.attitude_control?.yaw_active && robotState?.control?.axes?.surge === 0')
                assert 'yaw PID 生效' in page.locator('#robot-attitude-state').inner_text()
                Target.heading = 35
                page.wait_for_function('robotState?.attitude_control?.yaw_correction_pct === -5')
                page.evaluate('document.activeElement.blur()')
                page.keyboard.down('w')
                page.wait_for_function('robotState?.attitude_control?.yaw_active && robotState?.control?.mode === 3')
                assert not page.evaluate('robotState.attitude_control.pitch_active || robotState.attitude_control.roll_active')
                Target.heading = 35
                page.wait_for_function('robotState?.attitude_control?.yaw_correction_pct === -5')
                page.keyboard.down('d')
                page.wait_for_function('robotState?.control?.axes?.yaw === 40 && !robotState?.attitude_control?.yaw_active')
                assert not page.evaluate('robotState.attitude_control.pitch_active')
                Target.heading = 80
                page.keyboard.up('d')
                page.wait_for_function('robotState?.attitude_control?.heading_target_deg === 80')
                Target.fault = 'imu_unavailable'
                page.wait_for_function('robotState?.attitude_control?.reason === "imu_unavailable"')
                assert not page.evaluate('robotState.attitude_control.pitch_active')
                assert page.evaluate('wifiControlReady()'), 'Missing IMU incorrectly removed manual control'
                Target.fault = 'imu_stale'
                page.wait_for_function('robotState?.attitude_control?.reason === "imu_stale"')
                assert not page.evaluate('robotState.attitude_control.roll_active')
                Target.fault = ''
                Target.heading = 100
                page.wait_for_function('robotState?.attitude_control?.yaw_active')
                page.wait_for_function('robotState?.attitude_control?.heading_target_deg === 100')
                page.keyboard.up('w')
                page.wait_for_function('robotState?.attitude_control?.yaw_active && robotState?.control?.axes?.surge === 0 && robotState?.attitude_control?.heading_target_deg === 100')
                Target.heading = 105
                page.wait_for_function('robotState?.attitude_control?.yaw_correction_pct === -5')
                page.keyboard.down('s')
                page.wait_for_function('robotState?.control?.axes?.surge === -35')
                assert not page.evaluate('robotState.attitude_control.pitch_active || robotState.attitude_control.roll_active')
                page.keyboard.up('s')
                page.wait_for_function('robotState?.control?.axes?.surge === 0')
                page.keyboard.down('d')
                page.wait_for_function('robotState?.control?.axes?.yaw === 40')
                assert not page.evaluate('robotState.attitude_control.roll_active')
                page.keyboard.up('d')
                page.wait_for_function('robotState?.control?.axes?.yaw === 0')
                toggle.uncheck()
                page.wait_for_function('!robotState?.attitude_control?.enabled && !controlConfigPending')
                output=ROOT.parent/'build/analysis/yaw-pid-20261024';output.mkdir(parents=True,exist_ok=True)
                page.locator('#robot-attitude-settings').screenshot(path=str(output/'pid-ui.png'))
                page.set_viewport_size({'width':390,'height':844})
                assert page.evaluate('document.documentElement.scrollWidth<=innerWidth'),'PID form overflows mobile viewport'
                page.set_viewport_size({'width':1280,'height':900})
                assert [entry['action'] for entry in Target.controls if entry['action'].startswith('attitude_')] == ['attitude_on', 'attitude_off']
                assert sum(entry['action'] == 'acquire' for entry in Target.controls) == 1
                Target.supported = False
                page.evaluate('pollIdentity()')
                page.wait_for_function('document.getElementById("robot-attitude-enabled").disabled')
                assert not errors, errors
                page.close()
                Target.supported = True
                old_backend = browser.new_page()
                def legacy_metadata(route):
                    response = route.fetch()
                    value = response.json()
                    value.pop('features', None)
                    route.fulfill(response=response, json=value)
                for endpoint in ('state', 'session'):
                    old_backend.route('**/api/' + endpoint, legacy_metadata)
                old_backend.goto('http://127.0.0.1:' + str(web.server_port) + '/')
                old_backend.wait_for_function('wifiControlReady()')
                assert old_backend.locator('#robot-attitude-enabled').is_disabled()
                assert not old_backend.evaluate('serviceAttitudeControl')
                browser.close()
            diagnostics.stop()
            log = (Path(directory) / 'workstation.jsonl').read_text(encoding='utf-8')
            assert robotfish.DEFAULT_CONTROL_PASSWORD not in log
            print(json.dumps({'checks': 'PID editing/readback/history export, default off, legacy gates, hybrid yaw only, manual reanchor, missing/stale IMU, mobile layout and quiet logs passed'}))
        finally:
            diagnostics.stop()
            for connection in server.device_client.CLIENT._telemetry_connections.values():
                connection.close()
            for http in (target, web):
                http.shutdown()
                http.server_close()


if __name__ == '__main__':
    main()
