"""Regression checks for the host logic fault-injection audit.

Run with the bundled Python and -B. Add --browser for the two-page Edge check.
No real camera, serial port, fish, saved credentials, or production config is used.
The non-browser checks also run in the normal regression suite.
"""
import contextlib
from http.server import ThreadingHTTPServer
import io
import json
import os
from pathlib import Path
import socket
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

UPPER = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(UPPER), str(UPPER / 'tests'), str(UPPER / 'tools'),
               str(UPPER.parent / 'vision')]

import cv2
import numpy as np
import device_client
from lora_link import LoRaLink
from station import Workstation
from video_recorder import AsyncVideoRecorder
from vision_service import VisionService
from web_runtime import WebBridge

BROWSER = '--browser' in sys.argv


class HostLogicAudit(unittest.TestCase):
    def test_stopped_visual_sessions_do_not_fill_all_slots(self):
        view = {'motion': {'mode': 3}, 'attitude_control': {
            'heading_target_supported': True, 'heading_tail_guidance_supported': True,
            'heading_allocation_profile': 'hybrid_gait_tail_bias',
            'heading_tail_amplitude_gain': 1.5, 'heading_bias_priority_supported': True}}
        client = SimpleNamespace(is_robotfish=lambda host: True,
            control_binding=lambda *args: 'fixture-session', robot=lambda host: view,
            motion_control=lambda *args, **kwargs: {'accepted': True})
        station = Workstation()
        station.host = '192.168.4.1'
        with patch('station.device_client.CLIENT', client):
            for index in range(9):
                session = f'vision-{index}'
                try:
                    station.prepare_vision(session, 1)
                    station.vision_command('stop', {}, session, 2)
                except device_client.DeviceError:
                    raise AssertionError(
                        f'Visual session {index + 1} rejected after stopped sessions accumulated') from None
            self.assertEqual(len(station.sequences),1)
            station.vision_command('heading_target',{'heading_cw_deg':0,'surge_pct':10},session,3)
            with self.assertRaises(device_client.DeviceError):
                station.vision_command('heading_target',{'heading_cw_deg':0,'surge_pct':10},'vision-0',3)

    def test_stop_rejects_inflight_and_renewed_old_manual_input(self):
        station=Workstation();station.host='fixture';calls=[]
        def command(*args,**kwargs):
            calls.append(args[1])
            return {'accepted':True}
        client=SimpleNamespace(is_robotfish=lambda host:True,motion_control=command)
        axes={'surge':35,'yaw':0,'heave':0}
        with patch('station.device_client.CLIENT',client):
            station.target(axes,'session',1,client_id='page-a',manual_epoch=0)
            result=station.configure_control('stop',None,'session',1,client_id='page-b')
            self.assertEqual(result['manual_epoch'],1)
            for sequence in (2,3):
                with self.assertRaises(device_client.DeviceError) as error:
                    station.target(axes,'session',sequence,client_id='page-a',manual_epoch=0)
                self.assertEqual(error.exception.code,'manual_input_changed')
            with self.assertRaises(device_client.DeviceError):
                station.configure_control('heading_target',{'angle_deg':0,'surge_pct':35},
                                          'session',4,client_id='page-a',manual_epoch=0)
            station.target(axes,'session',4,client_id='page-a',manual_epoch=1)
        self.assertEqual(calls,['target','stop','target'])

    def test_old_process_output_cannot_replace_current_port_or_recording(self):
        service = VisionService()
        service.process = SimpleNamespace(poll=lambda: None, pid=2)
        service.port = 22222
        service.recording = {'active': True, 'saved': False, 'file': 'current.mp4'}
        old = SimpleNamespace(stdout=[
            'ROBOFISH_VISION_READY ' + json.dumps({'port': 11111}),
            'ROBOFISH_VISION_RECORDING ' + json.dumps({'saved': True, 'file': 'old.mp4'})])
        service._reader(old)
        self.assertEqual((service.port, service.recording['file']), (22222, 'current.mp4'))

    def test_old_stop_cannot_disconnect_a_newly_started_process(self):
        entered, resume = threading.Event(), threading.Event()
        service = VisionService()

        class Process:
            def __init__(self, old):
                self.old, self.exited, self.stdout, self.pid = old, False, [], 1 if old else 2

            def poll(self):
                return 0 if self.exited else None

            def wait(self, timeout):
                self.exited = True
                entered.set()
                if not resume.wait(3):
                    raise RuntimeError('Audit synchronization timed out')
                return 0

        old, new = Process(True), Process(False)
        service.process = old
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(service, 'action', return_value={}), \
                patch.object(service, 'state', return_value={}), \
                patch.object(service, 'model_path', return_value=Path('fixture.pt')), \
                patch.object(service.ready, 'wait', return_value=True), \
                patch('vision_service.vision_python', return_value=Path(sys.executable)), \
                patch('vision_service.subprocess.Popen', return_value=new):
            service.output = Path(folder)
            stopper = threading.Thread(target=service.stop, daemon=True)
            stopper.start()
            try:
                self.assertTrue(entered.wait(2))
                service.start(0, 'fixture', 0)
                new.stdout=['ROBOFISH_VISION_READY ' + json.dumps({'port':22222})]
                service._reader(new)
            finally:
                resume.set()
                stopper.join(3)
            self.assertFalse(stopper.is_alive())
            self.assertIs(service.process, new)
            self.assertEqual(service.port, 22222, 'Finishing the old stop cleared the new port')

    def test_auto_stop_is_not_rejected_by_a_full_pointer_action_queue(self):
        with patch.dict(os.environ, {'ROBOFISH_PARENT_TOKEN': 'fixture',
                                    'ROBOFISH_VISION_TOKEN': 'fixture'}):
            bridge = WebBridge(observer=True)
        client = bridge.app.test_client()
        headers = {'Authorization': 'Bearer fixture'}
        for _ in range(128):
            response = client.post('/action', json={'kind': 'move', 'x': .5, 'y': .5},
                                   headers=headers)
            self.assertEqual(response.status_code, 202)
        response = client.post('/action', json={'task': 'AUTO_STOP'}, headers=headers)
        self.assertEqual(response.status_code, 202, 'A saturated action queue rejected STOP')
        self.assertEqual(bridge.events.get_nowait()['task'],'AUTO_STOP')
        self.assertTrue(bridge.events.empty())

    def test_exit_cancels_a_queued_start_and_cannot_be_replaced_by_stop(self):
        with patch.dict(os.environ,{'ROBOFISH_PARENT_TOKEN':'fixture','ROBOFISH_VISION_TOKEN':'fixture'}):
            bridge=WebBridge(observer=True)
        client=bridge.app.test_client();headers={'Authorization':'Bearer fixture'}
        client.post('/action',json={'task':'AUTO_START'},headers=headers)
        for index in range(127):
            client.post('/action',json={'kind':'move','x':.5,'y':.5},headers=headers)
        self.assertEqual(client.post('/action',json={'task':'EXIT'},headers=headers).status_code,202)
        self.assertEqual(client.post('/action',json={'task':'AUTO_STOP'},headers=headers).status_code,202)
        self.assertEqual(client.post('/action',json={'task':'AUTO_START'},headers=headers).status_code,409)
        self.assertEqual(bridge.events.get_nowait()['task'],'EXIT')
        self.assertTrue(bridge.events.empty())

    def test_delayed_lora_stop_is_not_expired_like_a_motion_target(self):
        with tempfile.TemporaryDirectory() as folder:
            now, writes = [10.], []
            link = LoRaLink(sequence_file=Path(folder) / 'sequence.json', clock=lambda: now[0])
            link.host = 'fixture'
            link.serial = SimpleNamespace(write=lambda data: writes.append(data) or len(data),
                                          close=lambda: None)
            link.submit('fixture', 'stop', {'mode': 2})
            now[0] += 1.1
            link.step()
            fields = writes[0][3:].decode('ascii').strip().split(',')
            self.assertEqual(fields[0], '101', 'STOP expired and only a heartbeat was sent')
            self.assertEqual(fields[4], '17')

    def test_recording_duration_matches_observation_timestamps_when_frames_drop(self):
        with tempfile.TemporaryDirectory() as folder, contextlib.redirect_stdout(io.StringIO()):
            path = Path(folder) / 'timing.mp4'
            recorder = AsyncVideoRecorder(path, cv2.VideoWriter_fourcc(*'mp4v'), 120,
                                          (64, 48), path.with_suffix('.csv'),
                                          fields=['camera_receipt_time'])
            try:
                for index in range(6):
                    self.assertTrue(recorder.submit(np.zeros((48, 64, 3), np.uint8),
                                                    [100. + index / 30.],timestamp=100.+index/30.))
                    expected=1+index*4
                    deadline = time.monotonic() + 2
                    while recorder.frames < expected and time.monotonic() < deadline:
                        time.sleep(.001)
                    self.assertEqual(recorder.frames, expected)
            finally:
                recorder.close()
            capture = cv2.VideoCapture(str(path))
            try:
                duration = capture.get(cv2.CAP_PROP_FRAME_COUNT) / capture.get(cv2.CAP_PROP_FPS)
            finally:
                capture.release()
            self.assertAlmostEqual(duration, 6 / 30., places=2,
                                   msg='30 Hz observations encoded at 120 Hz play four times too fast')
            self.assertEqual(recorder.repeated_frames,18)


@unittest.skipUnless(BROWSER, 'Add --browser to run the loopback-only Edge check')
class BrowserLogicAudit(unittest.TestCase):
    def test_stop_in_another_page_requires_a_new_press_before_motion_resumes(self):
        from playwright.sync_api import sync_playwright
        from credentials import KeyStore
        from verify_autoconnect import Target
        import server

        Target.reset()
        client = device_client.DeviceClient()
        target = ThreadingHTTPServer(('127.0.0.1', 0), Target)
        web = ThreadingHTTPServer(('127.0.0.1', 0), server.Handler)
        host = f'127.0.0.1:{target.server_port}'
        url = f'http://127.0.0.1:{web.server_port}/'
        for http in (target, web):
            threading.Thread(target=http.serve_forever, daemon=True).start()
        try:
            with tempfile.TemporaryDirectory() as folder, \
                    patch.object(server, 'PORT', web.server_port), \
                    patch.object(server, 'STATION', Workstation()), \
                    patch('device_client.CLIENT', client), \
                    patch('device_client.gateways', return_value=[host]), \
                    patch('device_client.capture_network_context', return_value=None), \
                    patch('device_client.KEYS', KeyStore(Path(folder) / 'keys.json')), \
                    sync_playwright() as playwright:
                browser = playwright.chromium.launch(channel='msedge', headless=True)
                first = browser.new_page()
                try:
                    first.goto(url)
                    first.wait_for_function('wifiControlReady()')
                    second = browser.new_page()
                    second.goto(url)
                    second.wait_for_function('wifiControlReady()')
                    first.bring_to_front()
                    first.evaluate('configureControl("mode", 2)')
                    first.evaluate('document.activeElement.blur()')
                    first.keyboard.down('w')
                    first.wait_for_function('robotState.control.axes.surge === 35')
                    before = len(Target.controls)
                    second.evaluate('stopControl()')
                    stop = next(index for index in range(before, len(Target.controls))
                                if Target.controls[index]['action'] == 'stop')
                    first.wait_for_timeout(600)
                    resumed = [item for item in Target.controls[stop + 1:]
                               if item['action'] == 'target' and
                               any(item.get('axes', {}).values())]
                    self.assertEqual(len(resumed), 0, 'The other page renewed its old held input after STOP')
                    first.keyboard.up('w')
                    first.wait_for_function('heldKeys.size === 0')
                    first.keyboard.down('w')
                    first.wait_for_function('robotState.control.axes.surge === 35')
                finally:
                    with contextlib.suppress(Exception):
                        first.keyboard.up('w')
                        first.evaluate('stopControl()')
                    browser.close()
                    for connection in list(client._telemetry_connections.values()):
                        connection.close()
        finally:
            for http in (web, target):
                http.shutdown()
                http.server_close()


def loopback_only(event, args):
    if event == 'socket.connect' and isinstance(args[1], tuple):
        if args[1][0] not in ('127.0.0.1', 'localhost', '::1'):
            raise RuntimeError('Audit blocked a non-loopback connection')


if __name__ == '__main__':
    sys.addaudithook(loopback_only)
    unittest.main(argv=[arg for arg in sys.argv if arg != '--browser'], verbosity=2)
