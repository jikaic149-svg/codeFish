"""Offline HTTP/OTA regression. No hardware is contacted or flashed."""
import hashlib
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import struct
import sys
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import robotfish
import probe_upgrade


def image(project='robotfish', chip=9):
    header = bytearray(24)
    header[0:2] = bytes((0xe9, 1))
    header[23] = 1
    struct.pack_into('<H', header, 12, chip)
    desc = bytearray(256)
    struct.pack_into('<I', desc, 0, 0xabcd5432)
    desc[16:16+len(b'2.0.0-idf')] = b'2.0.0-idf'
    desc[48:48+len(project)] = project.encode()
    data = header + struct.pack('<II', 0x3c000020, len(desc)) + desc
    checksum = 0xef
    for byte in desc:
        checksum ^= byte
    data += bytes(15 - len(data) % 16) + bytes((checksum,))
    return bytes(data) + hashlib.sha256(data).digest()


HEALTH = dict(firmware='robotfish', hardware='robotfish-esp32s3', project_name='robotfish',
              version='2.0.0-idf', running_slot='app0', ota_confirmed=True, runtime_ready=True,
              sta_ip='192.168.4.2', sta_ssid='lab', ap_ip='192.168.4.1',
              wifi_control_allowed=True, control_protocol=1)
MOTION = dict(version=1, source='wifi', last_source='wifi', attempts=5, failures=0,
              mode=3, action=1, failsafe_active=False, lease_remaining_ms=2500,
              wifi_control_allowed=True, control_protocol=1, remote_active=False,
              remote_target=dict(surge_pct=40, yaw_pct=10, heave_pct=0),
              last_written=dict(tail_front_deg=91, tail_rear_deg=92,
                                pectoral_left_deg=89, pectoral_right_deg=88,
                                left_motor_pct=40, right_motor_pct=35),
              wifi=dict(accepted=0, rejected=0, session='', sequence=0))


class Target(BaseHTTPRequestHandler):
    CONTROL_PASSWORD = 'test-user-input'
    uploads = []
    paths = []
    controls = []
    session = ''
    sequence = 0
    last_motion = 0
    deny_control = False
    motion = copy.deepcopy(MOTION)

    @classmethod
    def reset(cls):
        cls.uploads = []
        cls.paths = []
        cls.controls = []
        cls.session = ''
        cls.sequence = 0
        cls.last_motion = 0
        cls.deny_control = False
        cls.motion = copy.deepcopy(MOTION)

    def log_message(self, *_):
        pass

    def reply(self, data, status=200):
        payload = json.dumps(data).encode()
        self.send_response(status)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self):
        type(self).paths.append(self.path)
        if self.path == '/get_ip':
            return self.reply(dict(HEALTH, running_slot='app1' if self.uploads else 'app0'))
        if self.path == '/motion_status':
            owner = type(self)
            if owner.motion['remote_active'] and time.monotonic() - owner.last_motion >= 3:
                owner.motion.update(remote_active=False, failsafe_active=True, lease_remaining_ms=0)
                owner.motion['remote_target'] = dict(surge_pct=0, yaw_pct=0, heave_pct=0)
            return self.reply(owner.motion)
        if self.path in ('/', '/update'):
            payload = (Path(__file__).resolve().parents[2] / 'main' / 'control.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(payload)
            return
        self.reply({'error': 'not found'}, 404)

    def do_POST(self):
        type(self).paths.append(self.path)
        data = self.rfile.read(int(self.headers['Content-Length']))
        if self.path == '/api/v1/control':
            owner = type(self)
            if self.headers.get('X-OTA-Password') != owner.CONTROL_PASSWORD:
                return self.reply({'error': 'Control password required'}, 401)
            command = json.loads(data)
            if owner.deny_control:
                return self.reply({'error': 'Control session expired'}, 409)
            action, session = command.get('action'), command.get('session')
            sequence = command.get('sequence', 0)
            if action == 'acquire':
                owner.session, owner.sequence = session, 0
                owner.motion.update(remote_active=False, failsafe_active=False)
                owner.motion['remote_target'] = dict(surge_pct=0, yaw_pct=0, heave_pct=0)
            else:
                if session != owner.session or sequence <= owner.sequence:
                    return self.reply({'error': 'Stale control session or sequence'}, 409)
                owner.sequence = sequence
                if action == 'target':
                    axes = command['axes']
                    active = any(axes.values())
                    owner.motion.update(remote_active=active, failsafe_active=False, lease_remaining_ms=3000 if active else 0)
                    owner.motion['remote_target'] = dict(surge_pct=axes['surge'], yaw_pct=axes['yaw'], heave_pct=axes['heave'])
                    owner.last_motion = time.monotonic()
                elif action == 'mode':
                    owner.motion['mode'] = command['mode']
                elif action in ('attitude_on', 'attitude_off'):
                    owner.motion.setdefault('attitude_control', {})['enabled'] = action == 'attitude_on'
                elif action == 'stop':
                    owner.motion.update(remote_active=False, failsafe_active=False, lease_remaining_ms=0)
                    owner.motion['remote_target'] = dict(surge_pct=0, yaw_pct=0, heave_pct=0)
                else:
                    return self.reply({'error': 'Invalid control action'}, 400)
            owner.controls.append(dict(command, received_at=time.monotonic()))
            owner.motion['wifi'].update(session=owner.session, sequence=owner.sequence, accepted=len(owner.controls))
            axes = owner.motion['remote_target']
            return self.reply(dict(accepted=True, session=owner.session, sequence=owner.sequence,
                mode=owner.motion['mode'], axes=dict(surge=axes['surge_pct'], yaw=axes['yaw_pct'], heave=axes['heave_pct']),
                lease_active=owner.motion['remote_active']))
        if self.path != '/ota_upload' or self.headers.get('X-OTA-Password') != 'test-user-input':
            return self.reply({'error': 'Invalid OTA password'}, 401)
        type(self).uploads.append(data)
        self.reply({'success': True})


class RobotFishTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.http = ThreadingHTTPServer(('127.0.0.1', 0), Target)
        cls.thread = threading.Thread(target=cls.http.serve_forever, daemon=True)
        cls.thread.start()
        cls.host = '127.0.0.1:' + str(cls.http.server_port)

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()
        cls.http.server_close()
        cls.thread.join()

    def setUp(self):
        Target.reset()

    def test_correct_image_and_rejection_before_upload(self):
        self.assertEqual(robotfish.inspect_image(image())['project_name'], 'robotfish')
        for bad in (b'', image(chip=0), image(project='embedding'),
                    image()[:-1], bytes([image()[0] ^ 1]) + image()[1:],
                    image()[:100] + bytes([image()[100] ^ 1]) + image()[101:],
                    bytes(robotfish.MAX_IMAGE_BYTES + 1)):
            with self.subTest(size=len(bad)), self.assertRaises(ValueError):
                robotfish.inspect_image(bad)
        self.assertFalse(Target.uploads)

    def test_cookie_free_six_channels_and_honest_capabilities(self):
        client = device_client.DeviceClient()
        identity = client.identity(self.host)
        self.assertEqual(identity['firmware'], 'robotfish')
        self.assertFalse(identity['camera'])
        self.assertFalse(identity['dap_client'])
        state = client.robot(self.host)
        self.assertEqual(state['actuators']['left_esc'], 40)
        self.assertEqual(state['actuators']['tail2'], 92)
        self.assertEqual(len([x for x in state['actuators'].values() if x is not None]), 6)
        self.assertFalse(state['battery']['valid'])
        self.assertTrue(state['wifi_control_allowed'])
        self.assertEqual(state['camera_stream_url'], '')
        self.assertIsNone(state['link']['round_trip_us'])
        with self.assertRaises(device_client.DeviceError):
            client.command(self.host, {'action': 1})
        self.assertEqual(Target.paths, ['/get_ip', '/motion_status'])

    def connected(self, session='test-session'):
        client = device_client.DeviceClient()
        client.identity(self.host)
        result = client.login(self.host, 'test-user-input', session=session)
        self.assertTrue(result['accepted'])
        self.assertEqual(result['session'], session)
        return client

    def test_wifi_acquire_target_mode_stop_and_monotonic_sequence(self):
        client = self.connected()
        result = client.wifi_control(self.host, 'target', 'test-session', 1,
            axes=dict(surge=35, yaw=-40, heave=-100))
        self.assertFalse(result['simulated'])
        self.assertEqual(client.robot(self.host)['control']['axes'], dict(surge=35, yaw=-40, heave=-100))
        self.assertTrue(client.robot(self.host)['control']['connected'])
        client.wifi_control(self.host, 'mode', 'test-session', 2, mode=2)
        self.assertEqual(client.robot(self.host)['control']['mode'], 2)
        client.wifi_control(self.host, 'stop', 'test-session', 3)
        self.assertEqual(client.robot(self.host)['control']['axes'], dict(surge=0, yaw=0, heave=0))
        self.assertEqual([x['action'] for x in Target.controls], ['acquire', 'target', 'mode', 'stop'])
        self.assertEqual([x.get('sequence', 0) for x in Target.controls], [0, 1, 2, 3])

    def test_wrong_control_password_and_disconnected_command_do_not_acquire(self):
        client = device_client.DeviceClient()
        client.identity(self.host)
        with self.assertRaises(device_client.DeviceError):
            client.wifi_control(self.host, 'target', 'unknown', 1, axes=dict(surge=50, yaw=0, heave=0))
        with self.assertRaisesRegex(device_client.DeviceError, '密码'):
            client.login(self.host, 'wrong-user-input', session='session')
        self.assertFalse(Target.controls)

    def test_conflict_latches_disconnected_without_retry_or_reacquire(self):
        client = self.connected()
        client.wifi_control(self.host, 'target', 'test-session', 1, axes=dict(surge=35, yaw=0, heave=0))
        before = len(Target.paths)
        Target.deny_control = True
        with self.assertRaises(device_client.DeviceError):
            client.wifi_control(self.host, 'target', 'test-session', 2, axes=dict(surge=35, yaw=0, heave=0))
        self.assertEqual(Target.paths[before:], ['/api/v1/control'])
        with self.assertRaises(device_client.DeviceError):
            client.wifi_control(self.host, 'target', 'test-session', 3, axes=dict(surge=35, yaw=0, heave=0))
        self.assertEqual(Target.paths[before:], ['/api/v1/control'])
        self.assertEqual([x['action'] for x in Target.controls], ['acquire', 'target'])

    def test_controller_takeover_invalidates_existing_session(self):
        client = self.connected()
        self.connected('second-session')
        with self.assertRaises(device_client.DeviceError):
            client.wifi_control(self.host, 'target', 'test-session', 1, axes=dict(surge=35, yaw=0, heave=0))
        self.assertFalse(client.robot(self.host)['control']['connected'])

    def test_local_sequence_and_station_axis_validation_never_contact_device(self):
        import station
        client = self.connected()
        before = len(Target.paths)
        for sequence in (0, -1, True, 1.5, 0x100000000):
            with self.subTest(sequence=sequence), self.assertRaises(device_client.DeviceError):
                client.wifi_control(self.host, 'target', 'test-session', sequence, axes=dict(surge=35, yaw=0, heave=0))
        workstation = station.Workstation()
        workstation.host = self.host
        for axes in ({}, dict(surge=0, yaw=0), dict(surge=101, yaw=0, heave=0),
                     dict(surge=True, yaw=0, heave=0), dict(surge=float('nan'), yaw=0, heave=0),
                     dict(surge=float('inf'), yaw=0, heave=0)):
            with self.subTest(axes=axes), self.assertRaises(device_client.DeviceError):
                workstation.target(axes, 'test-session', 1, 'wifi')
        self.assertEqual(len(Target.paths), before)

    def test_lease_expiry_remains_driver_state_not_feedback(self):
        state = robotfish.normalize(dict(MOTION, failsafe_active=True), self.host)
        self.assertEqual(state['safety_mode'], 'LEASE_EXPIRED')
        self.assertEqual(state['actuators']['tail1'], 91)

    def test_same_version_reflash_requires_slot_change_and_confirmation(self):
        with patch.object(robotfish.time, 'sleep'):
            result = robotfish.install(self.host, 'test-user-input', image(), lambda *_: None, lambda *_: None)
        self.assertTrue(result['complete'])
        self.assertEqual(result['slot'], 'app1')
        self.assertEqual(Target.uploads, [image()])
        self.assertNotIn('/control', Target.paths)

    def test_upload_ack_without_confirmed_new_slot_is_failure(self):
        for new in (HEALTH, dict(HEALTH, running_slot='app1', ota_confirmed=False)):
            # Return confirmed old slot before upload, then an unconfirmed candidate.
            replies = [HEALTH, {'success': True}] + [new] * 40
            with patch.object(robotfish, 'request', side_effect=replies), patch.object(robotfish.time, 'sleep'):
                with self.assertRaises(RuntimeError):
                    robotfish.install(self.host, 'test-user-input', image(), lambda *_: None, lambda *_: None)

    def test_wrong_ota_password_has_clear_error_and_does_not_upload(self):
        with self.assertRaisesRegex(ValueError, 'OTA'):
            robotfish.install(self.host, 'wrong-user-input', image(), lambda *_: None, lambda *_: None)
        self.assertFalse(Target.uploads)

    def test_bin_package_route_does_not_import_legacy_tools(self):
        import base64
        result = probe_upgrade.inspect({'name': 'robotfish.bin', 'data': base64.b64encode(image()).decode()})
        self.assertEqual(result['manifest']['format'], 'bin')
        with self.assertRaises(ValueError):
            probe_upgrade.install({'ticket': result['ticket'], 'transport': 'usb'}, lambda *_: None)


if __name__ == '__main__':
    unittest.main()
