"""Offline Wi-Fi connection checks. No sockets, flashing, or motion commands."""
import argparse
import copy
import json
from pathlib import Path
import re
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import sys


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--upper', type=Path)
options, remaining = parser.parse_known_args()
project = options.project.resolve()
sys.path.insert(0, str((options.upper or project / 'upper').resolve()))
import device_client

app_config = (project / 'main/config/AppConfig.h').read_text(encoding='utf-8')


def setting(name):
    match = re.search(r'\b' + re.escape(name) + r'\[\]\s*=\s*"([^"\n]*)"', app_config)
    if not match:
        raise ValueError('Missing firmware setting: ' + name)
    return match.group(1)


HOST = '192.168.4.1'
PASSWORD = setting('kOtaPassword')
HEALTH = dict(firmware='robotfish', hardware='robotfish-esp32s3', project_name='robotfish',
              version='2.2.6-vision', runtime_ready=True, ota_confirmed=True,
              sta_connected=False, sta_ip='0.0.0.0', sta_ssid='', ap_ip=HOST,
              ssid=setting('kSsid'), wifi_control_allowed=True, control_protocol=1)
MOTION = dict(version=1, wifi_control_allowed=True, control_protocol=1,
              remote_active=False, mode=3, wifi=dict(session='', sequence=0))


class ConnectionTests(unittest.TestCase):
    def setUp(self):
        self.health = copy.deepcopy(HEALTH)
        self.motion = copy.deepcopy(MOTION)
        self.calls = []
        self.control_error = None
        self.unreachable = False
        self.client = device_client.DeviceClient()
        self.keys = Mock()
        self.keys.has.return_value = False
        self.addCleanup(patch.stopall)
        patch.object(device_client, 'KEYS', self.keys).start()
        patch.object(device_client.http.client, 'HTTPConnection', self.connection).start()
        patch('socket.socket', side_effect=AssertionError('Offline test attempted socket I/O')).start()

    def connection(self, host, port, timeout=None):
        self.assertEqual((host, port), (HOST, 80))
        link = Mock()

        def request(method, path, body=None, headers=None):
            self.calls.append((method, path))
            if self.unreachable:
                raise OSError('Simulated network timeout')
            status, data = 200, {}
            if (method, path) == ('GET', '/get_ip'):
                data = self.health
            elif (method, path) == ('GET', '/motion_status'):
                data = self.motion
            elif (method, path) == ('POST', '/api/v1/control'):
                command = json.loads(body)
                self.assertEqual(command['action'], 'acquire', 'Only connection acquisition is allowed')
                self.assertNotIn('axes', command)
                if headers.get('X-OTA-Password') != PASSWORD:
                    status, data = 401, dict(error='invalid_password')
                elif self.control_error:
                    status, data = self.control_error
                else:
                    self.motion['wifi']['session'] = command['session']
                    data = dict(accepted=True, session=command['session'], sequence=0, lease_active=False)
            else:
                self.fail('Unexpected endpoint: ' + method + ' ' + path)
            payload = json.dumps(data).encode()
            link.getresponse.return_value = SimpleNamespace(
                status=status, read=lambda limit: payload[:limit], getheader=lambda name: None)

        link.request.side_effect = request
        return link

    def login(self):
        return self.client.login(HOST, PASSWORD, session='offline-connection-test')

    def test_ap_only_device_connects_with_sta_disconnected(self):
        identity = self.client.identity(HOST)
        self.assertFalse(identity['sta_connected'])
        self.assertEqual(identity['sta_ip'], '0.0.0.0')
        self.assertTrue(self.login()['accepted'])
        self.assertTrue(self.client.robot(HOST)['control']['connected'])
        self.assertFalse(self.motion['remote_active'])
        self.keys.save.assert_not_called()

    def test_device_discovery_does_not_authenticate_control(self):
        self.assertFalse(self.client.identity(HOST)['authenticated'])
        self.assertFalse(self.client.robot(HOST)['control']['connected'])
        self.assertNotIn(('POST', '/api/v1/control'), self.calls)

    def test_wifi_password_is_not_control_password(self):
        wifi_password = setting('kWifiPassword')
        self.assertNotEqual(wifi_password, PASSWORD)
        with self.assertRaises(device_client.DeviceError):
            self.client.login(HOST, wifi_password, session='offline-connection-test')
        self.assertFalse(self.client.identity(HOST)['authenticated'])
        self.assertEqual(self.motion['wifi']['session'], '')

    def test_unreachable_network_fails_before_authentication(self):
        self.unreachable = True
        with self.assertRaises(device_client.DeviceError):
            self.client.identity(HOST)
        self.assertNotIn(('POST', '/api/v1/control'), self.calls)
        self.assertFalse(self.client._sessions)

    def test_old_firmware_without_wifi_control_cannot_connect(self):
        self.health['wifi_control_allowed'] = False
        with self.assertRaises(device_client.DeviceError):
            self.login()
        self.assertNotIn(('POST', '/api/v1/control'), self.calls)

    def test_runtime_not_ready_cannot_establish_control(self):
        self.health['runtime_ready'] = False
        self.control_error = (503, dict(error='runtime_not_ready'))
        with self.assertRaisesRegex(device_client.DeviceError, 'runtime_not_ready'):
            self.login()
        self.assertFalse(self.client._sessions)

    def test_replaced_session_is_detected_without_reacquiring(self):
        self.login()
        self.motion['wifi']['session'] = 'another-client'
        self.assertFalse(self.client.robot(HOST)['control']['connected'])
        self.assertEqual(self.calls.count(('POST', '/api/v1/control')), 1)

    def test_firmware_ap_configuration_and_http_endpoints(self):
        source = (project / 'main/network.cpp').read_text(encoding='utf-8')
        self.assertEqual(re.findall(r'esp_wifi_set_mode\(\s*(\w+)\s*\)', source), ['WIFI_MODE_AP'])
        self.assertTrue(1 <= len(setting('kSsid').encode()) <= 32)
        self.assertTrue(8 <= len(setting('kWifiPassword').encode()) <= 63)
        routes = re.findall(r'\.uri\s*=\s*"([^"]+)"', source)
        self.assertIn('/get_ip', routes)
        self.assertIn('/api/v1/control', routes)
        self.assertNotIn('/api/v1/connect', routes)


if __name__ == '__main__':
    unittest.main(argv=[sys.argv[0]] + remaining)
