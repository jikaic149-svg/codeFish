"""Automatic connection uses verified local candidates and never sends motion."""
import copy
import json
import os
from pathlib import Path
import re
import socket
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import robotfish
from credentials import KeyStore
from test_robotfish import HEALTH, MOTION
from windows_network import _route_gateways


class Keys:
    def __init__(self): self.values, self.saves = {}, []
    def has(self, host): return host in self.values
    def get(self, host): return self.values.get(host, '')
    def save(self, host, password, **options):
        self.values[host] = password
        self.saves.append(options)


class AutoConnectTests(unittest.TestCase):
    def setUp(self):
        self.client, self.keys, self.calls = device_client.DeviceClient(), Keys(), []
        self.raw = copy.deepcopy(MOTION)
        self.password = robotfish.DEFAULT_CONTROL_PASSWORD
        self.key_patch = patch('device_client.KEYS', self.keys)
        self.key_patch.start()
        self.addCleanup(self.key_patch.stop)
        self.client._request = self.request

    def request(self, host, path, method='GET', body=None, **kwargs):
        self.calls.append((host, path, method, copy.deepcopy(body), kwargs))
        if path == '/get_ip': return 200, dict(HEALTH), None
        if path == '/motion_status': return 200, copy.deepcopy(self.raw), None
        if path == '/api/v1/control':
            if kwargs.get('password') != self.password:
                return 401, {'error': 'invalid_password'}, None
            self.raw['wifi']['session'] = body['session']
            return 200, dict(accepted=True, session=body['session'], sequence=0), None
        return 404, {'error': 'not_found'}, None

    def test_default_connect_requires_no_password_argument_and_returns_no_password(self):
        result = self.client.auto_connect('192.168.4.1', 'first-browser')
        self.assertTrue(result['accepted'])
        self.assertEqual(result['session'], 'first-browser')
        writes = [call for call in self.calls if call[2] == 'POST']
        self.assertEqual([call[3]['action'] for call in writes], ['acquire'])
        self.assertEqual(writes[0][4]['password'], robotfish.DEFAULT_CONTROL_PASSWORD)
        self.assertNotIn(robotfish.DEFAULT_CONTROL_PASSWORD, json.dumps(result))
        self.assertFalse(self.keys.saves)

    def test_second_page_reuses_the_existing_connection_without_acquire(self):
        self.client.auto_connect('192.168.4.1', 'first-browser')
        self.client._sessions['192.168.4.1']['client_sequence'] = 35
        result = self.client.auto_connect('192.168.4.1', 'second-browser')
        self.assertEqual((result['session'], result['sequence']), ('first-browser', 35))
        self.assertEqual(len([call for call in self.calls if call[2] == 'POST']), 1)

    def test_old_saved_probe_key_does_not_force_typing_the_factory_key(self):
        self.keys.values['192.168.4.1'] = 'previous-probe-key'
        result = self.client.auto_connect('192.168.4.1', 'factory-fish')
        self.assertTrue(result['accepted'])
        writes = [call for call in self.calls if call[2] == 'POST']
        self.assertEqual([call[4]['password'] for call in writes], ['previous-probe-key', robotfish.DEFAULT_CONTROL_PASSWORD])
        self.assertEqual([call[3]['action'] for call in writes], ['acquire', 'acquire'])

    def test_automatic_connection_does_not_take_over_active_motion(self):
        self.raw.update(remote_active=True)
        self.raw['wifi']['session'] = 'another-operator'
        with self.assertRaises(device_client.DeviceError) as raised:
            self.client.auto_connect('192.168.4.1', 'new-browser')
        self.assertEqual(raised.exception.code, 'control_session_changed')
        self.assertFalse(any(call[2] == 'POST' for call in self.calls))
        self.client.auto_connect('192.168.4.1', 'new-browser', force=True)
        self.assertEqual([call[3]['action'] for call in self.calls if call[2] == 'POST'], ['acquire'])

    def test_idle_remote_session_requires_explicit_takeover(self):
        self.raw['wifi']['session'] = 'idle-operator'
        self.assertFalse(self.raw['remote_active'])
        with self.assertRaises(device_client.DeviceError) as raised:
            self.client.auto_connect('192.168.4.1', 'new-browser')
        self.assertEqual(raised.exception.code, 'control_session_changed')
        self.assertFalse(any(call[2] == 'POST' for call in self.calls))
        self.assertTrue(self.client.auto_connect('192.168.4.1', 'new-browser', force=True)['accepted'])

    def test_custom_password_is_entered_once_then_loaded_from_the_key_store(self):
        self.password = 'new-key'
        with self.assertRaises(device_client.DeviceError) as raised:
            self.client.auto_connect('192.168.4.1', 'default-attempt')
        self.assertEqual(raised.exception.code, 'custom_password_required')
        self.assertEqual(len([call for call in self.calls if call[2] == 'POST']), 1)
        self.client.login('192.168.4.1', self.password, persist=True, session='manual-once')
        self.assertEqual(self.keys.saves, [{'minimum_length': 1}])
        self.client.forget('192.168.4.1')
        # A restarted fish has no controller; a still-owned session requires a click.
        self.raw['wifi']['session'] = ''
        result = self.client.auto_connect('192.168.4.1', 'next-start')
        self.assertTrue(result['accepted'])
        self.assertEqual(self.calls[-1][4]['password'], self.password)

    def test_discovery_only_reads_bounded_private_gateway_candidates(self):
        def request(host, path, method='GET', body=None, **kwargs):
            self.calls.append((host, path, method, body, kwargs))
            return 200, dict(HEALTH) if host == '192.168.4.1' else {'firmware': 'router'}, None
        self.client._request = request
        with patch('device_client.gateways', return_value=['10.0.0.1', '192.168.4.1', '8.8.8.8']):
            found = self.client.discover_robotfish()
        self.assertEqual(found['host'], '192.168.4.1')
        self.assertEqual([call[0] for call in self.calls], ['10.0.0.1', '192.168.4.1'])
        self.assertTrue(all(call[1:4] == ('/get_ip', 'GET', None) for call in self.calls))
        self.assertTrue(all(call[4] == {'timeout': .45, 'diagnose': False} for call in self.calls))

    def test_unverified_device_never_receives_a_password(self):
        def request(host, path, method='GET', body=None, **kwargs):
            self.calls.append((host, path, method, body, kwargs))
            return 200, {'firmware': 'router'}, None
        self.client._request = request
        with self.assertRaises(device_client.DeviceError):
            self.client.auto_connect('192.168.4.1', 'browser')
        self.assertTrue(all('password' not in call[4] and call[2] == 'GET' for call in self.calls))

    def test_default_key_matches_the_current_firmware_configuration(self):
        source = (Path(__file__).resolve().parents[2] / 'main/config/AppConfig.h').read_text(encoding='utf-8')
        self.assertEqual(robotfish.DEFAULT_CONTROL_PASSWORD, re.search(r'kOtaPassword\[\]\s*=\s*"([^"]+)"', source)[1])

    def test_gateway_table_parser_handles_order_duplicates_and_bad_lengths(self):
        def row(address, metric=10, destination=0, kind=4):
            values = [0] * 14
            values[0], values[3], values[5], values[9] = destination, struct.unpack('=I', socket.inet_aton(address))[0], kind, metric
            return struct.pack('=14I', *values)
        data = struct.pack('=I', 4) + row('10.0.0.1', 20) + row('192.168.4.1', 10) + row('192.168.4.1', 30) + row('10.0.0.2', 1, kind=2)
        self.assertEqual(_route_gateways(data), ['192.168.4.1', '10.0.0.1'])
        self.assertEqual(_route_gateways(data[:-1]), [])
        self.assertEqual(_route_gateways(b''), [])

    @unittest.skipUnless(os.name == 'nt', 'Windows DPAPI')
    def test_short_robotfish_password_is_encrypted_while_legacy_policy_stays(self):
        with tempfile.TemporaryDirectory() as directory:
            keys = KeyStore(Path(directory) / 'keys.json')
            with self.assertRaises(RuntimeError):
                keys.save('192.168.4.1', 'new-key')
            keys.save('192.168.4.1', 'new-key', minimum_length=1)
            self.assertEqual(keys.get('192.168.4.1'), 'new-key')
            self.assertNotIn('new-key', keys.path.read_text(encoding='utf-8'))


if __name__ == '__main__':
    unittest.main()
