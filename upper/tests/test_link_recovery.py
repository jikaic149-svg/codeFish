"""Fault injection: authentication survives jitter; motion never replays itself."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
import threading
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import windows_network
from station import Workstation
from test_realtime import RAW

HOST, SESSION = '192.168.4.1', 'fixture-session'
AXES = dict(surge=35, yaw=0, heave=0)


class LinkRecoveryTests(unittest.TestCase):
    def client(self, dual=False):
        client = device_client.DeviceClient()
        client._profiles[HOST] = 'robotfish'
        client._capabilities[HOST] = {'lora_protocol': 2} if dual else {}
        client._sessions[HOST] = dict(control_session=SESSION, password='fixture-secret', sequence=0)
        return client

    def test_page_sequences_are_independent_and_wire_sequences_are_shared(self):
        for dual in (False, True):
            with self.subTest(dual=dual):
                client, calls = self.client(dual), []
                client._motion_latest[HOST] = (dict(wifi=dict(session=SESSION, sequence=0),
                    lora=dict(healthy=False), mode=2), device_client.time.monotonic())
                def request(host, path, method, body, **kwargs):
                    calls.append(copy.deepcopy(body))
                    return 200, dict(accepted=True, session=SESSION, sequence=body['sequence']), None
                client._request = request
                client.motion_control(HOST, 'target', SESSION, 20, client_id='first-page', axes=AXES)
                stopped = client.motion_control(HOST, 'stop', SESSION, 1, client_id='second-page')
                self.assertTrue(stopped['accepted'])
                client.motion_control(HOST, 'mode', SESSION, 2, client_id='second-page', mode=2)
                client.motion_control(HOST, 'target', SESSION, 21, client_id='first-page', axes=AXES)
                self.assertEqual([call['sequence'] for call in calls], [1, 2, 3, 4])
                self.assertEqual([call['action'] for call in calls], ['target', 'stop', 'mode', 'target'])
                self.assertTrue(all('client_id' not in call for call in calls))
                for page, sequence in (('first-page', 20), ('second-page', 2)):
                    with self.assertRaises(device_client.DeviceError) as raised:
                        client.motion_control(HOST, 'target', SESSION, sequence, client_id=page, axes=AXES)
                    self.assertEqual(raised.exception.code, 'stale_client_sequence')
                self.assertEqual(len(calls), 4)

    def test_invalid_page_ids_cannot_bypass_request_ordering(self):
        client = self.client()
        with patch.object(client, '_request', return_value=(200, {'accepted': True}, None)) as request:
            for value in ('', 'x' * 65, 7, False, 'page\n'):
                with self.subTest(value=value), self.assertRaises(device_client.DeviceError):
                    client.motion_control(HOST, 'stop', SESSION, 1, client_id=value)
            self.assertFalse(request.called)

    def test_lost_reply_keeps_session_and_blocks_goals_until_confirmed_stop(self):
        for dual in (False, True):
            with self.subTest(dual=dual):
                client, calls = self.client(dual), []
                def request(host, path, method, body, **kwargs):
                    calls.append(copy.deepcopy(body))
                    if len(calls) == 1:
                        raise device_client.DeviceError('fixture lost reply', code='transport_unavailable', retryable=True)
                    return 200, dict(accepted=True, session=SESSION, sequence=body['sequence']), None
                client._request = request
                # Drive the actual Wi-Fi sender once; the MCU may already have accepted it.
                with self.assertRaises(device_client.DeviceError):
                    client.wifi_control(HOST, 'target', SESSION, 1, axes=AXES)
                self.assertTrue(client._sessions[HOST]['recovery_required'])
                self.assertEqual(client._sessions[HOST]['password'], 'fixture-secret')
                with self.assertRaisesRegex(device_client.DeviceError, '全停'):
                    client.motion_control(HOST, 'target', SESSION, 2, axes=AXES)
                self.assertEqual(len(calls), 1)
                # Fresh telemetry reports a larger sequence after a lost acknowledgment.
                client._motion_latest[HOST] = (dict(wifi=dict(session=SESSION, sequence=30), lora=dict(healthy=False), mode=2),
                                               device_client.time.monotonic())
                stopped = client.motion_control(HOST, 'stop', SESSION, 3)
                self.assertTrue(stopped['accepted'])
                self.assertEqual(calls[-1]['sequence'], 31)
                self.assertNotIn('recovery_required', client._sessions[HOST])
                new_axes = dict(surge=-10, yaw=20, heave=0)
                client.motion_control(HOST, 'target', SESSION, 4, axes=new_axes)
                self.assertEqual([entry['action'] for entry in calls], ['target', 'stop', 'target'])
                self.assertEqual(calls[-1]['axes'], new_axes)
                self.assertEqual(calls[-1]['sequence'], 32)
                with self.assertRaises(device_client.DeviceError):
                    client.motion_control(HOST, 'target', SESSION, 4, axes=AXES)
                self.assertEqual(len(calls), 3)

    def test_transient_and_permanent_control_errors_have_different_outcomes(self):
        for status, code, retryable, retained in (
                (503, 'control_timeout', True, True), (503, 'control_busy', True, True),
                (409, 'stale_sequence', True, True), (409, 'lora_primary_active', True, True),
                (409, 'command_expired', True, True),
                (409, 'neutral_pending', True, True), (401, 'invalid_password', False, False),
                (409, 'control_session_changed', False, False), (409, 'output_fault', False, False),
                (409, 'ota_locked', False, False)):
            with self.subTest(code=code):
                client = self.client()
                calls = []
                def request(*args, **kwargs):
                    calls.append(args)
                    return status, {'error': code}, None
                client._request = request
                with self.assertRaises(device_client.DeviceError) as raised:
                    client.wifi_control(HOST, 'target', SESSION, 1, axes=AXES)
                self.assertEqual((raised.exception.code, raised.exception.retryable), (code, retryable))
                self.assertEqual(HOST in client._sessions, retained)
                self.assertEqual(len(calls), 1)  # Never retry or reacquire in the sender.
                if code in ('lora_primary_active', 'neutral_pending'):
                    self.assertNotIn('recovery_required', client._sessions[HOST])

    def test_sample_started_before_login_cannot_revoke_the_new_session(self):
        client = self.client()
        raw = dict(copy.deepcopy(RAW), wifi_control_allowed=True, control_protocol=1,
                   wifi=dict(session='previous-session', sequence=0))
        def request(*args, **kwargs):
            client._sessions[HOST] = dict(control_session=SESSION, password='new-secret', sequence=0)
            return 200, copy.deepcopy(raw), None
        client._request = request
        view = client.robot(HOST)
        self.assertFalse(view['control']['connected'])
        self.assertFalse(view['control']['session_confirmed'])
        raw['wifi']['session'] = SESSION
        client._request = lambda *args, **kwargs: (200, copy.deepcopy(raw), None)
        view = client.robot(HOST)
        self.assertTrue(view['control']['connected'])
        self.assertTrue(view['control']['session_confirmed'])

    def test_telemetry_failure_discards_queued_lora_goal_but_keeps_stop(self):
        client = self.client(dual=True)
        def request(*args, **kwargs):
            raise device_client.DeviceError('fixture outage', code='transport_unavailable', retryable=True)
        client._request = request
        client.lora.pending = ('target', {'mode': 2, 'axes': AXES}, 0)
        with self.assertRaises(device_client.DeviceError):
            client.robot(HOST)
        self.assertIsNone(client.lora.pending)
        self.assertTrue(client._sessions[HOST]['recovery_required'])
        client.lora.pending = ('stop', {'mode': 2}, 0)
        with self.assertRaises(device_client.DeviceError):
            client.robot(HOST)
        self.assertEqual(client.lora.pending[0], 'stop')

    def test_slow_control_does_not_block_reads_or_allow_a_host_switch_race(self):
        workstation = Workstation()
        workstation.host = HOST
        entered, release = threading.Event(), threading.Event()
        calls = []
        def motion_control(host, *args, **kwargs):
            calls.append(host)
            entered.set()
            if not release.wait(3):
                raise AssertionError('fixture was not released')
            return {'accepted': True}
        client = SimpleNamespace(is_robotfish=lambda host=None: True, motion_control=motion_control,
                                 robot=lambda host: {'firmware': 'robotfish', 'host': host, 'online': True})
        with patch('station.device_client.CLIENT', client), ThreadPoolExecutor(max_workers=3) as executor:
            command = executor.submit(workstation.target, AXES, SESSION, 1, 'wifi')
            self.assertTrue(entered.wait(2))
            try:
                snapshot = executor.submit(workstation.read).result(timeout=.5)
                self.assertEqual(snapshot['host'], HOST)
                switched = executor.submit(workstation.configure, 'mission', '192.168.4.2')
                self.assertFalse(switched.done())
                self.assertEqual(workstation.host, HOST)
            finally:
                release.set()
            self.assertTrue(command.result(timeout=2)['accepted'])
            self.assertEqual(switched.result(timeout=2)['host'], '192.168.4.2')
        self.assertEqual(calls, [HOST])

    def test_fresh_takeover_is_reported_without_auto_acquire(self):
        client, calls = self.client(), []
        raw = dict(copy.deepcopy(RAW), wifi=dict(session='another-operator', sequence=50))
        def request(*args, **kwargs):
            calls.append(args)
            return 200, raw, None
        client._request = request
        view = client.robot(HOST)
        self.assertTrue(view['control']['session_confirmed'])
        self.assertFalse(view['control']['connected'])
        self.assertEqual([entry[1] for entry in calls], ['/motion_status'])

    def test_failed_reads_are_coalesced_and_backoff_is_reset_by_success(self):
        client, calls, now = self.client(), [], [100.0]
        def failure(*args, **kwargs):
            calls.append(args)
            raise device_client.DeviceError('fixture outage', code='transport_unavailable', retryable=True)
        client._request = failure
        with patch('device_client.time.monotonic', side_effect=lambda: now[0]):
            for _ in range(8):
                with self.assertRaises(device_client.DeviceError):
                    client.robot(HOST)
            self.assertEqual(len(calls), 1)
            now[0] += .3
            with self.assertRaises(device_client.DeviceError):
                client.robot(HOST)
            self.assertEqual(len(calls), 2)
            self.assertAlmostEqual(client._telemetry_failures[HOST]['retry_at'], 100.8)
            now[0] += .6
            raw = dict(copy.deepcopy(RAW), wifi=dict(session=SESSION, sequence=0))
            client._request = lambda *args, **kwargs: (200, raw, None)
            self.assertTrue(client.robot(HOST)['control']['connected'])
            self.assertNotIn(HOST, client._telemetry_failures)

    def test_new_login_bypasses_an_old_connection_backoff(self):
        client = self.client()
        client._request = lambda *args, **kwargs: (_ for _ in ()).throw(
            device_client.DeviceError('fixture outage', code='transport_unavailable', retryable=True))
        with self.assertRaises(device_client.DeviceError):
            client.robot(HOST)
        client._sessions[HOST] = dict(control_session='new-session', password='new-secret', sequence=0)
        raw = dict(copy.deepcopy(RAW), wifi=dict(session='new-session', sequence=0))
        client._request = lambda *args, **kwargs: (200, raw, None)
        view = client.robot(HOST)
        self.assertTrue(view['control']['connected'])
        self.assertTrue(view['control']['session_confirmed'])

    def test_windows_capture_is_read_only_throttled_and_off_request_thread(self):
        snapshot = {'interfaces': [{'name': 'fixture', 'ipv4': ['192.168.4.2']}], 'events': []}
        original = windows_network._last_capture
        windows_network._last_capture = float('-inf')
        try:
            with patch('windows_network.os.name', 'nt'), patch('windows_network.diagnostics.active', return_value=True), \
                    patch('windows_network.threading.Thread') as thread, \
                    patch('windows_network.time.monotonic', return_value=100), \
                    patch('windows_network.subprocess.run', return_value=subprocess.CompletedProcess([], 0, json.dumps(snapshot), '')) as run, \
                    patch('windows_network.diagnostics.event') as event:
                windows_network.capture()
                windows_network.capture()
                thread.assert_called_once()
                run.assert_not_called()
                thread.call_args.kwargs['target']()
                self.assertEqual(run.call_args.kwargs['timeout'], 8)
                command = run.call_args.args[0][-1]
                self.assertIn('Get-WinEvent', command)
                for forbidden in ('Set-Net', 'Restart-', 'Disable-', 'netsh', 'ipconfig', 'Invoke-WebRequest'):
                    self.assertNotIn(forbidden, command)
                event.assert_called_with('windows_network', **snapshot)
                windows_network.capture()
                thread.assert_called_once()  # Completed queries are rate limited as well.
        finally:
            windows_network._last_capture = original


if __name__ == '__main__':
    unittest.main()
