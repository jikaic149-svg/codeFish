"""Attitude configuration stays ordered and uses the existing command routes."""
import binascii
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import device_client
import robotfish
from lora_link import frame
from test_realtime import RAW

HOST, SESSION = '192.168.4.1', 'attitude-fixture'


class AttitudeTests(unittest.TestCase):
    def client(self, dual=True):
        client = device_client.DeviceClient()
        client._profiles[HOST] = 'robotfish'
        client._capabilities[HOST] = dict(attitude_control_supported=True, attitude_control_calibrated=True, lora_protocol=2 if dual else 0)
        client._sessions[HOST] = dict(control_session=SESSION, password='fixture-secret', sequence=0)
        client._motion_latest[HOST] = (dict(mode=2, wifi=dict(session=SESSION, sequence=0),
            lora=dict(healthy=False)), device_client.time.monotonic())
        return client

    def test_attitude_frames_match_firmware_golden_packets(self):
        for action, expected in (
                ('attitude_on', b'101,1,23120,42,23,3,0,0,0,1000,7010\n'),
                ('attitude_off', b'101,1,23120,42,24,3,0,0,0,1000,27492\n')):
            wire = frame(42, action, {'mode': 3})
            self.assertEqual(wire, expected)
            body, crc = wire.strip().rsplit(b',', 1)
            self.assertEqual(binascii.crc_hqx(body, 0xffff), int(crc))

    def test_wifi_and_primary_lora_configuration_routes(self):
        for dual in (False, True):
            with self.subTest(dual=dual):
                client, calls = self.client(dual), []
                def request(host, path, method, body, **options):
                    calls.append(body)
                    return 200, dict(accepted=True, session=SESSION, sequence=body['sequence']), None
                client._request = request
                for sequence, action in enumerate(('attitude_on', 'attitude_off'), 1):
                    self.assertTrue(client.motion_control(HOST, action, SESSION, sequence, client_id='page')['accepted'])
                self.assertEqual([entry['action'] for entry in calls], ['attitude_on', 'attitude_off'])
                self.assertTrue(all(set(entry) == {'action', 'session', 'sequence'} for entry in calls))
        client, queued = self.client(), []
        client._motion_latest[HOST][0]['lora']['healthy'] = True
        client.lora = SimpleNamespace(status=lambda: dict(connected=True, host=HOST),
            submit=lambda host, action, payload: queued.append((action, payload)) or dict(queued=True))
        with patch.object(client, '_request') as request:
            self.assertTrue(client.motion_control(HOST, 'attitude_on', SESSION, 1)['queued'])
            self.assertEqual(queued, [('attitude_on', {'mode': 2})])
            request.assert_not_called()

    def test_old_firmware_does_not_receive_unsupported_configuration(self):
        client = self.client()
        client._capabilities[HOST].pop('attitude_control_supported')
        with patch.object(client, '_request') as request:
            with self.assertRaises(device_client.DeviceError) as raised:
                client.motion_control(HOST, 'attitude_on', SESSION, 1)
            self.assertEqual(raised.exception.code, 'unsupported_control')
            request.assert_not_called()
        self.assertIn(HOST, client._sessions)

    def test_uncalibrated_profile_cannot_be_enabled_but_can_be_disabled(self):
        client = self.client()
        client._capabilities[HOST]['attitude_control_calibrated'] = False
        with patch.object(client, '_request') as request:
            with self.assertRaises(device_client.DeviceError) as raised:
                client.motion_control(HOST, 'attitude_on', SESSION, 1)
            self.assertEqual(raised.exception.code, 'attitude_not_calibrated')
            request.assert_not_called()
            request.return_value = (200, dict(accepted=True, session=SESSION, sequence=1), None)
            self.assertTrue(client.motion_control(HOST, 'attitude_off', SESSION, 1)['accepted'])
        self.assertIn(HOST, client._sessions)

    def test_slow_telemetry_cannot_publish_active_or_fresh_feedback(self):
        raw = copy.deepcopy(RAW)
        raw['attitude_control_supported'] = True
        raw['attitude_control'] = dict(enabled=True, reason='active', body_valid=True,
            pitch_calibrated=True, roll_calibrated=True, pitch_active=True, roll_active=True,
            yaw_calibrated=True, yaw_active=True, heading_valid=True, heading_locked=True,
            age_ms=20, maximum_age_ms=200, body_roll_deg=12, body_pitch_deg=-8,
            body_yaw_deg=30, heading_target_deg=25, yaw_correction_pct=-5,
            pitch_correction_pct=8.2, roll_correction_pct=-12.4)
        fresh = robotfish.normalize(raw, HOST)['attitude_control']
        self.assertTrue(fresh['pitch_active'] and fresh['roll_active'])
        self.assertEqual(fresh['body_roll_deg'], 12)
        self.assertTrue(fresh['heading_locked'] and fresh['yaw_active'])
        self.assertEqual(fresh['heading_target_deg'], 25)
        stale = robotfish.normalize(raw, HOST, 250)['attitude_control']
        self.assertTrue(stale['enabled'])
        self.assertFalse(stale['body_valid'] or stale['pitch_active'] or stale['roll_active'])
        self.assertIsNone(stale['body_roll_deg'])
        self.assertIsNone(stale['pitch_correction_pct'])
        self.assertFalse(stale['heading_locked'] or stale['yaw_active'])
        self.assertIsNone(stale['heading_target_deg'])
        self.assertFalse(robotfish.normalize(RAW, HOST)['attitude_control']['supported'])


if __name__ == '__main__':
    unittest.main()
